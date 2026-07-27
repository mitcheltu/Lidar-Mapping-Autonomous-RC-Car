//
//  ARDepthView.swift
//  PointCloudScanner
//
//  Wraps an ARSCNView. On every few ARKit frames it takes the LiDAR depth map,
//  unprojects each depth pixel into world space, samples color from the camera
//  image, and feeds the point into the voxel accumulator. The accumulated cloud
//  is rendered live as SceneKit points on top of the camera feed.
//

import SwiftUI
import ARKit
import SceneKit
import CoreImage
import ImageIO

struct ARDepthView: UIViewRepresentable {
    let accumulator: PointCloudAccumulator
    let streamer: PointCloudStreamer
    @Binding var isScanning: Bool
    let onPointCountUpdate: (Int) -> Void

    func makeCoordinator() -> Coordinator {
        Coordinator(accumulator: accumulator, streamer: streamer, onPointCountUpdate: onPointCountUpdate)
    }

    func makeUIView(context: Context) -> ARSCNView {
        let view = ARSCNView(frame: .zero)
        view.session.delegate = context.coordinator
        view.automaticallyUpdatesLighting = true
        view.scene.rootNode.addChildNode(context.coordinator.pointNode)
        context.coordinator.sceneView = view

        let config = ARWorldTrackingConfiguration()
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            config.frameSemantics.insert(.sceneDepth)
        }
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.smoothedSceneDepth) {
            config.frameSemantics.insert(.smoothedSceneDepth)
        }
        view.session.run(config)
        return view
    }

    func updateUIView(_ uiView: ARSCNView, context: Context) {
        context.coordinator.isScanning = isScanning
    }

    static func dismantleUIView(_ uiView: ARSCNView, coordinator: Coordinator) {
        uiView.session.pause()
    }

    // MARK: - Coordinator

    final class Coordinator: NSObject, ARSessionDelegate {
        let accumulator: PointCloudAccumulator
        let streamer: PointCloudStreamer
        let onPointCountUpdate: (Int) -> Void
        weak var sceneView: ARSCNView?
        let pointNode = SCNNode()
        var isScanning = true

        private let ciContext = CIContext(options: nil)
        private var lastPoseTime: TimeInterval = 0
        private var lastImageTime: TimeInterval = 0

        private let processingQueue = DispatchQueue(label: "pointcloud.processing", qos: .userInitiated)
        private let stateLock = NSLock()
        private var _isProcessing = false
        private var isProcessing: Bool {
            get { stateLock.lock(); defer { stateLock.unlock() }; return _isProcessing }
            set { stateLock.lock(); _isProcessing = newValue; stateLock.unlock() }
        }
        private var frameCounter = 0
        private var lastRenderTime: TimeInterval = 0

        // Tuning knobs.
        private let frameStride = 3        // process roughly every 3rd frame
        private let pixelStride = 2        // subsample the depth map by this factor
        private let maxDepth: Float = 5.0  // meters; iPhone LiDAR is reliable to ~5 m
        private let minConfidence: UInt8 = 1  // 0 = low, 1 = medium, 2 = high

        init(accumulator: PointCloudAccumulator,
             streamer: PointCloudStreamer,
             onPointCountUpdate: @escaping (Int) -> Void) {
            self.accumulator = accumulator
            self.streamer = streamer
            self.onPointCountUpdate = onPointCountUpdate
        }

        func session(_ session: ARSession, didUpdate frame: ARFrame) {
            guard isScanning else { return }
            frameCounter += 1
            guard frameCounter % frameStride == 0 else { return }
            guard !isProcessing else { return }
            guard let depth = frame.smoothedSceneDepth ?? frame.sceneDepth else { return }

            // Copy everything we need out of the frame; ARKit recycles ARFrames.
            let depthMap = depth.depthMap
            let confidenceMap = depth.confidenceMap
            let capturedImage = frame.capturedImage
            let intrinsics = frame.camera.intrinsics
            let cameraTransform = frame.camera.transform
            let imageResolution = frame.camera.imageResolution

            isProcessing = true
            processingQueue.async { [weak self] in
                guard let self = self else { return }
                self.process(depthMap: depthMap,
                             confidenceMap: confidenceMap,
                             capturedImage: capturedImage,
                             intrinsics: intrinsics,
                             cameraTransform: cameraTransform,
                             imageResolution: imageResolution)
                self.isProcessing = false
            }
        }

        // MARK: Depth -> world points

        private func process(depthMap: CVPixelBuffer,
                             confidenceMap: CVPixelBuffer?,
                             capturedImage: CVPixelBuffer,
                             intrinsics: simd_float3x3,
                             cameraTransform: simd_float4x4,
                             imageResolution: CGSize) {

            let depthWidth = CVPixelBufferGetWidth(depthMap)
            let depthHeight = CVPixelBufferGetHeight(depthMap)

            CVPixelBufferLockBaseAddress(depthMap, .readOnly)
            defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }
            if let cm = confidenceMap { CVPixelBufferLockBaseAddress(cm, .readOnly) }
            defer { if let cm = confidenceMap { CVPixelBufferUnlockBaseAddress(cm, .readOnly) } }
            CVPixelBufferLockBaseAddress(capturedImage, .readOnly)
            defer { CVPixelBufferUnlockBaseAddress(capturedImage, .readOnly) }

            guard let depthBase = CVPixelBufferGetBaseAddress(depthMap) else { return }
            let depthRowBytes = CVPixelBufferGetBytesPerRow(depthMap)

            let confBase = confidenceMap.flatMap { CVPixelBufferGetBaseAddress($0) }
            let confRowBytes = confidenceMap.map { CVPixelBufferGetBytesPerRow($0) } ?? 0

            // Captured image is YCbCr biplanar (plane 0 = luma, plane 1 = CbCr).
            let yBase = CVPixelBufferGetBaseAddressOfPlane(capturedImage, 0)
            let yRowBytes = CVPixelBufferGetBytesPerRowOfPlane(capturedImage, 0)
            let cbcrBase = CVPixelBufferGetBaseAddressOfPlane(capturedImage, 1)
            let cbcrRowBytes = CVPixelBufferGetBytesPerRowOfPlane(capturedImage, 1)
            let imgWidth = CVPixelBufferGetWidth(capturedImage)
            let imgHeight = CVPixelBufferGetHeight(capturedImage)

            // The camera intrinsics belong to the full-res captured image; scale
            // them down to the (smaller) depth-map resolution.
            let scaleX = Float(depthWidth) / Float(imageResolution.width)
            let scaleY = Float(depthHeight) / Float(imageResolution.height)
            let fx = intrinsics[0][0] * scaleX
            let fy = intrinsics[1][1] * scaleY
            let cx = intrinsics[2][0] * scaleX
            let cy = intrinsics[2][1] * scaleY

            for row in stride(from: 0, to: depthHeight, by: pixelStride) {
                let depthRow = (depthBase + row * depthRowBytes).assumingMemoryBound(to: Float32.self)
                let confRow = confBase.map { ($0 + row * confRowBytes).assumingMemoryBound(to: UInt8.self) }

                for col in stride(from: 0, to: depthWidth, by: pixelStride) {
                    let depth = depthRow[col]
                    if depth <= 0 || depth.isNaN || depth > maxDepth { continue }
                    if let confRow = confRow, confRow[col] < minConfidence { continue }

                    // Camera space (ARKit convention: +x right, +y up, -z forward).
                    let x = (Float(col) - cx) / fx * depth
                    let y = (Float(row) - cy) / fy * depth
                    let local = SIMD4<Float>(x, -y, -depth, 1)
                    let world4 = cameraTransform * local
                    let world = SIMD3<Float>(world4.x, world4.y, world4.z)

                    // Sample color from the captured image at the matching pixel.
                    var color = SIMD3<Float>(0.7, 0.7, 0.7)
                    let imgX = Int(Float(col) / scaleX)
                    let imgY = Int(Float(row) / scaleY)
                    if let yBase = yBase, let cbcrBase = cbcrBase,
                       imgX >= 0, imgY >= 0, imgX < imgWidth, imgY < imgHeight {
                        let yVal = Float((yBase + imgY * yRowBytes + imgX)
                            .assumingMemoryBound(to: UInt8.self).pointee)
                        let cbcrX = (imgX / 2) * 2
                        let cbcrPtr = (cbcrBase + (imgY / 2) * cbcrRowBytes + cbcrX)
                            .assumingMemoryBound(to: UInt8.self)
                        let cb = Float(cbcrPtr.pointee) - 128.0
                        let cr = Float((cbcrPtr + 1).pointee) - 128.0
                        let r = (yVal + 1.402 * cr) / 255.0
                        let g = (yVal - 0.344136 * cb - 0.714136 * cr) / 255.0
                        let b = (yVal + 1.772 * cb) / 255.0
                        color = SIMD3<Float>(min(max(r, 0), 1), min(max(g, 0), 1), min(max(b, 0), 1))
                    }

                    accumulator.add(position: world, color: color)
                }
            }

            // Stream to the computer (new points every frame; pose + video throttled).
            if streamer.isStreaming {
                streamer.sendPoints(accumulator.drainNew())
                let ts = CACurrentMediaTime()
                if ts - lastPoseTime > 0.1 {
                    lastPoseTime = ts
                    streamer.sendPose(cameraTransform)
                }
                if ts - lastImageTime > 0.2 {
                    lastImageTime = ts
                    sendJPEG(capturedImage)
                }
            }

            // Throttle the (relatively expensive) geometry rebuild.
            let now = CACurrentMediaTime()
            if now - lastRenderTime > 0.3 {
                lastRenderTime = now
                let verts = accumulator.snapshot()
                DispatchQueue.main.async { [weak self] in
                    self?.updateGeometry(with: verts)
                    self?.onPointCountUpdate(verts.count)
                }
            }
        }

        // MARK: Streaming helper

        private func sendJPEG(_ pixelBuffer: CVPixelBuffer) {
            let ci = CIImage(cvPixelBuffer: pixelBuffer)
            let scale = 480.0 / max(ci.extent.width, 1)
            let scaled = ci.transformed(by: CGAffineTransform(scaleX: scale, y: scale))
            let opts: [CIImageRepresentationOption: Any] =
                [CIImageRepresentationOption(rawValue: kCGImageDestinationLossyCompressionQuality as String): 0.4]
            if let data = ciContext.jpegRepresentation(of: scaled,
                                                       colorSpace: CGColorSpaceCreateDeviceRGB(),
                                                       options: opts) {
                streamer.sendImage(data)
            }
        }

        // MARK: Rendering

        private func updateGeometry(with points: [PointVertex]) {
            guard !points.isEmpty else { return }

            var positions = [SIMD3<Float>]()
            var colors = [SIMD3<Float>]()
            positions.reserveCapacity(points.count)
            colors.reserveCapacity(points.count)
            for p in points {
                positions.append(p.position)
                colors.append(p.color)
            }

            let vertexData = positions.withUnsafeBytes { Data($0) }
            let vertexSource = SCNGeometrySource(
                data: vertexData,
                semantic: .vertex,
                vectorCount: positions.count,
                usesFloatComponents: true,
                componentsPerVector: 3,
                bytesPerComponent: MemoryLayout<Float>.size,
                dataOffset: 0,
                dataStride: MemoryLayout<SIMD3<Float>>.stride)

            let colorData = colors.withUnsafeBytes { Data($0) }
            let colorSource = SCNGeometrySource(
                data: colorData,
                semantic: .color,
                vectorCount: colors.count,
                usesFloatComponents: true,
                componentsPerVector: 3,
                bytesPerComponent: MemoryLayout<Float>.size,
                dataOffset: 0,
                dataStride: MemoryLayout<SIMD3<Float>>.stride)

            let indices = [UInt32](0..<UInt32(positions.count))
            let element = SCNGeometryElement(indices: indices, primitiveType: .point)
            element.pointSize = 6.0
            element.minimumPointScreenSpaceRadius = 2.0
            element.maximumPointScreenSpaceRadius = 8.0

            let geometry = SCNGeometry(sources: [vertexSource, colorSource], elements: [element])
            let material = SCNMaterial()
            material.lightingModel = .constant
            material.isDoubleSided = true
            geometry.materials = [material]

            pointNode.geometry = geometry
        }
    }
}
