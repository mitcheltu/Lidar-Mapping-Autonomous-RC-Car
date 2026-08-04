//
//  ARDepthView.swift
//  RCCarLidarStreamer
//
//  Wraps an ARSCNView and acts as the car's LiDAR sensor.
//
//  The phone keeps NO map of its own. Each processed frame is unprojected into
//  world space, subsampled to a fixed budget, streamed, and forgotten. That is
//  deliberate: the laptop already maintains a log-odds voxel grid with ray
//  carving, so accumulating a second copy here only bought us a 400k cap whose
//  real effect was to silently stop streaming once it filled.
//
//  Depth is expensive, so it runs only when the car asks for it (SensorMode).
//  Pose is cheap and always streamed -- the car localises with it even when the
//  LiDAR is off.
//

import SwiftUI
import ARKit
import SceneKit
import CoreImage
import ImageIO

struct ARDepthView: UIViewRepresentable {
    let streamer: PointCloudStreamer

    func makeCoordinator() -> Coordinator {
        Coordinator(streamer: streamer)
    }

    func makeUIView(context: Context) -> ARSCNView {
        let view = ARSCNView(frame: .zero)
        view.session.delegate = context.coordinator
        view.automaticallyUpdatesLighting = true
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

    func updateUIView(_ uiView: ARSCNView, context: Context) {}

    static func dismantleUIView(_ uiView: ARSCNView, coordinator: Coordinator) {
        uiView.session.pause()
    }

    // MARK: - Coordinator

    final class Coordinator: NSObject, ARSessionDelegate {
        let streamer: PointCloudStreamer
        weak var sceneView: ARSCNView?

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

        // Reused between frames so a scan does not churn the allocator.
        private var positions = [SIMD3<Float>]()
        private var colors = [SIMD3<Float>]()

        // Tuning knobs.
        private let frameStride = 3          // process roughly every 3rd frame
        private let pixelStride = 2          // subsample the depth map by this
        private let maxDepth: Float = 5.0    // meters; LiDAR is reliable to ~5 m
        private let minConfidence: UInt8 = 1 // 0 = low, 1 = medium, 2 = high

        /// Hard bound on points sent per frame.
        ///
        /// This replaces the old accumulator cap, and is a different kind of
        /// limit: nothing is retained, we simply refuse to put more than this on
        /// the wire per frame. Sized to match the mapper's max_rays -- sending
        /// more just queues up work the laptop cannot keep up with anyway.
        private let maxPointsPerFrame = 2000

        init(streamer: PointCloudStreamer) {
            self.streamer = streamer
        }

        func session(_ session: ARSession, didUpdate frame: ARFrame) {
            let now = CACurrentMediaTime()

            // Pose first, and unconditionally: it is cheap, and the car needs
            // to know where it is even while the LiDAR is off.
            if now - lastPoseTime > 0.1 {
                lastPoseTime = now
                streamer.sendPose(frame.camera.transform)
            }

            guard streamer.mode.wantsDepth else { return }

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

        // MARK: Depth -> world points -> the wire

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

            // Decimate so a frame cannot exceed the budget. Computed per frame
            // because the depth map size can change with the session config.
            let candidates = max(1, (depthWidth / pixelStride) * (depthHeight / pixelStride))
            let keepEvery = max(1, Int((Double(candidates) / Double(maxPointsPerFrame)).rounded(.up)))
            var accepted = 0

            positions.removeAll(keepingCapacity: true)
            colors.removeAll(keepingCapacity: true)
            positions.reserveCapacity(maxPointsPerFrame)
            colors.reserveCapacity(maxPointsPerFrame)

            for row in stride(from: 0, to: depthHeight, by: pixelStride) {
                let depthRow = (depthBase + row * depthRowBytes).assumingMemoryBound(to: Float32.self)
                let confRow = confBase.map { ($0 + row * confRowBytes).assumingMemoryBound(to: UInt8.self) }

                for col in stride(from: 0, to: depthWidth, by: pixelStride) {
                    let depth = depthRow[col]
                    if depth <= 0 || depth.isNaN || depth > maxDepth { continue }
                    if let confRow = confRow, confRow[col] < minConfidence { continue }

                    accepted += 1
                    if accepted % keepEvery != 0 { continue }
                    if positions.count >= maxPointsPerFrame { break }

                    // Camera space (ARKit convention: +x right, +y up, -z forward).
                    let x = (Float(col) - cx) / fx * depth
                    let y = (Float(row) - cy) / fy * depth
                    let local = SIMD4<Float>(x, -y, -depth, 1)
                    let world4 = cameraTransform * local

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

                    positions.append(SIMD3<Float>(world4.x, world4.y, world4.z))
                    colors.append(color)
                }
                if positions.count >= maxPointsPerFrame { break }
            }

            // Send and forget. Nothing is retained between frames.
            streamer.sendPoints(positions: positions, colors: colors)

            let ts = CACurrentMediaTime()
            if ts - lastImageTime > 0.2 {
                lastImageTime = ts
                sendJPEG(capturedImage)
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
    }
}
