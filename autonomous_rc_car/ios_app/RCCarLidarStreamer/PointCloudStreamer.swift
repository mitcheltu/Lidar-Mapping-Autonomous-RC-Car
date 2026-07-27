//
//  PointCloudStreamer.swift
//  PointCloudScanner
//
//  Streams the live scan to a computer over Wi-Fi (TCP). Three message kinds:
//    'P' points : new voxels  — count(uint32) then per point: x,y,z float32 + r,g,b uint8
//    'O' pose   : camera 4x4 matrix as 16 float32 (column-major)
//    'I' image  : JPEG bytes of a downscaled camera frame
//  Every message is framed as: type(1 byte) + payloadLength(uint32 LE) + payload.
//  All numbers are little-endian to match the Python viewer's struct('<...').
//
//  The phone is the CLIENT; your computer runs pc_viewer.py as the server.
//  Enter the computer's LAN IP in the app and tap Connect.
//

import Foundation
import Network
import simd

final class PointCloudStreamer: ObservableObject {

    @Published var isStreaming = false
    @Published var status = "not connected"

    private var connection: NWConnection?
    private let queue = DispatchQueue(label: "pointcloud.stream")

    func start(host: String, port: UInt16 = 9000) {
        stop()
        let trimmed = host.trimmingCharacters(in: .whitespaces)
        guard !trimmed.isEmpty, let nwPort = NWEndpoint.Port(rawValue: port) else {
            DispatchQueue.main.async { self.status = "invalid host" }
            return
        }
        let conn = NWConnection(host: NWEndpoint.Host(trimmed), port: nwPort, using: .tcp)
        conn.stateUpdateHandler = { [weak self] state in
            DispatchQueue.main.async {
                switch state {
                case .ready:
                    self?.isStreaming = true;  self?.status = "connected"
                case .waiting(let e):
                    self?.isStreaming = false; self?.status = "waiting: \(e.localizedDescription)"
                case .failed(let e):
                    self?.isStreaming = false; self?.status = "failed: \(e.localizedDescription)"
                case .cancelled:
                    self?.isStreaming = false; self?.status = "not connected"
                default:
                    break
                }
            }
        }
        connection = conn
        conn.start(queue: queue)
    }

    func stop() {
        connection?.cancel()
        connection = nil
        DispatchQueue.main.async { self.isStreaming = false; self.status = "not connected" }
    }

    // MARK: - Framing

    private func send(type: UInt8, payload: Data) {
        guard let conn = connection, isStreaming else { return }
        var frame = Data()
        frame.append(type)
        var len = UInt32(payload.count).littleEndian
        withUnsafeBytes(of: &len) { frame.append(contentsOf: $0) }
        frame.append(payload)
        conn.send(content: frame, completion: .contentProcessed { _ in })
    }

    private func appendLE<T>(_ value: T, to data: inout Data) {
        var v = value
        withUnsafeBytes(of: &v) { data.append(contentsOf: $0) }
    }

    // MARK: - Messages

    func sendPoints(_ points: [PointVertex]) {
        guard isStreaming, !points.isEmpty else { return }
        var payload = Data(capacity: 4 + points.count * 15)
        appendLE(UInt32(points.count).littleEndian, to: &payload)
        for p in points {
            appendLE(p.position.x, to: &payload)
            appendLE(p.position.y, to: &payload)
            appendLE(p.position.z, to: &payload)
            payload.append(UInt8(max(0, min(255, Int((p.color.x * 255).rounded())))))
            payload.append(UInt8(max(0, min(255, Int((p.color.y * 255).rounded())))))
            payload.append(UInt8(max(0, min(255, Int((p.color.z * 255).rounded())))))
        }
        send(type: 0x50, payload: payload) // 'P'
    }

    func sendPose(_ m: simd_float4x4) {
        guard isStreaming else { return }
        var payload = Data(capacity: 64)
        for col in [m.columns.0, m.columns.1, m.columns.2, m.columns.3] {
            appendLE(col.x, to: &payload)
            appendLE(col.y, to: &payload)
            appendLE(col.z, to: &payload)
            appendLE(col.w, to: &payload)
        }
        send(type: 0x4F, payload: payload) // 'O'
    }

    func sendImage(_ jpeg: Data) {
        guard isStreaming else { return }
        send(type: 0x49, payload: jpeg) // 'I'
    }
}
