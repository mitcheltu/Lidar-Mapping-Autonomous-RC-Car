//
//  PointCloudStreamer.swift
//  RCCarLidarStreamer
//
//  Streams the live scan to the laptop over Wi-Fi (TCP), and listens for the
//  commands coming back the other way.
//
//  Phone -> laptop:
//    'P' 0x50 points : count(uint32) then per point: x,y,z float32 + r,g,b uint8
//    'O' 0x4F pose   : camera 4x4 matrix as 16 float32 (column-major)
//    'I' 0x49 image  : JPEG bytes of a downscaled camera frame
//  Laptop -> phone:
//    'M' 0x4D mode   : ASCII "IDLE" | "SCAN" | "DRIVE" -- when depth is wanted
//    'D' 0x44 drive  : ASCII "L<left>R<right>" (legacy BLE relay; ignored now
//                      that the laptop drives the ESP32 directly over WiFi)
//
//  Every message is framed as: type(1 byte) + payloadLength(uint32 LE) + payload.
//  All numbers are little-endian to match the Python side's struct('<...').
//
//  The phone is the CLIENT; the laptop runs bridge_node as the server on :9000.
//

import Foundation
import Network
import QuartzCore     // CACurrentMediaTime
import simd

/// What the car currently wants from the LiDAR. Depth is by far the most
/// expensive thing the phone does, so the laptop turns it on only when it is
/// actually going to use the data.
enum SensorMode: String {
    case idle  = "IDLE"     // pose only -- parked, planning, waiting
    case scan  = "SCAN"     // depth, during a commanded 360-degree spin
    case drive = "DRIVE"    // depth, while the car is moving

    /// Whether depth frames should be unprojected and streamed at all.
    var wantsDepth: Bool {
        switch self {
        case .idle:         return false
        case .scan, .drive: return true
        }
    }
}

final class PointCloudStreamer: ObservableObject {

    @Published var isStreaming = false
    @Published var status = "not connected"
    /// Default IDLE: never burn battery on depth until the car asks for it.
    @Published var mode: SensorMode = .idle
    @Published var pointsPerSecond = 0

    private var connection: NWConnection?
    private let queue = DispatchQueue(label: "pointcloud.stream")
    private var inbox = Data()

    private var sentThisSecond = 0
    private var rateWindowStart = CACurrentMediaTime()

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
                    self?.receiveLoop()
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
        inbox.removeAll(keepingCapacity: false)
        DispatchQueue.main.async {
            self.isStreaming = false
            self.status = "not connected"
            self.mode = .idle          // back to cheap when the link drops
            self.pointsPerSecond = 0
        }
    }

    // MARK: - Receiving (laptop -> phone)

    private func receiveLoop() {
        connection?.receive(minimumIncompleteLength: 1, maximumLength: 8192) {
            [weak self] data, _, isComplete, error in
            guard let self = self else { return }
            if let data = data, !data.isEmpty {
                self.inbox.append(data)
                self.drainInbox()
            }
            if isComplete || error != nil { return }
            self.receiveLoop()
        }
    }

    /// Pull whole frames out of the inbox; a partial frame stays buffered.
    ///
    /// The header is read byte by byte rather than with an unaligned load: TCP
    /// gives no alignment guarantees, and Data slices keep their parent's
    /// indices, which makes offset arithmetic here easy to get subtly wrong.
    private func drainInbox() {
        while true {
            guard inbox.count >= 5 else { return }
            let header = [UInt8](inbox.prefix(5))
            let type = header[0]
            let length = Int(UInt32(header[1])
                             | (UInt32(header[2]) << 8)
                             | (UInt32(header[3]) << 16)
                             | (UInt32(header[4]) << 24))
            guard inbox.count >= 5 + length else { return }   // wait for the rest

            let payload = Data(inbox.dropFirst(5).prefix(length))
            inbox = Data(inbox.dropFirst(5 + length))
            handle(type: type, payload: payload)
        }
    }

    private func handle(type: UInt8, payload: Data) {
        switch type {
        case 0x4D:   // 'M' mode
            guard let text = String(data: payload, encoding: .ascii),
                  let newMode = SensorMode(rawValue: text.trimmingCharacters(in: .whitespacesAndNewlines).uppercased())
            else { return }
            DispatchQueue.main.async { self.mode = newMode }
        default:
            break    // unknown types are ignored, so either end can add messages
        }
    }

    // MARK: - Framing

    private func send(type: UInt8, payload: Data) {
        guard let conn = connection, isStreaming else { return }
        var frame = Data(capacity: 5 + payload.count)
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

    /// Send a batch of world-space points straight from the depth pass.
    ///
    /// Deliberately takes flat buffers rather than an accumulated cloud: the
    /// phone keeps no map of its own, so there is nothing to grow unbounded and
    /// nothing to cap. The laptop's log-odds voxel grid does the accumulating,
    /// and repeated observations of the same surface are useful to it -- they
    /// are the evidence that keeps a voxel occupied against ray carving.
    func sendPoints(positions: [SIMD3<Float>], colors: [SIMD3<Float>]) {
        guard isStreaming, !positions.isEmpty else { return }
        let n = min(positions.count, colors.count)
        var payload = Data(capacity: 4 + n * 15)
        appendLE(UInt32(n).littleEndian, to: &payload)
        for i in 0..<n {
            appendLE(positions[i].x, to: &payload)
            appendLE(positions[i].y, to: &payload)
            appendLE(positions[i].z, to: &payload)
            payload.append(UInt8(max(0, min(255, Int((colors[i].x * 255).rounded())))))
            payload.append(UInt8(max(0, min(255, Int((colors[i].y * 255).rounded())))))
            payload.append(UInt8(max(0, min(255, Int((colors[i].z * 255).rounded())))))
        }
        send(type: 0x50, payload: payload) // 'P'
        noteSent(n)
    }

    private func noteSent(_ count: Int) {
        sentThisSecond += count
        let now = CACurrentMediaTime()
        if now - rateWindowStart >= 1.0 {
            let rate = Int(Double(sentThisSecond) / (now - rateWindowStart))
            sentThisSecond = 0
            rateWindowStart = now
            DispatchQueue.main.async { self.pointsPerSecond = rate }
        }
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
