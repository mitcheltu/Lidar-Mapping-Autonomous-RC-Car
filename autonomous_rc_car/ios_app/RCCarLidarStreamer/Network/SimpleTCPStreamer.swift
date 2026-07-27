import Foundation
import Network

final class SimpleTCPStreamer {
    private var connection: NWConnection?
    private let queue = DispatchQueue(label: "simple.tcp")

    func connect(host: String, port: UInt16 = 9002) {
        let endpoint = NWEndpoint.Host(host)
        let nwPort = NWEndpoint.Port(rawValue: port) ?? .init(9002)
        let conn = NWConnection(host: endpoint, port: nwPort, using: .tcp)
        connection = conn
        conn.start(queue: queue)
    }

    func send(points: [[Float]]) {
        guard let conn = connection else { return }
        var payload = Data()
        var count = UInt32(points.count).littleEndian
        withUnsafeBytes(of: &count) { payload.append(contentsOf: $0) }
        for point in points {
            if point.count >= 3 {
                var x = point[0]
                var y = point[1]
                var z = point[2]
                withUnsafeBytes(of: &x) { payload.append(contentsOf: $0) }
                withUnsafeBytes(of: &y) { payload.append(contentsOf: $0) }
                withUnsafeBytes(of: &z) { payload.append(contentsOf: $0) }
                let r: UInt8 = 255
                let g: UInt8 = 0
                let b: UInt8 = 0
                payload.append(r)
                payload.append(g)
                payload.append(b)
            }
        }

        var frame = Data()
        frame.append(0x50)
        var length = UInt32(payload.count).littleEndian
        withUnsafeBytes(of: &length) { frame.append(contentsOf: $0) }
        frame.append(payload)
        conn.send(content: frame, completion: .contentProcessed { _ in })
    }

    func disconnect() {
        connection?.cancel()
        connection = nil
    }
}
