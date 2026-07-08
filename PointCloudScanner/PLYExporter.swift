//
//  PLYExporter.swift
//  PointCloudScanner
//
//  Writes the accumulated point cloud to an ASCII .PLY file. PLY opens in
//  MeshLab, CloudCompare, Blender, and most 3D tools, and is easy to inspect
//  by hand. For very large clouds you'd switch to binary PLY, but ASCII is the
//  most portable and is fine for a few hundred thousand points.
//

import Foundation

enum PLYExporter {

    static func writePLY(points: [PointVertex], to url: URL) throws {
        var text = ""
        text.reserveCapacity(points.count * 40 + 256)

        // Header
        text += "ply\n"
        text += "format ascii 1.0\n"
        text += "comment Created by PointCloudScanner (iPhone LiDAR)\n"
        text += "element vertex \(points.count)\n"
        text += "property float x\n"
        text += "property float y\n"
        text += "property float z\n"
        text += "property uchar red\n"
        text += "property uchar green\n"
        text += "property uchar blue\n"
        text += "end_header\n"

        // Body
        for p in points {
            let r = UInt8(clamping: Int((p.color.x * 255).rounded()))
            let g = UInt8(clamping: Int((p.color.y * 255).rounded()))
            let b = UInt8(clamping: Int((p.color.z * 255).rounded()))
            text += "\(p.position.x) \(p.position.y) \(p.position.z) \(r) \(g) \(b)\n"
        }

        try text.write(to: url, atomically: true, encoding: .ascii)
    }
}
