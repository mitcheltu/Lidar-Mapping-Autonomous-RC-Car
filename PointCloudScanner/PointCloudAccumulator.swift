//
//  PointCloudAccumulator.swift
//  PointCloudScanner
//
//  Thread-safe voxel-grid accumulator. As you wave the phone around, LiDAR
//  depth samples are unprojected into world space and dropped into a voxel
//  grid so we keep ~one averaged, colored point per grid cell. This keeps the
//  cloud from exploding into millions of duplicate points sitting on the same
//  surface.
//

import Foundation
import simd

/// A single colored point in the cloud. Color components are 0...1.
struct PointVertex {
    var position: SIMD3<Float>
    var color: SIMD3<Float>
}

final class PointCloudAccumulator {

    /// Edge length of each voxel cell, in meters. 0.015 = 1.5 cm.
    private let voxelSize: Float
    /// Safety cap so memory can't grow unbounded during a long scan.
    private let maxPoints: Int

    private struct Cell {
        var posSum: SIMD3<Float>
        var colSum: SIMD3<Float>
        var count: Float
    }

    private var cells: [SIMD3<Int32>: Cell] = [:]
    private var pendingNew: [PointVertex] = []   // new voxels since last drain (for streaming)
    private let lock = NSLock()

    init(voxelSize: Float = 0.015, maxPoints: Int = 400_000) {
        self.voxelSize = voxelSize
        self.maxPoints = maxPoints
    }

    /// Add one world-space sample. Points landing in the same voxel are averaged.
    func add(position: SIMD3<Float>, color: SIMD3<Float>) {
        let key = SIMD3<Int32>(
            Int32((position.x / voxelSize).rounded(.down)),
            Int32((position.y / voxelSize).rounded(.down)),
            Int32((position.z / voxelSize).rounded(.down))
        )

        lock.lock()
        if var c = cells[key] {
            c.posSum += position
            c.colSum += color
            c.count += 1
            cells[key] = c
        } else if cells.count < maxPoints {
            cells[key] = Cell(posSum: position, colSum: color, count: 1)
            pendingNew.append(PointVertex(position: position, color: color))
        }
        lock.unlock()
    }

    /// Return points added to new voxels since the last call (for network streaming),
    /// and clear the pending buffer. Lightweight — sends only what's new.
    func drainNew() -> [PointVertex] {
        lock.lock()
        let out = pendingNew
        pendingNew.removeAll(keepingCapacity: true)
        lock.unlock()
        return out
    }

    var count: Int {
        lock.lock(); defer { lock.unlock() }
        return cells.count
    }

    /// Snapshot the current cloud as averaged points (safe to call from any thread).
    func snapshot() -> [PointVertex] {
        lock.lock()
        let copy = cells
        lock.unlock()

        var out = [PointVertex]()
        out.reserveCapacity(copy.count)
        for (_, c) in copy {
            let inv = 1.0 / c.count
            out.append(PointVertex(position: c.posSum * inv, color: c.colSum * inv))
        }
        return out
    }

    func clear() {
        lock.lock()
        cells.removeAll(keepingCapacity: true)
        pendingNew.removeAll(keepingCapacity: true)
        lock.unlock()
    }
}
