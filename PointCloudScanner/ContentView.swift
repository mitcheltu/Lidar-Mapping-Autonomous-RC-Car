//
//  ContentView.swift
//  PointCloudScanner
//
//  UI: live camera + point cloud, a running point count, and Pause / Clear /
//  Export controls. Export writes a .PLY and hands it to the iOS share sheet
//  so you can AirDrop / save it to Files.
//

import SwiftUI

final class ScannerModel: ObservableObject {
    let accumulator = PointCloudAccumulator()
}

struct ContentView: View {
    @StateObject private var model = ScannerModel()
    @StateObject private var streamer = PointCloudStreamer()
    @State private var isScanning = true
    @State private var pointCount = 0
    @State private var statusMessage = ""
    @State private var shareURL: URL?
    @State private var showShare = false
    @State private var hostIP = ""

    var body: some View {
        ZStack(alignment: .bottom) {
            ARDepthView(accumulator: model.accumulator,
                        streamer: streamer,
                        isScanning: $isScanning) { count in
                pointCount = count
            }
            .edgesIgnoringSafeArea(.all)

            VStack(spacing: 10) {
                Text("\(pointCount) points")
                    .font(.headline)
                    .padding(.horizontal, 14).padding(.vertical, 8)
                    .background(.ultraThinMaterial, in: Capsule())

                if !statusMessage.isEmpty {
                    Text(statusMessage)
                        .font(.caption)
                        .padding(.horizontal, 12).padding(.vertical, 6)
                        .background(.ultraThinMaterial, in: Capsule())
                }

                HStack(spacing: 14) {
                    Button(isScanning ? "Pause" : "Resume") {
                        isScanning.toggle()
                    }
                    .buttonStyle(.borderedProminent)

                    Button("Clear") {
                        model.accumulator.clear()
                        pointCount = 0
                        statusMessage = ""
                    }
                    .buttonStyle(.bordered)

                    Button("Export PLY") { export() }
                        .buttonStyle(.borderedProminent)
                        .tint(.green)
                }
                .padding(.bottom, 28)
            }
        }
        .overlay(alignment: .top) { connectionBar }
        .sheet(isPresented: $showShare) {
            if let url = shareURL {
                ShareSheet(items: [url])
            }
        }
    }

    private var connectionBar: some View {
        VStack(spacing: 4) {
            HStack(spacing: 8) {
                TextField("Computer IP (e.g. 192.168.1.20)", text: $hostIP)
                    .textFieldStyle(.roundedBorder)
                    .keyboardType(.numbersAndPunctuation)
                    .autocorrectionDisabled(true)
                    .textInputAutocapitalization(.never)
                    .frame(maxWidth: 210)
                Button(streamer.isStreaming ? "Stop" : "Stream") {
                    if streamer.isStreaming { streamer.stop() }
                    else { streamer.start(host: hostIP) }
                }
                .buttonStyle(.borderedProminent)
                .tint(streamer.isStreaming ? .red : .blue)
                Circle()
                    .fill(streamer.isStreaming ? Color.green : Color.gray)
                    .frame(width: 10, height: 10)
            }
            Text(streamer.status)
                .font(.caption2)
                .foregroundStyle(.secondary)
        }
        .padding(10)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 14))
        .padding(.top, 12)
    }

    private func export() {
        isScanning = false
        statusMessage = "Exporting…"
        let accumulator = model.accumulator
        DispatchQueue.global(qos: .userInitiated).async {
            let points = accumulator.snapshot()
            let name = "scan-\(Int(Date().timeIntervalSince1970)).ply"
            let url = FileManager.default.temporaryDirectory.appendingPathComponent(name)
            do {
                try PLYExporter.writePLY(points: points, to: url)
                DispatchQueue.main.async {
                    shareURL = url
                    showShare = true
                    statusMessage = "Exported \(points.count) points"
                }
            } catch {
                DispatchQueue.main.async {
                    statusMessage = "Export failed: \(error.localizedDescription)"
                }
            }
        }
    }
}

/// Thin wrapper around UIActivityViewController for SwiftUI.
struct ShareSheet: UIViewControllerRepresentable {
    let items: [Any]
    func makeUIViewController(context: Context) -> UIActivityViewController {
        UIActivityViewController(activityItems: items, applicationActivities: nil)
    }
    func updateUIViewController(_ vc: UIActivityViewController, context: Context) {}
}
