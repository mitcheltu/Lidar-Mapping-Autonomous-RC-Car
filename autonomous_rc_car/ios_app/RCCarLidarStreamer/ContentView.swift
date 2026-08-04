//
//  ContentView.swift
//  RCCarLidarStreamer
//
//  UI for a phone bolted to a robot: connect to the laptop, then show what the
//  car is asking of the LiDAR and whether data is actually flowing.
//
//  There is deliberately no point count, no Clear and no Export. The phone no
//  longer accumulates a cloud -- it unprojects, streams and forgets -- so those
//  controls had nothing left to act on. The map lives on the laptop, where you
//  can see it in Rerun.
//

import SwiftUI

struct ContentView: View {
    @StateObject private var streamer = PointCloudStreamer()
    @State private var hostIP = ""

    var body: some View {
        ZStack(alignment: .bottom) {
            ARDepthView(streamer: streamer)
                .edgesIgnoringSafeArea(.all)

            VStack(spacing: 10) {
                modeBadge
                if streamer.mode.wantsDepth {
                    Text("\(streamer.pointsPerSecond) pts/s")
                        .font(.caption)
                        .padding(.horizontal, 12).padding(.vertical, 6)
                        .background(.ultraThinMaterial, in: Capsule())
                }
                Text("The car controls the LiDAR. Scan with 's' in the laptop console.")
                    .font(.caption2)
                    .foregroundStyle(.secondary)
                    .multilineTextAlignment(.center)
                    .padding(.horizontal, 24)
                    .padding(.bottom, 28)
            }
        }
        .overlay(alignment: .top) { connectionBar }
    }

    private var modeBadge: some View {
        let (label, tint): (String, Color) = {
            switch streamer.mode {
            case .idle:  return ("LiDAR idle", .gray)
            case .scan:  return ("SCANNING", .green)
            case .drive: return ("driving", .blue)
            }
        }()
        return Text(label)
            .font(.headline)
            .foregroundStyle(.white)
            .padding(.horizontal, 16).padding(.vertical, 8)
            .background(tint.opacity(0.85), in: Capsule())
    }

    private var connectionBar: some View {
        VStack(spacing: 4) {
            HStack(spacing: 8) {
                TextField("Laptop IP (e.g. 192.168.1.20)", text: $hostIP)
                    .textFieldStyle(.roundedBorder)
                    .keyboardType(.numbersAndPunctuation)
                    .autocorrectionDisabled(true)
                    .textInputAutocapitalization(.never)
                    .frame(maxWidth: 210)
                Button(streamer.isStreaming ? "Stop" : "Connect") {
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
}
