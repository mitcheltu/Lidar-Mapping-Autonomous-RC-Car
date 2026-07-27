//
//  CarController.swift
//  PointCloudScanner
//
//  BLE link from the iPhone (central) to the ESP32 (peripheral). This is the
//  bridge you'll use once the app drives the robot: call `drive(left:right:)`
//  with motor speeds in -100...100 and the ESP32 firmware turns them into PWM.
//
//  The UUIDs below use the common "Nordic UART Service" (NUS) layout. They MUST
//  match the UUIDs in the ESP32 firmware (see firmware/esp32_car.ino).
//
//  Not wired into the scanning UI yet — it's here so the perception app and the
//  robot can share one project. Instantiate it, wait for `isConnected`, then
//  send commands from your control loop.
//

import Foundation
import CoreBluetooth

final class CarController: NSObject, ObservableObject,
                           CBCentralManagerDelegate, CBPeripheralDelegate {

    static let serviceUUID = CBUUID(string: "6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
    static let rxCharUUID  = CBUUID(string: "6E400002-B5A3-F393-E0A9-E50E24DCCA9E") // iPhone -> ESP32

    @Published var isConnected = false

    private var central: CBCentralManager!
    private var peripheral: CBPeripheral?
    private var rxChar: CBCharacteristic?

    override init() {
        super.init()
        central = CBCentralManager(delegate: self, queue: nil)
    }

    // MARK: Central lifecycle

    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn {
            central.scanForPeripherals(withServices: [Self.serviceUUID], options: nil)
        }
    }

    func centralManager(_ central: CBCentralManager,
                        didDiscover peripheral: CBPeripheral,
                        advertisementData: [String: Any],
                        rssi RSSI: NSNumber) {
        self.peripheral = peripheral
        peripheral.delegate = self
        central.stopScan()
        central.connect(peripheral, options: nil)
    }

    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        peripheral.discoverServices([Self.serviceUUID])
    }

    func centralManager(_ central: CBCentralManager,
                        didDisconnectPeripheral peripheral: CBPeripheral,
                        error: Error?) {
        DispatchQueue.main.async { self.isConnected = false }
        central.scanForPeripherals(withServices: [Self.serviceUUID], options: nil)
    }

    // MARK: Peripheral discovery

    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        for service in peripheral.services ?? [] {
            peripheral.discoverCharacteristics([Self.rxCharUUID], for: service)
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didDiscoverCharacteristicsFor service: CBService,
                    error: Error?) {
        for ch in service.characteristics ?? [] where ch.uuid == Self.rxCharUUID {
            rxChar = ch
            DispatchQueue.main.async { self.isConnected = true }
        }
    }

    // MARK: Commands

    /// Send left/right motor speeds, each -100...100 (negative = reverse).
    /// Wire format is a short ASCII line, e.g. "L60R60\n".
    func drive(left: Int, right: Int) {
        guard let peripheral = peripheral, let rxChar = rxChar else { return }
        let l = max(-100, min(100, left))
        let r = max(-100, min(100, right))
        let cmd = "L\(l)R\(r)\n"
        if let data = cmd.data(using: .ascii) {
            peripheral.writeValue(data, for: rxChar, type: .withoutResponse)
        }
    }

    func stop() { drive(left: 0, right: 0) }
}
