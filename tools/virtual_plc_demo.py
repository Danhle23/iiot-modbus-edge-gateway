#!/usr/bin/env python3
"""
virtual_plc_demo.py - End-to-End RS485 Modbus-RTU Cabinet & Cold-Chain Simulator
Generates raw wire-level Modbus-RTU frames (with CRC-16) and demonstrates
phase-loss & thermal excursion fault detection for industrial retrofit demos.
Author: Le Thanh Danh (Danhle23)
"""

import json
import struct


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if (crc & 1) else (crc >> 1)
    return crc


def build_slave_holding_response(slave_id: int, regs: list[int]) -> bytes:
    header = bytes([slave_id, 0x03, len(regs) * 2])
    payload = b"".join(struct.pack(">h", r) for r in regs)
    body = header + payload
    crc = crc16_modbus(body)
    return body + struct.pack("<H", crc)


def run_simulation() -> None:
    scenarios = [
        ("t=01s [NORMAL]",     [2215, 2202, 2220, 142, -185]),
        ("t=02s [NORMAL]",     [2210, 2198, 2216, 145, -184]),
        ("t=03s [TRANSIENT]",  [2205, 2195, 1400, 150, -182]),
        ("t=04s [FAULT 2/3]",  [2208, 2192,  820, 295, -105]),
        ("t=05s [ALARM LATCH]",[2204, 2190,  790, 338,  -98]),
    ]

    consecutive = 0
    for label, regs in scenarios:
        frame = build_slave_holding_response(slave_id=1, regs=regs)
        assert crc16_modbus(frame[:-2]) == struct.unpack("<H", frame[-2:])[0]

        va, vb, vc = regs[0] / 10.0, regs[1] / 10.0, regs[2] / 10.0
        irms = regs[3] / 10.0
        temp_c = regs[4] / 10.0

        fault = (min(va, vb, vc) < 175.0) or (temp_c > -12.0) or (irms > 32.0)
        consecutive = (consecutive + 1) if fault else 0
        alarm = consecutive >= 3

        telemetry = {
            "step": label,
            "raw_rs485_hex": frame.hex(" ").upper(),
            "va_v": va, "vb_v": vb, "vc_v": vc,
            "irms_a": irms, "temp_c": temp_c,
            "alarm_latched": alarm,
        }
        print(json.dumps(telemetry))


if __name__ == "__main__":
    run_simulation()
