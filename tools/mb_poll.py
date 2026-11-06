#!/usr/bin/env python3
"""Bench test for the emulator: acts as a Modbus RTU master over a USB-serial adapter.

Needs pyserial. The adapter must be RS-232 if it connects after the transceiver, or
3.3 V TTL if it connects straight to GPIO16/17.

    python tools/mb_poll.py COM5
    python tools/mb_poll.py COM5 --baud 19200 --start 3000 --qty 10 --loop
"""

import argparse
import struct
import sys
import time

import serial

EXCEPTIONS = {1: "illegal function", 2: "illegal address", 3: "illegal value"}


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def read_registers(port, slave, fc, start, qty):
    request = struct.pack(">BBHH", slave, fc, start, qty)
    request += struct.pack("<H", crc16(request))

    port.reset_input_buffer()
    port.write(request)

    # Enough to read an exception reply, then the rest of a normal one
    reply = port.read(5)
    if len(reply) < 5:
        raise TimeoutError(f"no reply, got {reply.hex(' ') or 'nothing'}")

    if not reply[1] & 0x80:
        reply += port.read(reply[2] + 5 - len(reply))
        if len(reply) < reply[2] + 5:
            raise TimeoutError(f"short reply: {reply.hex(' ')}")

    if struct.unpack("<H", reply[-2:])[0] != crc16(reply[:-2]):
        raise ValueError(f"bad CRC: {reply.hex(' ')}")

    if reply[1] & 0x80:
        raise ValueError(f"exception {reply[2]:02d}: {EXCEPTIONS.get(reply[2], 'unknown')}")

    return struct.unpack(f">{qty}H", reply[3:-2])


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("port")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--parity", choices="NEO", default="N")
    parser.add_argument("--stop", type=int, choices=(1, 2), default=1)
    parser.add_argument("--slave", type=int, default=1)
    parser.add_argument("--fc", type=int, choices=(3, 4), default=4)
    parser.add_argument("--start", type=int, default=3000, help="wire address, zero-based")
    parser.add_argument("--qty", type=int, default=100)
    parser.add_argument("--loop", action="store_true", help="poll once a second until Ctrl+C")
    args = parser.parse_args()

    prefix = "3x" if args.fc == 4 else "4x"

    with serial.Serial(args.port, args.baud, parity=args.parity, stopbits=args.stop,
                       timeout=1) as port:
        while True:
            try:
                values = read_registers(port, args.slave, args.fc, args.start, args.qty)
                for i, value in enumerate(values):
                    print(f"{prefix}{args.start + i + 1}: {value}")
            except (TimeoutError, ValueError) as error:
                print(f"error: {error}", file=sys.stderr)
                if not args.loop:
                    return 1

            if not args.loop:
                return 0
            print()
            time.sleep(1)


if __name__ == "__main__":
    sys.exit(main())
