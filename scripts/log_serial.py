#!/usr/bin/env python3
"""Capture CSV-style Serial output from thesis_xy.ino to a log file.

Matches the tags printed by the EVAL_MODE instrumentation in thesis_xy.ino:
FPS, LATENCY_US, TOUCH, TRANSITION, SIZE, RECORD_SAMPLE, REPLAY_SAMPLE.

Examples:
    python3 log_serial.py --list
    python3 log_serial.py --port /dev/tty.usbmodemXXXX --tag FPS --duration 60
    python3 log_serial.py --port /dev/tty.usbmodemXXXX --tag TOUCH --out test3_center.csv
"""
import argparse
import csv
import sys
import time
from datetime import datetime

import serial
import serial.tools.list_ports


def list_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return
    print("Available ports:")
    for p in ports:
        print(f"  {p.device}  {p.description}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="Serial port, e.g. /dev/tty.usbmodemXXXX or COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--out", default=None, help="Output CSV path (default: log_<tag|all>_<timestamp>.csv)")
    parser.add_argument("--duration", type=float, default=None, help="Stop after N seconds (default: run until Ctrl+C)")
    parser.add_argument("--tag", default=None,
                         help="Only keep lines starting with this tag (FPS, LATENCY_US, TOUCH, "
                              "TRANSITION, SIZE, RECORD_SAMPLE, REPLAY_SAMPLE). Default: keep everything.")
    parser.add_argument("--echo", action="store_true", help="Also print each captured line to stdout")
    parser.add_argument("--list", action="store_true", help="List available serial ports and exit")
    args = parser.parse_args()

    if args.list:
        list_ports()
        return

    if not args.port:
        parser.error("--port is required (use --list to see available ports)")

    out_path = args.out or f"log_{args.tag or 'all'}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    print(f"Opening {args.port} @ {args.baud} baud...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"Failed to open {args.port}: {e}", file=sys.stderr)
        sys.exit(1)

    # The board resets on connect; give it time to finish setup()/calibration
    # before we start counting lines, and drop whatever it wrote during that.
    time.sleep(2)
    ser.reset_input_buffer()

    start = time.monotonic()
    kept = 0
    total = 0

    print(f"Logging to {out_path}" + (f" (tag={args.tag})" if args.tag else "") + " — Ctrl+C to stop.")

    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        try:
            while True:
                if args.duration is not None and (time.monotonic() - start) >= args.duration:
                    break

                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                total += 1

                if args.tag and not line.startswith(args.tag + ","):
                    if args.echo:
                        print(f"(skip) {line}")
                    continue

                writer.writerow(line.split(","))
                f.flush()
                kept += 1

                if args.echo:
                    print(line)
        except KeyboardInterrupt:
            print("\nStopped by user.")
        finally:
            ser.close()

    print(f"Done. {kept}/{total} lines captured to {out_path}")


if __name__ == "__main__":
    main()
