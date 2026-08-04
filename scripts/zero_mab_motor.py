#!/usr/bin/env python3
"""Zero (set-origin) MAB-backend motors directly over CAN, without starting the
ROS 2 driver node at all.

Sends only RunZero_Message (reg 0x8C = 1, classic CAN frame, long timeout) -- mirrors
MabFdCan::set_zero_position() in mab_fd_can.cpp. This assumes the motor is ALREADY enabled
(control mode active); it does not write the mode/state registers itself.

Only for CAN interfaces using the "mab" comm_backend (see can_backends / comm_backend_default
in the driver's param file). Do NOT run this against a motor that the ROS driver node already
has configured/active on the same CAN interface -- both would fight over the bus and steal each
other's replies.

Usage:
    python3 zero_mab_motor.py --config ../config/test_params.yaml
    python3 zero_mab_motor.py --config ../config/test_params.yaml --joint joint_test1 --yes
"""

import argparse
import os
import socket
import struct
import sys
import time

import yaml

# ---------------------------------------------------------------------------
# MAB register-protocol frames (mirrors mab_fd_can.hpp, #pragma pack(1))
# ---------------------------------------------------------------------------

WRITE_FRAME_ID = 0x40
REG_RUN_ZERO = 0x8C


def build_run_zero_frame() -> bytes:
    return struct.pack("<BBhb", WRITE_FRAME_ID, 0x00, REG_RUN_ZERO, 1)


# ---------------------------------------------------------------------------
# SocketCAN FD send / receive
# ---------------------------------------------------------------------------

PF_CAN = 29
CAN_RAW = 1
SOL_CAN_RAW = 101  # SOL_CAN_BASE(100) + CAN_RAW(1)
CAN_RAW_FD_FRAMES = 5

# struct can_frame:   { __u32 can_id, __u8 len, __u8 pad, __u8 res0, __u8 len8_dlc, __u8 data[8] }
CAN_FRAME_FMT = "=IBBBB8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)
# struct canfd_frame: { __u32 can_id, __u8 len, __u8 flags, __u8 res0, __u8 res1, __u8 data[64] }
CANFD_FRAME_FMT = "=IBBxx64s"
CANFD_FRAME_SIZE = struct.calcsize(CANFD_FRAME_FMT)

DEFAULT_TIMEOUT_S = 2.0
ZERO_TIMEOUT_S = 10.0  # zeroing takes a few seconds on real hardware


def open_socket(interface: str, timeout_s: float) -> socket.socket:
    sock = socket.socket(PF_CAN, socket.SOCK_RAW, CAN_RAW)
    sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
    sock.bind((interface,))
    sock.settimeout(timeout_s)
    return sock


def send_classic(sock: socket.socket, can_id: int, data: bytes) -> None:
    if len(data) > 8:
        raise ValueError(f"classic CAN frame payload must be <=8 bytes, got {len(data)}")
    frame = struct.pack(CAN_FRAME_FMT, can_id, len(data), 0, 0, len(data), data.ljust(8, b"\x00"))
    sock.send(frame)


def recv_matching(sock: socket.socket, can_id: int, deadline_s: float):
    """Read frames (FD or classic-sized) until one from can_id arrives or we time out."""
    end = time.monotonic() + deadline_s
    while time.monotonic() < end:
        remaining = max(0.0, end - time.monotonic())
        sock.settimeout(remaining if remaining > 0 else 0.001)
        try:
            raw = sock.recv(CANFD_FRAME_SIZE)
        except (TimeoutError, socket.timeout, OSError):
            return None
        if len(raw) >= CAN_FRAME_SIZE:
            rx_can_id = struct.unpack_from("<I", raw)[0]
            if rx_can_id == can_id:
                return raw
    return None


def flush_rx_queue(sock: socket.socket) -> None:
    sock.setblocking(False)
    try:
        while True:
            if not sock.recv(CANFD_FRAME_SIZE):
                break
    except (BlockingIOError, OSError):
        pass
    finally:
        sock.setblocking(True)


# ---------------------------------------------------------------------------
# Config parsing
# ---------------------------------------------------------------------------

def load_mab_motors(config_path: str, joint_filter=None):
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    params = config["cubemars_hardware_node"]["ros__parameters"]
    joints = params["joints"]
    joint_defs = params["joint_defintions"]
    can_backends = params.get("can_backends", {})
    default_backend = params.get("comm_backend_default", "cubemars")

    motors = []
    for name in joints:
        if joint_filter and name not in joint_filter:
            continue
        joint_def = joint_defs[name]
        interface = joint_def["can_interface"]
        backend = can_backends.get(interface, default_backend)
        if backend != "mab":
            print(f"  skipping {name}: interface {interface} uses backend '{backend}', not 'mab'")
            continue
        motors.append((name, interface, joint_def["can_id"]))
    return motors


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def zero_motor(name: str, interface: str, can_id: int) -> bool:
    print(f"\n[{name}]  {interface}  id=0x{can_id:X}")
    sock = open_socket(interface, DEFAULT_TIMEOUT_S)
    try:
        # Trigger zero (long timeout - zeroing takes a few seconds on real hardware). Assumes the
        # motor is already enabled; this alone will not put it into control mode.
        flush_rx_queue(sock)
        send_classic(sock, can_id, build_run_zero_frame())
        reply = recv_matching(sock, can_id, ZERO_TIMEOUT_S)
        if reply is None:
            print(f"    No reply to run-zero within {ZERO_TIMEOUT_S:.0f}s.")
            print("    WARNING: zero command was sent but no ACK received - motor state unknown.")
            return False
        print("    origin set (ack received)")
        return True
    finally:
        sock.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Zero (set-origin) MAB-backend motors directly over CAN, "
                    "without starting the ROS 2 driver node."
    )
    default_config = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "config", "test_params.yaml"
    )
    parser.add_argument("--config", default=default_config,
                         help=f"Path to cubemars driver config YAML (default: {default_config})")
    parser.add_argument("--joint", action="append", dest="joints", default=None,
                         help="Zero only this joint (repeatable). Default: all MAB joints in config.")
    parser.add_argument("--yes", action="store_true", help="Skip the confirmation prompt")
    args = parser.parse_args()

    motors = load_mab_motors(args.config, joint_filter=set(args.joints) if args.joints else None)

    print(f"Config: {args.config}")
    print(f"Motors to zero ({len(motors)}):")
    for name, interface, can_id in motors:
        print(f"  {name:<20} interface={interface:<6} can_id={can_id}")

    if not motors:
        print("Nothing to do.")
        sys.exit(1)

    print("\nWARNING: this redefines each motor's current physical position as zero. It assumes")
    print("the motor is already enabled (control mode active) -- it will NOT enable it. Do NOT")
    print("run this while the ROS driver node already has these motors configured/active on the")
    print("same CAN interface.")

    if not args.yes:
        reply = input("\nProceed? [y/N] ").strip().lower()
        if reply != "y":
            print("Aborted.")
            sys.exit(1)

    failures = []
    for name, interface, can_id in motors:
        if not zero_motor(name, interface, can_id):
            failures.append(name)

    if failures:
        print(f"\nFailed to zero: {', '.join(failures)}")
        sys.exit(1)
    print("\nAll motors zeroed successfully.")


if __name__ == "__main__":
    main()
