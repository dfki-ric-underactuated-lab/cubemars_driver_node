#!/usr/bin/env python3
"""Persist pending register changes (e.g. maxTorque, zero position) to flash
for every active motor in a cubemars_driver_node config YAML.

For each joint listed (uncommented) under
  cubemars_hardware_node.ros__parameters.joints
(can_interface / can_id looked up in ros__parameters.joint_defintions),
sends a WRITE_REGISTER frame that triggers runSaveCmd (reg 0x080, uint8=1)
— mirrors configure_motor.py's build_save_frame.

Usage:
    python3 save_motors.py --config ../config/test_params.yaml
    python3 save_motors.py --config ../config/test_params.yaml --yes
"""

import argparse
import os
import socket
import struct
import sys

import yaml

# ---------------------------------------------------------------------------
# Message  (mirrors configure_motor.py's WRITE_REGISTER save frame)
# ---------------------------------------------------------------------------

WRITE_REGISTER = 0x40   # Md80FrameId_E::FRAME_WRITE_REGISTER (0x41 = READ)
REG_RUN_SAVE   = 0x080   # uint8 (any non-zero triggers flash save)


def build_save_frame() -> bytes:
    """WRITE_REGISTER frame that triggers runSaveCmd (flush all regs to flash)."""
    return struct.pack("<BBHB", WRITE_REGISTER, 0x00, REG_RUN_SAVE, 1)


# ---------------------------------------------------------------------------
# SocketCAN FD send / receive  (same as configure_motor.py)
# ---------------------------------------------------------------------------

PF_CAN            = 29
CAN_RAW           = 1
SOL_CAN_RAW       = 101   # SOL_CAN_BASE(100) + CAN_RAW(1)
CAN_RAW_FD_FRAMES = 5

# struct canfd_frame: { __u32 can_id, __u8 len, __u8 flags, __u8 res0, __u8 res1, __u8 data[64] }
CANFD_FRAME_FMT  = "=IBBxx64s"
CANFD_FRAME_SIZE = struct.calcsize(CANFD_FRAME_FMT)

RECEIVE_TIMEOUT_S = 2.0


def send_and_receive(interface: str, can_id: int, data: bytes):
    sock = socket.socket(PF_CAN, socket.SOCK_RAW, CAN_RAW)
    sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
    sock.bind((interface,))
    sock.settimeout(RECEIVE_TIMEOUT_S)

    frame = struct.pack(CANFD_FRAME_FMT, can_id, len(data), 0, data.ljust(64, b"\x00"))
    sock.send(frame)

    try:
        raw = sock.recv(CANFD_FRAME_SIZE)
        rx_can_id, rx_len, _, rx_data = struct.unpack(CANFD_FRAME_FMT, raw)
        return rx_can_id, rx_data[:rx_len]
    except (TimeoutError, OSError):
        return None
    finally:
        sock.close()


# ---------------------------------------------------------------------------
# Response decode (mirrors configure_motor.py)
# ---------------------------------------------------------------------------

REGISTER_RESPONSE_HEADER_FMT  = "<BBH"
REGISTER_RESPONSE_HEADER_SIZE = struct.calcsize(REGISTER_RESPONSE_HEADER_FMT)

KNOWN_REGISTERS = {
    REG_RUN_SAVE: ("B", "runSaveCmd (uint8)"),
}


def _decode_response(can_id: int, data: bytes) -> None:
    print(f"    RX  ID=0x{can_id:03X}  len={len(data)}  raw={data.hex().upper()}")
    if len(data) < REGISTER_RESPONSE_HEADER_SIZE:
        return
    if data[0] not in (0x40, 0x41):
        return
    frame_id, padding, reg_id = struct.unpack_from(REGISTER_RESPONSE_HEADER_FMT, data)
    value_bytes = data[REGISTER_RESPONSE_HEADER_SIZE:]
    if reg_id in KNOWN_REGISTERS:
        fmt, label = KNOWN_REGISTERS[reg_id]
        sz = struct.calcsize(fmt)
        if len(value_bytes) >= sz:
            val = struct.unpack_from(f"<{fmt}", value_bytes)[0]
            print(f"        register_id 0x{reg_id:04X}  value {val}  ({label})")


# ---------------------------------------------------------------------------
# Config parsing
# ---------------------------------------------------------------------------

def load_active_motors(config_path: str):
    """Return [(joint_name, can_interface, can_id), ...] for every joint
    listed under ros__parameters.joints (commented-out entries are already
    absent from the parsed YAML list)."""
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    params = config["cubemars_hardware_node"]["ros__parameters"]
    joints = params["joints"]
    joint_defs = params["joint_defintions"]

    motors = []
    for name in joints:
        joint_def = joint_defs[name]
        motors.append((name, joint_def["can_interface"], joint_def["can_id"]))
    return motors


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Persist pending register changes to flash for every active motor in a config YAML."
    )
    default_config = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "config", "test_params.yaml"
    )
    parser.add_argument("--config", default=default_config,
                        help=f"Path to cubemars driver config YAML (default: {default_config})")
    parser.add_argument("--yes", action="store_true",
                        help="Skip the confirmation prompt")
    args = parser.parse_args()

    motors = load_active_motors(args.config)

    print(f"Config: {args.config}")
    print(f"Active motors ({len(motors)}):")
    for name, interface, can_id in motors:
        print(f"  {name:<20} interface={interface:<6} can_id={can_id}")

    if not args.yes:
        reply = input("\nSave pending register changes to flash for all motors above? [y/N] ").strip().lower()
        if reply != "y":
            print("Aborted.")
            sys.exit(1)

    save_payload = build_save_frame()

    for name, interface, can_id in motors:
        print(f"\n[{name}]  {interface}  id={can_id}")
        print(f"  save  payload={save_payload.hex().upper()}")
        result = send_and_receive(interface, can_id, save_payload)
        if result is None:
            print(f"    No response within {RECEIVE_TIMEOUT_S:.0f}s.")
            print("    WARNING: save command was sent but no ACK received.")
        else:
            _decode_response(*result)


if __name__ == "__main__":
    main()
