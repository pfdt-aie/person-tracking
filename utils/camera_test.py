import socket
import time
import struct

import config as cfg

# Config
CAMERA_IP = cfg.CAMERA_IP
PORT = cfg.GIMBAL_PORT
BUFFER_SIZE = 1024

def crc16_ccitt(data: bytes) -> int:
    """CRC16-CCITT (XMODEM) implementation for SIYI SDK v2.0"""
    crc = 0
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

def build_packet(cmd_id: int, data: bytes = b'', seq: int = 0) -> bytes:
    """
    SIYI Header: STX (2 bytes) | CTRL (1 byte) | Data_Len (2 bytes) | Seq (2 bytes) | CMD_ID (1 byte)
    STX = 0x5566
    CTRL = 0x01 (Needs ACK)
    """
    stx = b'\x55\x66'
    ctrl = b'\x01'
    data_len = struct.pack("<H", len(data))
    sequence = struct.pack("<H", seq)
    cmd = struct.pack("B", cmd_id)
    
    packet_header = stx + ctrl + data_len + sequence + cmd
    payload = packet_header + data
    
    crc = crc16_ccitt(payload)
    return payload + struct.pack("<H", crc)

def send_command(sock, cmd_id, data=b'', seq=0):
    pkt = build_packet(cmd_id, data, seq=seq)
    sock.sendto(pkt, (CAMERA_IP, PORT))
    
    print(f"Sent CMD {hex(cmd_id)}: {' '.join(f'{b:02X}' for b in pkt)}")
    
    try:
        sock.settimeout(1.0)
        reply, addr = sock.recvfrom(BUFFER_SIZE)
        print(f"Reply from {addr}: {' '.join(f'{b:02X}' for b in reply)}")
        return reply
    except socket.timeout:
        print(f"Timeout: No reply from {CAMERA_IP} on port {PORT}")
        return None

def main() -> int:
    """Run a small SIYI UDP smoke test against the configured camera."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        current_seq = 1

        # 1. Request Firmware Version (Standard Handshake)
        print("--- Requesting Hardware ID ---")
        send_command(sock, 0x00, b'', current_seq)
        current_seq += 1
        time.sleep(0.5)

        # 2. Rotation Test
        # Data for 0x07: Yaw (-100 to 100), Pitch (-100 to 100)
        # Using signed bytes (e.g., 20 yaw, -10 pitch)
        print("--- Rotating Gimbal ---")
        yaw = 20
        pitch = 20
        rotation_data = struct.pack("bb", yaw, pitch)
        send_command(sock, 0x07, rotation_data, current_seq)

        time.sleep(1)

        # 3. Stop
        send_command(sock, 0x07, struct.pack("bb", 0, 0), current_seq + 1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
