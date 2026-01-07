import binascii

def crc16_modbus(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

# New announce packet from log (Addr=0x01): FF 0A 81 01 01 ED ED 16 50 4E 27 C9 5D
# CRC in log: BE C9 (LE) -> 0xC9BE
data_ann = bytes.fromhex("FF0A810101EDED16504E27C95D")
print(f"CRC for Announce (Addr=0x01): {crc16_modbus(data_ann):04X}")

# User's test payload: 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D
# Length byte: 0x0C (12)
data_test = bytes.fromhex("FF0C0102030405060708090A0B0C0D")
crc_test = crc16_modbus(data_test)
print(f"Valid CRC for your test packet: {crc_test:04X} (LE: {crc_test&0xFF:02X} {crc_test>>8:02X})")

# Real Poll for Temperature Min (Reg 10 = 0x000A) on Addr 0x01
# FF (Sync) 03 (Len-1: MsgType + Addr + RegLo + RegHi = 4 bytes)
# 04 (Read) 01 (Addr) 0A (RegLo) 00 (RegHi)
data_poll = bytes.fromhex("FF0304010A00")
crc_poll = crc16_modbus(data_poll)
print(f"Valid Poll Packet (Temp Min): FF 03 04 01 0A 00 {crc_poll&0xFF:02X} {crc_poll>>8:02X}")
