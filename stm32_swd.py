import argparse
import binascii
import time
from pathlib import Path
import serial


def read_line(ser, timeout=30):
    old = ser.timeout
    ser.timeout = timeout
    try:
        line = ser.readline().decode('ascii', 'replace').strip()
        if not line:
            raise TimeoutError('STM32 programmer response timeout')
        return line
    finally:
        ser.timeout = old


def cmd(ser, text):
    ser.write((text + '\n').encode('ascii'))
    ser.flush()
    return read_line(ser)


def parse_ihex(path):
    upper = 0
    records = []
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        raw = bytes.fromhex(line[1:])
        if (sum(raw) & 0xFF) != 0:
            raise ValueError('Intel HEX checksum error')
        n = raw[0]
        addr = (raw[1] << 8) | raw[2]
        typ = raw[3]
        data = raw[4:4+n]
        if typ == 0:
            records.append(((upper << 16) + addr, data))
        elif typ == 1:
            break
        elif typ == 4:
            upper = (data[0] << 8) | data[1]
    return records


def main():
    ap = argparse.ArgumentParser(description='ESP32 SWD programmer for STM32F1')
    ap.add_argument('port')
    ap.add_argument('image', nargs='?')
    ap.add_argument('--info', action='store_true')
    ap.add_argument('--temp', action='store_true', help='read the latest temperature reported by STM32')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--address', type=lambda x: int(x, 0), default=0x08000000)
    args = ap.parse_args()

    segments = None
    if args.image:
        path = Path(args.image)
        if path.suffix.lower() in ('.hex', '.ihex'):
            segments = parse_ihex(path)
        else:
            segments = [(args.address, path.read_bytes())]
    elif not args.info and not args.temp:
        ap.error('image is required unless --info or --temp is used')

    with serial.Serial(args.port, args.baud, timeout=2, write_timeout=5) as ser:
        # Do not leave DTR/RTS asserted: common ESP32 auto-reset circuits use
        # these control lines and can otherwise hold the programmer in reset.
        ser.dtr = False
        ser.rts = False
        time.sleep(1.0)
        ser.reset_input_buffer()
        if args.temp:
            # Give the STM32 a moment to emit a fresh periodic sample.
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                r = cmd(ser, 'TEMP')
                if r.startswith('OK TEMP_'):
                    print(r)
                    return
                time.sleep(0.2)
            raise RuntimeError('no STM32 temperature received')

        info = cmd(ser, 'INFO')
        print(info)
        if not info.startswith('OK '):
            raise RuntimeError(info)
        if args.info:
            return
        if not segments:
            raise RuntimeError('image contains no data records')
        for address, data in segments:
            r = cmd(ser, f'ERASE 0x{address:X} 0x{len(data):X}')
            print(r)
            if r != 'OK ERASE':
                raise RuntimeError(r)

        for address, data in segments:
            for off in range(0, len(data), 256):
                block = data[off:off+256]
                crc = binascii.crc32(block) & 0xFFFFFFFF
                ser.write(f'WRITE 0x{address+off:08X} {len(block)} 0x{crc:08X}\n'.encode('ascii'))
                ser.flush()
                r = read_line(ser)
                if r != 'READY':
                    raise RuntimeError(f'write handshake failed: {r}')
                ser.write(block)
                ser.flush()
                r = read_line(ser)
                print(r)
                if not r.startswith('OK WRITE'):
                    raise RuntimeError(r)

        r = cmd(ser, 'RESET')
        print(r)
        if r != 'OK RESET':
            raise RuntimeError(r)
        print('FLASH COMPLETE')


if __name__ == '__main__':
    main()
