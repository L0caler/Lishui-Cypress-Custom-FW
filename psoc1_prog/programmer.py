import sys
import os
import time
import serial
from array import array

BLOCK_SIZE = 64


class Program:
    def __init__(self, intelhex):
        self.program = b''
        self.secure = b''
        self.checksum = b''

        with open(intelhex, 'r') as f:
            mode = 0
            for i, l in enumerate(f.readlines()):
                l = l.rstrip()
                if not l.startswith(':') or len(l) < 11:
                    continue
                size = int(l[1:3], 16)
                addr = int(l[3:7], 16)
                rtype = int(l[7:9], 16)
                if len(l) != 11 + size * 2:
                    raise ValueError(
                        f"{intelhex}:{i} invalid number of characters, given {size * 2} found {len(l) - 11}")
                data = bytes.fromhex(l[9:-2])
                csum = (-sum(bytes.fromhex(l[1:-2]))) & 0x0FF
                csum_given = int(l[-2:], 16)
                if csum != csum_given:
                    raise ValueError(f"{intelhex}:{i} checksum failed, given 0x{csum_given.to_bytes(1, 'big').hex()} "
                                     f"calculated 0x{csum.to_bytes(1, 'big').hex()}")
                if rtype == 0:
                    if mode == 0:
                        if addr != len(self.program):
                            raise ValueError(f"{intelhex}:{i} expected address to be continuous")
                        self.program += data
                    elif mode == 16:
                        if size != BLOCK_SIZE:
                            raise ValueError(f"{intelhex}:{i} expected data size was {BLOCK_SIZE} bytes, found {size}")
                        self.secure = data
                    elif mode == 32:
                        self.checksum = data
                elif rtype == 4:
                    mode = int.from_bytes(data, 'big')
                elif rtype == 1:
                    break


class PSoC1Prog:
    # Silicon ID -> part name. Taken from the Cypress PSoC Programmer device
    # database, <PSoC Programmer>/Devices/*.dat. Keep in step with the device
    # table in src/devices.h, which also holds the flash sizes.
    DEVICE_IDS = {
        # CY8C27x43, 16 KB
        0x0009: "CY8C27143",
        0x000A: "CY8C27243",
        0x000B: "CY8C27443",
        0x000C: "CY8C27543",
        0x000D: "CY8C27643",
        # CY8C21x23, 4 KB
        0x0017: "CY8C21123",
        0x0018: "CY8C21223",
        0x0019: "CY8C21323",
        # CY8C24x23A, 4 KB
        0x0032: "CY8C24123A",
        0x0033: "CY8C24223A",
        0x0034: "CY8C24423A",
        # CY8C21x34, 8 KB
        0x0036: "CY8C21234",
        0x0037: "CY8C21334",
        0x0038: "CY8C21434",
        0x0040: "CY8C21534",
        0x0049: "CY8C21634",
        # CY8C21x12, 8 KB
        0x0837: "CY8C21312",
        0x0840: "CY8C21512",
        # CY8C23x33, 8 KB
        0x08B0: "CY8C23433",
        0x08B1: "CY8C23533",
        0x08B2: "CY8C23033",
        # CY8C24x33, 8 KB
        0x00B0: "CY8C24533",
        0x00B1: "CY8C24633",
        0x00B2: "CY8C24033",
        # CapSense derivatives of CY8C21x34, 8 KB
        0x0638: "CY8CTST110_32LTXI",
        0x0639: "CY8CTST110_00PVXI",
        0x0738: "CY8CTMG110_32LTXI",
        0x0739: "CY8CTMG110_00PVXI",
    }

    INIT_MODES = {'auto': 0, 'reset': 1, 'power': 2}
    MODE_NAMES = {0: 'auto', 1: 'reset', 2: 'power cycle'}

    def __init__(self, serial_port):
        self.ser = serial.Serial(port=serial_port, baudrate=115200, timeout=2, write_timeout=2)
        # Opening the port normally resets the board, but if it did not then
        # the greeting from a previous run is still queued up.
        self.ser.reset_input_buffer()
        self.ser.write(b'\n')
        # The board reboots when the port opens, so allow 10 s for the
        # prompt. Never change ser.timeout to do that: on Windows it makes
        # pyserial re-apply the port settings, the USB serial bridge then
        # reconfigures its UART, and at 115200 that corrupts bytes in flight.
        self._read_deadline(b'> ', 10)
        buf1 = os.urandom(BLOCK_SIZE)
        self._write_blk(buf1)
        buf2 = self._read_blk()
        if buf1 != buf2:
            print("Sent: " + buf1.hex(), file=sys.stderr)
            print("Recv: " + buf2.hex(), file=sys.stderr)
            raise ValueError("Programmer self-test failed!")

    def _read_deadline(self, until, seconds):
        """Read until `until` (a byte count or a terminator) or `seconds`
        pass, without touching ser.timeout (see __init__)."""
        deadline = time.monotonic() + seconds
        buf = b''
        while time.monotonic() < deadline:
            if isinstance(until, int):
                buf += self.ser.read(until - len(buf))
                if len(buf) >= until:
                    break
            else:
                buf += self.ser.read_until(until)
                if buf.endswith(until):
                    break
        return buf

    def _wait(self, timeout=None):
        # Acquiring a target can take seconds, because a power cycle holds the
        # supply down for a while, so some commands need
        # longer than the port's default timeout.
        x = self._read_deadline(4, timeout if timeout is not None else 2)
        if len(x) == 0:
            raise ValueError("Chip response timeout")
        if x != b'\r\n> ':
            raise ValueError("Programmer resturned unexpected result: " + x.hex())

    def reinitialise(self, mode=None):
        if mode is None:
            self.ser.write(b'i')
        else:
            self.ser.write(b'I' + array('B', [self.INIT_MODES[mode]]).tobytes())
        self._wait(timeout=30)

    def get_init_mode(self):
        self.ser.write(b'm')
        res = self.ser.read_until(b'\r\n')
        self._wait_prompt()
        return int(res.strip(), 16)

    def get_errors(self):
        """Read and clear the firmware's WAIT-AND-POLL timeout counter.

        Anything other than zero means the target stopped answering, so the
        operation that ran before it cannot be trusted.
        """
        self.ser.write(b'E')
        res = self.ser.read_until(b'\r\n')
        self._wait_prompt()
        return int(res.strip(), 16)

    def _wait_prompt(self):
        x = self.ser.read(2)
        if x != b'> ':
            raise ValueError("Programmer resturned unexpected result: " + x.hex())

    def reset_device(self):
        self.ser.write(b'a')
        self._wait(timeout=30)

    def get_device_id(self):
        self.ser.write(b'D')
        res = self.ser.read(2)
        self._wait()
        return int.from_bytes(res, 'little')

    def get_firmware_id(self):
        self.ser.write(b'F')
        res = self.ser.read(2)
        self._wait()
        return int.from_bytes(res, 'little')

    def get_flash_blocks(self):
        """Flash size of the attached target in 64 byte blocks, 0 if unknown."""
        self.ser.write(b'N')
        res = self.ser.read(2)
        self._wait()
        return int.from_bytes(res, 'little')

    def set_flash_blocks(self, blocks):
        """Override the flash size the firmware took from its device table."""
        self.ser.write(b'B' + array('B', [blocks & 0xFF, (blocks >> 8) & 0xFF]).tobytes())
        self._wait()

    def set_supply(self, volts):
        """Pick the INITIALIZE-3 vector set: 3 for Vdd <= 3.6V, 5 above."""
        self.ser.write(b'v' + array('B', [1 if volts >= 4 else 0]).tobytes())
        self._wait()

    def erase_memory(self):
        self.ser.write(b'e')
        self._wait()

    def read_checksum(self):
        self.ser.write(b'C')
        res = self.ser.read(2)
        self._wait()
        return res[::-1]

    def read_memory(self, n, offset=0x80):
        # read n x 64 byte memory blocks
        blocks = b''
        for i in range(n):
            self.ser.write(b'r' + array('B', [i, offset]).tobytes())
            self._wait()
            block = self._read_blk()
            blocks += block
            print(f"\rReading 0x{i.to_bytes(1, 'big').hex()} [{(i+1)/n*100:.2f}%]", end='', file=sys.stderr)
        print('', file=sys.stderr)
        return blocks

    def get_device_name(self):
        devid = self.get_device_id()
        return self.DEVICE_IDS.get(devid, devid.to_bytes(2, 'big').hex())

    def _write_blk(self, data):
        if len(data) != BLOCK_SIZE:
            raise ValueError(f"Data is not {BLOCK_SIZE} bytes")
        self.ser.write(b't')
        self.ser.write(data)
        self._wait()
        return

    def _read_blk(self):
        self.ser.write(b's')
        blk = self.ser.read(BLOCK_SIZE)
        self._wait()
        return blk

    def write_program(self, data):
        self.erase_memory()
        count = len(data) // BLOCK_SIZE
        for i in range(count):
            self._write_blk(data[i*BLOCK_SIZE:(i+1)*BLOCK_SIZE])
            self.ser.write(b'w' + array('B', [i]).tobytes())
            self._wait()
            print(f"\rWriting 0x{i.to_bytes(1, 'big').hex()} [{(i+1)/count*100:.2f}%]", end='', file=sys.stderr)
        print('', file=sys.stderr)

    def write_secure(self, data):
        self._write_blk(data)
        self.ser.write(b'x')
        self._wait()

    MON_ESCAPE = b'\x1d'    # Ctrl-], leaves monitor mode on the firmware

    def monitor(self, baud=19200, echo=True, hexdump=False):
        """Bridge this terminal to the target's UART on P1[0]/P1[1] through
        the firmware's monitor mode, until Ctrl-] or Ctrl-C is pressed."""
        import threading

        fw = self.get_firmware_id()
        if fw < 0x0004:
            raise ValueError(f"programmer firmware 0x{fw:04x} has no monitor mode, "
                             "flash the Arduino with fw 0x0004 or later")

        self.ser.write(b'k' + str(int(baud)).encode() + b'\r')
        res = self._read_deadline(b'> ', 2)
        rate = res.strip().rstrip(b'>').strip()
        if rate != str(int(baud)).encode():
            raise ValueError(f"programmer refused {baud} baud, it answered {res!r}")

        self.ser.write(b'M')
        res = self._read_deadline(b'\r\n', 2)
        if res.strip() != rate:
            raise ValueError(f"programmer did not enter monitor mode, it answered {res!r}")

        log(f"monitor at {baud} baud on P1[0] (target TX) / P1[1] (target RX)")
        log("Ctrl-] or Ctrl-C to quit" + (", sent keys shown as [tx ...]" if echo else ""))

        stop = threading.Event()
        out = sys.stdout

        # Never change ser.timeout here (see __init__); poll in_waiting instead.
        def reader():
            while not stop.is_set():
                n = self.ser.in_waiting
                if not n:
                    time.sleep(0.005)
                    continue
                data = self.ser.read(n)
                if hexdump:
                    out.write(''.join(f"<{b:02x}>" for b in data))
                else:
                    out.write(data.decode('ascii', 'replace'))
                out.flush()

        def send(key):
            self.ser.write(key)
            if echo:
                out.write(f"[tx {key!r}]")
                out.flush()

        rx = threading.Thread(target=reader, daemon=True)
        rx.start()
        try:
            if os.name == 'nt':
                import msvcrt
                while True:
                    if not msvcrt.kbhit():
                        time.sleep(0.01)
                        continue
                    key = msvcrt.getch()
                    if key in (b'\x00', b'\xe0'):   # arrow / function key prefix
                        msvcrt.getch()
                        continue
                    if key in (self.MON_ESCAPE, b'\x03'):
                        break
                    send(key)
            else:
                import termios
                import tty
                import select
                fd = sys.stdin.fileno()
                old = termios.tcgetattr(fd)
                tty.setraw(fd)
                try:
                    while True:
                        if not select.select([fd], [], [], 0.05)[0]:
                            continue
                        key = os.read(fd, 1)
                        if key in (self.MON_ESCAPE, b'\x03'):
                            break
                        send(key)
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old)
        except KeyboardInterrupt:
            pass
        finally:
            stop.set()
            rx.join()
            # Leave monitor mode, so the programmer is back at its prompt
            self.ser.write(self.MON_ESCAPE)
            self._read_deadline(b'> ', 2)
            out.write("\n")
            log("monitor closed")

    def close(self):
        self.ser.close()

def log(text):
    print("PSoC1_Prog: " + text, file=sys.stderr)

if __name__ == '__main__':
    import argparse
    import sys

    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("port", help="Serial port")
    parser.add_argument("cmd", choices=['flash', 'checksum', 'device', 'read', 'erase', 'reset', 'fw', 'monitor'],
        help="Command to run\n" +
        "flash - write .hex to device\n" +
        "checksum - returns program checksum from device\n" +
        "device - returns device name or identification hex\n" +
        "read - dumps device program to file\n" +
        "erase - deletes all devices program memory\n" +
        "reset - restarts device\n"
        "fw - returns programmer firmware version\n"
        "monitor - serial terminal to the target's UART (P1[0] TX, P1[1] RX)\n"
    )
    parser.add_argument("-i", "--input", help="Input intel hex file for flashing")
    parser.add_argument("-o", "--output", help="Output binary for memory dump")
    parser.add_argument("--offset", type=int, default=0x80, help="Memory dump read address offset")
    parser.add_argument("--count", type=int, default=None,
                        help="Memory dump read count in 64 byte blocks (default: the device's flash size)")
    parser.add_argument("--read", action="store_true", help="Read back program when flashing to double check")
    parser.add_argument("--reset", action="store_true", help="Reset device after command is complete")
    parser.add_argument("--init", type=bool, default=True, help="Reinitialise programming mode on device")
    parser.add_argument("--init-mode", choices=['auto', 'reset', 'power'], default=None,
                        help="How to enter programming mode\n"
                             "auto  - XRES first, power cycle if that finds nothing (default)\n"
                             "reset - XRES pulse only\n"
                             "power - power cycle only, for packages without an XRES pin\n"
                             "        such as the 28 pin CY8C24533")
    parser.add_argument("--vdd", type=float, default=None,
                        help="Target supply voltage, picks the INITIALIZE-3 vector set (3 or 5)")
    parser.add_argument("--blocks", type=int, default=None,
                        help="Override the target's flash size in 64 byte blocks")
    parser.add_argument("--ignore-checksum", action="store_true",
                        help="Report a checksum mismatch after flashing but do not fail")
    parser.add_argument("--monitor", action="store_true",
                        help="Open the serial monitor after the command (and --reset) is done")
    parser.add_argument("--baud", type=int, default=19200,
                        help="Target UART line rate for the monitor (default 19200, max 57600)")
    parser.add_argument("--hex", action="store_true",
                        help="Monitor: show received bytes as hex instead of text")
    parser.add_argument("--no-echo", action="store_true",
                        help="Monitor: do not print the keys that are sent")

    args = parser.parse_args()

    prog = PSoC1Prog(args.port)
    log("programmer initialised")

    if args.cmd == 'monitor':
        # No programming mode here: that would halt the target's application.
        # --reset restarts the target first, which needs it acquired once so
        # the firmware knows to power cycle it (the CY8C24533 has no XRES).
        if args.reset:
            log("restarting target")
            prog.reinitialise(args.init_mode)
            prog.reset_device()
        prog.monitor(args.baud, echo=not args.no_echo, hexdump=args.hex)
        exit(0)
    if args.vdd is not None:
        prog.set_supply(args.vdd)
    if args.init or args.init_mode is not None:
        prog.reinitialise(args.init_mode)
    if args.blocks is not None:
        prog.set_flash_blocks(args.blocks)
    # auto mode may have tried and abandoned a power cycle, so start clean
    prog.get_errors()

    devid = prog.get_device_id()
    blocks = prog.get_flash_blocks()
    if args.cmd != 'fw':
        name = prog.DEVICE_IDS.get(devid, 'unknown')
        mode = prog.MODE_NAMES.get(prog.get_init_mode(), '?')
        size = f"{blocks * BLOCK_SIZE} bytes of flash" if blocks else "flash size unknown"
        log(f"target {name} (id 0x{devid:04x}), acquired by {mode}, {size}")

    if args.cmd in ('flash', 'read') and not blocks:
        log("the flash size of this target is unknown; add it to the device "
            "table in src/devices.h or pass --blocks")
        exit(1)

    if args.cmd == 'flash':
        if args.input is None:
            log("input is not specified")
            exit(1)
        ifile = Program(args.input)
        if len(ifile.program) % BLOCK_SIZE:
            log(f"program is not a whole number of {BLOCK_SIZE} byte blocks")
            exit(1)
        if len(ifile.program) > blocks * BLOCK_SIZE:
            log(f"program is {len(ifile.program)} bytes but the target only has "
                f"{blocks * BLOCK_SIZE} bytes of flash")
            exit(1)
        log("erasing memory")
        prog.erase_memory()
        log("writing program")
        prog.write_program(ifile.program)
        log("writing secure")
        prog.write_secure(ifile.secure)
        errs = prog.get_errors()
        if errs:
            log(f"the target stopped answering {errs} time(s) while programming, "
                "so the flash contents are not trustworthy")
            exit(1)
        log("checking checksum")
        csum = prog.read_checksum()
        if csum != ifile.checksum:
            log(f"checksum mismatch, device={csum.hex()} file={ifile.checksum.hex()}")
            if not args.ignore_checksum:
                exit(1)
        if args.read:
            mem_data = prog.read_memory(len(ifile.program) // BLOCK_SIZE)
            if mem_data != ifile.program:
                log(f"written program does not match one on device")
                exit(1)
        log("success")
    elif args.cmd == 'checksum':
        if args.input is not None:
            log("file checksum: " + Program(args.input).checksum.hex())
        print(prog.read_checksum().hex())
    elif args.cmd == 'device':
        print(prog.get_device_name())
    elif args.cmd == 'read':
        if args.output is None:
            log("output is not specified")
            exit(1)
        with open(args.output, 'wb') as f:
            f.write(prog.read_memory(args.count or blocks, args.offset))
        log("success")
    elif args.cmd == 'erase':
        prog.erase_memory()
        log("success")
    elif args.cmd == 'reset':
        prog.reset_device()
        exit(0)
    elif args.cmd == 'fw':
        print(prog.get_firmware_id())
    if args.reset:
        prog.reset_device()
    if args.monitor:
        prog.monitor(args.baud, echo=not args.no_echo, hexdump=args.hex)
    exit(0)
