#!/usr/bin/python3
# -*- coding: utf-8 -*-
'''Driver for the MFRC522 RFID reader (ISO/IEC 14443 A, e.g. MIFARE tags) over SPI.

Originally based on https://github.com/mxgxw/MFRC522-python (modifications by
Simon Monk). Reworked for fast, reliable tag polling on a moving robot:

- `mfrc522_to_card` waits correctly for the reply or for the reader's own
  timeout (the original wait loop misused the bitwise `~` operator and gave up
  after 20 register reads, so slow or weak replies were often lost),
- a short reader timeout (a few ms instead of 15 ms), so a poll without a tag
  is cheap and the tag is polled more often while it is under the antenna,
- the receiver gain is configurable (maximum 48 dB by default),
- `read_uid()` performs a complete wake-up + anticollision and halts the tag,
  so it answers again on the very next poll.
'''
import time

import RPi.GPIO as GPIO
import spi


class MFRC522:
    NRSTPD = 25

    MAX_LEN = 16

    PCD_IDLE       = 0x00
    PCD_AUTHENT    = 0x0E
    PCD_RECEIVE    = 0x08
    PCD_TRANSMIT   = 0x04
    PCD_TRANSCEIVE = 0x0C
    PCD_RESETPHASE = 0x0F
    PCD_CALCCRC    = 0x03

    PICC_REQIDL    = 0x26  # REQA: wakes only tags in IDLE state
    PICC_REQALL    = 0x52  # WUPA: wakes tags in IDLE and HALT state
    PICC_ANTICOLL  = 0x93
    PICC_SELECTTAG = 0x93
    PICC_AUTHENT1A = 0x60
    PICC_AUTHENT1B = 0x61
    PICC_READ      = 0x30
    PICC_WRITE     = 0xA0
    PICC_DECREMENT = 0xC0
    PICC_INCREMENT = 0xC1
    PICC_RESTORE   = 0xC2
    PICC_TRANSFER  = 0xB0
    PICC_HALT      = 0x50

    MI_OK       = 0
    MI_NOTAGERR = 1
    MI_ERR      = 2

    Reserved00     = 0x00
    CommandReg     = 0x01
    CommIEnReg     = 0x02
    DivlEnReg      = 0x03
    CommIrqReg     = 0x04
    DivIrqReg      = 0x05
    ErrorReg       = 0x06
    Status1Reg     = 0x07
    Status2Reg     = 0x08
    FIFODataReg    = 0x09
    FIFOLevelReg   = 0x0A
    WaterLevelReg  = 0x0B
    ControlReg     = 0x0C
    BitFramingReg  = 0x0D
    CollReg        = 0x0E
    Reserved01     = 0x0F

    Reserved10     = 0x10
    ModeReg        = 0x11
    TxModeReg      = 0x12
    RxModeReg      = 0x13
    TxControlReg   = 0x14
    TxAutoReg      = 0x15
    TxSelReg       = 0x16
    RxSelReg       = 0x17
    RxThresholdReg = 0x18
    DemodReg       = 0x19
    Reserved11     = 0x1A
    Reserved12     = 0x1B
    MifareReg      = 0x1C
    Reserved13     = 0x1D
    Reserved14     = 0x1E
    SerialSpeedReg = 0x1F

    Reserved20        = 0x20
    CRCResultRegM     = 0x21
    CRCResultRegL     = 0x22
    Reserved21        = 0x23
    ModWidthReg       = 0x24
    Reserved22        = 0x25
    RFCfgReg          = 0x26
    GsNReg            = 0x27
    CWGsPReg          = 0x28
    ModGsPReg         = 0x29
    TModeReg          = 0x2A
    TPrescalerReg     = 0x2B
    TReloadRegH       = 0x2C
    TReloadRegL       = 0x2D
    TCounterValueRegH = 0x2E
    TCounterValueRegL = 0x2F

    Reserved30      = 0x30
    TestSel1Reg     = 0x31
    TestSel2Reg     = 0x32
    TestPinEnReg    = 0x33
    TestPinValueReg = 0x34
    TestBusReg      = 0x35
    AutoTestReg     = 0x36
    VersionReg      = 0x37
    AnalogTestReg   = 0x38
    TestDAC1Reg     = 0x39
    TestDAC2Reg     = 0x3A
    TestADCReg      = 0x3B
    Reserved31      = 0x3C
    Reserved32      = 0x3D
    Reserved33      = 0x3E
    Reserved34      = 0x3F

    # Receiver gain values for RFCfgReg (bits 6:4)
    GAIN_18DB = 0x00
    GAIN_23DB = 0x10
    GAIN_33DB = 0x40  # chip default
    GAIN_38DB = 0x50
    GAIN_43DB = 0x60
    GAIN_48DB = 0x70  # maximum

    # Timer clock: 13.56 MHz / (2*TPrescaler + 1); TPrescaler = 0x0A9 -> 40 kHz, 25 us/tick
    _TIMER_TICK = 25e-6

    ser_num = []

    def __init__(self, dev='/dev/spidev0.0', spd=1000000, gain=GAIN_48DB, timeout=0.005,
                 rx_threshold=None, poll_sleep=0.0005):
        '''Open the SPI device and initialise the reader.

        Args:
            dev: SPI device.
            spd: SPI clock in Hz.
            gain: receiver gain (one of GAIN_*).
            timeout: how long the reader waits for a tag reply [s].
            rx_threshold: optional value for RxThresholdReg (None = chip default 0x84).
            poll_sleep: sleep between status reads while waiting for a reply [s]
                (0 = busy wait, lowest latency but one CPU core at 100 %).
        '''
        self.gain = gain
        self.timeout = timeout
        self.poll_sleep = poll_sleep
        self.rx_threshold = rx_threshold
        self.spi_fd = spi.openSPI(device=dev, speed=spd)
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.NRSTPD, GPIO.OUT)
        GPIO.output(self.NRSTPD, 1)
        self.mfrc522_init()

    # ---------------------------------------------------------------- low level

    def mfrc522_reset(self):
        self.write_mfrc522(self.CommandReg, self.PCD_RESETPHASE)
        # Wait until the oscillator is running and the soft reset has finished
        deadline = time.monotonic() + 0.1
        time.sleep(0.001)
        while self.read_mfrc522(self.CommandReg) & 0x10 and time.monotonic() < deadline:
            time.sleep(0.001)

    def write_mfrc522(self, addr, val):
        spi.transfer(self.spi_fd, ((addr << 1) & 0x7E, val))

    def read_mfrc522(self, addr):
        val = spi.transfer(self.spi_fd, (((addr << 1) & 0x7E) | 0x80, 0))
        return val[1]

    def close_mfrc522(self):
        try:
            self.antenna_off()
        finally:
            spi.closeSPI(self.spi_fd)
            GPIO.cleanup(self.NRSTPD)

    def set_bit_mask(self, reg, mask):
        tmp = self.read_mfrc522(reg)
        self.write_mfrc522(reg, tmp | mask)

    def clear_bit_mask(self, reg, mask):
        tmp = self.read_mfrc522(reg)
        self.write_mfrc522(reg, tmp & (~mask) & 0xFF)

    def antenna_on(self):
        temp = self.read_mfrc522(self.TxControlReg)
        if (temp & 0x03) != 0x03:
            self.set_bit_mask(self.TxControlReg, 0x03)

    def antenna_off(self):
        self.clear_bit_mask(self.TxControlReg, 0x03)

    def version(self):
        '''Chip version (0x91 = v1.0, 0x92 = v2.0, 0x88 = clone, 0x00/0xFF = no chip).'''
        return self.read_mfrc522(self.VersionReg)

    def set_timeout(self, timeout):
        '''Set how long the reader waits for a tag reply [s] (timer starts after transmission).'''
        self.timeout = timeout
        reload = max(1, min(0xFFFF, int(round(timeout / self._TIMER_TICK))))
        self.write_mfrc522(self.TReloadRegH, (reload >> 8) & 0xFF)
        self.write_mfrc522(self.TReloadRegL, reload & 0xFF)

    def set_gain(self, gain):
        self.gain = gain
        self.write_mfrc522(self.RFCfgReg, gain & 0x70)

    def mfrc522_to_card(self, command, send_data):
        '''Execute `command` with `send_data`; return (status, back_data, back_bits).'''
        back_data = []
        back_len = 0
        status = self.MI_ERR
        irq_en = 0x00
        wait_irq = 0x00

        if command == self.PCD_AUTHENT:
            irq_en = 0x12
            wait_irq = 0x10  # IdleIRq
        elif command == self.PCD_TRANSCEIVE:
            irq_en = 0x77
            wait_irq = 0x30  # RxIRq | IdleIRq

        self.write_mfrc522(self.CommandReg, self.PCD_IDLE)   # stop any active command
        self.write_mfrc522(self.CommIEnReg, irq_en | 0x80)
        self.write_mfrc522(self.CommIrqReg, 0x7F)            # clear all interrupt bits
        self.write_mfrc522(self.FIFOLevelReg, 0x80)          # flush FIFO

        for b in send_data:
            self.write_mfrc522(self.FIFODataReg, b)

        self.write_mfrc522(self.CommandReg, command)

        if command == self.PCD_TRANSCEIVE:
            self.set_bit_mask(self.BitFramingReg, 0x80)  # StartSend

        # Wait for the reply or for the reader's timer (TimerIRq). The wall-clock
        # deadline is only a safety net in case the chip does not respond at all.
        # Between register reads sleep for `poll_sleep` to keep the CPU load low.
        deadline = time.monotonic() + self.timeout + 0.025
        timed_out = True
        while True:
            n = self.read_mfrc522(self.CommIrqReg)
            if n & wait_irq:
                timed_out = False
                break
            if n & 0x01:  # TimerIRq: no reply from the tag
                break
            if time.monotonic() > deadline:
                break
            if self.poll_sleep > 0.0:
                time.sleep(self.poll_sleep)

        self.clear_bit_mask(self.BitFramingReg, 0x80)

        if timed_out:
            self.write_mfrc522(self.CommandReg, self.PCD_IDLE)
            return (self.MI_NOTAGERR, back_data, back_len)

        if self.read_mfrc522(self.ErrorReg) & 0x1B:  # BufferOvfl, CollErr, ParityErr, ProtocolErr
            return (self.MI_ERR, back_data, back_len)

        status = self.MI_OK
        if n & irq_en & 0x01:
            status = self.MI_NOTAGERR

        if command == self.PCD_TRANSCEIVE:
            n = self.read_mfrc522(self.FIFOLevelReg)
            last_bits = self.read_mfrc522(self.ControlReg) & 0x07
            if last_bits != 0:
                back_len = (n - 1) * 8 + last_bits
            else:
                back_len = n * 8

            if n == 0:
                n = 1
            if n > self.MAX_LEN:
                n = self.MAX_LEN

            for _ in range(n):
                back_data.append(self.read_mfrc522(self.FIFODataReg))

        return (status, back_data, back_len)

    # ---------------------------------------------------------- ISO 14443 A

    def mfrc522_request(self, req_mode):
        '''Send REQA (PICC_REQIDL) or WUPA (PICC_REQALL); return (status, back_bits).'''
        self.write_mfrc522(self.BitFramingReg, 0x07)  # short frame, 7 bits
        (status, back_data, back_bits) = self.mfrc522_to_card(self.PCD_TRANSCEIVE, [req_mode])

        if status != self.MI_OK or back_bits != 0x10:  # ATQA is 16 bits
            status = self.MI_ERR

        return (status, back_bits)

    def mfrc522_anticoll(self):
        '''Cascade level 1 anticollision; return (status, uid) with uid = 4 bytes + BCC.'''
        self.write_mfrc522(self.BitFramingReg, 0x00)
        self.write_mfrc522(self.CollReg, 0x80)  # ValuesAfterColl=1 (recommended)

        (status, back_data, back_bits) = self.mfrc522_to_card(self.PCD_TRANSCEIVE,
                                                              [self.PICC_ANTICOLL, 0x20])

        if status == self.MI_OK:
            if len(back_data) == 5 and back_bits == 40:
                check = 0
                for i in range(4):
                    check ^= back_data[i]
                if check != back_data[4]:
                    status = self.MI_ERR
            else:
                status = self.MI_ERR

        return (status, back_data)

    def mfrc522_halt(self):
        '''Send HLTA. The tag goes to HALT (if selected) or back to IDLE, so the next
        WUPA wakes it immediately. The tag does not answer HLTA, so the expected
        result is a timeout; nothing is returned.'''
        self.write_mfrc522(self.BitFramingReg, 0x00)
        # HLTA = 50 00 + CRC_A (0x57, 0xCD), precomputed to avoid using the CRC unit
        saved = self.timeout
        self.set_timeout(0.001)
        try:
            self.mfrc522_to_card(self.PCD_TRANSCEIVE, [self.PICC_HALT, 0x00, 0x57, 0xCD])
        finally:
            self.set_timeout(saved)

    def read_uid(self, retries=1):
        '''Poll for a tag once. Return the UID (4 bytes + BCC) as a list, or None.

        Uses WUPA, so tags in HALT state answer too, i.e. a tag that stays in the
        field is reported on every call (presence detection). `retries` extra
        attempts are made if the tag answered the wake-up but anticollision failed.
        '''
        for _ in range(1 + retries):
            (status, _atqa) = self.mfrc522_request(self.PICC_REQALL)
            if status != self.MI_OK:
                return None  # no tag answered within the timeout
            (status, uid) = self.mfrc522_anticoll()
            if status == self.MI_OK:
                # Put the tag back to IDLE/HALT (it is now in READY state, in which
                # it would ignore the next wake-up request).
                self.mfrc522_halt()
                return uid
        return None

    def calculate_crc(self, p_in_data):
        self.write_mfrc522(self.CommandReg, self.PCD_IDLE)
        self.write_mfrc522(self.DivIrqReg, 0x04)       # clear CRCIRq
        self.write_mfrc522(self.FIFOLevelReg, 0x80)    # flush FIFO
        for b in p_in_data:
            self.write_mfrc522(self.FIFODataReg, b)
        self.write_mfrc522(self.CommandReg, self.PCD_CALCCRC)
        deadline = time.monotonic() + 0.05
        while not (self.read_mfrc522(self.DivIrqReg) & 0x04):
            if time.monotonic() > deadline:
                break
        self.write_mfrc522(self.CommandReg, self.PCD_IDLE)
        return [self.read_mfrc522(self.CRCResultRegL), self.read_mfrc522(self.CRCResultRegM)]

    def mfrc522_select_tag(self, ser_num):
        buf = [self.PICC_SELECTTAG, 0x70] + list(ser_num[:5])
        buf += self.calculate_crc(buf)
        (status, back_data, back_len) = self.mfrc522_to_card(self.PCD_TRANSCEIVE, buf)

        if (status == self.MI_OK) and (back_len == 0x18):
            return back_data[0]
        else:
            return 0

    def mfrc522_auth(self, auth_mode, BlockAddr, Sectorkey, ser_num):
        buff = [auth_mode, BlockAddr] + list(Sectorkey) + list(ser_num[:4])

        (status, back_data, back_len) = self.mfrc522_to_card(self.PCD_AUTHENT, buff)

        if status != self.MI_OK:
            return self.MI_ERR
        if not (self.read_mfrc522(self.Status2Reg) & 0x08):
            return self.MI_ERR

        return status

    def mfrc522_stop_crypto1(self):
        self.clear_bit_mask(self.Status2Reg, 0x08)

    def mfrc522_read(self, block_addr):
        recv_data = [self.PICC_READ, block_addr]
        recv_data += self.calculate_crc(recv_data)
        (status, back_data, back_len) = self.mfrc522_to_card(self.PCD_TRANSCEIVE, recv_data)
        if status != self.MI_OK:
            return None
        if len(back_data) == 16:
            return back_data
        else:
            return None

    def mfrc522_write(self, block_addr, write_data):
        buff = [self.PICC_WRITE, block_addr]
        buff += self.calculate_crc(buff)
        (status, back_data, back_len) = self.mfrc522_to_card(self.PCD_TRANSCEIVE, buff)
        if status != self.MI_OK or back_len != 4 or (back_data[0] & 0x0F) != 0x0A:
            status = self.MI_ERR

        if status == self.MI_OK:
            buf = list(write_data[:16])
            buf += self.calculate_crc(buf)
            (status, back_data, back_len) = self.mfrc522_to_card(self.PCD_TRANSCEIVE, buf)
            if status != self.MI_OK or back_len != 4 or (back_data[0] & 0x0F) != 0x0A:
                print('Error while writing')

    def mfrc522_dump_classic_1k(self, key, uid):
        for i in range(64):
            status = self.mfrc522_auth(self.PICC_AUTHENT1A, i, key, uid)
            if status == self.MI_OK:
                self.mfrc522_read(i)
            else:
                print('Authentication error')

    def mfrc522_init(self):
        GPIO.output(self.NRSTPD, 1)

        self.mfrc522_reset()

        # Timer: TAuto=1 (starts automatically after transmission), 25 us per tick
        self.write_mfrc522(self.TModeReg, 0x80)
        self.write_mfrc522(self.TPrescalerReg, 0xA9)
        self.set_timeout(self.timeout)

        self.write_mfrc522(self.TxAutoReg, 0x40)  # 100 % ASK modulation
        self.write_mfrc522(self.ModeReg, 0x3D)    # CRC preset 0x6363 (ISO 14443-3)
        if self.rx_threshold is not None:
            self.write_mfrc522(self.RxThresholdReg, self.rx_threshold)
        self.set_gain(self.gain)
        self.antenna_on()
