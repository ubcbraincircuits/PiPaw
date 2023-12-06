"""
!/usr/bin/python3

# Python library to interface with the chip LS7366R for the Raspberry Pi
# Written by Federico Bolanos
# Refactored by Brian Han
# Last Edit: Nov 27th 2023
"""
import spidev
from time import sleep


class Encoder():
    """
    Usage: import encoder then create an object by calling obj.Encoder(csx, clk, btmd)
    e.g. lever.Encoder(0, 1000000, 4)
    """
    # Commands
    CLEAR_COUNTER = 0x20
    CLEAR_STATUS = 0x30
    READ_COUNTER = 0x60
    READ_STATUS = 0x70
    WRITE_MODE0 = 0x88
    WRITE_MODE1 = 0x90

    # Modes
    NQ_COUNT = 0x00
    ONEX_COUNT = 0x01
    TWOX_COUNT = 0x02
    FOURX_COUNT = 0x03

    FOURBYTE_COUNTER = 0x00
    THREEBYTE_COUNTER = 0x01
    TWOBYTE_COUNTER = 0x02
    ONEBYTE_COUNTER = 0x03

    BYTE_MODE = [ONEBYTE_COUNTER, TWOBYTE_COUNTER, THREEBYTE_COUNTER, FOURBYTE_COUNTER]

    # Variables
    max_val = 4294967295
    counter_size = 4 #Default 4

    def __init__(self, csx, clk, btmd):
        self.counter_size = btmd # Sets the byte mode that will be used

        self.spi = spidev.SpiDev() # Initialize object
        self.spi.open(0, csx) # Which CS line will be used
        self.spi.max_speed_hz = clk # Speed of clk (modifies speed transaction)

        # Init the Encoder
        print(f'Clearing Encoder CS{csx}\'s Count...\t', self.clear_counter())
        print(f'Clearing Encoder CS{csx}\'s Status..\t', self.clear_status())

        self.spi.xfer2([self.WRITE_MODE0, self.ONEX_COUNT])

        sleep(.1) # Rest

        self.spi.xfer2([self.WRITE_MODE1, self.BYTE_MODE[self.counter_size-1]])

    def close(self):
        print('\nThanks for using me! :)')
        self.spi.close()

    def clear_counter(self):
        self.spi.xfer2([self.CLEAR_COUNTER])
        return '[DONE]'

    def clear_status(self):
        self.spi.xfer2([self.CLEAR_STATUS])
        return '[DONE]'

    def read_counter(self):
        read_transaction = [self.READ_COUNTER]

        for i in range(self.counter_size):
            read_transaction.append(0)

        data = self.spi.xfer2(read_transaction)

        encoder_count = 0
        for i in range(self.counter_size):
            encoder_count = (encoder_count << 8) + data[i+1]

        if data[1] != 255:
            return encoder_count

        return encoder_count - (self.max_val+1)

    def read_status(self):
        data = self.spi.xfer2([self.READ_STATUS, 0xFF])
        return data[1]
