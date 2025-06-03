# LibreCellular RFE-Mini test scripts - 20250302
# See: https://github.com/myriadrf/lc-rfe-ctl

import board
import digitalio
import busio
import time

### Monitor pin
monitor_pin = digitalio.DigitalInOut(board.GP6)
monitor_pin.direction = digitalio.Direction.INPUT
monitor_pin.pull = digitalio.Pull.UP

### RF control pins / TDD
tdd_mcu_pin = digitalio.DigitalInOut(board.GP12)
tdd_ext_pin = digitalio.DigitalInOut(board.GP13)
pa_bypass_pin = digitalio.DigitalInOut(board.GP14)
lna_bypass_pin = digitalio.DigitalInOut(board.GP15)

### RF Switches
sw3_vctl_pin = digitalio.DigitalInOut(board.GP19)
sw1_v1_pin = digitalio.DigitalInOut(board.GP20)
sw1_v2_pin = digitalio.DigitalInOut(board.GP21)
sw2_vctl_pin = digitalio.DigitalInOut(board.GP22)

### Power Measurement
#SPI0 - GP16: PM_SDO, GP18: PM_SCK
pm_spi = busio.SPI(clock=board.GP18, MISO=board.GP16)
#GP17: PM_CONV
pm_conv_pin = digitalio.DigitalInOut(board.GP17)

### Step Attenuator
#SPI1 - GP27: ATTEN_DATA, GP26: ATTEN_CLK
atten_spi = busio.SPI(clock=board.GP26, MOSI=board.GP27)
#GP28: ATTEN_LE
atten_le_pin = digitalio.DigitalInOut(board.GP28)

### Channel defs
RF_CH_A = 0
RF_CH_B = 1


def init_monitor_mode():
    tdd_mcu_pin.direction = digitalio.Direction.INPUT
    tdd_ext_pin.direction = digitalio.Direction.INPUT
    pa_bypass_pin.direction = digitalio.Direction.INPUT
    lna_bypass_pin.direction = digitalio.Direction.INPUT
    sw3_vctl_pin.direction = digitalio.Direction.INPUT
    sw1_v1_pin.direction = digitalio.Direction.INPUT
    sw1_v2_pin.direction = digitalio.Direction.INPUT
    sw2_vctl_pin.direction = digitalio.Direction.INPUT


def init_normal_mode():
    tdd_mcu_pin.direction = digitalio.Direction.OUTPUT
    tdd_ext_pin.direction = digitalio.Direction.OUTPUT
    pa_bypass_pin.direction = digitalio.Direction.OUTPUT
    lna_bypass_pin.direction = digitalio.Direction.OUTPUT
    sw3_vctl_pin.direction = digitalio.Direction.OUTPUT
    sw1_v1_pin.direction = digitalio.Direction.OUTPUT
    sw1_v2_pin.direction = digitalio.Direction.OUTPUT
    sw2_vctl_pin.direction = digitalio.Direction.OUTPUT

    ### Power monitor (LTC5587)
    pm_conv_pin.direction = digitalio.Direction.OUTPUT
    pm_conv_pin.value = True

    ### Step attenuator (F1956)
    atten_le_pin.direction = digitalio.Direction.OUTPUT
    atten_le_pin.value = True

def reverse_bits(byte_value):
    """Reverse the bits in a byte to convert between MSB and LSB first."""
    result = 0
    for i in range(8):
        # Shift result left and add the least significant bit of byte_value
        result = (result << 1) | (byte_value & 1)
        byte_value >>= 1
    return result

def set_rx_atten(channel, atten_value):
    """
    Set the RF attenuation value for the specified channel.

    Args:
        channel (int): Channel enum (RF_CH_A or RF_CH_B)
        atten_value (float): Attenuation value in dB (range: 0.00 - 31.75)
    """
    step = 0.25

    # Max attenuation value is 31.75, clip there
    if atten_value > 31.75:
        atten_value = 31.75

    # Prep attenuation word
    atten_word = int(atten_value / step)  # F1956 has fixed 0.25 attenuation steps
    atten_word &= 0b01111111  # Make sure D7 is always 0

    # Prep address word
    # CH A has A0 connected to GND
    # CH B has A0 connected to VCC
    if channel == RF_CH_A:
        addr_word = 0
    elif channel == RF_CH_B:
        addr_word = 1

    # F1956 wants data LSB first, so we flip 'em
    atten_word_reversed = reverse_bits(atten_word)
    addr_word_reversed = reverse_bits(addr_word)

    # Begin transfer...
    atten_le_pin.value = False  # ensure LE low to begin transfer
    time.sleep(0.01)

    # Write 2 bytes
    tx_buf = bytearray([atten_word_reversed, addr_word_reversed])
    atten_spi.try_lock()
    atten_spi.configure(baudrate=100000, phase=0, polarity=0)
    atten_spi.write(tx_buf)
    atten_spi.unlock()
    time.sleep(0.001)

    # End transfer
    atten_le_pin.value = True  # LE high to latch data
    #pg 8: It is recommended that Latch enable be left high
    #when the device is not being programmed.


# Check state of the pin on power on, and if the jumper is closed
# then do not initialise any other pins, just sit in a loop...
if monitor_pin.value == False:
    print("monitor mode active, just printing pin states...")
    while True:
        # print the state of all input pins setup in monitor mode
        print("   TDD_MCU:", tdd_mcu_pin.value)
        print("   TDD_EXT:", tdd_ext_pin.value)
        print(" PA_BYPASS:", pa_bypass_pin.value)
        print("LNA_BYPASS:", lna_bypass_pin.value)
        print("  SW3_VCTL:", sw3_vctl_pin.value)
        print("    SW1_V1:", sw1_v1_pin.value)
        print("    SW1_V2:", sw1_v2_pin.value)
        print("  SW2_VCTL:", sw2_vctl_pin.value)
        print("------------------")
        time.sleep(2)
else:  # set things up for normal operation mode...
    print("normal mode active, initialising things...")

    print("MAKE SURE ALL JUMPERS ARE REMOVED - PICO DRIVES CONTROL SIGNALS!")
    print("MAKE SURE ALL JUMPERS ARE REMOVED - PICO DRIVES CONTROL SIGNALS!")
    print("MAKE SURE ALL JUMPERS ARE REMOVED - PICO DRIVES CONTROL SIGNALS!")
    time.sleep(1)

    init_normal_mode()

    #########################################
    ### testing - refer to the google doc ###
    #########################################
    test_mode = 5  # <<<< CHANGE THIS ONLY
    #########################################
    #########################################


    if test_mode == 1:
        print("Test 1 - RX Path, LNA Bypass, 0dB attenuation")
        print("Check VNA ports - TX and RX has different directions!")

        tdd_ext_pin.value = True
        tdd_mcu_pin.value = True
        lna_bypass_pin.value = True
        sw2_vctl_pin.value = True
        set_rx_atten(RF_CH_A, 0)

        while True:
            pass

    elif test_mode == 2:
        print("Test 2 - RX Path, LNA Enable, 0dB attenuation")
        print("Check VNA ports - TX and RX has different directions!")

        tdd_ext_pin.value = True
        tdd_mcu_pin.value = True
        lna_bypass_pin.value = False # diff
        sw2_vctl_pin.value = True
        set_rx_atten(RF_CH_A, 0)

        while True:
            pass

    elif test_mode == 3:
        print("Test 3 - TX Path, PA Bypass")
        print("Check VNA ports - TX and RX has different directions!")

        tdd_ext_pin.value = True
        tdd_mcu_pin.value = True
        pa_bypass_pin.value = True
        sw1_v1_pin.value = True
        sw1_v2_pin.value = False

        while True:
            pass

    elif test_mode == 4:
        print("Test 4 - TX Path, PA Enable")
        print("Check VNA ports - TX and RX has different directions!")

        tdd_ext_pin.value = True
        tdd_mcu_pin.value = True
        pa_bypass_pin.value = False # diff
        sw1_v1_pin.value = True
        sw1_v2_pin.value = False

        while True:
            pass

    elif test_mode == 5:
        print("Test 4 - Step attenuator at different levels")
        print("Check SMA connectors!")

        tdd_ext_pin.value = True
        tdd_mcu_pin.value = True
        lna_bypass_pin.value = True
        sw2_vctl_pin.value = True

        while True:
            cur = 0.0
            step = 0.25

            # Loop from 0 to max in 0.25 dB steps
            while cur <= 31.75:
                print(f"Setting attenuation: {cur:.2f} dB")
                set_rx_atten(RF_CH_A, cur)
                time.sleep(0.1)
                cur += 0.25
    else:
        print("not sure what to do here...")
        time.sleep(1)
