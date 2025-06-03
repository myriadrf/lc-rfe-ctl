# LibreCellular RFE-Mini test scripts - 202503015
# See: https://github.com/myriadrf/lc-rfe-ctl

import board
import digitalio
import busio
import time

# Setup SPI and control pins
spi = busio.SPI(clock=board.GP26, MOSI=board.GP27)
le_pin = digitalio.DigitalInOut(board.GP28)
le_pin.direction = digitalio.Direction.OUTPUT
le_pin.value = True  # LE is active low, so start high

# Channel definitions to match RF_CH_A and RF_CH_B in C code
RF_CH_A = 0
RF_CH_B = 1

def reverse_bits(byte_value):
    """Reverse the bits in a byte to convert between MSB and LSB first."""
    result = 0
    for i in range(8):
        # Shift result left and add the least significant bit of byte_value
        result = (result << 1) | (byte_value & 1)
        byte_value >>= 1
    return result

def set_rx_attenuation(channel, atten_value):
    """
    Set the RF attenuation value for the specified channel.

    Args:
        channel (int): Channel enum (RF_CH_A or RF_CH_B)
        atten_value (float): Attenuation value in dB
    """
    step = 0.25

    # Max attenuation value is 31.75, clip there
    if atten_value > 31.75:
        atten_value = 31.75

    # Prep attenuation word
    atten_word = int(atten_value / step)  # F1956 has fixed 0.25 attenuation steps
    atten_word &= 0b01111111  # Make sure D7 is always 0

    # Prep address word
    # atten a has A0 connected to GND
    # atten b has A0 connected to VCC
    if channel == RF_CH_A:
        addr_word = 0
    elif channel == RF_CH_B:
        addr_word = 1

    # F1956 wants data LSB first...
    atten_word_reversed = reverse_bits(atten_word)
    addr_word_reversed = reverse_bits(addr_word)

    # Begin transfer
    le_pin.value = False  # ensure LE low to begin transfer
    time.sleep(0.01)     # 1ms delay

    # Write 2 bytes
    tx = bytearray([atten_word_reversed, addr_word_reversed])
    #tx = bytearray([0x80, 0])
    spi.try_lock()
    spi.configure(baudrate=100000, phase=0, polarity=0)
    spi.write(tx)
    spi.unlock()

    time.sleep(0.001)  # 0.1ms delay

    # Toggle LE pin
    le_pin.value = True  # LE high to latch data
    #pg 8: It is recommended that Latch enable be left high
    #when the device is not being programmed.


def test_attenuator(channel):
    """
    Test function for the F1956 attenuator.
    Increments attenuation from 0 to maximum in 0.25 dB steps,
    with a 0.3 second delay between each step.

    Args:
        channel: RF channel to test (RF_CH_A or RF_CH_B)
    """
    # Starting attenuation
    current_attenuation = 0.0

    # Maximum attenuation (31.75 dB)
    max_attenuation = 31.75

    # Step size for F1956
    step_size = 0.25

    print(f"Starting attenuation test on channel {'A' if channel == RF_CH_A else 'B'}")
    print(f"Incrementing from 0 dB to {max_attenuation} dB in {step_size} dB steps")
    print(f"Delay between steps: 0.3 seconds")

    # Loop from 0 to max in 0.25 dB steps
    while current_attenuation <= max_attenuation:
        print(f"Setting attenuation: {current_attenuation:.2f} dB")

        # Set the attenuation using our function
        set_rx_attenuation(channel, current_attenuation)

        # Wait 0.3 seconds before next step
        time.sleep(0.1)

        # Increment by 0.25 dB
        current_attenuation += step_size

    print(f"Test complete. Final attenuation: {max_attenuation} dB")

    # Return to 0 dB after test
    print("Returning to 0 dB")
    set_rx_attenuation(channel, 0)


while True:
    print("set atten")
    set_rx_attenuation(RF_CH_A, 31.75)
    time.sleep(2)
    #test_attenuator(RF_CH_A)




