from color_sensor.connectors.i2c import I2C


class TCS9548A:
    def __init__(self, bus_number, address):
        self.i2c = I2C(bus_number, address)

    def enable_channel(self, channel):
        if channel < 0 or channel > 7:
            raise ValueError("Channel must be between 0 and 7")
        self.i2c.write_byte(0x00, 1 << channel)
