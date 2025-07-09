from smbus2 import SMBus

class I2C:
    def __init__(self, bus_number=1, address=0x29):
        self.bus = SMBus(bus_number)
        self.address = address

    def read_byte(self, register):
        return self.bus.read_byte_data(self.address, register)

    def write_byte(self, register, value):
        self.bus.write_byte_data(self.address, register, value)
    
    def write_block_data(self, register, datas):
        self.bus.write_i2c_block_data(self.address, register, datas)
    
    def read_block_data(self, register, length):
        return self.bus.read_i2c_block_data(self.address, register, length)

    def close(self):
        self.bus.close()

    def __del__(self):
        self.close()