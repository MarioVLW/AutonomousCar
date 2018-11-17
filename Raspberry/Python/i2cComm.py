import smbus

class i2cCommunication:

    def __init__(self, addressRightWheel, addressLeftWheel, addressRack, bus):
        self.bus = smbus.SMBus(bus)
        self.addressRightWheel = addressRightWheel
        self.addressLeftWheel = addressLeftWheel
        self.addressRack = addressRack

    def sendReferences(self, references):
        self.bus.write_byte_data(self.addressLeftWheel,0,references["leftWheelVel"])
        self.bus.write_byte_data(self.addressRightWheel,0,references["rightWheelVel"])
        self.bus.write_byte_data(self.addressRack,0,references["pinionPosition"])

