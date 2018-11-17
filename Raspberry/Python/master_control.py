# Before try sudo i2cdetect -y 1
import RPi.GPIO as GPIO
import carDynamic
import i2cComm
import time

CONSTANT = {
    "COEFFICIENT_FRICTION" : 0.6,  # Coefficient of friction between the wheel and the floor 0.6 to 0.9
    "GRAVITY" : 9.81,              # Gravity value in m/s^2
    "WHEEL_RADIUS" : 0,            # Wheel radius in meters
    "CAR_LENGTH" : 0,              # Car lenght distance in meters
    "CAR_WIDTH" : 0,               # Car width distance in meters
    "MAX_VEL_RPM" : 80,            # Maximum angular velocity in RPM
    "PINION_RADIUS" : 0            # Radius of the pinion in meters
}

# Global variables
angleOffset = 0            # Angle offset from the camera in radians
isTurnRight = True         # Flag to check the direction of the curve

# Use the lower-level numbering system for GPIO
GPIO.setmode(GPIO.BCM)

try:
    # Instance of modules
    dynamic = carDynamic.carDynamic()
    dynamic.__init__(CONSTANT_DICT=CONSTANT)

    i2c = i2cComm.i2cCommunication()
    i2c.__init__(addressRightWheel=0x20, addressLeftWheel=0x23, addressRack=0x40)

    while True:
        dynamic.updateRackPosition(isTurnRight=isTurnRight, angle=angleOffset)
        dynamic.updateDistances(isTurnRight=isTurnRight, angle=angleOffset)
        dynamic.updateMaxVelocity()
        dynamic.updateMotorVelocities()

        i2c.sendReferences(dynamic.getReferences())

except KeyboardInterrupt:
    GPIO.cleanup()