import math

class carDynamic():

    def __init__(self, CONSTANT_DICT):
        self.PINION_RADIUS = CONSTANT_DICT["PINION_RADIUS"]
        self.MAX_VEL_RPM = CONSTANT_DICT["MAX_VEL_RPM"]
        self.CAR_WIDTH = CONSTANT_DICT["CAR_WIDTH"]
        self.COEFFICIENT_FRICTION = CONSTANT_DICT["COEFFICIENT_FRICTION"]
        self.GRAVITY = CONSTANT_DICT["GRAVITY"]
        self.WHEEL_RADIUS = CONSTANT_DICT["WHEEL_RADIUS"]
        self.CAR_LENGTH = CONSTANT_DICT["CAR_LENGTH"]
        self.references = {
            "rightWheelVel" : 0,
            "leftWheelVel" : 0,
            "pinionPosition" : 0
        }

        self.insideAngleArray = [40.61,31.16,30.71,29.57,27.96,26.36,25.39,24.76,23.16,21.56,19.3,19.02,17.485,15.94,14.4,12.87,12.04,11.95,10.35,8.75,8.04,7.15,5.55,3.95,3.49,2.35,0.75,0]
        self.motorPositionArray = [24.97,20,19.93,19,18,17,16.89,16,15,14,13.18,13,12,11,10,9,8.46,8,7,6,5.74,5,4,3,2.52,2,1,0]

    def rackPos2angle(self, position):
        return (180*position)/(math.pi*self.PINION_RADIUS)

    def radPerSec2revPerMin(self, vel):
        return (vel*60)/(2*math.pi)

    def updateRackPosition(self, isTurnRight, angle):
        position = -1
        angleFinder = round(angle,1)
        varianceAllow = 0.01

        while -1 == position:
            try:
                index = self.insideAngleArray.index(angleFinder)

                position = self.motorPositionArray[index]
                position = self.rackPos2angle(position)
            except:
                angleFinder += varianceAllow
                angleFinder = round(angleFinder,2)

        if isTurnRight:
            position += 100

        self.references.update({
            "pinionPosition" : round(position)
        })

    def updateDistances(self, isTurnRight, angle):

        angleRadians = math.radians(angle)

        if isTurnRight:
            self.rightWheelDistance = self.CAR_LENGTH/math.tan(angleRadians)
            self.leftWheelDistance = self.rightWheelDistance + self.CAR_WIDTH
            firstCateto = self.rightWheelDistance + (self.CAR_WIDTH / 2)
        else:
            self.leftWheelDistance = self.CAR_LENGTH / math.tan(angleRadians)
            self.rightWheelDistance = self.leftWheelDistance + self.CAR_WIDTH
            firstCateto = self.leftWheelDistance + (self.CAR_WIDTH / 2)

        secondCateto = self.CAR_LENGTH/2
        self.carCenterDistance = math.sqrt(math.pow(firstCateto,2) + math.pow(secondCateto,2))

    def updateMaxVelocity(self):
        maxLinearVelocity = math.sqrt(self.carCenterDistance*self.GRAVITY*self.COEFFICIENT_FRICTION)
        self.maxAngularVelocity = maxLinearVelocity/self.carCenterDistance

    def updateMotorVelocities(self):
        rightLinearVel = self.maxAngularVelocity*self.rightWheelDistance
        leftLinearVel = self.maxAngularVelocity*self.leftWheelDistance

        rightAngularVel = self.radPerSec2revPerMin(rightLinearVel/self.WHEEL_RADIUS)
        leftAngularVel = self.radPerSec2revPerMin(leftLinearVel/self.WHEEL_RADIUS)

        if rightAngularVel > self.MAX_VEL_RPM:
            rightAngularVel = self.MAX_VEL_RPM

        if leftAngularVel > self.MAX_VEL_RPM:
            leftAngularVel = self.MAX_VEL_RPM

        self.references.update({
            "leftWheelVel" : round(leftAngularVel),
            "rightWheelVel" : round(rightAngularVel)
        })

    def getReferences(self):
        return self.references