import math

class carDynamic():

    def __init__(self, CONSTANT_DICT):
        self.CAR_WIDTH = CONSTANT_DICT["CAR_WIDTH"]
        self.COEFFICIENT_FRICTION = CONSTANT_DICT["COEFFICIENT_FRICTION"]
        self.GRAVITY = CONSTANT_DICT["GRAVITY"]
        self.WHEEL_RADIUS = CONSTANT_DICT["WHEEL_RADIUS"]
        self.CAR_LENGTH = CONSTANT_DICT["CAR_LENGTH"]
        self.references = {
            "rightWheelVel" : 0,
            "leftWheelVel" : 0,
            "rackPosition" : 0
        }
        self.rightAngleArray = [1,2,3,4]
        self.leftAngleArray = [5,6,7,8]
        self.motorPositionArray = [9,10,11,12]

    def updateRackPosition(self, isTurnRight, angle):
        position = -1
        angleFinder = angle
        varianceAllow = 1

        while -1 == position:
            try:
                if isTurnRight:
                    index = self.rightAngleArray.index(angleFinder)
                else:
                    index = self.leftAngleArray.index(angleFinder)

                position = self.motorPositionArray[index]
            except:
                angleFinder += varianceAllow

        self.references.update({
            "rackPosition" : position
        })

    def updateDistances(self, isTurnRight, angle):

        if isTurnRight:
            self.rightWheelDistance = self.CAR_LENGTH/math.tan(angle)
            self.LeftWheelDistance = self.rightWheelDistance + self.CAR_WIDTH
            firstCateto = self.rightWheelDistance + (self.CAR_WIDTH / 2)
        else:
            self.LeftWheelDistance = self.CAR_LENGTH / math.tan(angle)
            self.rightWheelDistance = self.LeftWheelDistance + self.CAR_WIDTH
            firstCateto = self.LeftWheelDistance + (self.CAR_WIDTH / 2)

        secondCateto = self.CAR_LENGTH/2
        self.carCenterDistance = math.sqrt(math.pow(firstCateto,2) + math.pow(secondCateto,2))

    def updateMaxVelocity(self):
        maxLinearVelocity = math.sqrt(self.carCenterDistance*self.GRAVITY*self.COEFFICIENT_FRICTION)
        self.maxAngularVelocity = maxLinearVelocity/self.carCenterDistance

    def updateMotorVelocities(self):
        rightLinearVel = self.maxAngularVelocity*self.rightWheelDistance
        leftLinearVel = self.maxAngularVelocity*self.LeftWheelDistance

        rightAngularVel = rightLinearVel/self.WHEEL_RADIUS
        leftAngularVel = leftLinearVel/self.WHEEL_RADIUS

        self.references.update({
            "leftWheelVel" : leftAngularVel,
            "rightWheelVel" : rightAngularVel
        })

    def getReferences(self):
        return self.references