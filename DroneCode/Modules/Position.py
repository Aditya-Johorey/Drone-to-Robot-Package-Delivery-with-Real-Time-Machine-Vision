import numpy as np
from enum import Enum

class Direction(Enum):
    LEFT = 0
    FORWARD = 1
    RIGHT = 2
    BACKWARD = 3

class Rotation(Enum):
    CW = 1
    NO_ROTATION = 0
    CCW = -1

class Position:
    def __init__(self, package_coords):

        self.position = np.zeros((2,1))
        self.groundRobotToDrone = np.zeros((2, 1))
        self.droneToPackage = np.zeros((2, 1))
        self.distanceMoved = 0.0
        self.direction= Direction.FORWARD
        self.rotation = None
        self.boxNumber = 0
        self.package_coords = package_coords


    def parcelToRobot(self, droneToPackage):

        # Position of the robot wrt the drone
        droneToRobot = np.array([[100], [0]])

        # Take in account for the ground robot being 100cm to the right of the initial position:
        # This means we need to subtract 1m from the X axis of the drone's position first

        self.groundRobotToDrone = self.position - droneToRobot
        # Modified the package coordinates to match the world frame

        temp = droneToPackage[1, 0]
        droneToPackage[1, 0] = -droneToPackage[0, 0]
        droneToPackage[0, 0] = -temp

        # Calculating the package coordinates in the world frame

        # print("Coords:",self.groundRobotToDrone, droneToPackage)

        packageCoordinates = self.groundRobotToDrone.flatten() + droneToPackage.flatten()
        print(f"Coordinates of the pacakge: {packageCoordinates}")

        # TODO: Return the coordinates
        # return self.position + droneToPackage
        self.package_coords.x, self.package_coords.y = list(packageCoordinates)

    def changeDirection(self, cwORccw):
        if cwORccw:
            if self.direction is Direction.BACKWARD:
                self.direction = Direction.LEFT
            else:
                self.direction = Direction(self.direction.value + 1)
        else:
            if self.direction is Direction.LEFT:
                self.direction = Direction.BACKWARD
            else:
                self.direction = Direction(self.direction.value - 1)

        print(f"The Drone is looking in the {self.direction.name} direction")

    def readDistance(self, distance = 0):
        self.distanceMoved = distance
        self.calculate()

    def calculate(self):

        # Based on the  Direction the position of the Drone is being updated in the global frame
        if self.direction == Direction.LEFT:
            self.position[0, 0] -= self.distanceMoved
        elif self.direction == Direction.RIGHT:
            self.position[0, 0] += self.distanceMoved
        elif self.direction == Direction.FORWARD:
            self.position[1, 0] += self.distanceMoved
        elif self.direction == Direction.BACKWARD:
            self.position[1, 0] -= self.distanceMoved

        print(f"Drone position: {self.position[0, 0]}, {self.position[1, 0]}")