import numpy as np
import time
import threading

class FlightModule:
    def __init__(self, flight, positionManager):
        self.tello = flight
        print("Drone Ready to take off")
        self.parcelDetected = False
        self.parcelPostion = np.zeros((2, 1))
        self.positionManager = positionManager


    #Basic flight commands
    def takeoff(self):
        self.tello.takeoff()
        print("Drone has Taken Off")

    def land(self):
        self.tello.land()
        print("Drone has Landed")

    def stop(self):
        self.tello.send_rc_control(0, 0, 0, 0)
        print("Drone has stopped")

    # Vertical and Horizontal Movements
    def forward(self, distance):

        if not self.parcelDetected:
            print(f"Drone is moving forward by {distance} cm")
            self.tello.move_forward(distance)
            self.positionManager.readDistance(distance = distance)
        else:
            self.stop()

    def backwards(self, distance):
        if not self.parcelDetected:
            print(f"Drone is moving backward by {distance} cm")
            self.tello.move_back(distance)
            self.positionManager.readDistance(distance=distance)
        else:
            self.stop()

    def left(self, distance):
        if not self.parcelDetected:
            print(f"Drone is moving left by {distance} cm")
            self.tello.move_left(distance)
            self.positionManager.readDistance(distance= distance)
        else:
            self.stop()

    def right(self, distance):
        if not self.parcelDetected:
            print(f"Drone is moving right by {distance} cm")
            self.tello.move_right(distance)
            self.positionManager.readDistance(distance= distance)
        else:
            self.stop()

    def up(self, distance):
        self.tello.move_up(distance)
        print(f"Drone is moving up by {distance} cm")

    def down(self, distance):
        self.tello.move_down(distance)
        print(f"Drone is moving down by {distance} cm")

    # Rotation functions
    def rotate(self, angle):
        if angle > 0:
            self.tello.rotate_clockwise(angle)
            print(f'Drone is rotating clockwise by {angle}°')
            self.positionManager.changeDirection(True)
        else:
            self.tello.rotate_counter_clockwise(-angle)
            print(f'Drone is rotating counter-clockwise by {-angle}°')
            self.positionManager.changeDirection(False)

    # Analytics Function
    def getIMUData(self):
        return {
            "roll": self.tello.get_roll(),
            "pitch":self.tello.get_pitch(),
            "yaw":self.tello.get_yaw(),
            "height": self.tello.get_distance_tof(),
            'acceleration':{
                'x': self.tello.get_acceleration_x(),
                'y':self.tello.get_acceleration_y(),
                'z':self.tello.get_acceleration_z()
            }
        }

    # Set Speed
    def speed(self, speed):
        self.tello.set_speed(speed)
        print(f'Drone speed is set to {speed} cm/s')

    def monitor(self):
        try:
            while not self.parcelDetected:
                time.sleep(0.05)
            print("Parcel detected in monitor thread!")
            raise RuntimeError("Parcel detected, stopping movement")
        except RuntimeError as e:
            print("Traversal Stopped:", e)
            self.stop()
            self.positionManager.parcelToRobot(self.parcelPostion)
        return

    def flightPrep(self):

        """
        Goal of this section is to prepare for the flight.
        Any preliminary operations to be done here.

        :return: Nothing

        """
        self.speed(speed = 30)


    def phase1(self):
        """
        Goal of this phase is to reach near the aruco tags and align itself to the arucos

        :return: Nothing
        """
        print("Entering: PHASE 1")
        # Step 1: Take off
        self.takeoff()

        # Step 2: Rise to 1.5 m
        self.up(distance= 150 - 80)

        # step 2.1: Reboot the IMU to refresh its
        self.stop()  # Don't touch this line at any cost

        # Step 3: Move forward
        self.forward(distance=280) # 280cm instead of 300 because the drone tends to drift to the left.

        # Step 4: Rotate clockwise 90 deg
        self.rotate(angle=90)
        print("Ending: PHASE 1")

    def phase2(self):
        # TODO: Make any modifications here if necessary
        """
        Goal of this phase is to search the package and make precise position estimation of it

        :return: Nothing
        """

        print("Entering: PHASE2: ")

        # Move forward
        try:
            monitor_thread = threading.Thread(target=self.monitor)
            monitor_thread.start()

            # Splitting the 500cm journey into small 100 cm journeys to maintain accuracy
            distanceToTravel = 500
            intervalsForSearching = 50
            for x in range(0, distanceToTravel, intervalsForSearching):
                self.forward(distance = intervalsForSearching)
                if self.parcelDetected: break
            monitor_thread.join()

        except RuntimeError as e:
            print("Traversal Stopped:", e)
            print("Parcel coordinates sent to the ground robot")
        else:
            print("Traversal Complete")
            return
        finally:
            print("Ending: PHASE 2")
            return

    def phase3(self):

        """
        Goal of this phase is to clean up and go back to the initial position
        :return: Nothing
        """

        print("Entering: PHASE 3")

        # Step 1: Rotate the drone 180 deg
        self.rotate(angle = 90)
        self.rotate(angle = 90)
        print("Going back to origin")

        # Step 2: Get back to origin
        if self.positionManager.position[0, 0] < 400:
            self.tello.go_xyz_speed(x=int(self.positionManager.position[0,0]),
                                    y=int(self.positionManager.position[1, 0]),
                                    z=-25,
                                    speed=30)
        else:
            self.tello.go_xyz_speed(x=int(self.positionManager.position[0,0]//2),
                        y=int(self.positionManager.position[1, 0]//2),
                        z=-25,
                        speed=30)
            self.tello.go_xyz_speed(x=int(self.positionManager.position[0,0]//2),
                        y=int(self.positionManager.position[1, 0]//2),
                        z=-25,
                        speed=30)


        # Step 3: Land
        self.land()
        print("Ending: PHASE 3")
        return
    def traversal(self):

        # Prepare for flight
        self.flightPrep()

        # Phase 1
        self.phase1()

        # Phase 2
        self.phase2()

        # Phase 3: # TODO: Modify phase 3 after you have set up the aruco detection
        self.phase3()

    def Lawnmower(self, x, y, speed=50):

        self.speed(speed)

        left_xdim = x - 40
        turning = 1
        try:
            monitor_thread = threading.Thread(target=self.monitor)
            monitor_thread.start()

            while left_xdim > 0:
                self.forward(y)
                self.rotate(turning * 90)
                self.forward(30)
                self.rotate(turning * 90)
                turning *= -1
                left_xdim -= 30

            monitor_thread.join()
        except RuntimeError as e:
            print("Traversal Stopped:", e)
            self.land()
        else:
            print("Traversal Complete")



