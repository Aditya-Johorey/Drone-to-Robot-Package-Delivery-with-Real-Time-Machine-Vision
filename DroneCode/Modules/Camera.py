import time
import threading
import cv2 as cv
from djitellopy import Tello
import numpy as np



class Streaming:
    def __init__(self, tello, flightMod):
        self.tello = tello
        self.frame = None
        self.running = False
        self.lock = threading.Lock()
        self.flightController = flightMod
        self.threadLife = None


        # Load camera calibration data
        # self.data = np.load("C:\\Users\\Aaditya\\PycharmProjects\\drone\\dronenv\\calibration_data2.npz")
        self.data = np.load('/home/robotlab1/src/drone/DroneCode/Modules/calibration_data2.npz')
        self.parcelId = [10]  # ArUco marker ID
        self.parcelDetected = False
        self.droneToAruco = np.zeros((2, 1))

        # self.arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
        self.arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
        self.parameters = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.arucoDict, self.parameters)
        self.parcelAlignment = False

        # Start Tello's video stream
        self.tello.streamon()
        self.frame_reader = self.tello.get_frame_read()

        # Variables
        self.prev_xyz = None  # Will store the smoothed [x, y, z]
        self.mirrorOffsetAngles = {
            'x': -0.31,
            'y':-0.11
        } # in radians

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.update, daemon=True)
        self.thread.start()

    def update(self):
        while self.running:
            frame = self.frame_reader.frame
            if frame is not None:
                with self.lock:
                    self.frame =  cv.rotate(cv.flip(frame, 1), cv.ROTATE_180)
                    # self.frame = frame

    def read(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False
        self.thread.join()
        self.tello.streamoff()
        cv.destroyAllWindows()

    def euler_to_rotation_matrix(self, roll, pitch, yaw):
        # Convert degrees to radians
        roll, pitch, yaw = np.radians([roll, pitch, yaw])

        # Rotation matrices
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        return Rz @ Ry @ Rx

    def alignment(self, frame, marker_corners):

        # Define marker's 3D corner positions in object space
        marker_length = 0.071  # 7.1 cm
        half_len = marker_length / 2
        obj_points = np.array([
            [-half_len, half_len, 0],
            [half_len, half_len, 0],
            [half_len, -half_len, 0],
            [-half_len, -half_len, 0]
        ], dtype=np.float32)

        success, rvec, tvec = cv.solvePnP(
            obj_points,
            marker_corners,
            self.data['K'],
            self.data['dist'],

        )

        if success:
            cv.drawFrameAxes(frame, self.data['K'], self.data['dist'], rvec, tvec, 0.03)

            # --- Get IMU orientation ---
            imu = self.flightController.getIMUData()
            # imu = self.imu_buffer.get_data()
            print(f"IMU: pitch={imu['pitch']}°, roll={imu['roll']}°, yaw={imu['yaw']}°, Height = {imu['height']}cm")
            pitch = imu['pitch']
            roll = imu['roll']
            yaw = imu['yaw']

            # # --- Apply orientation correction ---
            tvec_cam = tvec.reshape(3, 1)  # shape (3,1)


            R_imu = self.euler_to_rotation_matrix(roll, pitch, yaw)
            tvec_drone = R_imu @ tvec_cam

            raw_xyz = (tvec_drone.flatten() * 100).tolist()  # Convert to cm

            alpha = 0.2  # smoothing factor

            if self.prev_xyz is None:
                self.prev_xyz = raw_xyz
                xyz = raw_xyz
            else:
                xyz = [
                    alpha * new + (1 - alpha) * prev
                    for new, prev in zip(raw_xyz, self.prev_xyz)
                ]
                self.prev_xyz = xyz  # update for next frame

            if not self.parcelAlignment:
                print(f"Marker ID: {self.parcelId}")
                print("Translation Vector (tvec):", tvec.flatten())
                print("Vertical distance ", np.linalg.norm(tvec))
                # print("Rotation Vector (rvec):", rvec.flatten())
            print("XYZ: ", xyz[0], xyz[1], xyz[2], xyz)

            # Account for aruco's position wrt drone
            with self.lock:
                self.flightController.parcelPostion[0, 0] = xyz[0]
                self.flightController.parcelPostion[1, 0] = xyz[1]

        return frame

    def detectAruco(self, frame):
        corners, ids, rejected = self.detector.detectMarkers(frame)

        if ids is not None:
            # print("Detected Markers:", ids)
            cv.aruco.drawDetectedMarkers(frame, corners, ids)

            if any(i in ids for i in self.parcelId):

                i = ids.flatten().tolist().index(self.parcelId[0])
                marker_corners = corners[i].reshape(4, 2)  # 4 image points

                # Finding Pacakge's position wrt the drone
                if not self.parcelDetected:
                    frame = self.alignment(
                        frame=frame,
                        marker_corners=marker_corners
                    )

                with self.lock:
                    self.parcelDetected = True
                    print("Parcel has been detected")
                    self.flightController.parcelDetected = True

        return frame, self.parcelDetected

    def viewStream(self):
        try:
            while True:
                frame = self.read()
                if frame is not None:
                    frame = cv.undistort(frame, self.data['K'], self.data["dist"])
                    frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
                    img, detected = self.detectAruco(frame)
                    cv.imshow("Real-Time Tello Feed", img)

                if cv.waitKey(1) & 0xFF == ord('q'):
                    break

                # time.sleep(0.3)
        finally:
            self.stop()

