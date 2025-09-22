# The code in this file is from this YouTube video - https://www.youtube.com/watch?v=JOZ9XoFDEYE

import cv2
from djitellopy import tello

def run_bottom_video(drone):
    while True:
        # get the most recent frame
        frame = drone.get_frame_read().frame
        # crop the frame to only show the visible image
        crop_img = frame[0:240, 0:320]
        # show the cropped image
        cv2.imshow("Frame", crop_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # cleanup and end the program
    cv2.destroyAllWindows()
    print("Ending the Tello object...")
    drone.end()
    print("...Tello object ended. Terminating program.")

def main():
    # initialise drone object
    drone = tello.Tello()
    # establish connection with the drone
    drone.connect()

    # set the drone's camera direction
    drone.set_video_direction(drone.CAMERA_DOWNWARD)

    # start the drone's camera stream
    drone.streamon()

    # call the function to stream video from the drone's bottom camera
    run_bottom_video(drone)

if __name__ == "__main__":
    main()
