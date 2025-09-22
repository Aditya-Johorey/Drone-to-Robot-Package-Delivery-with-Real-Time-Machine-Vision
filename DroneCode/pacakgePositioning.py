from djitellopy import Tello
import time
from Modules.FlightDemo import FlightModule
from Modules.Position import Position
from Modules.Camera import Streaming
import threading

class PackageCoordinates:
    def __init__(self, x, y):
        self.x = x
        self.y = y

package_coords = PackageCoordinates(0,0)

# Initiate tello
tello = Tello()
tello.connect() # Connect tello via wifi

# Initialise the Position Manager
positionManager = Position(package_coords)

# Initialise the Flight controller and pass on the Position Manager
drone = FlightModule(tello, positionManager)

# Initialise the Camera Module and pass on the tello and Flight Controller
vidStream = Streaming(tello, drone)

print(f"Battery: {tello.get_battery()}%")

def main() -> PackageCoordinates:
    # Start the video stream loop, passing frames to visualizer
    vidStream.start()  # Don't touch this line pls :)
    streamThread = threading.Thread(target=vidStream.viewStream)
    streamThread.start()

    # Start flight logic in a background thread
    flight_thread = threading.Thread(target=drone.traversal)
    flight_thread.start()

    time.sleep(5)

    # After plot is closed, wait for drone thread to finish
    flight_thread.join()
    # plot_thread.join()
    streamThread.join()
    vidStream.stop()
    tello.end()

    return package_coords
    

def testing():
    drone.takeoff()

    # Any traversal algorithm
    drone.forward(distance=200)

    drone.land()

if __name__ == "__main__":
    main()