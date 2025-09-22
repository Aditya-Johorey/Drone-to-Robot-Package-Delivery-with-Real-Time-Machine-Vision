import time
from typing import Callable, Tuple

class DroneCoords:
    def __init__(self):
        self.x: int = 0
        self.y: int = 0
        self.z: int = 0

class PackageCoords:
    x: int = 140
    y: int = 270

def run_drone_mission(update_drone_coords: Callable[[int,int,int], None]) -> Tuple[int,int]:
    drone_coords = DroneCoords()

    # simulate drone flying up
    for z in range (0, 301):
        if z%20 == 0:
            time.sleep(0.1)
            drone_coords.z = z
            update_drone_coords(drone_coords.x, drone_coords.y, drone_coords.z)

    # simulate drone flying in a simplified (impossible) flying pattern
    step_count = 0
    is_package_located = False
    for y in range(0, PackageCoords.y + 10):
        for x in range(0, PackageCoords.x + 10):
            if step_count%500 == 0:
                drone_coords.x = x
                drone_coords.y = y
                update_drone_coords(drone_coords.x, drone_coords.y, drone_coords.z)
                time.sleep(0.05)
            if x == PackageCoords.x and y == PackageCoords.y:
                is_package_located = True
                break
            step_count += 1
        if is_package_located == True:
            break

    return (PackageCoords.x, PackageCoords.y)
