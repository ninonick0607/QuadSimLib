#!/usr/bin/env python3
"""
QuadSim SDK — Flight Demo

  python examples/flight_demo.py
"""

import time
from quadsim import QuadSim


def main():
    with QuadSim() as sim:
        drone = sim.drone()

        print(f"Connected. Position: {drone.get_position()}")

        # Takeoff
        drone.takeoff(altitude=3.0, speed=1.0)


        # Fly to a point
        drone.fly_to(x=5.0, y=0.0, z=3.0, speed=2.0)

        # Hover
        drone.hover(duration=3.0)
        waypoints = [
            (5.0, 5.0, 3.0),
            (0.0, 5.0, 3.0),
            (0.0, 0.0, 3.0),
            (5.0, 0.0, 3.0),
        ]
        drone.fly_path(waypoints, speed=2.0)

        # Yaw
        drone.yaw_to(heading_deg=90.0)

        # Land
        drone.land(speed=0.5)


    print("Disconnected.")


if __name__ == "__main__":
    main()