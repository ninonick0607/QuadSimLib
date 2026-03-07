#!/usr/bin/env python3
"""
QuadSim SDK — End-to-End Flight Demo

Demonstrates the full high-level API:
  connect → takeoff → fly_to → hover → fly_path → yaw_to → land → disconnect

Run with the QuadSim Unity scene playing:
  python examples/flight_demo.py

Requirements:
  pip install quadsim   (or pip install -e . from the repo root)
"""

import time

from quadsim import QuadSimApi


def main():
    # ── Connect ──────────────────────────────────────────────────
    api = QuadSimApi()
    api.connect()
    print(f"Connected. Status: {api.get_status()}")
    print(f"Starting position: {api.get_position()}")

    try:
        # ── Takeoff ──────────────────────────────────────────────
        print("\n--- Takeoff to 3m ---")
        api.takeoff(altitude=3.0, speed=1.0)
        print(f"Airborne at: {api.get_position()}")
        print(f"Is airborne: {api.is_airborne()}")

        # ── Fly to a waypoint ────────────────────────────────────
        print("\n--- Fly to (5, 3, 0) ---")
        api.fly_to(x=5.0, y=3.0, z=0.0, speed=2.0)
        print(f"Arrived at: {api.get_position()}")

        # ── Hover for 3 seconds ─────────────────────────────────
        print("\n--- Hover for 3s ---")
        api.hover(duration=3.0)
        print(f"After hover: {api.get_position()}")

        # ── Fly a path (square pattern) ──────────────────────────
        print("\n--- Fly square path ---")
        waypoints = [
            (5.0, 3.0, 5.0),
            (0.0, 3.0, 5.0),
            (0.0, 3.0, 0.0),
            (5.0, 3.0, 0.0),
        ]
        api.fly_path(waypoints, speed=2.0)
        print(f"Path complete at: {api.get_position()}")

        # ── Yaw to heading ───────────────────────────────────────
        print("\n--- Yaw to 90° ---")
        api.yaw_to(heading_deg=90.0)
        print(f"Heading: {api.get_attitude()}")

        # ── Land ─────────────────────────────────────────────────
        print("\n--- Landing ---")
        api.land(speed=0.5)
        print(f"Landed at: {api.get_position()}")
        print(f"Is airborne: {api.is_airborne()}")

    except Exception as e:
        print(f"\nError during flight: {e}")
        # Best-effort emergency land
        try:
            api.land(speed=1.0, timeout=10.0)
        except Exception:
            pass

    finally:
        api.disconnect()
        print("\nDisconnected.")


if __name__ == "__main__":
    main()