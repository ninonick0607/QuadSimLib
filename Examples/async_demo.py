#!/usr/bin/env python3
"""
QuadSim SDK — Async Flight Demo

Demonstrates non-blocking commands with Future polling and cancellation.

Run with the QuadSim Unity scene playing:
  python examples/async_demo.py
"""

import time

from quadsim import QuadSimApi


def main():
    with QuadSimApi() as api:
        print(f"Connected. Position: {api.get_position()}")

        # ── Blocking takeoff first ──────────────────────────────
        api.takeoff(altitude=3.0)
        print(f"Airborne at: {api.get_position()}")

        # ── Async fly_to with polling ────────────────────────────
        print("\n--- Async fly_to (10, 3, 5) with position polling ---")
        future = api.fly_to_async(x=10.0, y=3.0, z=5.0, speed=2.0)

        while not future.done:
            pos = api.get_position()
            print(f"  En route: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
            time.sleep(0.5)

        if future.exception:
            print(f"  Flight failed: {future.exception}")
        else:
            print(f"  Arrived at: {api.get_position()}")

        # ── Async fly_to with cancellation ───────────────────────
        print("\n--- Async fly_to with early cancellation ---")
        future = api.fly_to_async(x=0.0, y=3.0, z=0.0, speed=1.0)

        # Let it fly for 2 seconds, then cancel
        time.sleep(2.0)
        print(f"  Cancelling at: {api.get_position()}")
        future.cancel()
        future.wait()  # Wait for the thread to actually exit
        print(f"  Holding at: {api.get_position()}")

        # ── Async with .wait() ───────────────────────────────────
        print("\n--- Async fly_to with wait() ---")
        future = api.fly_to_async(x=0.0, y=3.0, z=0.0, speed=2.0)
        future.wait(timeout=30.0)
        print(f"  Arrived at: {api.get_position()}")

        # ── Interruption: new command cancels old ────────────────
        print("\n--- Command interruption demo ---")
        future1 = api.fly_to_async(x=20.0, y=3.0, z=20.0, speed=1.0)
        time.sleep(1.0)
        print(f"  Mid-flight: {api.get_position()}")

        # Starting a new command automatically cancels the old one
        future2 = api.fly_to_async(x=0.0, y=3.0, z=0.0, speed=3.0)
        print(f"  Old future cancelled: {future1.cancelled}")
        future2.wait(timeout=30.0)
        print(f"  Final position: {api.get_position()}")

        # ── Land ─────────────────────────────────────────────────
        api.land()
        print(f"\nLanded at: {api.get_position()}")

    print("Disconnected.")


if __name__ == "__main__":
    main()