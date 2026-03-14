"""
Fallback Behavior Script
This script simulates fallback logic for the GPS transmission system.
It checks GPS fix quality and telemetry status and determines
the appropriate system mode.
"""

def fallback_behavior(gps_fix_type, telemetry_ok):
    
    # RTK Fixed (Best accuracy)
    if gps_fix_type == 5:
        print("Mode: RTK FIXED - Normal Operation")

    # RTK degraded but GPS still available
    elif gps_fix_type > 0 and gps_fix_type < 5:
        print("Mode: RTK DEGRADED - Switching to Degraded GPS Mode")

    # GPS signal lost
    elif gps_fix_type == 0:
        print("Mode: GPS LOST - Entering Emergency Mode")

    # Telemetry lost (independent check)
    if not telemetry_ok:
        print("Mode: TELEMETRY LOST - Switching to Autonomous Mode")


def test_fallback():
    print("---- Test Case 1: RTK Fix ----")
    fallback_behavior(5, True)

    print("\n---- Test Case 2: RTK Degraded ----")
    fallback_behavior(3, True)

    print("\n---- Test Case 3: GPS Lost ----")
    fallback_behavior(0, True)

    print("\n---- Test Case 4: Telemetry Lost ----")
    fallback_behavior(5, False)


if __name__ == "__main__":
    test_fallback()
