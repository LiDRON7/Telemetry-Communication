import time
from wakeup import wakeup

# Wake up Pixhawk and request streams
the_connection = wakeup()

try:
    while True:
        # Get attitude
        attitude = the_connection.recv_match(type='ATTITUDE', blocking=False)
        if attitude:
            print(f"Attitude -> Roll: {attitude.roll:.2f}, Pitch: {attitude.pitch:.2f}, Yaw: {attitude.yaw:.2f}")
            print(f"Angular rates -> p: {attitude.rollspeed:.2f}, q: {attitude.pitchspeed:.2f}, r: {attitude.yawspeed:.2f}\n")

        time.sleep(0.05)  # slow down printing a bit
except KeyboardInterrupt:
    print("Stopped by user")
