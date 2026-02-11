from pymavlink import mavutil

def wakeup():
    """
    Establishes a MAVLink connection to the Pixhawk 6X flight controller via USB, sends an initial heartbeat to 
    wake the autopilot, waits for confirmation, requests telemetry and data streams and finally returns the active
    MAVLink connection 
    """

    # Creating a MAVLink serial connection over USB
    # Pixhawk typically appears as /dev/ttyACM0 or /dev/ttyaACM1
    the_connection = mavutil.mavlink_connection('/dev/ttyACM0')
    
    #Sending an initial hearrtbeat message, prompting PX4 to initialize MAVLink connection 
    the_connection.mav.heartbeat_send(
        0, #type
        0, #autopilot
        0, #base_mode
        0, #custom_mode
        0, #system_status
        0, #mavlink_version
    )

    # Check the heartbeat to confirm that connection is active and working 
    the_connection.wait_heartbeat()
    print(f"Heartbeat from target_system: {the_connection.target_system}, MAV_TYPE: {the_connection.mav_type}")

    # Request telemetry data streams from the Pixhawk, streams via USB must be explicitly requested 
    the_connection.mav.request_data_stream_send(
            the_connection.target_system,
            the_connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,  # Request all available data streams
            10,
            1
    )

    return the_connection



if __name__ == "__main__":
    wakeup()
