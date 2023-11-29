#!/usr/bin/env python3

import rospy
import math
import time
import threading
import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

# The URI for the Crazyflie to connect to
URI = 'radio://0/80/2M/E7E7E7E7E7'

def run_sequence(cf_uri):
    # Wait for the ROS node to fully initialize
    time.sleep(1)

    try:
        with SyncCrazyflie(cf_uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            # We take off when the commander is created
            with MotionCommander(scf) as mc:
                time.sleep(1)  # Short delay before takeoff

                # Check if the drone is already flying
                # if mc._is_flying:
                #     rospy.logwarn("Drone is already flying! Attempting to land.")
                #     mc.land()
                #     time.sleep(1)  # Give some time to land

                # rospy.loginfo("Taking off!")
                # mc.take_off(1.0, 0.5)
                # time.sleep(3)  # Wait for the drone to stabilize

                rospy.loginfo("Flying in a circle.")
                radius = 0.5  # meters
                velocity = 0.5  # meters/second

                # Calculate the circumference of the circle
                circumference = 2 * math.pi * radius

                # Calculate the time to complete one full circle
                time_to_complete_circle = circumference / velocity

                # Flying in a circle to the right
                mc.circle_right(radius, velocity=velocity)
                
                # Wait for the time it takes to complete the circle
                time.sleep(time_to_complete_circle)

                # Stop any further motion (if required)
                mc.stop()

                # Land and end the program
                rospy.loginfo("Landing!")
                mc.land()

    except Exception as e:
        rospy.logerr("An error occurred: %s", e)
    finally:
        rospy.loginfo("Sequence completed or interrupted.")

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('crazyflie_takeoff_circle_land', anonymous=False)

    # Initialize the low-level drivers (only once)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    run_sequence_thread = threading.Thread(target=run_sequence, args=(URI,))
    run_sequence_thread.start()

    # Keep the node alive while the threads run
    rospy.spin()
