#!/usr/bin/env python3

import rospy
from tf2_msgs.msg import TFMessage
import time
import math
import threading
from cflib.utils import uri_helper
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
import cflib.crtp

class CrazyflieController:
    def __init__(self, uri, crazyflie_frame_id, lander_frame_id):
        self.uri = uri
        self.crazyflie_frame_id = crazyflie_frame_id
        self.lander_frame_id = lander_frame_id
        self.drone_x = 0
        self.drone_y = 0
        self.drone_z = 0
        self.lander_x = 0
        self.lander_y = 0
        self.lander_z = 0
        self.lock = threading.Lock()

    def crazyflie(self):
        with SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            with MotionCommander(scf) as mc:
                # Drone pose 
                print ('drone_init_pose_x:', self.drone_x)
                print ('drone_init_pose_y:', self.drone_y) 
                print ('drone_init_pose_z:', self.drone_z)
                drone_abs_init_pose = math.sqrt((0 - self.drone_x)**2 + (0 - self.drone_y)**2 + (0 - self.drone_z)**2)
                print ('absolute_drone_pose:', drone_abs_init_pose)

                #Lander pose
                print ('lander_pose_x:', self.lander_x)
                print ('lander_pose_y:', self.lander_y) 
                print ('lander_pose_z:', self.lander_z)
                lander_abs_init_pose = math.sqrt((0 - self.lander_x)**2 + (0 - self.lander_y)**2 + (0 - self.lander_z)**2)
                print ('absolute_lander_pose:', lander_abs_init_pose)

                time.sleep(1)

                # Calculate the desired position (20 cm above the lander pose)
                with self.lock:
                    desired_x = self.lander_x
                    desired_y = self.lander_y
                    desired_z = self.lander_z + 0.2  # 20 cm above the lander pose

                # Move above the lander by moving up
                mc.up(0.2, velocity=0.1)
                time.sleep(2)

                # Move horizontally to the desired position
                mc.move_distance(self.drone_x - desired_x, self.drone_y - desired_y, 0, velocity=0.1)

                print('del_x', (self.drone_x - desired_x))
                print('del_y', (self.drone_y - desired_y))
                print('del_z', (self.drone_z - desired_z))

                # Hover for 1.2 seconds
                time.sleep(2)

                # Land the Crazyflie
                mc.land()
                pose_error_x = self.drone_x - desired_x
                pose_error_y = self.drone_y - desired_y
                pose_error_z = self.drone_z - desired_z
                pose_error = math.sqrt(pose_error_x**2 + pose_error_y**2 + pose_error_z**2)

                print ('landing_pose_error_x:', pose_error_x)
                print ('landing_pose_error_y:', pose_error_y)
                print ('landing_pose_error_z:', pose_error_z)
                print ('landing_pose_error:', pose_error)
                

    def optitrack_callback(self, msg):
        for transform in msg.transforms:
            frame_id = transform.child_frame_id
            if frame_id == self.crazyflie_frame_id:
                with self.lock:
                    self.drone_x = transform.transform.translation.x
                    self.drone_y = transform.transform.translation.y
                    self.drone_z = transform.transform.translation.z
            elif frame_id == self.lander_frame_id:
                with self.lock:
                    self.lander_x = transform.transform.translation.x
                    self.lander_y = transform.transform.translation.y
                    self.lander_z = transform.transform.translation.z

if __name__ == '__main__':
    rospy.init_node('crazyflie_tracker')

    cflib.crtp.init_drivers()

    # Add your Crazyflie URI
    uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

    controller = CrazyflieController(uri, "Crazyflie1", "Lander2")

    t1 = threading.Thread(target=controller.crazyflie)
    position_topic = '/tf'
    pose_subscriber = rospy.Subscriber(position_topic, TFMessage, controller.optitrack_callback)

    t1.start()

    rospy.spin()
