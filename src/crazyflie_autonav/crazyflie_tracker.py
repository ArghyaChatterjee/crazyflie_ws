#!/usr/bin/env python3

import rospy
from tf2_msgs.msg import TFMessage

import logging
import sys
import time
import threading
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

drone = "\"Crazyflie1\""
drone_x = 0
drone_y = 0
drone_z = 0
drone_xr = 0
drone_yr = 0
drone_zr = 0
drone_wr = 0
wand = "\"wand\""
wand_x = 0
wand_y = 0
wand_z = 0
wand_xr = 0
wand_yr = 0
wand_zr = 0
wand_wr = 0

def crazyflie():
    global drone_x
    global drone_y
    global drone_z
    global wand_x
    global wand_y
    global wand_z
    window = 0.1
    offset = 1

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with MotionCommander(scf) as mc:
            time.sleep(2)
            while True:
                onTarget = True
                if float(wand_z)-window > float(drone_z):
                    mc.up(0.1)
                    onTarget = False
                elif float(wand_z)+window < float(drone_z):
                    mc.down(0.1)
                    onTarget = False
                if float(wand_y)-offset-window > float(drone_y):
                    mc.forward(0.1)
                    onTarget = False
                elif float(wand_y)-offset+window < float(drone_y):
                    mc.back(0.1)
                    onTarget = False
                if float(wand_x)-window > float(drone_x):
                    mc.right(0.1)
                    onTarget = False
                elif float(wand_x)+window < float(drone_x):
                    mc.left(0.1)
                    onTarget = False
                if onTarget:
                    mc.turn_right(15,15)
                    for i in range(12):
                        mc.left(0.5)
                        time.sleep(1)
                        mc.turn_right(30,30)
                        time.sleep(1)
                    mc.turn_left(15,15)
                    time.sleep(1)
                    mc.back(2)
                    mc.land()



def optitrack(msg):
    global drone
    global drone_x
    global drone_y
    global drone_z
    global drone_xr
    global drone_yr
    global drone_zr
    global drone_wr
    global wand
    global wand_x
    global wand_y
    global wand_z
    global wand_xr
    global wand_yr
    global wand_zr
    global wand_wr

    crazy1 = msg.transforms.pop().__str__().replace(" ","").split("\n")
    title = crazy1[6].split(":")[1]
    if title == drone:
        drone_x = crazy1[9].split(":")[1]
        drone_y = crazy1[10].split(":")[1]
        drone_z = crazy1[11].split(":")[1]
        drone_xr = crazy1[13].split(":")[1]
        drone_yr = crazy1[14].split(":")[1]
        drone_zr = crazy1[15].split(":")[1]
        drone_wr = crazy1[16].split(":")[1]
    elif title == wand:
        wand_x = crazy1[9].split(":")[1]
        wand_y = crazy1[10].split(":")[1]
        wand_z = crazy1[11].split(":")[1]
        wand_xr = crazy1[13].split(":")[1]
        wand_yr = crazy1[14].split(":")[1]
        wand_zr = crazy1[15].split(":")[1]
        wand_wr = crazy1[16].split(":")[1]


if __name__ == '__main__':
    rospy.init_node('crazyflie_tracker')
    
    cflib.crtp.init_drivers()

    t1 = threading.Thread(target=crazyflie)

    position_topic = '/tf'
    pose_subscriber = rospy.Subscriber(position_topic, TFMessage, optitrack)

    t1.start()

    rospy.spin()

       
