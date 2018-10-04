"""
Demo of the ffmpeg based mambo vision code (basically flies around and saves out photos as it flies)

Author: Amy McGovern
"""

import sys
sys.path.append('../pyparrot/')

from Minidrone import Mambo
from DroneVision import DroneVision
import threading
import cv2
import cv2.aruco as aruco
import cv2.aruco as aruco2
import numpy

import time
import math


HEIGHT_FRONTCAM = 480 # y
WIDTH_FRONTCAM = 640 # x

HEIGHT_CENTER = HEIGHT_FRONTCAM/2
WIDTH_CENTER = WIDTH_FRONTCAM/2

TURN_CNT = 58
THRESHOLD = 30

LAND_ID = 27
TARGET_ID = 18
AWAY_ID = 43
ATTACK_ID = 42
DELTA = 6

PITCH_DELTA = 3

TARGET_DISTANCE = 110

MAX_LAST_MOVE_CNT = 2
DEFAULT_MOVE = (0, 0, 0, -2)

NUM_MOTION = 4
MOTIONS = ("front", "back", "right", "left")

# set this to true if you want to fly for the demo
show_groundcam = False
show_vision = True
testFlying = False
terminate = False
dance = False

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)

def half(last_move):
    if last_move > 0:
        last_move //= 2
    elif last_move < 0:
        last_move = -last_move
        last_move //= 2
        last_move = -last_move
    return last_move

def get_distance(pos1, pos2):
    x1, y1 = pos1
    x2, y2 = pos2
    diff_x = math.fabs(x1-x2)
    diff_y = math.fabs(y1-y2)
    return math.sqrt(diff_x*diff_x + diff_y*diff_y)


def middle(corner):

    roll = pitch = yaw = vertical_movement = 0

    sum_y = 0
    sum_x = 0
    for vertex in corner:
        sum_y += vertex[1]
        sum_x += vertex[0]

    pos_y = sum_y / 4
    pos_x = sum_x / 4

    if pos_x >= WIDTH_CENTER + THRESHOLD:
        return False
    elif pos_x <= WIDTH_CENTER - THRESHOLD:
        return False

    if pos_y >= HEIGHT_CENTER:
        diff_y = pos_y - HEIGHT_CENTER
        vertical_movement = -(diff_y/THRESHOLD + 2)*3
    elif pos_y <= HEIGHT_CENTER - THRESHOLD*2:
        diff_y = HEIGHT_CENTER - pos_y
        vertical_movement = (diff_y/THRESHOLD)*3

    if math.fabs(vertical_movement) >= 9:
        return False

    return True


def next_pos(corner):
    roll = pitch = yaw = vertical_movement = 0
    vertical_movement = -2

    sum_y = 0
    sum_x = 0
    for vertex in corner:
        sum_y += vertex[1]
        sum_x += vertex[0]

    pos_y = sum_y/4
    pos_x = sum_x/4

    if pos_x >= WIDTH_CENTER + THRESHOLD/2:
        diff_x = pos_x - WIDTH_CENTER
        print(pos_y, pos_x, "rotate_right") # yaw should be positive
        yaw = (diff_x//THRESHOLD + 1)*3

    elif pos_x <= WIDTH_CENTER - THRESHOLD/2:
        diff_x = WIDTH_CENTER - pos_x
        print(pos_y, pos_x, "rotate_left") # yas should be negative
        yaw = -(diff_x//THRESHOLD + 1)*3

    if math.fabs(yaw) >= 9:
        return roll, pitch, yaw, vertical_movement

    if pos_y >= HEIGHT_CENTER - THRESHOLD/2:
        diff_y = pos_y - HEIGHT_CENTER
        vertical_movement = -(diff_y/THRESHOLD + 2)*3
        print(pos_y, pos_x, "MOVE_DOWN")

    elif pos_y <= HEIGHT_CENTER - THRESHOLD:
        print(pos_y, pos_x, "MOVE_UP")
        diff_y = HEIGHT_CENTER - pos_y
        vertical_movement = (diff_y/THRESHOLD + 1)*3

    if math.fabs(vertical_movement) >= 8:
        return roll, pitch, yaw, vertical_movement


    side = get_distance(corner[0], corner[1])
    print("side : ", side)
    if side <= TARGET_DISTANCE:
        print("MOVE FRONT")
        pitch = PITCH_DELTA
        if side <= TARGET_DISTANCE - 20:
            pitch += PITCH_DELTA
        if side <= TARGET_DISTANCE - 50:
            pitch += PITCH_DELTA
        if side <= TARGET_DISTANCE - 90:
            pitch += PITCH_DELTA
        if side <= TARGET_DISTANCE - 100:
            pitch += PITCH_DELTA

    return roll, pitch, yaw, vertical_movement/2


class UserVision:
    def __init__(self, vision):
        self.index = 0
        self.vision = vision
        self.mambo = vision.drone_object
    def save_pictures(self, args):
        print("in save pictures on image %d " % self.index)
        img = self.vision.get_latest_valid_picture()
        if (img is not None):
            filename = "test_image_%06d.png" % self.index
            cv2.imwrite(filename, img)
            self.index +=1
            #print(self.index)

    def show_pictures(self, args):
        global dictionary
        global terminate
        #print("in show pictures on image %d " % self.index)
        img = self.vision.get_latest_valid_picture()
        if (img is not None):
            res = aruco.detectMarkers(img, dictionary)
            if len(res[0]) > 0:
                aruco.drawDetectedMarkers(img, res[0], res[1])
                if 8 in res[1]:
                    terminate = True

        #
        #    else:
        #        mambo.fly_direct(roll=0, pitch=0.1, yaw=0, vertical_movement=0, duration=0.1)
        #        mambo.smart_sleep(0.1)

        #   print(res[0], res[1])
            cv2.imshow('show', img)
            cv2.waitKey(1)
        #    if k == 27:
        #        global terminate
        #        terminate = True
        #        cv2.destroyAllWindows()

    def do_groundcam(self, args):
        global show_groundcam
        global terminate
        mambo = self.mambo
        while not terminate:
            mambo.take_picture()
            picture_names = mambo.groundcam.get_groundcam_pictures_names()  # get list of availible files
            while len(picture_names) == 0:
                mambo.take_picture()
                picture_names = mambo.groundcam.get_groundcam_pictures_names()  # get list of availible files

            frame = mambo.groundcam.get_groundcam_picture(picture_names[len(picture_names) - 1], True)  # get frame which is the first in the array
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            if frame is not None:
                res = aruco.detectMarkers(gray, dictionary)
                if len(res[0]) > 0 and LAND_ID in res[1]:
                    terminate = True
                    print("Detect MARK_LAND in groundcam")

                if show_groundcam:
                    if len(res[0]) > 0:
                    #   print(res[0], res[1])
                        aruco.drawDetectedMarkers(frame, res[0], res[1])
                    cv2.imshow('groundcam', frame)
                    cv2.waitKey(30)

# you will need to change this to the address of YOUR mambo
mamboAddr = "e0:14:d0:63:3d:d0"

# make my mambo object
# remember to set True/False for the wifi depending on if you are using the wifi or the BLE to connect
mambo = Mambo(mamboAddr, use_wifi=True)
print("trying to connect to mambo now")
success = mambo.connect(num_retries=3)
print("connected: %s" % success)

if (success):
    # get the state information
    print("sleeping")
    mambo.smart_sleep(1)
    mambo.ask_for_state_update()
    mambo.smart_sleep(1)

    print("Preparing to open vision")
    mamboVision = DroneVision(mambo, is_bebop=False, buffer_size=30)
    userVision = UserVision(mamboVision)
#    mamboVision.set_user_callback_function(userVision.show_pictures, user_callback_args=None)
    mamboVision.set_user_callback_function(userVision.do_groundcam, user_callback_args=None)

    success = mamboVision.open_video()
    print("Success in opening vision is %s" % success)

    if (success):
        print("Vision successfully started!")
        #removed the user call to this function (it now happens in open_video())
        #mamboVision.start_video_buffering()

        print("taking off!")
        mambo.safe_takeoff(5)
        mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=DELTA*3, duration=1)
        mambo.smart_sleep(1)
        motion = 0
        wait_cnt = 0
        last_move = (0, 0, 0, 0)
        last_time = time.time()

        while not terminate:
            img = userVision.vision.get_latest_valid_picture()

            if (img is not None):
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, dictionary)

                if show_vision:
                    if len(corners) > 0:
                        #print(corners, ids)
                        img = aruco.drawDetectedMarkers(img, corners, ids)

                    cv2.imshow('show', img)
                    k = cv2.waitKey(1)
                    if k == 27:
                        terminate = True
                        cv2.destroyAllWindows()

                if len(corners) > 0:
                    for idx in range(0, len(ids)):
                        marker_id = ids[idx]
                        corner = corners[idx][0]

                        if marker_id == AWAY_ID:
                            mambo.fly_direct(roll=0, pitch=0, yaw=-25, vertical_movement=0, duration=1)
                            mambo.smart_sleep(1)
                            mambo.fly_direct(roll=0, pitch=-DELTA*3, yaw=10, vertical_movement=0, duration=2)
                            mambo.smart_sleep(3)
                            mambo.fly_direct(roll=0, pitch=DELTA, yaw=-25, vertical_movement=0, duration=1)
                            mambo.smart_sleep(1)

                        elif marker_id == ATTACK_ID and middle(corner):
                            mambo.fly_direct(roll=0, pitch=-DELTA*3, yaw=0, vertical_movement=10, duration=0.8)
                            mambo.smart_sleep(0.8)
                            mambo.fly_direct(roll=0, pitch=100, yaw=0, vertical_movement=0, duration=0.5)
                            mambo.smart_sleep(0.5)
                            mambo.fly_direct(roll=0, pitch=-35, yaw=0, vertical_movement=0, duration=0.5)
                            mambo.smart_sleep(0.5)
                            mambo.fly_direct(roll=0, pitch=0, yaw=70, vertical_movement=-5, duration=1)
                            mambo.smart_sleep(1)


                        elif marker_id == TARGET_ID:
                            side = get_distance(corner[0], corner[1])
                            wait_cnt = 0
                            if side >= TARGET_DISTANCE:
                                pitch_back = -DELTA*2
                                r, p, y, vm = last_move
                                pitch_back -= half(p)

                                print("flip")
                                mambo.fly_direct(roll=0, pitch=pitch_back, yaw=0, vertical_movement=DELTA*2, duration=1)
                                mambo.smart_sleep(1.5)
                                mambo.flip(direction=MOTIONS[motion]) # ("front", "back", "right", "left")):
                                mambo.smart_sleep(4)
                                motion += 1
                                motion %= NUM_MOTION
                            else:
                                r, p, y, vm = next_pos(corner)
                                last_move = r, p, y, vm
                                mambo.fly_direct(roll=r, pitch=p, yaw=y, vertical_movement=vm, duration=0.10)

                        elif marker_id == LAND_ID:
                            terminate = True
                            print("Detect MARK_LAND in frontcam")


                else:
                    wait_cnt += 1
                    if last_move != (0, 0, 0, 0):
                        r, p, y, vm = last_move
                        r = half(r)
                        p = half(p)
                        y = half(y)
                        vm = half(vm)
                        mambo.fly_direct(roll=r, pitch=p, yaw=y, vertical_movement=vm, duration=0.10)
                        last_move = (r, p, y, 0)
                    else:
                        r, p, y, vm = DEFAULT_MOVE
                        mambo.fly_direct(roll=r, pitch=p, yaw=y, vertical_movement=vm, duration=0.10)

                    if wait_cnt == TURN_CNT:
                        mambo.fly_direct(roll=0, pitch=0, yaw=80, vertical_movement=0, duration=0.5)
                        wait_cnt = 0


                next_time = time.time()
                print(next_time - last_time)
                last_time = next_time


        print("landing")
        print("flying state is %s" % mambo.sensors.flying_state)
        mambo.safe_land(5)


        # done doing vision demo
        print("Ending the sleep and vision")
        mamboVision.close_video()

        mambo.smart_sleep(5)

    print("disconnecting")
    mambo.disconnect()
