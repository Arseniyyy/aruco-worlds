import math

import rospy
import cv2
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range, Image
from clover.srv import SetLEDEffect
from cv_bridge import CvBridge
from time import time


rospy.init_node('flight_video_capture')

# bridge = CvBridge()
# out = cv2.VideoWriter('output_test.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 40, (320, 240))

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)
land = rospy.ServiceProxy('land', Trigger)


def navigate_wait(x=0, y=0, z=1.5, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)


# def image_callback(data):
#     cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
#     out.write(cv_image)
#     cv2.imwrite('images/{}.jpg'.format(time()), cv_image)


def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)


dist = 0  # range
def range_callback(msg):
    global dist
    dist = msg.range


def led_dist():
    z = get_telemetry(frame_id='navigate_target').z
    if z - dist - 0.1 > 0.5:
        set_effect(effect='blink_fast', r=255)
    else:
        set_effect(g=255)
    rospy.sleep(2)
    set_effect()  # turn off light


def write_to_file(count, x, y, z):
    with open('flight_data.txt', 'a') as f:
        f.write(f"{count}. x: {x}, y: {y}, z: {z}\n")


def square(first, second, third, fourth):
    navigate_wait(frame_id=f'aruco_{first}')
    telemetry = get_telemetry(frame_id='navigate_target')
    write_to_file(1, x=telemetry.x, y=telemetry.y, z=telemetry.z)
    rospy.sleep(2)

    navigate_wait(frame_id=f'aruco_{second}')
    telemetry = get_telemetry(frame_id='navigate_target')
    write_to_file(2, x=telemetry.x, y=telemetry.y, z=telemetry.z)
    rospy.sleep(2)

    navigate_wait(frame_id=f'aruco_{third}')
    telemetry = get_telemetry(frame_id='navigate_target')
    write_to_file(3, x=telemetry.x, y=telemetry.y, z=telemetry.z)
    rospy.sleep(2)

    navigate_wait(frame_id=f'aruco_{fourth}')
    telemetry = get_telemetry(frame_id='navigate_target')
    write_to_file(4, x=telemetry.x, y=telemetry.y, z=telemetry.z)
    rospy.sleep(2)



# rospy.Subscriber('main_camera/image_raw', Image, image_callback)
rospy.Subscriber('rangefinder/range', Range, range_callback)

with open('flight_data.txt', 'w') as f:
    f.write('')

navigate_wait(frame_id='body', auto_arm=True)
square(116, 128, 125, 113)

land_wait()

# out.release()
