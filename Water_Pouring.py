#!/usr/bin/python3|
#|
coding=utf8|
# Date: 2022/06/30
import sys
import time
import rospy
from kinematics import ik_transform
from armpi_pro import bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

if sys.version_info.major == 2:
    print( 'Please run this program with python3!')
    sys.exit(0)
ik = ik_transform.ArmIK()
def stop():
    target = ik.setPitchRanges((0.00, 0.12, 0.08), -90, -180, -90)
    if target:
        servo_data = target [1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data[' servos']), (6, servo_data[' servo6'])))


if __name__=='__main__':
    rospy.init_node('Water_Pouring',log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    rospy.sleep(1.0)

    target = ik.setPitchRanges((-0.96, 0.12, 0.24), -180, -0, -90)
    if target:
        servo_data = target [1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 100), (2, 1000), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data[' servos']), (6, servo_data[' servo6'])))
    time.sleep(1.0)


    
    target = ik.setPitchRanges((0.0, 0.0, 0.0), -90, 0, -90)
    if target:
        servo_data = target [1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 1000), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data[' servos']), (6, servo_data[' servo6'])))

    time.sleep(1.0)


    target = ik.setPitchRanges((0.0, 0.0, 0.4), -90, 270, -270)
    if target:
        servo_data = target [1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 0), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data[' servos']), (6, servo_data[' servo6'])))

    time.sleep(1.0)

    target = ik.setPitchRanges((0.0, 0.0, 0.4), -90, 0, -270)
    if target:
        servo_data = target [1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 0), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data[' servos']), (6, servo_data[' servo6'])))

    time.sleep(3.0)


    target = ik.setPitchRanges((0.0, 0.12, 0.15), -90, -180, 0)
    if target:
        servo_data = target [1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data[' servos']), (6, servo_data[' servo6'])))

    time.sleep(1.0)

    target = ik.setPitchRanges((0.0, 0.12, 0.15), -90, -180, 0)
    if target:
        servo_data = target [1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 1000), (2, 0), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data[' servos']), (6, servo_data[' servo6'])))

    time.sleep(1.0)

    target = ik.setPitchRanges((0.96, 0.12, 0.15), -90, -180, 0)
    if target:
        servo_data = target [1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 1000), (2, 0), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data[' servos']), (6, servo_data[' servo6'])))

    time.sleep(1.0)





    target = ik.setPitchRanges((0.0, 0.96, 0.15), -90, -180, 0)
    if target:
        servo_data = target [1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 1000), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data[' servos']), (6, servo_data[' servo6'])))

    time.sleep(1.0)

    target = ik.setPitchRanges((0.96, 0.12, 0.15), -90, -180, -90)
    if target:
        servo_data = target [1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 1000), (2, 1000), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data[' servos']), (6, servo_data[' servo6'])))

    time.sleep(1.0)

    target = ik.setPitchRanges((0.96, 0.12, 0.24), -90, -180, -90)
    if target:
        servo_data = target [1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 1000), (2, 1000), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data[' servos']), (6, servo_data[' servo6'])))

    time.sleep(1.0)

    target = ik.setPitchRanges((0.0, 0.12, 0.24), -90, -180, -90)
    if target:
        servo_data = target [1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 1000), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data[' servos']), (6, servo_data[' servo6'])))

    time.sleep(1.0)








    
    target = ik.setPitchRanges((0.0, 0.96, 0.15), -90, -180, 0)

    if target:
        servo_data = target [1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 1000), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data[' servos']), (6, servo_data[' servo6'])))


    time. sleep(1.0)

    
    target = ik.setPitchRanges ((0.96, 0.12, 0.15), -90, -180, -90)
    if target:
        servo_data = target [1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 1000), (2, 1000), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data[' servos']), (6, servo_data[' servo6'])))

    time.sleep(1.0)

    
    target = ik.setPitchRanges((0.96, 0.12, 0.24), -90, -180, -90)
    if target:
        servo_data = target [1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 1000), (2, 1000), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data[' servos']), (6, servo_data[' servo6'])))

    time.sleep(1.0)

    
    target = ik.setPitchRanges((0.0, 0.12, 0.24), -90, -180, -90)
    if target:
        servo_data = target [1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 1000), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data[' servos']), (6, servo_data[' servo6'])))


    time. sleep(1.0)
    target = ik.setPitchRanges((0.0, 0.12, 0.24), -90, -180, -90)
    if target:
        servo_data = target [1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 0), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data[' servos']), (6, servo_data[' servo6'])))



    time.sleep(4.0)


