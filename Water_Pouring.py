#!/usr/bin/python3
# coding=utf8
# Date: 2022/06/30

# Final Project

## 1. Introduction
*This project is a cumulative work that uti-
lizes the various concepts of robotic manipulation, such
as forward kinematics, inverse kinematics, Jacobians,
and Python coding to achieve the successful movement
of an ARMPI robot arm run by a Raspberry PI onboard
computer. Within this paper are the methods and prac-
tices that were followed to set up the Linux environment
for the ROS, or Robotic Operating System, as well as
the mathematical derivations of how to calculate joint
angles for the robot arm.*

## 2. Data Loading and Preprocessing
*Load the files and process it.*

import sys
import time
import rospy
from kinematics import ik_transform
from armpi_pro import bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
ik = ik_transform.ArmIK()

## 3. Stop Function
*Default position of the robot arm.*

def stop():
    target = ik.setPitchRanges((0.00, 0.12, 0.08), -90, -180, -90)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']), (5, servo_data['servos']), (6, servo_data['servo6'])))

## 4. Main Function
*Houses all of the joint angles required to move the robot to perform the water pouring action.*

if __name__=='__main__':
    rospy.init_node('Water_Pouring', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    rospy.sleep(1.0)

    target = ik.setPitchRanges((-0.96, 0.12, 0.24), -180, -0, -90)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 100), (2, 1000), (3, servo_data['servo3']),
                        (4, servo_data['servo4']), (5, servo_data['servos']), (6, servo_data['servo6'])))
    time.sleep(1.0)
    
    target = ik.setPitchRanges((0.0, 0.0, 0.0), -90, 0, -90)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 1000), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']), (5, servo_data['servos']), (6, servo_data['servo6'])))

    time.sleep(1.0)

## 5. Open claws for accepting water bottle
*The proper joint angles are sent to the robot to manipulate the claws to open.*

    target = ik.setPitchRanges((0.0, 0.0, 0.4), -90, 270, -270)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 0), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']), (5, servo_data['servos']), (6, servo_data['servo6'])))

    time.sleep(1.0)

    target = ik.setPitchRanges((0.0, 0.0, 0.4), -90, 0, -270)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 0), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']), (5, servo_data['servos']), (6, servo_data['servo6'])))

    time.sleep(3.0)

    target = ik.setPitchRanges((0.0, 0.12, 0.15), -90, -180, 0)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']), (5, servo_data['servos']), (6, servo_data['servo6'])))

    time.sleep(1.0)

    target = ik.setPitchRanges((0.0, 0.12, 0.15), -90, -180, 0)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 1000), (2, 0), (3, servo_data['servo3']),
                        (4, servo_data['servo4']), (5, servo_data['servos']), (6, servo_data['servo6'])))

## 6. Pouring
*Bring bottle closer to ground and tilt.*

    time.sleep(1.0)

    target = ik.setPitchRanges((0.96, 0.12, 0.15), -90, -180, 0)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 1000), (2, 0), (3, servo_data['servo3']),
                        (4, servo_data['servo4']), (5, servo_data['servos']), (6, servo_data['servo6'])))

    time.sleep(1.0)

    target = ik.setPitchRanges((0.0, 0.96, 0.15), -90, -180, 0)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 1000), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']), (5, servo_data['servos']), (6, servo_data['servo6'])))

    time.sleep(1.0)

    target = ik.setPitchRanges((0.96, 0.12, 0.15), -90, -180, -90)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 1000), (2, 1000), (3, servo_data['servo3']),
                        (4, servo_data['servo4']), (5, servo_data['servos']), (6, servo_data['servo6'])))

## 7. Return to user
*After pouring has completed, bring the bottle upwards towards the user.*

    time.sleep(1.0)

    target = ik.setPitchRanges((0.96, 0.12, 0.24), -90, -180, -90)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 1000), (2, 1000), (3, servo_data['servo3']),
                        (4, servo_data['servo4']), (5, servo_data['servos']), (6, servo_data['servo6'])))

    time.sleep(1.0)

    target = ik.setPitchRanges((0.0, 0.12, 0.24), -90, -180, -90)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 1000), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']), (5, servo_data['servos']), (6, servo_data['servo6'])))

    time.sleep(1.0)

    target = ik.setPitchRanges((0.0, 0.96, 0.15), -90, -180, 0)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 1000), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']), (5, servo_data['servos']), (6, servo_data['servo6'])))

    time.sleep(1.0)

    target = ik.setPitchRanges((0.96, 0.12, 0.15), -90, -180, -90)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 1000), (2, 1000), (3, servo_data['servo3']),
                        (4, servo_data['servo4']), (5, servo_data['servos']), (6, servo_data['servo6'])))

    time.sleep(1.0)

## 8. Claw Open
*Gently release claw to allow user to retrieve bottle.*

    target = ik.setPitchRanges((0.96, 0.12, 0.24), -90, -180, -90)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 1000), (2, 1000), (3, servo_data['servo3']),
                        (4, servo_data['servo4']), (5, servo_data['servos']), (6, servo_data['servo6'])))

    time.sleep(1.0)

    target = ik.setPitchRanges((0.0, 0.12, 0.24), -90, -180, -90)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 1000), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']), (5, servo_data['servos']), (6, servo_data['servo6'])))

## 9. Default position
*Finish steps and go reset joint angles to default positions.*

    time.sleep(1.0)
    target = ik.setPitchRanges((0.0, 0.12, 0.24), -90, -180, -90)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 0), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']), (5, servo_data['servos']), (6, servo_data['servo6'])))
    time.sleep(4.0)

## 10. Conclusion
*The current results demonstrate the feasibility of
using an ARMPI robot controlled by a Raspberry
Pi for executing manipulation tasks. The successful
movement of joints according to programmed angles
confirms that the kinematic models and initial setup
are correct.
However, further work is needed to refine the sys-
tem. Future steps include:
• Fine-tuning the joint movements for smoother
operation.
• Implementing feedback mechanisms for error cor-
rection.
• Developing a more user-friendly interface for
controlling the robot.
• Enhancing the precision of the pouring action
through calibration.
• Extending the functionality for more complex
tasks and experimenting with machine learning
algorithms for adaptive control.
By addressing these areas, we aim to improve the
robot’s performance and reliability, ultimately con-
tributing to the broader field of robotic manipulation.*