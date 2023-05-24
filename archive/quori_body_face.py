#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import time


def send_body_face(start_position, start_emotion, end_position, end_emotion, duration):
    # print('Start: {}, End: {}, Duration {}'.format(start_position, end_position, duration))
    #Start point
    traj = JointTrajectory()
    traj.joint_names = ["r_shoulder_pitch", "r_shoulder_roll", "l_shoulder_pitch", "l_shoulder_roll", "waist_pitch"]
    point_1 = JointTrajectoryPoint()
    point_1.time_from_start = rospy.Duration(duration / 2)
    point_1.positions = start_position
    traj.points=[point_1]
    movement_pub.publish(traj)

    emotion_to_send = Float64MultiArray()
    emotion_to_send.data = start_emotion
    emotion_pub.publish(emotion_to_send)

    time.sleep(duration/2)

    #End point
    traj = JointTrajectory()
    traj.joint_names = ["r_shoulder_pitch", "r_shoulder_roll", "l_shoulder_pitch", "l_shoulder_roll", "waist_pitch"]

    point_2 = JointTrajectoryPoint()
    point_2.time_from_start = rospy.Duration(duration / 2)
    point_2.positions = end_position
    traj.points=[point_2]
    movement_pub.publish(traj)

    emotion_to_send = Float64MultiArray()
    emotion_to_send.data = end_emotion
    emotion_pub.publish(emotion_to_send)

    time.sleep(duration/2)

def callback(data):
    #Order [neutral, 'joy', 'sadness', 'anger', 'disgust', 'fear', 'surprise']
    #Message [emotion, intensity, duration]

    emotion = emotion_mapping[int(data.data[0])]
    intensity = data.data[1]
    duration = data.data[2]
    if duration < 8 and not emotion == 'neutral':
        duration = 8

    arm_y = np.array([2.21, -0.40])
    arm_sides = np.array([0, -1.1])
    arm_high = np.array([2.4, -1.1])
    arm_forward = np.array([1, -1.1])

    torso_forward = -0.47
    torso_backward = 0.21

    if emotion == 'neutral':
        neutral_positions = np.array([0, -1.1, 0, -1.1, 0])
        a = -0.05
        b = 0.05
        start_position = (neutral_positions + (b-a) * np.random.random_sample((5,)) + a).tolist()
        end_position = (neutral_positions + (b-a) * np.random.random_sample((5,)) + a).tolist()

        neutral_emotion = np.array([0.1, 0, 0, 0, 0, 0])
        a = 0
        b = 0.1
        start_emotion = (neutral_emotion + (b-a) * np.random.random_sample((6,)) + a).tolist()
        end_emotion = (neutral_emotion + (b-a) * np.random.random_sample((6,)) + a).tolist()

    elif emotion == 'happy':
        #Torso backward
        torso = intensity*torso_backward

        #Arm to y
        distance = arm_y - arm_sides + [1e-5, 1e-5]
        distance_to_move = distance*intensity
        end_arm = (arm_y - arm_sides) / (distance) * distance_to_move

        start_position = [end_arm[0], end_arm[1], end_arm[0], end_arm[1], torso]
        end_position = [0, -1.1, 0, -1.1, 0]

        start_emotion = [intensity, 0, 0, 0, 0, 0]
        end_emotion = [0, 0, 0, 0, 0, 0]

    elif emotion == 'sad':
        #Torso forward
        torso = intensity*torso_forward

        #Arm stays at sides
        start_position = [0, -1.1, 0, -1.1, torso]
        end_position = [0, -1.1, 0, -1.1, 0]

        start_emotion = [0, intensity, 0, 0, 0, 0]
        end_emotion = [0, 0, 0, 0, 0, 0]

    elif emotion == 'surprise':
        #Torso backward
        torso = intensity*torso_backward

        #Arm to high
        distance = arm_high - arm_sides + [1e-5, 1e-5]
        distance_to_move = distance*intensity 
        end_arm = (arm_high - arm_sides) / (distance) * distance_to_move

        start_position = [end_arm[0], end_arm[1], end_arm[0], end_arm[1], torso]
        end_position = [0, -1.1, 0, -1.1, 0]

        start_emotion = [0, 0, 0, 0, 0, intensity]
        end_emotion = [0, 0, 0, 0, 0, 0]
    
    elif emotion == 'interest':
        #Torso slightly forward
        torso = intensity*torso_forward*0.5

        #Right Arm to forward
        distance = arm_forward - arm_sides + [1e-5, 1e-5]
        distance_to_move = distance*intensity
        end_arm = (arm_forward - arm_sides) / (distance) * distance_to_move

        start_position = [end_arm[0], end_arm[1], 0, -1.1, torso]
        end_position = [0, -1.1, 0, -1.1, 0]

        start_emotion = [0.25*intensity, 0, 0, 0, 0, 0.25*intensity]
        end_emotion = [0, 0, 0, 0, 0, 0]
    
    
    send_body_face(start_position, start_emotion, end_position, end_emotion, duration)

def listener():

    rospy.init_node('quori_body_face', anonymous=True)

    rospy.Subscriber("quori_body_face", Float64MultiArray, callback)

    rospy.spin()


if __name__ == '__main__':
    emotion_pub = rospy.Publisher('quori/face_generator_emotion', Float64MultiArray, queue_size=10)
    movement_pub = rospy.Publisher('quori/joint_trajectory_controller/command', JointTrajectory, queue_size=10)
    emotion_mapping = ['neutral', 'happy', 'sad', 'surprise', 'interest']

    listener()