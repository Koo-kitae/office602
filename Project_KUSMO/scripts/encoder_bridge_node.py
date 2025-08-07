#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32
from md.msg import md_robot_msg1

# left_ticks와 right_ticks를 발행할 Publisher들을 전역 변수로 선언
pub_left_ticks = None
pub_right_ticks = None

def callback_motor_status(msg):
    """
    /md_robot_message1 토픽을 구독했을 때 실행되는 콜백 함수
    """
    # Int32 메시지 타입을 생성
    left_tick_msg = Int32()
    right_tick_msg = Int32()

    # /md_robot_message1 메시지에서 motor_pos 값을 추출하여
    # 각각의 Int32 메시지에 담습니다.
    # (모터 1번을 오른쪽, 2번을 왼쪽으로 가정)
    left_tick_msg.data = msg.motor2_pos
    right_tick_msg.data = msg.motor1_pos

    # /left_ticks와 /right_ticks 토픽으로 발행
    pub_left_ticks.publish(left_tick_msg)
    pub_right_ticks.publish(right_tick_msg)

def main():
    global pub_left_ticks, pub_right_ticks

    rospy.init_node('encoder_bridge_node')

    # /left_ticks와 /right_ticks 토픽을 발행할 Publisher 생성
    pub_left_ticks = rospy.Publisher('/left_ticks', Int32, queue_size=10)
    pub_right_ticks = rospy.Publisher('/right_ticks', Int32, queue_size=10)

    # /md_robot_message1 토픽을 구독
    rospy.Subscriber('/md_robot_message1', md_robot_msg1, callback_motor_status)
    
    rospy.loginfo("Encoder Bridge Node is running...")

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
