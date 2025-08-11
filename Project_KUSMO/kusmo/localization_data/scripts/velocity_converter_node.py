#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from std_msgs.msg import Float32
from md.msg import md_robot_msg1 # rostopic info로 확인한 메시지 타입을 import 합니다.

def rpm_to_mps(rpm, wheel_radius=0.107):
  """
  모터의 RPM 값을 바퀴의 선속도(m/s)로 변환합니다.
  (바퀴 반지름은 터틀봇3 버거 기준이며, 로봇에 맞게 수정이 필요할 수 있습니다.)
  """
  # 분당 회전수를 초당 회전수로 변환
  rps = rpm / 60.0
  
  # 선속도(m/s) 계산
  linear_velocity = rps * (2 * math.pi * wheel_radius)
  
  return linear_velocity

# 생성한 Publisher들을 전역 변수로 선언하여 콜백 함수 내에서 접근 가능하게 함
pub_motor1_mps = None
pub_motor2_mps = None

def message_callback(msg):
  """
  /md_robot_message1 토픽을 구독했을 때 실행되는 콜백 함수
  """
  # 메시지에서 motor1_rpm, motor2_rpm 값을 가져옵니다.
  rpm1 = msg.motor1_rpm
  rpm2 = msg.motor2_rpm

  # 각 모터의 RPM 값을 m/s 속도로 변환합니다.
  velocity1_mps = rpm_to_mps(rpm1)
  velocity2_mps = rpm_to_mps(rpm2)

  # 변환된 m/s 값을 각각의 토픽으로 발행합니다.
  pub_motor1_mps.publish(velocity1_mps)
  pub_motor2_mps.publish(velocity2_mps)

  # 현재 상태를 로그로 출력하여 확인
  rospy.loginfo(f"RPM In (M1, M2): ({rpm1}, {rpm2}) -> MPS Out: ({velocity1_mps:.4f}, {velocity2_mps:.4f})")

def main():
  global pub_motor1_mps, pub_motor2_mps

  rospy.init_node('velocity_converter_node')

  # m/s로 변환된 값을 발행할 Publisher들을 생성합니다.
  pub_motor1_mps = rospy.Publisher('/mps/motor1', Float32, queue_size=10)
  pub_motor2_mps = rospy.Publisher('/mps/motor2', Float32, queue_size=10)

  # /md_robot_message1 토픽을 구독하고, 메시지가 수신되면 message_callback 함수를 실행합니다.
  rospy.Subscriber('/md_robot_message1', md_robot_msg1, message_callback)
  
  rospy.loginfo("Velocity Converter Node is running...")
  rospy.loginfo("Subscribing to /md_robot_message1")
  rospy.loginfo("Publishing to /mps/motor1 and /mps/motor2")

  # 노드가 종료될 때까지 대기
  rospy.spin()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
