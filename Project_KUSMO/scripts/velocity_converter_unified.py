#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Vector3   # 두 값을 동시에 보내기 위해 Vector3 메시지 사용
from md.msg import md_robot_msg1

def rpm_to_mps(rpm, wheel_radius=0.107):
  """모터의 RPM 값을 바퀴의 선속도(m/s)로 변환합니다."""
  rps = rpm / 60.0
  linear_velocity = rps * (2 * math.pi * wheel_radius)
  return linear_velocity

# 통합된 속도 값을 발행할 Publisher
pub_motors_mps = None

def message_callback(msg):
  """/md_robot_message1 토픽 구독 시 실행되는 콜백 함수"""
  rpm1 = msg.motor1_rpm
  rpm2 = msg.motor2_rpm

  velocity1_mps = rpm_to_mps(rpm1)
  velocity2_mps = rpm_to_mps(rpm2)

  # Vector3 메시지를 생성하고, x, y 필드에 각 모터의 속도를 저장합니다.
  mps_msg = Vector3()
  mps_msg.x = velocity1_mps  # 모터1 속도
  mps_msg.y = velocity2_mps  # 모터2 속도
  mps_msg.z = 0.0            # 이 필드는 사용하지 않음

  pub_motors_mps.publish(mps_msg)
  rospy.loginfo(f"RPM In (M1, M2): ({rpm1}, {rpm2}) -> MPS Out (x, y): ({velocity1_mps:.4f}, {velocity2_mps:.4f})")

def main():
  global pub_motors_mps
  rospy.init_node('velocity_converter_unified_node') # 노드 이름 변경

  pub_motors_mps = rospy.Publisher('/motors_mps', Vector3, queue_size=10)
  rospy.Subscriber('/md_robot_message1', md_robot_msg1, message_callback)
  
  rospy.loginfo("Unified Velocity Converter Node is running...")
  rospy.loginfo("Publishing unified motor speeds to /motors_mps")

  rospy.spin()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
