# -*- coding: utf-8 -*-

import os
import threading
import time

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

from enum import Enum
from enum import IntEnum

ROS_PACKAGE_NAME = os.environ.get("PYBOT_SCOUT_ROS_PACKAGE", "roller_eye")

_ros_srv = __import__(ROS_PACKAGE_NAME + ".srv", fromlist=["*"])
_ros_msg = __import__(ROS_PACKAGE_NAME + ".msg", fromlist=["*"])

for _module in (_ros_srv, _ros_msg):
  for _name in dir(_module):
    if not _name.startswith('_'):
      globals()[_name] = getattr(_module, _name)

class reg(IntEnum):
    person = 0
    home = 1
    dog = 2
    cat = 3
    motion = 1001

class MEDIA_TYPE(IntEnum):
    PIC = 1
    VIDEO = 2

DEGREE_3_RAD  = 0.052333
DURATION_10_S = 10000

class PyBotScoutRosBridge(threading.Thread):
  _velPub = None

  def __init__(self):
    super(PyBotScoutRosBridge, self).__init__()
    rospy.init_node('PyBotScoutBridgeNode', anonymous=False)
    self._velPub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    self._speakerPub = rospy.Publisher('/speaker_cmd', Int32, queue_size = 10)
    self.start()

  def run(self):
    rospy.spin()
    print "PyBotScoutRosBridge thread exit"

  def call_service_test(self):
    get_record_file_num = rospy.ServiceProxy('/RecorderAgentNode/record_get_file_num', record_get_file_num)
    print "call get_record_file_num"
    resp = get_record_file_num(0)
    print "get_record_file_num resp:%d" % resp.count

  def call_service_alog_move(self, xDistance, yDistance, speed):
    ALGO_MOVE_SERVICE = "/UtilNode/algo_move"
    algo_move_proxy = rospy.ServiceProxy(ALGO_MOVE_SERVICE, algo_move)
    print "call call_service_algo_move"
    resp = algo_move_proxy(ALGO_MOVE_SERVICE, xDistance, yDistance, speed)
    print "call_service_alog_move resp:%d" % resp.ret

  def call_service_alog_action(self, xSpeed, ySpeed, rotatedSpeed, time):
    ALGO_ROLL_SERVICE = "/UtilNode/algo_action"
    algo_action_proxy = rospy.ServiceProxy(ALGO_ROLL_SERVICE, algo_action)
    print "call call_service_alog_action"
    resp = algo_action_proxy(xSpeed, ySpeed, rotatedSpeed, time)
    print "call_service_alog_action resp:%d" % resp.ret

  def call_service_alog_roll(self, angle, rotatedSpeed, timeout = DURATION_10_S, error = DEGREE_3_RAD):
    ALGO_ROLL_SERVICE = "/UtilNode/algo_roll"
    algo_roll_proxy = rospy.ServiceProxy(ALGO_ROLL_SERVICE, algo_roll)
    print "call call_service_alog_roll"
    resp = algo_roll_proxy(angle, rotatedSpeed, timeout, error)
    print "call_service_alog_roll resp:%d" % resp.ret

  def call_service_record_start(self, media_type = MEDIA_TYPE.PIC, mode = 1, duration = 3, count = 1):
    SERVICE_NAME = "/RecorderAgentNode/record_start"
    proxy = rospy.ServiceProxy(SERVICE_NAME, record_start)
    proxy(media_type, mode, duration, count)

  def call_service_record_stop(self, media_type = MEDIA_TYPE.PIC):
    SERVICE_NAME = "/RecorderAgentNode/record_stop"
    proxy = rospy.ServiceProxy(SERVICE_NAME, record_stop)
    proxy(media_type)

  def sub_topic_ai_detect(self, cb):
    sub = rospy.Subscriber("/CoreNode/obj", detect, cb)
    return sub

  def sub_topic_ai_motion_detect(self, cb):
    sub = rospy.Subscriber("/CoreNode/motion", detect, cb)
    return sub

  def sub_topic_proximity(self, topic, cb):
    from sensor_msgs.msg import Range
    sub = rospy.Subscriber(topic, Range, cb)
    return sub

  def publish_cmd_vel(self, linearX, linearY, angularZ):
    vel = Twist()
    vel.linear.x = linearX
    vel.linear.y = linearY
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = angularZ
    try:
      self._velPub.publish(vel)
    except:
      print "publish vel cmd failed!"

  def publish_raw_cmd_vel(self, vel):
    try:
      self._velPub.publish(vel)
    except:
      print "publish raw vel cmd failed!"

  def publish_speaker_cmd(self, effect_id):
    try:
      self._speakerPub.publish(Int32(data=int(effect_id)))
      return True
    except Exception as exc:
      print "publish speaker cmd failed: %s" % str(exc)
      return False

  def call_service_programming_exception_handle(self, msg):
    SERVICE_NAME = "/AppNode/programming_exception_handle"
    proxy = rospy.ServiceProxy(SERVICE_NAME, programming_exception_handle)
    proxy(msg)

  def call_service_programming_meta_handle(self, msg):
    SERVICE_NAME = "/AppNode/programming_meta_handle"
    proxy = rospy.ServiceProxy(SERVICE_NAME, programming_meta_handle)
    proxy(msg)

  def call_service_programming_msg_handle(self, msg_type, msg):
    SERVICE_NAME = "/AppNode/programming_msg_handle"
    proxy = rospy.ServiceProxy(SERVICE_NAME, programming_msg_handle)
    proxy(msg_type, msg)

  def close(self):
    print "close PyBotScoutRosBridge!"
    rospy.signal_shutdown("close PyBotScoutRosBridge!")
