'''
Author: xindong324 xindong324@163.com
Date: 2024-02-22 20:51:17
LastEditors: xindong324 xindong324@163.com
LastEditTime: 2024-02-23 00:58:31
FilePath: /uneven_planner/src/perching/planning/scripts/set_state.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler,euler_matrix

import math

state = ModelState()
ModelState.model_name = "landing_pad_test"

t_start = 0
flag_trigger = False

mpitch = 0

t_test = 3.3

pub = None


omega = 2.0



def positionCallback(msg):
	global flag_trigger
	t_start = rospy.Time.now()
	flag_trigger = True
	

def fsm_cb(event):
	global state
	# if(flag_trigger == False):
	# 	return
	print("set")
	t = (rospy.Time.now() - t_start).to_sec()
	mpitch = 0.6*math.sin(omega*t)
	quat = quaternion_from_euler(0,mpitch,0)
	state.pose.position.x = 5
	state.pose.position.y = 0
	state.pose.position.z = 2

	state.pose.orientation.x = quat[0]
	state.pose.orientation.y = quat[1]
	state.pose.orientation.z = quat[2]
	state.pose.orientation.w = quat[3]
	pub.publish(state)

    

	

if __name__ == "__main__":
	rospy.init_node("pytalker",anonymous=True)
	# sub = rospy.Subscriber("/move_base_simple/goal",PoseStamped, callback=positionCallback, queue_size=1)
	t_start = rospy.Time.now()
	
    ############################################
	mpitch = 0.6*math.sin(omega*t_test)
	quat = quaternion_from_euler(mpitch,0,0)
	print("pit:",mpitch)
	print("q:",quat)
	print("mat:",euler_matrix(mpitch,0,0))

    ##################################################

	pub = rospy.Publisher("/gazebo/set_model_state",ModelState,queue_size=10) #Publisher 函数第一个参数是话题名称，第二个参数 数据类型，现在就是我们定义的msg 最后一个是缓冲区的大小
	fc_pub = rospy.Timer(rospy.Duration(0.01), fsm_cb)
	print("init done")
	#更新频率是1hz
	rospy.spin()
