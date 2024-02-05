#!usr/bin/env python
import rospy
import can
import time
import os
import cantools
import rospkg
from can_ros_interface.msg import AS

'''
rostopic pub -r 100 /AS_out can_ros_interface/AS '{AS_trq_bl: 1, AS_trq_br: 1, AS_trq_fl: 1, AS_trq_fr: 1, AS_steer_ang: 0, AS_active: 1, AS_mission: 0, AS_R2D: 1, ASSI: 0}'
rostopic pub -r 100 /AS_out can_ros_interface/AS '{AS_trq_bl: 0, AS_trq_br: 0, AS_trq_fl: 0, AS_trq_fr: 0, AS_steer_ang: 0, AS_active: 0, AS_mission: 0, AS_R2D: 1, ASSI: 0}'

rostopic pub -r 100 /AS_out can_ros_interface/AS '{AS_trq_bl: 0, AS_trq_br: 0, AS_trq_fl: 0, AS_trq_fr: 0, AS_steer_ang: 0, AS_active: 0, AS_mission: 0, AS_R2D: 1, ASSI: 1}'
'''

rospack = rospkg.RosPack()

pkg_path = rospack.get_path('can_ros_interface')

# Set dbc file

database_path = os.path.join(pkg_path, 'powertrain_AS_11_15_23.dbc')

# Fix any existing error frames
os.system("sudo ip link set can0 down")
os.system("sudo ifconfig can0 txqueuelen 1000")
os.system("sudo ip link set can0 up type can bitrate 500000")

db = cantools.database.load_file(database_path)

bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=500000)

AS_Setpt_trq_msg = db.get_message_by_name('AS_Setpt_trq')

AS_Setpt_steer_msg = db.get_message_by_name('AS_Setpt_steer')

DV_system_status_msg = db.get_message_by_name('DV_system_status')


def callback(data):
  
  # Set the data for 4 wheel torques
  AS_Setpt_trq_data = AS_Setpt_trq_msg.encode({ 'AS_trq_fl': data.AS_trq_fl,
                                                'AS_trq_fr': data.AS_trq_fr,
                                                'AS_trq_bl': data.AS_trq_bl,
                                                'AS_trq_br': data.AS_trq_br
                                                })
  
  # Set the data for steering angle
  AS_Setpt_steer_data = AS_Setpt_steer_msg.encode({'AS_steer_ang': data.AS_steer_ang})

  DV_system_status_data = DV_system_status_msg.encode({ 'AS_state': data.ASSI,
                                                        'EBS_State': 0,
                                                        'AMI_state': 0,
                                                        'Steering_state': 0,
                                                        'Service_brake_state': 0,
                                                        'Lap_counter': 0,
                                                        'Cones_count_actual': 0,
                                                        'Cones_count_all': 0,
                                                        })

  try:
      # bus_message_trq = can.Message(arbitration_id=AS_Setpt_trq_msg.frame_id, data=AS_Setpt_trq_data, dlc=8, is_extended_id=False, channel='can0', is_rx=False)
      # bus_message_steer = can.Message(arbitration_id=AS_Setpt_steer_msg.frame_id, data=AS_Setpt_steer_data, dlc=3, is_extended_id=False, channel='can0', is_rx=False)
      bus_message_status = can.Message(arbitration_id=DV_system_status_msg.frame_id, data=DV_system_status_data, dlc=5, is_extended_id=False, channel='can0', is_rx=False)
      
      bus.send(bus_message_status)
      # bus.send(bus_message_trq)
      # bus.send(bus_message_steer)
      print(f"Message sent on {bus.channel_info}")
  except can.CanError:
      print(f"Message on {bus.channel_info} NOT sent")

  


if __name__ == '__main__':


  # anonymous=False, we only want one of these to ever be running
  rospy.init_node('can_ros_interface_node')

  # AS is the message type
  rospy.Subscriber('AS_out', AS, callback)

  rospy.spin()



