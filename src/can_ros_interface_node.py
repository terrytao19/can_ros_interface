#!usr/bin/env python
import rospy
import can
import time
import os
import cantools
import rospkg
from can_ros_interface.msg import AS

'''
rostopic pub -r 100 /AS_out can_ros_interface/AS '{AS_trq_bl: 1, AS_trq_br: 1, AS_trq_fl: 1, AS_trq_fr: 1, AS_steer_ang: 1, AS_active: 1, AS_mission: 1, AS_R2D: 1, ASSI: 1}'
'''

rospack = rospkg.RosPack()

pkg_path = rospack.get_path('can_ros_interface')

# Set dbc file

database_path = os.path.join(pkg_path, 'powertrain_AS.dbc')

# Fix any existing error frames
os.system("sudo ip link set can0 down")
os.system("sudo ip link set can0 up type can bitrate 500000 restart-ms 100")
os.system("sudo ip link set can0 up type can bitrate 500000")

db = cantools.database.load_file(database_path)

bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=500000)

AS_Setpt_trq_msg = db.get_message_by_name('AS_Setpt_trq')

AS_Setpt_steer_msg = db.get_message_by_name('AS_Setpt_steer')

AS_status_msg = db.get_message_by_name('AS_status')


def callback(data):
  
  # Set the data for 4 wheel torques
  AS_Setpt_trq_data = AS_Setpt_trq_msg.encode({ 'AS_trq_fl': data.AS_trq_fl,
                                                'AS_trq_fr': data.AS_trq_fr,
                                                'AS_trq_bl': data.AS_trq_bl,
                                                'AS_trq_br': data.AS_trq_br
                                                })
  
  # Set the data for steering angle
  AS_Setpt_steer_data = AS_Setpt_steer_msg.encode({'AS_steer_ang': data.AS_steer_ang})

  # Set the data for AS_status
  AS_status_data = AS_status_msg.encode({ 'AS_active': data.AS_active,
                                          'AS_mission': data.AS_mission,
                                          'AS_R2D': data.AS_R2D,
                                          'ASSI': data.ASSI
                                          })

  try:
      bus_message_trq = can.Message(arbitration_id=AS_Setpt_trq_msg.frame_id, data=AS_Setpt_trq_data, dlc=8, is_extended_id=False, channel='can0', is_rx=False)
      bus_message_steer = can.Message(arbitration_id=AS_Setpt_steer_msg.frame_id, data=AS_Setpt_steer_data, dlc=3, is_extended_id=False, channel='can0', is_rx=False)
      bus_message_status = can.Message(arbitration_id=AS_status_msg.frame_id, data=AS_status_data, dlc=1, is_extended_id=False, channel='can0', is_rx=False)
      bus.send(bus_message_trq)
      bus.send(bus_message_steer)
      bus.send(bus_message_status)
      print(f"Message sent on {bus.channel_info}")
  except can.CanError:
      print(f"Message on {bus.channel_info} NOT sent")

  


if __name__ == '__main__':


  # anonymous=False, we only want one of these to ever be running
  rospy.init_node('can_ros_interface_node')

  # AS is the message type
  rospy.Subscriber('AS_out', AS, callback)

  rospy.spin()



