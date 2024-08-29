#!usr/bin/env python
import rospy
import can
import time
import os
import cantools
import rospkg
from can_ros_interface.msg import AS
from can_ros_interface.msg import MR23_CAN

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

def extract_signal_value(data, start_bit, length, factor, offset):
    # Extract the raw signal value from the CAN data
    raw_value = int.from_bytes(data, byteorder='little', signed=True)

    # Shift and mask to get the signal value
    signal_mask = (1 << length) - 1
    signal_value = (raw_value >> start_bit) & signal_mask

    # Apply factor and offset to get the physical value
    physical_value = signal_value * factor + offset

    return physical_value

def can_to_ros():
    # Set up CAN bus interface
    # bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=500000)
    # Main loop to receive and publish CAN messages
   # message = bus.recv() # receive can message
    can_data = MR23_CAN() #using bus
    print("hello")
    with can.Bus() as bus:
        for msg in bus:
            print(msg.data)
    # if bus.recv() is not None:
    #     print("Here")
    #     #fix the id, it is under AMK_TargetTorque
    #     if message.arbitration_id == 388 or 389 or 392 or 393:  # ID for AS_Setpt_trq message
    #         can_data.header.stamp = rospy.Time.now()
    #         print(message.data[388])
    #         print(message.data[0])
    #         print((message.data[6] << 8 | message.data[7]) * 0.0098)
    #         signal_value = extract_signal_value(msg.data, start_bit=48, length=16, factor=0.0098, offset=0.0)
    #         print("JELLO")
    #         # can_data.AS_trq_fr = (message.data[6] << 8 | message.data[7]) * 0.0098
    #         # can_data.AS_trq_fl = (message.data[4] << 8 | message.data[5]) * 0.0098
    #         # can_data.AS_trq_br = (message.data[2] << 8 | message.data[3]) * 0.0098
    #         # can_data.AS_trq_bl = (message.data[0] << 8 | message.data[1]) * 0.0098
    #         pub_trq.publish(can_data) 
        ros.spinOnce()
  


if __name__ == '__main__':


  # anonymous=False, we only want one of these to ever be running
  rospy.init_node('can_ros_interface_node')

  # AS is the message type
  rospy.Subscriber('AS_out', AS, callback)
  
  # Create ROS publisher
  pub = rospy.Publisher('MR23_CAN', MR23_CAN, queue_size=20)
  
  while not rospy.is_shutdown():
    can_to_ros()


