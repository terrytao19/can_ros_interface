import rospy
import can
import time
import os
import cantools
import rospkg
from can_ros_interface.msg import VehicleCan
from pprint import pprint


rospack = rospkg.RosPack()

pkg_path = rospack.get_path('can_ros_interface')

# Chassis dbc file
chassis_database = os.path.join(pkg_path, 'chassis.dbc')
ch_db = cantools.database.load_file(chassis_database)


# Configure Steering CAN interface
os.system("sudo ip link set can2 down")
os.system("sudo ifconfig can2 txqueuelen 1000")
os.system("sudo ip link set can2 up type can bitrate 500000")
steering_bus = can.interface.Bus(channel='can2', bustype='socketcan')



if __name__ == '__main__':
    rospy.init_node('can2ros_steering', anonymous=False)

    # Create ROS publisher
  
    steering_pub = rospy.Publisher('vehicle_steering_can', VehicleCan, queue_size=20)

    while not rospy.is_shutdown():
      

        #process steering can messages
        chassis_msg = steering_bus.recv()
        steering_can_msg = VehicleCan()
        steering_can_msg.header.stamp = rospy.Time.now()

        print(chassis_msg.arbitration_id)

        # if(chassis_msg.arbitration_id == 0x22):
        #     print("HERE")
            # decoded_message = ch_db.decode_message("0x" + str(chassis_msg.arbitration_id), chassis_msg.data)
            
        # print(decoded_message)

        # if 'SteeringAngle' in decoded_message:
        #     vehicle_can_msg.steering_angle = decoded_message['SteeringAngle']
        # else:
        #     #indicates unknown message
        #     vehicle_can_msg.steering_angle = -1.0  

        # steering_pub.publish(steering_can_msg)


    
    
    
