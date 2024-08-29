import rospy
import can
import time
import os
import cantools
import rospkg
from can_ros_interface.msg import VehicleCan

rospack = rospkg.RosPack()

pkg_path = rospack.get_path('can_ros_interface')

# Powertrain dbc file
powertrain_database = os.path.join(pkg_path, 'powertrain.dbc')
pt_db = cantools.database.load_file(powertrain_database)

# Chassis dbc file
chassis_database = os.path.join(pkg_path, 'chassis.dbc')
ch_db = cantools.database.load_file(chassis_database)

# Configure RES CAN interface
os.system("sudo ip link set can0 down")
os.system("sudo ifconfig can0 txqueuelen 1000")
os.system("sudo ip link set can0 up type can bitrate 500000")
res_bus = can.interface.Bus(channel='can0', bustype='socketcan')

# Configure Steering CAN interface
os.system("sudo ip link set can2 down")
os.system("sudo ifconfig can2 txqueuelen 1000")
os.system("sudo ip link set can2 up type can bitrate 500000")
steering_bus = can.interface.Bus(channel='can2', bustype='socketcan')


# This will enable can on the RES
os.system("cansend can0 000#0100000000000000")
print("Enabled RES can")

# Enable Steering can
os.system("cansend can2 000#0100000000000000")
print("Enabled Steering can")

if __name__ == '__main__':
    rospy.init_node('can2ros', anonymous=False)

    # Create ROS publisher
    res_pub = rospy.Publisher('vehicle_can', VehicleCan, queue_size=20)
    steering_pub = rospy.Publisher('vehicle_steering_can', VehicleCan, queue_size=20)

    while not rospy.is_shutdown():
        print("Hello world")
        message = res_bus.recv()

        vehicle_can_msg = VehicleCan()
        vehicle_can_msg.header.stamp = rospy.Time.now()

        res_hex = message.data.hex()
        
        if res_hex == "0000000000000000":
            vehicle_can_msg.res_switch = False
            vehicle_can_msg.res_button = False
            vehicle_can_msg.res_enabled = False

        elif res_hex == "0100008000005a01":
            vehicle_can_msg.res_switch = False
            vehicle_can_msg.res_button = False
            vehicle_can_msg.res_enabled = False

        elif res_hex == "0100008000006401":
            vehicle_can_msg.res_switch = False
            vehicle_can_msg.res_button = False
            vehicle_can_msg.res_enabled = True

        elif res_hex == "0300008000006401":
            vehicle_can_msg.res_switch = True
            vehicle_can_msg.res_button = False
            vehicle_can_msg.res_enabled = True

        elif res_hex == "0700008000006401":
            vehicle_can_msg.res_switch = True
            vehicle_can_msg.res_button = True
            vehicle_can_msg.res_enabled = True

        elif res_hex == "0500008000006401":
            vehicle_can_msg.res_switch = False
            vehicle_can_msg.res_button = True
            vehicle_can_msg.res_enabled = True

        else:
            vehicle_can_msg.res_switch = False
            vehicle_can_msg.res_button = False
            vehicle_can_msg.res_enabled = False

        res_pub.publish(vehicle_can_msg)

        #process steering can messages
        steering_message = steering_bus.recv()
        steering_can_msg = VehicleCan()
        steering_can_msg.header.stamp = rospy.Time.now()

        decoded_message = ch_db.decode_message(steering_message.arbitration_id, steering_message.data)

        print(decoded_message)
        # if 'SteeringAngle' in decoded_message:
        #     vehicle_can_msg.steering_angle = decoded_message['SteeringAngle']
        # else:
        #     #indicates unknown message
        #     vehicle_can_msg.steering_angle = -1.0  

        # steering_pub.publish(steering_can_msg)


    
    
    
