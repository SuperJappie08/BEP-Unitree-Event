#!/usr/bin/env python3
from serial import Serial
import argparse
import rospy
# from sensor_msgs.msg import Range 
from std_msgs.msg import String

# def get_point_cloud(msg: Range):
#     print(msg.header.stamp ,msg.range)

print("test")
if __name__ == '__main__':
    parser = argparse.ArgumentParser("bep_serial")
    parser.add_argument("--usb", type=str, default="/dev/ttyUSB0", required=False)
    # parser.add_argument("__name", type=str, default="bep_serial", required=False)
    args = parser.parse_known_intermixed_args()[0]
    device_id = args.usb
    print("started")
       
    rospy.init_node("bep_serial")

    # sub = rospy.Subscriber("/camera1/range_visual_face", Range, get_point_cloud)
    pub = rospy.Publisher("/bep/serial_data", String, tcp_nodelay=True)

    port: Serial = Serial(device_id, baudrate=115200)
    print("Connect to serial")
    # if not port.is_open:
    #     port.open()
    try:
        _ = port.read_until(b"// \n")

        last = rospy.Time.now()
        while raw_data:=port.read_until(b"// \n"):
            data: str = raw_data.lstrip().decode('iso-8859-1').rstrip("// ********************************************************** // \n").lstrip()
            # print("newmsg\n")
            # new = rospy.Time.now()
            # print(new - last)
            # print(new, data)
            # last = new
            pub.publish(String(data))
            
            
            # print(datetime.datetime.now(), data.splitlines()[6])
    finally:
        port.close()
    print("this")
