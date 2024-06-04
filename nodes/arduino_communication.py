#!/usr/bin/env python3
import serial
import numpy as np
from std_msgs.msg import String
import rospy


class ArduinoMsg:
    """get the IR sensor message from the aruino and send it to ros
    """

    def __init__(self) -> None:
        rospy.init_node("arduino_msg", anonymous=True)

        # Load Aruidno parameters
        self.port = rospy.get_param("~arduino_port")
        self.baud_rate = int(rospy.get_param("~arduino_baud"))
        self.timeout = float(rospy.get_param("~arduino_timeout"))

        # Load Ros parameters
        self.publish_rate = rospy.Rate(int(rospy.get_param("~publish_rate")))

        # Init the Serial instance for Arduino connection
        self.arduino = serial.Serial(
            self.port, self.baud_rate, timeout=self.timeout)
        self.arduino.reset_input_buffer()
        rospy.loginfo("Successfully connected to the Arduino!")

        # Init a publisher for conveyor status
        self.convey_status_pub = rospy.Publisher(
            "conveyor_status", String, queue_size=10)
        self.convey_status = "run"
        self.ir_sensor_threshold = 800

    def publish_status(self):
        while not rospy.is_shutdown():
            try:
                readings = self.arduino.readline().decode("utf-8").rstrip()
                # rospy.loginfo(f"readings value type is {type(readings)}")
                ir_value = float(readings)
                rospy.loginfo(f"processed value ir_value is {ir_value}")
                if ir_value < self.ir_sensor_threshold:
                    self.convey_status = "stop"
                elif ir_value > self.ir_sensor_threshold:
                    self.convey_status = "run"

            except ValueError:
                pass

            self.convey_status_pub.publish(self.convey_status)
            self.publish_rate.sleep()


if __name__ == "__main__":
    arduino_msg = ArduinoMsg()
    arduino_msg.publish_status()
