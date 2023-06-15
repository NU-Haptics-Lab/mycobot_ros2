#!/usr/bin/env python2
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from pymycobot.mycobot import MyCobot

class Sync_plan(Node):
    def __init__(self):
        super().__init__("sync_plan")
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', 1000000)
        self.declare_parameter('speed', 80.0)
   
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.speed = self.get_parameter("speed").get_parameter_value().double_value
        self.mc = MyCobot(port,str(baud))

        self.get_logger().info("port:%s, baud:%d" % (port, baud))

        # initial joint angles
        joints = [0, 0, -1.57, 0, 0, -1.57]
        self.mc.send_radians(joints, 20)



        self.subscription = self.create_subscription(
            JointState,
            "cmd/joint_states",
            self.listener_callback,
            10
        )

    def listener_callback(self, data):
        # rclpy.loginfo(rclpy.get_caller_id() + "%s", data)
        data_list = []
        for index, value in enumerate(data.position):
            data_list.append(value)

        self.mc.send_radians(data_list, self.speed)


def main(args=None):
    rclpy.init(args=args)
    sync_plan = Sync_plan()
    
    rclpy.spin(sync_plan)
    
    sync_plan.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

