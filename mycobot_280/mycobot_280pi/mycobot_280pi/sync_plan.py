#!/usr/bin/env python2
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32

from pymycobot.mycobot import MyCobot

class Sync_plan(Node):
    def __init__(self):
        super().__init__("sync_plan")
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', 1000000)
        self.declare_parameter('speed', 5)
   
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.speed = self.get_parameter("speed").get_parameter_value().integer_value
        self.mc = MyCobot(port,str(baud))

        self.get_logger().info("port:%s, baud:%d" % (port, baud))

        # initial joint angles
        joints = [0, 0, -1.57, 0, 0, -1.57]
        self.mc.send_radians(joints, 20)

        self.m_ACTIVE = False
        self.create_subscription(Bool, "activate", self.activateCb, 1 )
        self.create_subscription(JointState, "cmd/joint_commands", self.listener_callback, 1)
        self.status_pub = self.create_publisher(Float32, "active", 1)

        # timer
        self.create_timer(0.5, self.timerCB)

    def activateCb(self, msg):
        self.m_ACTIVE = msg.data

    def timerCB(self):
        self.status_pub.publish(Float32(data=float(self.m_ACTIVE)))

    def listener_callback(self, msg):
        # rclpy.loginfo(rclpy.get_caller_id() + "%s", data)

        data_list = []
        for index, value in enumerate(msg.position):
            data_list.append(value)

        if self.m_ACTIVE:
            self.mc.send_radians(data_list, self.speed)


def main(args=None):
    rclpy.init(args=args)
    sync_plan = Sync_plan()
    
    rclpy.spin(sync_plan)
    
    sync_plan.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

