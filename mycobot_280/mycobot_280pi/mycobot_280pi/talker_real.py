import math

import rclpy
from pymycobot.mycobot import MyCobot
# from pymycobot.mycobotsocket import MyCobotSocket
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class Talker(Node):
    def __init__(self):
        super().__init__("talker_real")
        
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', 1000000)
        self.declare_parameter('joint_prefix', "mycobot_")
   
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.joint_prefix = self.get_parameter("joint_prefix").get_parameter_value().string_value

        self.get_logger().info("port:%s, baud:%d" % (port, baud))
        self.mc = MyCobot(port,str(baud))

    def start(self):
        pub = self.create_publisher(
            msg_type=JointState,
            topic="joint_states",
            qos_profile=10
        )
        rate = self.create_rate(30)  # 30hz

        # pub joint state
        joint_state_send = JointState()
        joint_state_send.header = Header()

        joint_state_send.name = [
            self.joint_prefix + "joint2_to_joint1",
            self.joint_prefix + "joint3_to_joint2",
            self.joint_prefix + "joint4_to_joint3",
            self.joint_prefix + "joint5_to_joint4",
            self.joint_prefix + "joint6_to_joint5",
            self.joint_prefix + "joint6output_to_joint6",
        ]
        
        joint_state_send.effort = []
        
        while rclpy.ok():
            
            rclpy.spin_once(self)
            # get real angles from server.
            res = self.mc.get_angles()
            vels = self.mc.get_servo_speeds()
            try:
                if res[0] == res[1] == res[2] == 0.0:
                    continue
                radians_list = [
                    res[0] * (math.pi / 180),
                    res[1] * (math.pi / 180),
                    res[2] * (math.pi / 180),
                    res[3] * (math.pi / 180),
                    res[4] * (math.pi / 180),
                    res[5] * (math.pi / 180),
                ]
                vels_list = [
                    vels[0] * (math.pi / 180),
                    vels[1] * (math.pi / 180),
                    vels[2] * (math.pi / 180),
                    vels[3] * (math.pi / 180),
                    vels[4] * (math.pi / 180),
                    vels[5] * (math.pi / 180),
                ]
                # self.get_logger().info("vels: {}".format(vels_list))

                # publish angles.
                joint_state_send.header.stamp = self.get_clock().now().to_msg()
                joint_state_send.position = radians_list
                joint_state_send.velocity = vels_list
                pub.publish(joint_state_send)
                rate.sleep()
            except Exception as e:
                pass
                # print(e)
            
            


def main(args=None):
    rclpy.init(args=args)
    
    talker = Talker()
    talker.start()
    rclpy.spin(talker)
    
    talker.destroy_node()
    rclpy.shutdown()
    


if __name__ == "__main__":
    main()
