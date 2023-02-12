import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient
from irobot_create_msgs.action import Dock, Undock

from geometry_msgs.msg import Twist, Transform
import math


class DockClient(Node):

    def __init__(self):
        super().__init__('dock_client')
        self._dock_client = ActionClient(self, Dock, 'dock')
        self._undock_client = ActionClient(self, Undock, 'undock')
    
    def undock(self):
        goal_msg = Undock.Goal()

        self._undock_client.wait_for_server()

        return self._undock_client.send_goal_async(goal_msg)
    
    def dock(self):
        goal_msg = Dock.Goal()

        self._dock_client.wait_for_server()

        return self._dock_client.send_goal_async(goal_msg)


class CmdVelPublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_publisher')
    
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.dt = 0.05  # seconds
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.i = 0

    def get_behavior(self):
        
        t = self.i * self.dt - 40.

        if t < 0:
            return (0.0, 0.0)
        if t < 25:
            return (1.0, 0.0)
        if t < 35:
            return (0.0, math.pi/3)
        if t < 55:
            return (1.0, 0.0)
        if t < 70:
            return (0.5, -math.pi/2)
            
        self.destroy_node()
        return
    
    def timer_callback(self):
        vx, wz = self.get_behavior() 
        if not (vx == 0.0 and wz == 0.0):
            msg = Twist()
            msg.linear.x = vx
            msg.angular.z = wz
            self.publisher_.publish(msg)
        self.i += 1

def main(args=None):
    rclpy.init()

    dock_client = DockClient()

    future = dock_client.undock()

    rclpy.spin_until_future_complete(dock_client, future)

    cmvel = CmdVelPublisher()
    rclpy.spin(cmvel)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
