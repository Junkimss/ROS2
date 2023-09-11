import rclpy
import random
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Node를 상속받아 사용
class pub_twist(Node):
  def __init__(self):
    # Node 생성자 호출 , 노드이름 pub
    super().__init__('pub')

    # 통신상태가 좋지 않을시 publish할 데이터를 버퍼에 10개까지 저장.
    qos_profile = QoSProfile(depth =10)

    # publisher 생성
    # 매시지 타입 : Twist
    # cmd_vel 토픽 이름.
    self.pub = self.create_publisher(
      Twist,
      'cmd_vel',
      qos_profile
    )
    # 10hz 초당 10번 pub_msg 실행.
    self.timer = self.create_timer(0.1,self.pub_msg)

  # twist_msg 에 random값 저장 및 출력.
  #
  def pub_msg(self):
    twist_msg =Twist()
    twist_msg.linear.x = random.uniform(-10,10)
    twist_msg.linear.x = round(twist_msg.linear.x,1)
    twist_msg.linear.y = random.uniform(-10,10)
    twist_msg.linear.y = round(twist_msg.linear.y,1)
    twist_msg.linear.z = random.uniform(-10,10)
    twist_msg.linear.z = round(twist_msg.linear.z,1)

    twist_msg.angular.x = random.uniform(0,360)
    twist_msg.angular.x = round(twist_msg.angular.x,1)
    twist_msg.angular.y = random.uniform(0,360)
    twist_msg.angular.y = round(twist_msg.angular.y,1)
    twist_msg.angular.z = random.uniform(0,360)
    twist_msg.angular.z = round(twist_msg.angular.z,1)

    # twist_msg 발송.
    self.pub.publish(twist_msg)
    self.get_logger().info('Published Twist: linear.x={0}, linear.y={1}, linear.z={2}, angular.z={3},angular.z={4}, angular.z={5}'.format(twist_msg.linear.x,twist_msg.linear.y,twist_msg.linear.z,twist_msg.angular.x,twist_msg.angular.y,twist_msg.angular.z))


def main(args = None):
  rclpy.init(args=args)
  node = pub_twist()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.get_logger().info("Keyboard Interrupt (SIGINT)")
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()
