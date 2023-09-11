import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

# 얘도 Node 를 상속받아서 사용할 예정.
class HelloworldSubscriber(Node):

  # 생성자 정의로 'super().__init__("Helloworld_subscriber")를 이용하여 Node 클래스의 생성자를 호출하고
  # 노드 이름을 Helloworld_subscriber 로 정의
  # QoS를 호출하여 통신 상태가 원활하지 않을 시 subscriber 버퍼를 10개까지 저장하도록함.
  # create_subscription 함수를 이용하여 helloworld_subscriber 라는 서브스크라이버를 설정하고
  # 토픽 메시지 타입 , 토픽의 이름, 수신받을 메시지를 처리할 콜백함수와 , qos를 설정.
  def __init__(self):
    super().__init__('Helloworld_subscriber')
    qos_profile = QoSProfile(depth = 10)

    self.helloworld_subscriber = self.create_subscription(
      String,
      'helloworld',
      self.subscribe_topic_message,
      qos_profile
    )

  # 콜백 함수.
  # printf 와 같은 get_logger를 사용하여 받은 msg를 출력.
  def subscribe_topic_message(self,msg):
    self.get_logger().info("Received message : {0}".format(msg.data))



  # HelloworldSubscriber() 함수를 node로 사용하겠다.
def main(args =None ):
  rclpy.init(args=args)
  node = HelloworldSubscriber()
  try :
    rclpy.spin(node)
  except KeyboardInterrupt:
      node.get_logger().info('Keyboard Interrupt (SIGINT)')
  finally:
      node.destroy_node()
      rclpy.shutdown()

if __name__ == '__main__':
  main()





