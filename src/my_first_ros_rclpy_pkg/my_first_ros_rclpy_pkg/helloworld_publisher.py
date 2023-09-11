import rclpy
from rclpy.node import Node
# rclpy의 노드 사용
from rclpy.node import QoSProfile
# QoS 설정을 위하여 QoSProfile 클래스 사용
from std_msgs.msg import String
# Publish하는 메시지의 타입은 std_msgs.msg 모듈의 String 메시지 인터페이스를 사용


# 메인 클래스 Node 클래스 상속
class HelloworldPublisher(Node):

  def __init__(self):
    # Node 클래스의 생성자를 호출하고 노드 이름을 지정
    super().__init__("helloworld_publisher")

    # 퍼블리셔의 QoS 설정을 위하여 'QoS 설정을 위하여 QoSProfile 호출하고 기본 depth를 10으로 설정
    # 통신상태가 원할하지 않을시 퍼블리시할 데이터를 버퍼에 10개까지 저장.
    qos_profile = QoSProfile(depth = 10)

    # Node 클래스의 create_publisher 함수를 이용하여 helloworld_publisher 라는 퍼블리셔를 설정.
    # 변수로 토픽에 사용할 토픽 메시지 타입과 토픽의 이름, QoS 설정을 기입하도록 되어 있으며
    # 토픽 메시지 타입으로 String, 토픽 이름으로 helloworld, QoS 설정으로 좀전에 설정한 qos_profile 설정
    self.helloworld_publisher = self.create_publisher(String,'helloworld',qos_profile)

    # Node 클래스의 create_timer함수를 이용하여 콜백함수를 수행하는 구문
    # 첫번째 매개변수 1 : timer_period_sec  1초마다 지정한 콜백 함수를 실행하라.
    # 1초마다 publih_hellow_msg 함수를 실행하게 된다 . count 는 콜백함수에 사용되는 카운터 값.
    self.timer = self.create_timer(1,self.publish_helloworld_msg)
    self.count = 0


  # 콜백함수
  # publish 할 메시지는 String타입으로 msg이라는 이름으로 선언.
  # 보낼 메시지는 msg.data에 저장. Hello world : 1 과 같이 매번 콜백함수가 실행될 때 마다 1씩 증가.
  # get_logger 는 콘솔창에 출력하는 함수로 종류에 따라 debug, info, warning, error, fatal 5종류가 있다.
  # 일반적인 정보 전달에는 info 를 사용.    printf 같은거다.
  def publish_helloworld_msg(self):
    msg = String()
    msg.data = 'Hello World : {0}'.format(self.count)
    self.helloworld_publisher.publish(msg)
    self.get_logger().info('Published message: {0}'.format(msg.data))
    self.count += 1

  # 마지막으로 main 함수 rclpy.init를 이용하여 초기화하고 위에서 작성한 HelloworldPublisher 클래스를 node라는 이름으로 생성
  # rclpy.spin 함수를 이용하여 생성한 노드를 spin시켜 지정된 콜백함수가 실행될 수 있도록 하고 있다.
  # 종료 같은 인크럽트 시그널 예외 상황에서는 node를 소멸시키고 rclpy.shutdown 함수로 노드를 종료하게 된다.
def main(args = None):
  rclpy.init(args=args)
  node = HelloworldPublisher()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.get_logger().info("Keyboard Interrupt (SIGINT)")
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()

