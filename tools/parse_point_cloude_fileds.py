import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.exceptions import ROSInterruptException

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber')
        
        # 检查话题是否存在
        topic_name = '/scanner/cloud'
        available_topics = self.get_topic_names_and_types()

        # 如果没有该话题，打印提示信息并退出
        if topic_name not in [topic[0] for topic in available_topics]:
            self.get_logger().error(f"Topic '{topic_name}' not found. Exiting.")
            rclpy.shutdown()
            return

        # 如果话题存在，创建订阅者
        self.subscription = self.create_subscription(
            PointCloud2,
            topic_name,  # 订阅点云话题
            self.point_cloud_callback,
            10
        )
    
    def point_cloud_callback(self, msg):
        # 打印 PointCloud2 消息的字段名称和数据类型
        for field in msg.fields:
            print(f"Field Name: {field.name}, Data Type: {field.datatype}")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSubscriber()
    
    # 只在订阅者创建成功后，才进行 rclpy.spin
    if hasattr(node, 'subscription') and node.subscription:  
        rclpy.spin(node)
    else:
        # 如果没有创建订阅者，退出程序
        rclpy.shutdown()

if __name__ == '__main__':
    main()
