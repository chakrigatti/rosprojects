import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
from rclpy.time import Time

class TF2Broadcaster(Node):

    def __init__(self):
        super().__init__('tf2_broadcaster')
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_dict={}
        # Set timer to publish transforms at regular intervals
        self.timer = self.create_timer(0.1, self.broadcast_transforms)
        self.create_transform(frame_name='frame1',
                              parent_frame='world',
                              translation={'x':1.0,'y':1.0,'z':1.0},
                              rotation={'x':0.0,'y':0.0,'z':0.0,'w':1.0}
                              )
    
    def create_transform(self, frame_name,parent_frame,translation,rotation):
        t1 = None
        if frame_name not in self.tf_dict:
            t1 = TransformStamped()
            self.tf_dict[frame_name] = t1
            t1.header.frame_id = parent_frame
            t1.child_frame_id = frame_name
        else:
            t1 = self.tf_dict[frame_name]

        t1.header.stamp = self.get_clock().now().to_msg()
        if translation is not None :
            t1.transform.translation.x = translation['x']
            t1.transform.translation.y = translation['y']
            t1.transform.translation.z = translation['z']

        if rotation is not None:
            t1.transform.rotation.x = rotation['x']
            t1.transform.rotation.y = rotation['y']
            t1.transform.rotation.z = rotation['z']
            t1.transform.rotation.w = rotation['w']
        

    def broadcast_transforms(self):
        for x in self.tf_dict:
            self.tf_dict[x].header.stamp = self.get_clock().now().to_msg()
            self.broadcaster.sendTransform(self.tf_dict[x])



def main(args=None):
    rclpy.init(args=args)
    node = TF2Broadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
