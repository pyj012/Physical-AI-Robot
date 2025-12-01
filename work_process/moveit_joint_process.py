import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket
import time
class JointStateSubscriber(Node):
    def __init__(self, resultQ):
        print("Loading Joint State Process")
        load_start= time.time()
        super().__init__('joint_state_subscriber')
        self.resultQ = resultQ
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            1)

        self.subscription # prevent unused variable warning
        # Setup socket client
        self.joint_names_dict = {
            "L_shoulder_joint1":1,
            "L_shoulder_joint2":2,
            "L_elbow_joint":3,
            "L_arm_joint":4,
            "L_wrist_joint1":5,
            "L_wrist_joint2":11,
            "L_end_joint":0,
            "R_shoulder_joint1":6,
            "R_shoulder_joint2":7,
            "R_elbow_joint":8,
            "R_arm_joint":9,
            "R_wrist_joint1":10,
            "R_wrist_joint2":12,
            "R_end_joint":0
        }

        self.joint_dict={}
        self.prev_joint_dict={}
        self.inver_motor_id = [1,2,3,5,7,8,10,12]
        self.send_time = 0
        print("Load Joint State Complete : ",time.time()-load_start )


    def listener_callback(self, msg):
        self.send_time =time.time()
        joint_names = msg.name
        joint_positions = msg.position  # This contains the angles of the joints
        info_str = ""
        for name, position in zip(joint_names, joint_positions):
            # if ""name in 
            degrees = round(position * (180 / 3.14159),2)
            motor_id = self.joint_names_dict[name]
            if motor_id in self.inver_motor_id: 
                self.joint_dict[motor_id]= -degrees

            else:
                self.joint_dict[motor_id]= degrees

        # if self.joint_dict != self.prev_joint_dict:
        #     self.prev_joint_dict = self.joint_dict.copy()
        if self.resultQ.qsize()==0:
            self.resultQ.put(self.joint_dict.copy())
        time.sleep(0.001)

class Moveit_Joint_Class():  
    def __init__(self, resultQ):
        self.resultQ= resultQ
        rclpy.init()
        joint_state_subscriber = JointStateSubscriber(self.resultQ)
        rclpy.spin(joint_state_subscriber)
        # Shutdown
        joint_state_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    Moveit_Joint_Class(None)