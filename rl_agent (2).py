import rclpy
from rclpy.node import Node
import rerun as rr
import numpy as np
import time

class RLAgent(Node):
    def __init__(self):
        super().__init__('rl_agent')
        self.get_logger().info('RL Agent Node Started')
        
        # Initialize Rerun
        rr.init("unitree_g1_rl", spawn=True)
        
        # Placeholders for RL
        self.observation_space = 10
        self.action_space = 12 # 12 DOF for G1 (approx)
        
        # Timer for control loop (e.g., 50Hz)
        self.create_timer(0.02, self.control_loop)
        
        self.step_count = 0

    def control_loop(self):
        # 1. Get Observation (Placeholder)
        obs = np.random.randn(self.observation_space)
        
        # 2. Inference (Placeholder for PPO/SAC)
        action = np.random.uniform(-1, 1, self.action_space)
        
        # 3. Publish Action (Placeholder)
        # self.publisher.publish(action)
        
        # 4. Log to Rerun
        rr.log("agent/action", rr.BarChart(action))
        rr.log("agent/observation", rr.Tensor(obs))
        rr.log("agent/reward", rr.Scalar(np.sin(self.step_count * 0.1)))
        
        self.step_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = RLAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
