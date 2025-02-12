import rclpy
import rclpy.logging
from rclpy.node import Node
import numpy as np
import os
import torch
import torch.nn as nn
from eufs_msgs.msg import CanState
from rl_autonomous_driving import environment as env
from rl_autonomous_driving import ddpg
from itertools import count

class RlControler(Node):
    def __init__(self, max_steps=10000, test_iterations=500, exploration_noise=0.1, load=False):
        super().__init__('rl_controler')
        self.get_logger().info("Mission State Node Started")
        self.subscription = self.create_subscription(
            CanState,
            '/ros_can/state',
            self.mission_state_callback,
            10)
        
        self.car_state = None
        self.car_mission = None
        self.rewards = []
        
        self.environment = env.EufsEnv(delay=0.1)
        state_dim = self.environment.observation_space.shape[0]
        action_dim = self.environment.action_space.shape[0]
        max_action = min(self.environment.action_space.high)
        #TODO: change max_action to list of max_actions
        self.agent = ddpg.DDPG(state_dim, action_dim, max_action)
        
        self.load = False        
        self.max_steps = max_steps
        self.test_iterations = test_iterations
        self.load = load
        self.exploration_noise = exploration_noise
        self.log_interval = 50
        
    def mission_state_callback(self, msg):
        self.car_mission = msg.ami_state
        self.car_state = msg.as_state
        self.rl_controller()
        
    def rl_controller(self):
        if self.car_state == 2 and self.car_mission == 22:
          # Training
            self.get_logger().info("Training the model")
            if self.load: self.agent.load()
            total_steps = 0
            for i in range(self.test_iterations):
                self.get_logger().info("Episode: {}".format(i))
                state = self.environment.reset()[0]
                episode_reward = 0
                step = 0
                for t in count():
                    self.get_logger().info("Step: {}".format(t))
                    action = self.agent.select_action(state)
                    action = (action + np.random.normal(0, self.exploration_noise, size=1)).clip(
                        self.environment.action_space.low, self.environment.action_space.high)
                  
                    self.get_logger().info("Action: {}".format(action))
                    next_state, reward, done, _ = self.environment.step(action)
                    self.get_logger().info("Next State: {}, Reward: {}, Done: {}".format(next_state, reward, done))
                    self.agent.replay_buffer.push((state, next_state, action, reward, np.float(done)))
                    state = next_state
                    if done or t > self.max_steps:
                        self.get_logger().info("End of Episode")
                        break
                        
                    step += 1
                    episode_reward += reward
                    self.get_logger().info("Episode: {}, Total Steps: {}, Episode Reward: {}".format(i, total_steps, episode_reward))
                    
                self.get_logger().info("Training the model")
                total_steps += step+1
                self.agent.save()
                self.get_logger().info("Episode: {}, Total Steps: {}, Episode Reward: {}".format(i, total_steps, episode_reward))
                self.agent.update()
                
                self.rewards.append(episode_reward)
                
                if i % self.log_interval == 0:
                    self.agent.save()
                    self.get_logger().info("Model saved at episode: {}".format(i))
                    
                    # save rewards in file
                    np.save("rewards", self.rewards)
                     
        
            self.environment.reset()
            
        elif self.car_state == 2 and self.car_mission == 23:
            #TODO: test the model
            pass    
    
    
def main(args=None):
    rclpy.init(args=args)
    rl_controler = RlControler()
    rclpy.spin(rl_controler)
    rl_controler.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    