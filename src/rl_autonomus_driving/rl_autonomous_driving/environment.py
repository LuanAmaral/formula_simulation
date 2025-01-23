import rclpy
from rclpy.node import Node
from eufs_msgs.msg import ConeArrayWithCovariance
from eufs_msgs.msg import CarState
import numpy as np
from gym import Env 
from gym import spaces
import quaternion
from Ackermann.msg import AckermannDriveStamped
import time
from std_srvs.srv import Trigger

class EnvRosInterface(Node):
    def __init__(self):
        super().__init__('env')
        self.get_logger().info('Environment node has been initialized')
        self.subscriptions = []
        self.yellow_cones = []
        self.blue_cones = []
        self.orange_cones = []
        self.rate = self.create_rate(10)
        self._clock = self.get_clock()
        
        # subscribe to the topic /cone
        subscription = self.create_subscription(
            ConeArrayWithCovariance,
            '/cones',
            self.cones_callback,
            10)   
        
        self.subscriptions.append(subscription)

        subscription = self.create_subscription(
            CarState,
            '/odometry_integration/car_state',
            self.car_state_callback,
            10)
        
        self.publisher = self.create_publisher(AckermannDriveStamped, '/cmd', 10)
        
        self.reset_vehicle_pos_srv = self.create_client(Trigger, '/ros_can/reset_vehicle_pos')
        self.reset_cone_pos_srv = self.create_client(Trigger, '/ros_can/reset_cone_pos')
            
    def cones_callback(self, msg):
        self.yellow_cones = msg.yellow_cones.point
        self.blue_cones = msg.blue_cones.point
        self.orange_cones = msg.big_orange_cones.point
        self._clock = self.get_clock()
    
    def car_state_callback(self, msg):
        self.car_state = msg
        self._clock = self.get_clock()
        
    def publish_action(self, action):
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = action[0]
        msg.drive.acceleration = action[1]
        self.publisher.publish(msg)
      
    def resetVehiclePos(self):
        """Requests race car model position reset"""
        self.node.get_logger().debug(
            "Requesting race_car_model position reset")

        if self.reset_vehicle_pos_srv.wait_for_service(timeout_sec=1):
            request = Trigger.Request()
            result = self.reset_vehicle_pos_srv.call_async(request)
            self.node.get_logger().debug("Vehicle position reset successful")
            self.node.get_logger().debug(result)
        else:
            self.node.get_logger().warn(
                "/ros_can/reset_vehicle_pos service is not available")
    
    def resetConePos(self):
        """Requests gazebo_cone_ground_truth to reset cone position"""
        self.node.get_logger().debug(
            "Requesting gazebo_cone_ground_truth cone position reset")

        if self.reset_cone_pos_srv.wait_for_service(timeout_sec=1):
            request = Trigger.Request()
            result = self.reset_cone_pos_srv.call_async(request)
            self.node.get_logger().debug("Cone position reset successful")
            self.node.get_logger().debug(result)
        else:
            self.node.get_logger().warn(
                "/ros_can/reset_cone_pos service is not available")


class EufsEnv(Env):
    def __init__(self, delay=0.1):
        self.ros_interface = EnvRosInterface()
        self.action_space = spaces.Box(low=np.array([-1, -2]), high=np.array([1, 2]), dtype=np.float32) # Steering angle and acc
        self.i = 0
        
        # Observation space:
        # 3 yellow cones (x, y, z)
        # 3 blue cones (x, y, z)
        # 4 orange cones (x, y, z)
        # Position of the car (x, y, z)
        # Orientation of the car (x, y, z, w)
        # Vel of the car (x, y, z, yaw)
        # Acc of the car (x, y, z)
        self.nb_car_states = 44
        self.observation_space = spaces.Box(
            low=np.array( self.nb_car_states*[-np.inf] ),
            high=np.array( self.nb_car_states*[np.inf] ),
            dtype=np.float32
        )
        
        self.reward_range = (-np.inf, np.inf)
        
        self.previous_state = self.nb_car_states*[None]
        self.previous_orange_cones_pos = self._average_cone_pos(self.ros_interface.orange_cones)
        self.previous_time = self.ros_interface.get_clock().now().nanoseconds
        
        self.init_time = self.previous_time
        self.lap_time = self.previous_time
        
         
    def _get_cones_position(self):
        yellow_cones_pos = []
        blue_cones_pos = []
        orange_cones_pos = []
        
        for i in range(3):
            if i < len(self.ros_interface.yellow_cones):
                yellow_cones_pos.append([
                    self.ros_interface.yellow_cones[-3+i].x,
                    self.ros_interface.yellow_cones[-3+i].y,
                    self.ros_interface.yellow_cones[-3+i].z
                    ])
            else:
                yellow_cones_pos.append(3*[np.inf])
            
            if i < len(self.ros_interface.blue_cones):
                blue_cones_pos.append([
                    self.ros_interface.blue_cones[-3+i].x,
                    self.ros_interface.blue_cones[-3+i].y,
                    self.ros_interface.blue_cones[-3+i].z
                    ])
            else:
                blue_cones_pos.append(3*[np.inf])
            
        for i in range(4):                
            if i < len(self.ros_interface.orange_cones):
                orange_cones_pos.append([
                    self.ros_interface.orange_cones[i].x,
                    self.ros_interface.orange_cones[i].y,
                    self.ros_interface.orange_cones[i].z
                    ])
            else:
                orange_cones_pos.append(3*[np.inf])
                
        # transform the list in one dimension
        yellow_cones_pos = [item for sublist in yellow_cones_pos for item in sublist]
        blue_cones_pos = [item for sublist in blue_cones_pos for item in sublist]
        orange_cones_pos = [item for sublist in orange_cones_pos for item in sublist]
        
        return yellow_cones_pos, blue_cones_pos, orange_cones_pos
    
    def _get_car_state(self):
        car_state = [
            self.ros_interface.car_state.pose.position.x,
            self.ros_interface.car_state.pose.position.y,
            self.ros_interface.car_state.pose.position.z,
            self.ros_interface.car_state.pose.orientation.x,
            self.ros_interface.car_state.pose.orientation.y,
            self.ros_interface.car_state.pose.orientation.z,
            self.ros_interface.car_state.pose.orientation.w, 
            self.ros_interface.car_state.twist.twist.linear.x,
            self.ros_interface.car_state.twist.twist.linear.y,
            self.ros_interface.car_state.twist.twist.linear.z,
            self.ros_interface.car_state.twist.twist.angular.z, 
            self.ros_interface.car_state.linear_acceleration.x,
            self.ros_interface.car_state.linear_acceleration.y,
            self.ros_interface.car_state.linear_acceleration.z,
        ]
        return car_state
    
    def _get_state(self):
        yellow_cones_pos, blue_cones_pos, orange_cones_pos = self._get_cones_position()
        car_state = self._get_car_state()
        
        state = yellow_cones_pos + blue_cones_pos + orange_cones_pos + car_state
        return state      
    
    def _global_to_car_frame(self, pos, car_pos, car_orientation):
        # pos: [x, y, z]
        # car_pos: [x, y, z]
        # car_orientation: [x, y, z, w]
        # return: [x, y, z]
        q = np.quaternion(car_orientation[3], car_orientation[0], car_orientation[1], car_orientation[2])
        pos_car_frame = quaternion.rotate_vectors(q, np.array(pos) - np.array(car_pos))
        return pos_car_frame
    
    def _average_cone_pos(self, orange_cones_pos_car_frame):
        # orange_cones_pos_car_frame: [[x, y, z], [x, y, z], [x, y, z], [x, y, z]]
        # return: [[x, y, z], [x,y,z]]
        average_cone_pos_left = [0, 0, 0]
        average_cone_pos_right = [0, 0, 0]
        
        if orange_cones_pos_car_frame[0][1] == np.inf or \
            orange_cones_pos_car_frame[1][1] == np.inf or \
            orange_cones_pos_car_frame[2][1] == np.inf or \
            orange_cones_pos_car_frame[3][1] == np.inf:
            return [average_cone_pos_left, average_cone_pos_right]
        
        for i in range(4):
            if orange_cones_pos_car_frame[i][1] > 0:
                average_cone_pos_left = np.add(average_cone_pos_left, orange_cones_pos_car_frame[i])
            else:
                average_cone_pos_right = np.add(average_cone_pos_right, orange_cones_pos_car_frame[i])
        
        average_cone_pos_left = average_cone_pos_left / 2
        average_cone_pos_right = average_cone_pos_right / 2
        
        return [average_cone_pos_left, average_cone_pos_right]
        
    def _is_left_side(self, cone_1, cone_2, car_pos):
        if cone_1[0] == np.inf or cone_2[0] == np.inf:
            return True        
        return (cone_1[0] - car_pos[0]) * (cone_2[1] - car_pos[1]) - (cone_2[0] - car_pos[0]) * (cone_1[1] - car_pos[1]) > 0

    def _is_right_side(self, cone_1, cone_2, car_pos):
        if cone_1[0] == np.inf or cone_2[0] == np.inf:
            return True
        return (cone_1[0] - car_pos[0]) * (cone_2[1] - car_pos[1]) - (cone_2[0] - car_pos[0]) * (cone_1[1] - car_pos[1]) < 0
    
    def _get_reward(self, state):
        reward = 0
        yellow_cones = state[:9]
        blue_cones = state[9:18]
        orange_cones = state[18:30]
        car_pos = state[30:33]
        car_orientation = state[33:37]
        car_vel = state[37:41]
        car_acc = state[41:44]
        
        time = self.ros_interface.get_clock().now().nanoseconds
        
        # check if the car is on the track
        on_track = True
        for i in range(2):
            on_track = on_track and self._is_left_side(yellow_cones[3*i:3*(i+1)], yellow_cones[3*(i+1):3*(i+2)], car_pos)
            on_track = on_track and self._is_right_side(blue_cones[3*i:3*(i+1)], blue_cones[3*(i+1):3*(i+2)], car_pos)
        
        if not on_track:
            reward -= 100
            
        # Check if one lap is completed
        orange_cones_pos_car_frame = [self._global_to_car_frame(orange_cones[3*i:3*(i+1)], car_pos, car_orientation) for i in range(4)]
        
        org_cones_pos_left, org_cones_pos_right = self._average_cone_pos(orange_cones_pos_car_frame)
        
        if (org_cones_pos_left[0] < 0 and org_cones_pos_right[0] < 0) and \
            (self.previous_orange_cones_pos[0][0] > 0 or self.previous_orange_cones_pos[1][0] > 0):
            reward += 1000-10*(time-self.lap_time)/1e9
            self.lap_time = time
            
        # Check velocity
        reward += 0.1 * car_vel[0]
        
        # Check acceleration jerk
        previous_acc = self.previous_state[35:38]
        reward -= 5 * np.linalg.norm(np.array(car_acc) - np.array(previous_acc))
       
        # Update previous state  
        self.previous_time = time
        self.previous_state = state
        self.previous_orange_cones_pos = [org_cones_pos_left, org_cones_pos_right]
        
        return reward
        
    
    def step(self, action):
        info = {"simulated_time": self.ros_interface.get_clock().now().nanoseconds}
        
        self.i += 1
        done = False
        
        # TODO:
        # Use the action to control the car
        # Delay to get the new state
        # Get the new state 
        # Calculate the reward
        # Check if the episode is done
        # Return the state, reward, done and info
        
        self.ros_interface.publish_action(action)
        self.ros_interface.rate.sleep()
        state = self._get_state()
        self.reward = self._get_reward(state)
                
        #TODO: Check if a Lap has been completed
        
        self.previous_state = state
         
        return state, self.reward, done, info

    def reset(self):
        # Implement the reset function
        # TODO: Reset the car position and orientation
        self.ros_interface.resetVehiclePos()
        self.ros_interface.resetConePos()
        self.i = 0
             
        self.previous_state = self.nb_car_states*[None]
        self.ros_interface.rate.sleep() 
        
        self.previous_orange_cones_pos = self._average_cone_pos(self.ros_interface.orange_cones)
        self.previous_time = self.ros_interface.get_clock().now().nanoseconds
        self.init_time = self.previous_time
        self.lap_time = self.previous_time
        
        state = self._get_state()
        return [state]