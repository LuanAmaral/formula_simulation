import rclpy
from rclpy.node import Node
from eufs_msgs.msg import ConeArrayWithCovariance
from eufs_msgs.msg import CarState
import numpy as np
from gym import Env 
from gym import spaces
import quaternion
from ackermann_msgs.msg  import AckermannDriveStamped 
from std_srvs.srv import Trigger
import time

INF = 50000

class EufsEnv(Env, Node):
    def __init__(self, delay=0.1):
        super().__init__('eufs_env')
        self.get_logger().info('Environment node has been initialized')
         
        # ROS subscriber and publisher and services
        self.cone_subs = self.create_subscription(
            ConeArrayWithCovariance,
            '/cones',
            self._cones_callback,
            10)
        
        self.subs_car_state = self.create_subscription(
            CarState,
            '/odometry_integration/car_state',
            self._car_state_callback,
            10)
        
        self.publisher = self.create_publisher(AckermannDriveStamped, '/cmd', 10)
         
        self.reset_vehicle_pos_srv = self.create_client(Trigger, '/ros_can/reset_vehicle_pos')
        self.reset_cone_pos_srv = self.create_client(Trigger, '/ros_can/reset_cone_pos')
        
        # World variables
        self.yellow_cones = []
        self.blue_cones = []
        self.orange_cones = []
        self.rate = self.create_rate(10)
        self._clock = self.get_clock()
        self.car_state = 14*[0.0]          
        
        self.action_space = spaces.Box(
            low=np.array([-1, -2], dtype=np.float32),
            high=np.array([1, 2], dtype=np.float32),
            dtype=np.float32) # Steering angle and acc
        
        # Env variables
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
            low=np.array( self.nb_car_states*[-INF], dtype=np.float32 ),
            high=np.array( self.nb_car_states*[INF], dtype=np.float32 ),
            dtype=np.float32
        )
        
        self.reward_range = (-INF, INF)
        
        self.i = 0
        self.previous_state = self.nb_car_states*[0.0]
        self.previous_orange_cones_pos = [3*[-INF], 3*[-INF]]
        self.previous_time = self.get_clock().now().nanoseconds
        
        self.init_time = self.previous_time
        self.lap_time = self.previous_time
        
    def _cones_callback(self, msg):
        self.yellow_cones = msg.yellow_cones
        self.blue_cones = msg.blue_cones
        self.orange_cones = msg.orange_cones
        self._clock = self.get_clock()        
    
    def _car_state_callback(self, msg : CarState):
        self.car_state.append(msg.pose.position.x)
        self.car_state.append(msg.pose.position.y)
        self.car_state.append(msg.pose.position.z)
        self.car_state.append(msg.pose.orientation.x)
        self.car_state.append(msg.pose.orientation.y)
        self.car_state.append(msg.pose.orientation.z)
        self.car_state.append(msg.pose.orientation.w)
        self.car_state.append(msg.twist.twist.linear.x)
        self.car_state.append(msg.twist.twist.linear.y)
        self.car_state.append(msg.twist.twist.linear.z)
        self.car_state.append(msg.twist.twist.angular.z)
        self.car_state.append(msg.linear_acceleration.x)
        self.car_state.append(msg.linear_acceleration.y)
        self.car_state.append(msg.linear_acceleration.z)
        self._clock = self.get_clock()
        
    def _publish_action(self, action):
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = action[0]
        msg.drive.acceleration = action[1]
        self.publisher.publish(msg)
      
    def _resetVehiclePos(self):
        """Requests race car model position reset"""
        self.get_logger().debug(
            "Requesting race_car_model position reset")

        if self.reset_vehicle_pos_srv.wait_for_service(timeout_sec=1):
            request = Trigger.Request()
            result = self.reset_vehicle_pos_srv.call_async(request)
            self.get_logger().debug("Vehicle position reset successful")
            self.get_logger().debug(result)
        else:
            self.get_logger().warn(
                "/ros_can/reset_vehicle_pos service is not available")
    
    def _resetConePos(self):
        """Requests gazebo_cone_ground_truth to reset cone position"""
        self.get_logger().debug(
            "Requesting gazebo_cone_ground_truth cone position reset")

        if self.reset_cone_pos_srv.wait_for_service(timeout_sec=1):
            request = Trigger.Request()
            result = self.reset_cone_pos_srv.call_async(request)
            self.get_logger().debug("Cone position reset successful")
            self.get_logger().debug(result)
        else:
            self.get_logger().warn(
                "/ros_can/reset_cone_pos service is not available")
         
    def _get_cones_position(self):
        yellow_cones_pos = []
        blue_cones_pos = []
        orange_cones_pos = []
        
        for i in range(3):
            if i < len(self.yellow_cones):
                yellow_cones_pos.append([
                    self.yellow_cones[-3+i].x,
                    self.yellow_cones[-3+i].y,
                    self.yellow_cones[-3+i].z
                    ])
            else:
                yellow_cones_pos.append(3*[INF])
            
            if i < len(self.blue_cones):
                blue_cones_pos.append([
                    self.blue_cones[-3+i].x,
                    self.blue_cones[-3+i].y,
                    self.blue_cones[-3+i].z
                    ])
            else:
                blue_cones_pos.append(3*[INF])
            
        for i in range(4):                
            if i < len(self.orange_cones):
                orange_cones_pos.append([
                    self.orange_cones[i].x,
                    self.orange_cones[i].y,
                    self.orange_cones[i].z
                    ])
            else:
                orange_cones_pos.append(3*[INF])
                
        # transform the list in one dimension
        yellow_cones_pos = [item for sublist in yellow_cones_pos for item in sublist]
        blue_cones_pos = [item for sublist in blue_cones_pos for item in sublist]
        orange_cones_pos = [item for sublist in orange_cones_pos for item in sublist]
        
        return yellow_cones_pos, blue_cones_pos, orange_cones_pos
    
    def _get_state(self):
        yellow_cones_pos, blue_cones_pos, orange_cones_pos = self._get_cones_position()
        state = yellow_cones_pos + blue_cones_pos + orange_cones_pos + self.car_state
        return state      
    
    def _global_to_car_frame(self, pos, car_pos, car_orientation):
        # pos: [x, y, z]
        # car_pos: [x, y, z]
        # car_orientation: [x, y, z, w]
        # return: [x, y, z]
        q = np.quaternion(car_orientation[3], car_orientation[0], car_orientation[1], car_orientation[2])
        if q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w == 0:
            return [np.inf, np.inf, np.inf]
        pos_car_frame = quaternion.rotate_vectors(q, np.array(pos) - np.array(car_pos))
        return pos_car_frame
    
    def _average_cone_pos(self, orange_cones_pos_car_frame):
        # orange_cones_pos_car_frame: [[x, y, z], [x, y, z], [x, y, z], [x, y, z]]
        # return: [[x, y, z], [x,y,z]]
        average_cone_pos_left = [0, 0, 0]
        average_cone_pos_right = [0, 0, 0]
        
        if len(self.orange_cones) < 4:
            return [average_cone_pos_left, average_cone_pos_right]
        
        if orange_cones_pos_car_frame[0][1] == INF or \
            orange_cones_pos_car_frame[1][1] == INF or \
            orange_cones_pos_car_frame[2][1] == INF or \
            orange_cones_pos_car_frame[3][1] == INF :
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
        if cone_1[0] == INF or cone_2[0] == INF:
            return True        
        return (cone_1[0] - car_pos[0]) * (cone_2[1] - car_pos[1]) - (cone_2[0] - car_pos[0]) * (cone_1[1] - car_pos[1]) > 0

    def _is_right_side(self, cone_1, cone_2, car_pos):
        if cone_1[0] == INF or cone_2[0] == INF:
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
        
        time = self.get_clock().now().nanoseconds
        
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
        
        # Penilize null velocity
        if car_vel[0] == 0:
            reward -= 10
             
        # Check acceleration jerk
        previous_acc = self.previous_state[35:38]
        
        reward -= 5 * np.linalg.norm(np.array(car_acc) - np.array(previous_acc))
       
        # Update previous state  
        self.previous_time = time
        self.previous_state = state
        self.previous_orange_cones_pos = [org_cones_pos_left, org_cones_pos_right]
        
        return reward
        
    
    def step(self, action):
        info = {"simulated_time": self.get_clock().now().nanoseconds}
        self.i += 1
        done = False
        
        # TODO:
        # Use the action to control the car
        # Delay to get the new state
        # Get the new state 
        # Calculate the reward
        # Check if the episode is done
        # Return the state, reward, done and info
        
        self._publish_action(action)
        self.get_logger().info("Action: {}".format(action))
        # TODO: fix the delay
        time.sleep(0.1)
        self.get_logger().info("Step: {}".format(self.i))
        state = self._get_state()
        self.reward = self._get_reward(state)
                
        #TODO: Check if a Lap has been completed
        
        self.previous_state = state
         
        return state, self.reward, done, info

    def reset(self):
        # Implement the reset function
        # TODO: Reset the car position and orientation
        self._resetVehiclePos()
        self._resetConePos()
        self.i = 0
             
        self.previous_state = self.nb_car_states*[0.0]
        # TODO: fix the delay
        time.sleep(0.1)
         
        self.previous_orange_cones_pos = self._average_cone_pos(self.orange_cones)
        self.previous_time = self.get_clock().now().nanoseconds
        self.init_time = self.previous_time
        self.lap_time = self.previous_time
        state = self._get_state()
        return [state]
    
    
def main(args=None):
    rclpy.init(args=args)
    env = EufsEnv()
    rclpy.spin(env)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    