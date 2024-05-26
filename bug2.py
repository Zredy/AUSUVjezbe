import math
import rclpy
from rclpy.node import Node
import tf_transformations as transform
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry

class Bug2(Node):
    #Initialization
    def __init__(self):
        super().__init__('bug0')
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.location_callback, 10)
        self.fl_sensor_sub = self.create_subscription(Range, '/fl_range_sensor', self.fl_sensor_callback, 10)
        self.fr_sensor_sub = self.create_subscription(Range, '/fr_range_sensor', self.fr_sensor_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.bug_algorithm_timer = self.create_timer(0.1, self.bug_algorithm_callback)
        self.goal_x = None
        self.goal_y = None
        self.current_x = None
        self.current_y = None
        self.current_theta = None
        self.fl_sensor_value = 0.0
        self.fr_sensor_value = 0.0
        self.cmd_vel_msg = Twist()
        self.state = 'head_goal'
        self.d1 = 0.55 # wall detection distance
        self.d2 = 0.48 # wall follow sensor distance
        self.wall_turn_dir = 1 # 1 turn left, -1 turn right
        self.dist_to_goal = 0
        self.fast_turn = 0.5
        self.avoid_dist = 0.5
        self.closest_x = 0
        self.closest_y = 0
        self.closest_dist = 999999
        self.start_fw_dist = 999999
        self.goal_line_coef = None
        self.next_exit = False

        self.bug_level = 'bug2'

    #Method for goal update
    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.state = 'head_goal'

    #Method for robot current position
    def location_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
        self.current_theta = transform.euler_from_quaternion(q)[2] #[-pi, pi]

    #Methods for updating sensor values
    def fl_sensor_callback(self, msg):
        self.fl_sensor_value_prev = self.fl_sensor_value
        self.fl_sensor_value = msg.range
        

    def fr_sensor_callback(self, msg):
        self.fr_sensor_value_prev = self.fr_sensor_value
        self.fr_sensor_value = msg.range
        

    def bug_algorithm_callback(self):
        if self.goal_x == None:
            return
        
        print("Current X: " , self.current_x)
        print("Current Y: " , self.current_y)
        print("Current theta: " , self.current_theta)
        print("FL Sensor: " , self.fl_sensor_value)
        print("FR Sensor: " , self.fr_sensor_value)
        print("Goal X: " , self.goal_x)
        print("Goal Y: " , self.goal_y)
        print("Closest X: ", self.closest_x)
        print("Closest Y: ", self.closest_y)
        print("Closest dist: ", self.closest_dist)
        print("Goal dist: ", self.dist_to_goal)
        print("Next exit? ", self.next_exit)

        
        self.calculate_vals()
        self.online = self.am_i_lineing()
        if self.dist_to_goal < 0.1:
            return
        msg = Twist()

        if self.state == 'park':
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        if self.state == 'head_goal':
            if self.head_goal() != None:
                msg = self.head_goal()

        if (self.fl_sensor_value < self.d1 
            and self.fr_sensor_value < self.d1 
            and msg.linear.x == 0):

            self.state = 'turn_wall'
            

        if self.state == 'turn_wall':
            if (self.fr_sensor_value < (self.d2 * 1.035)
                and self.fr_sensor_value > (self.d2 * 0.955)
                and self.fl_sensor_value > 0.7
                or abs(self.fr_sensor_value - self.fr_sensor_value_prev) > 1.0):

                self.state = 'follow_wall'
                self.closest_dist = self.dist_to_goal
            else:
                msg.angular.z = self.wall_turn_dir * 0.3
        up_coef = 1.02
        low_coef = 0.97

        if self.state == 'follow_wall':
            #BUG0 LOGIC
            if self.bug_level == 'bug0':
                if abs(self.rel_angle) < 0.2:

                    self.state = 'head_goal'
            
            #BUG1 LOGIC
            if self.bug_level == 'bug1':
                if (self.next_exit
                    and self.check_point_prox(self.closest_x,self.closest_y, 0.04)):

                    self.state = 'head_goal'
                    self.calculate_goal_line()

                if (self.am_i_lineing() 
                    and self.closest_dist < self.dist_to_goal*0.9):
                    self.next_exit = True
                    print("HIT THE START LINE!")

            #BUG2 LOGIC
            if self.bug_level == 'bug2':
                if (self.online == True
                    and self.state == 'follow_wall'
                    and self.next_exit == True
                    and self.dist_to_goal < self.closest_dist):
                    print("HIT THE M LINE!")

                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    
                    self.next_exit = False
                    self.state = 'head_goal'
                    return
                    

            if (self.fr_sensor_value < (self.d2 * up_coef) 
                and self.fr_sensor_value > (self.d2 * low_coef) 
                and self.fl_sensor_value > 0.7):

                msg.linear.x = 0.5
                msg.angular.z = 0.0
            elif(self.fr_sensor_value < (self.d2 * up_coef) ): 
                msg.angular.z = 0.7
                msg.linear.x = 0.3
            elif(self.fr_sensor_value > (self.d2 * low_coef) ): 
                msg.angular.z = -0.7
                msg.linear.x = 0.3
            self.next_exit = True

        #print("Rel Y: ", self.rel_y)
        #print("Rel X: ", self.rel_x)
        print("Rel Ang: ", self.rel_angle)
        #print("Heading: ", self.heading)
        print("State: ", self.state)
        print("On line: ", self.am_i_lineing())
        print("#################################################")

        self.cmd_pub.publish(msg)

    def head_goal(self):
        msg = Twist()
        if(abs(self.dist_to_goal) < 0.08):
            self.state = 'park'

        if abs(self.rel_angle) > 0.14:
            if self.fl_sensor_value < self.avoid_dist and self.fr_sensor_value > self.avoid_dist*1.5: 
                msg.angular.z = -self.fast_turn
                msg.linear.x = 0.2
            elif self.fr_sensor_value < self.avoid_dist and self.fl_sensor_value > self.avoid_dist*1.5:
                msg.angular.z = self.fast_turn
                msg.linear.x = 0.2
            else:
                if self.rel_angle < -math.pi:
                    self.rel_angle = (2*math.pi) + self.rel_angle
                elif self.rel_angle > math.pi:
                    self.rel_angle = (-2*math.pi) + self.rel_angle
                msg.angular.z =  -1.0 * self.rel_angle
                msg.linear.x = 0.0
        elif ((self.fl_sensor_value > self.d1 or 
            self.fr_sensor_value > self.d1) 
            and self.fl_sensor_value >= 0.5*self.d1 
            and self.fr_sensor_value >= 0.5*self.d1):
            #msg.linear.x = min(0.4, max(0, self.dist_to_goal))
            msg.linear.x = 0.3
            if self.fl_sensor_value < self.avoid_dist and self.fr_sensor_value > self.avoid_dist*2:
                msg.angular.z = -self.fast_turn
            elif self.fr_sensor_value < self.avoid_dist and self.fl_sensor_value > self.avoid_dist*2:
                msg.angular.z = self.fast_turn
            else:
                msg.angular.z = 0.0
        else:
            return None
        return msg

    def calculate_vals(self):
        self.dist_to_goal = math.sqrt((self.current_x - self.goal_x)**2 + (self.current_y-self.goal_y)**2);
        
        if self.state == 'follow_wall':
            if (self.dist_to_goal < self.closest_dist
                and self.bug_level == 'bug1'):
                self.closest_dist = self.dist_to_goal
                self.closest_x = self.current_x
                self.closest_y = self.current_y

        self.rel_x = self.current_x-self.goal_x
        self.rel_y = self.current_y-self.goal_y
        self.heading = math.atan2(self.current_y-self.goal_y, self.current_x-self.goal_x)
        self.dir_goal = self.heading / abs(self.heading)
        if self.heading > 0:
           self.heading -= math.pi
        else:
            self.heading += math.pi

        if (self.current_theta < 0 and self.heading > 0) :
            self.rel_angle = self.current_theta - abs(self.heading)
        else:
            self.rel_angle = self.current_theta - self.heading

        if self.goal_line_coef == None:
            self.calculate_goal_line()

    def calculate_goal_line(self):
        '''
        y - y1 = (y2-y1)/(x2-x1) *(x-x1)
        '''
        x1 = self.current_x
        y1 = self.current_y
        x2 = self.goal_x
        y2 = self.goal_y
        self.goal_line_coef = abs(y2 - y1) / abs(x2-x1)
        self.goal_line_off = (-self.goal_line_coef * x1) + y1

    def am_i_lineing(self):
        #loša fora
        y = self.current_y
        x = self.current_x
        k = self.goal_line_coef
        c = self.goal_line_off
        val = ((k*x) + c) / y
        if ((abs(val)-1) < 0.12
            and (abs(val)-1) > -0.12) :
            return True
        else:
            return False
    
    def check_point_prox(self,ch_x,ch_y,prec):
        a = ch_x / self.current_x
        b = ch_y / self.current_y
        up_val = 1 + prec
        low_val = 1 - prec
        #print("Prox A: ", a)
        #print("Prox B: ", b)
        if (a > low_val and a < up_val 
            and b > low_val and b < up_val):
            return True
        else:
            return False


def main(args=None):

    rclpy.init(args=args)
    bug_node = Bug2()
    rclpy.spin(bug_node)
    bug_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()