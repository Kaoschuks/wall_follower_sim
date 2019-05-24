#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    ANGLE_VELOCITY_WEIGHT = 0.8
    MIN_DISTANCE_TO_WALL = 0.75
    MIN_VELOCITY = 0.1

    def __init__(self):
 	self.control_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        self.control_rate = rospy.Rate(20)
        
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan_callback)
        self.control_action = None
        self.lines = None
        self.control_mode = 'ackermann'
  
    def scan_callback(self,laser_scan_data):
	min_angle = laser_scan_data.angle_min
	max_angle = laser_scan_data.angle_max
	angle_increment = laser_scan_data.angle_increment

	ranges = np.array(laser_scan_data.ranges)
	ranges = np.nan_to_num(ranges)
	angles = np.arange(laser_scan_data.angle_min, laser_scan_data.angle_max, laser_scan_data.angle_increment)

	NUMBER_PRIMITIVES = 11
	NUMBER_MEASUREMENTS_PER_RANGE = len(ranges) / NUMBER_PRIMITIVES

	desires = [] # (desire, angle, speed)
	for primitive in range(1,NUMBER_PRIMITIVES-1):
		ranges_area = ranges[NUMBER_MEASUREMENTS_PER_RANGE*(primitive-1):NUMBER_MEASUREMENTS_PER_RANGE*(primitive+2)]
		angles_area = angles[NUMBER_MEASUREMENTS_PER_RANGE*(primitive-1):NUMBER_MEASUREMENTS_PER_RANGE*(primitive+2)]
		danger_min = ranges_area.min()
		if danger_min > self.MIN_DISTANCE_TO_WALL:
			angles_mean = angles_area.mean()
			desire = danger_min - abs(angles_mean)
			print("Area", primitive, danger_min, angles_mean, desire)
			desires.append((desire, angles_mean, self.VELOCITY - self.ANGLE_VELOCITY_WEIGHT*angles_mean))


	desires.sort(reverse=True)

	control_action = AckermannDriveStamped()
        control_action.header.stamp = rospy.rostime.Time().now()
        
	if len(desires) > 0:
		print("Best desire: " , desires[0])
		control_action.drive.speed = max(desires[0][2], self.MIN_VELOCITY)
		control_action.drive.steering_angle = desires[0][1]
        else:
		control_action.drive.speed = self.MIN_VELOCITY
		control_action.drive.steering_angle = 1.0

        self.control_pub.publish(control_action)

	#front_ranges = ranges[40:60]
	#print("Num ranges: " + str(len(ranges)))
	#print("Expected: ", (max_angle-min_angle)/angle_increment, min_angle, max_angle, angle_increment)
	#min_distance = front_ranges.min()
#	print(min_distance)
#        self.lines= self.scan_to_lines(laser_scan_data)
#        self.set_control()
#	control_action = AckermannDriveStamped()
        #set header
 #       control_action.header.stamp = rospy.rostime.Time().now()
            #fixed velocity and heading


#	if min_distance > 3.0:
#        	control_action.drive.speed = self.VELOCITY
#		control_action.drive.steering_angle = 0.0
       # else#
	#	control_action.drive.speed = self.VELOCITY / 4
#		control_action.drive.steering_angle = 1.0
      #  self.control_pub.publish(control_action)

#        self.lines = None
        #self.publish_lines(self.lines, color = [0,0,1])
#        return
   
    def scan_to_lines(self, laser_scan_data):
        ro_points = self.partition_points(laser_scan_data)
        cartesian_points = self.range_to_cartesian(ro_points)
        lines = self.cartesian_to_lines(cartesian_points)
        return lines

    def partition_points(self,laser_scan_data):
        range_orientation_points = np.asarray(laser_scan_data.ranges)
        orientation_points = np.arange(laser_scan_data.angle_min, \
                                        laser_scan_data.angle_max+laser_scan_data.angle_increment/2,\
                                        laser_scan_data.angle_increment)
        range_orientation_points = np.vstack([range_orientation_points, orientation_points]).T
        return self.preprocess_points(laser_scan_data, range_orientation_points)    
        return range_orientation_points
       
    def range_to_cartesian(self,range_orientation_points):
        num_of_points = range_orientation_points.shape[0]
        cartesian_points = np.zeros([num_of_points,2])
        for i, r in enumerate(range_orientation_points[:,0]):
                #calculate x
                cartesian_points[i,0]=r*np.cos(range_orientation_points[i,1])
                #calculate y
                cartesian_points[i,1]=r*np.sin(range_orientation_points[i,1])
        return cartesian_points
    
    def preprocess_points(self, laser_scan_data,range_orientation_points,\
                        max_angle_limit = np.pi, min_angle_limit = np.pi/4,
                        recurse_depth = 0):
        #prune impossible points
        new_rop = []
        for rop in range_orientation_points:
            if self.SIDE==1:
                #only look to the front left
                if rop[1]<=min(max_angle_limit, laser_scan_data.angle_max) and \
                    rop[1] >= -min_angle_limit:
                        new_rop.append(rop)
            elif self.SIDE == -1:
                if rop[1]>=max(-max_angle_limit, laser_scan_data.angle_min) and \
                    rop[1]<= min_angle_limit:
                        new_rop.append(rop)
        '''
        if len(new_rop) < 10 and recurse_depth <1:
            return self.preprocess_points(laser_scan_data,range_orientation_points,\
                        np.pi*2/3, 0, 1) 
        '''
        return np.asarray(new_rop)

    def cartesian_to_lines(self,cartesian_points):
        lines = hough_line_detection(cartesian_points,line_num = 2)
        return lines
  
    def choose_line(self, wall_lines):
        #pick line to follow
        lines_mbx=np.zeros([len(wall_lines),3])
        for i, l in enumerate(wall_lines):
            lines_mbx[i,0]=l[0]
            lines_mbx[i,1]=l[1]
            lines_mbx[i,2]=-l[1]/l[0]
        #print(lines_mbx)
        lines_mbx[lines_mbx[:,2].argsort()] #sort by x-intercept
        closest_line_in_front = None
        closest_line_to_left = None
        closest_line_to_right = None
        for lmbx in lines_mbx:
            
            if lmbx[2]>0:
                if closest_line_in_front is None:
                    closest_line_in_front = lmbx
                elif lmbx[2]<closest_line_in_front[2]:
                    closest_line_in_front = lmbx
            
            #closest line to left
            #print(line_to_point_distance(lmbx[0:2]))
            if line_to_point_distance(lmbx[0:2])>0:
                if closest_line_to_left is None:
                    closest_line_to_left = lmbx
                elif abs(line_to_point_distance(lmbx[0:2]))<abs(line_to_point_distance(closest_line_to_left[0:2])):
                    closest_line_to_left = lmbx
            #closest line to right
            if line_to_point_distance(lmbx[0:2])<0:
                if closest_line_to_right is None:
                    closest_line_to_right = lmbx
                elif abs(line_to_point_distance(lmbx[0:2]))<abs(line_to_point_distance(closest_line_to_right[0:2])):
                    closest_line_to_right = lmbx
        
        #if there is a very close line (obstacle) to the front
        if closest_line_in_front is not None:
            if closest_line_in_front[2]<1.92*0.6+self.DESIRED_DISTANCE:
                #if abs(closest_line_in_front[1])>1e2:
                self.control_mode = 'turn'
                pass
                return closest_line_in_front[0:2]
        
        #if there is a side to follow
        if self.SIDE==1 and closest_line_to_left is not None:
            return closest_line_to_left[0:2]
        elif closest_line_to_right is not None:
            return closest_line_to_right[0:2]   
        else:
            print('no line chosen!')
            self.control_mode = 'turn' 
            return None

    def set_control(self, zero=False):
        wall_lines = self.lines
        if zero or wall_lines is None:
            control_action = AckermannDriveStamped()
            #set header
            control_action.header.stamp = rospy.rostime.Time().now()
            #fixed velocity and heading
            control_action.drive.speed = self.VELOCITY
            self.control_action = control_action
            return True
        #pick line to follow
        wall_line = self.choose_line(wall_lines)
        if wall_line is None:
            return False
        assert(len(wall_line)==2)    #mx+b form
        control_action = AckermannDriveStamped()
        #set header
        control_action.header.stamp = rospy.rostime.Time().now()
        #calculate control action
        control_action.drive.speed = self.VELOCITY
        reference_line = self.compute_reference_line(wall_line)
        self.publish_lines([reference_line], color=[1,0,1])
        if self.control_mode == 'turn':
            control_action.drive.steering_angle = self.turn()
            self.control_mode = 'ackermann'
        else:
            control_action.drive.steering_angle = self.compute_ackermann_action(reference_line)
            #control_action.drive.steering_angle_velocity = 0.03
            #publish
        self.control_action = control_action
        #print(control_action)
        return True
    
    def publish_lines(self, lines, frame='laser', color = [1,1,0]):
        marker_array = []
        for line in lines:
            marker = Marker()
            marker.header.stamp = rospy.rostime.Time().now()
            marker.header.frame_id = frame
            marker.type = Marker.LINE_STRIP
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.
            #compute line
            p1 = Point()
            p1.x = 0.
            p1.y = line[1]
            p2 = Point()
            p2.x = -line[1]/line[0]
            p2.y = 0.
            marker.points.append(p1)
            marker.points.append(p2)
            marker_array.append(marker)
        self.marker_pub.publish(marker_array)

    def compute_reference_line(self, wall_line, select_criterion = 'closest'):
        #given a wall, compute the reference line to follow
        assert(len(wall_line)==2)   #mx+b form
        wall_1 = np.asarray([wall_line[0], \
                    wall_line[1]-self.DESIRED_DISTANCE*np.sqrt(1+wall_line[0]**2)])
        wall_2 = np.asarray([wall_line[0], \
                     wall_line[1]+self.DESIRED_DISTANCE*np.sqrt(1+wall_line[0]**2)])
        if select_criterion == 'closest':
            dist_wall_1 = line_to_point_distance(wall_1)
            dist_wall_2 = line_to_point_distance(wall_2)
            if abs(dist_wall_1) < abs(dist_wall_2):
                return wall_1
            elif abs(dist_wall_2) < abs(dist_wall_1):
                return wall_2
        elif select_criterion == 'intercept':
            #calculate x intercept
            x_int_1 = -wall_1[1]/wall_1[0]
            x_int_2 = -wall_2[1]/wall_2[0]
            if abs(x_int_1)<abs(x_int_2):
                return wall_1
            elif abs(x_int_2)<abs(x_int_1):
                return wall_2
        if self.SIDE == 1:
            #wall is on the left
            return wall_1
        else:
            #wall is on the right
            return wall_2
        
    def compute_pd_action(self, wall_line):
        steering_angle = 0
        dist = line_to_point_distance(wall_line)
        dist_err = dist-self.DESIRED_DISTANCE*self.SIDE
        p_gain = 1
        #print('wl',wall_line)
        #print(dist, dist_err)
        steering_angle += p_gain*dist_err
        steering_angle = max(-self.STEERING_ANGLE,min(self.STEERING_ANGLE,steering_angle))
        return steering_angle

    def compute_ackermann_action(self, reference_line):
        steering_angle = 0
        L1 = self.L1
        L = self.WHEEL_BASE
        d = line_to_point_distance(reference_line)
        phi = np.arctan2(d,L1)
        eta = phi + np.arctan(reference_line[1])
        delta = np.arctan2(2*L*np.sin(eta),L1)
        #delta = max(-self.STEERING_ANGLE,min(self.STEERING_ANGLE,delta))
        return delta
    
    def turn(self):
        print('turning')
        if self.SIDE == -1:
            return self.STEERING_ANGLE
        else:
            return -self.STEERING_ANGLE



if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()

    rospy.spin()
