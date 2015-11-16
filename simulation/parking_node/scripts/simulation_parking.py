#! /usr/bin/env python

## @package parking_node
#  This module allows the tas_car to do the parking task
#
#  The car starts behind two boxes which are located on the left wall with a certain distance to each other.
#  The car decides if that distance is big enough to be able to park into that gap.
#  Laser scanners at the front and at the back of the car are used to locate the boxes
#  and not to touch the boxes during the parking movement

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import tf

## This class sends movement commands to the car
#
#  The car accepts msgs of the type geometry_msgs/Twist
#  which are generated in this class according to the
#  desired movement
class controller:
    
    ## The constructor
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    ## This method moves the car for a specified time
    #  @param speed The movement speed
    #  @param angle The angular speed of the movement
    #  @param time The duration of the movement    
    def moveForTime(self, speed, angle, time):
    	
        counter = 0
        duration = 0.1
    	freq = time/duration
    	
        # generating the message
        twi = Twist()
    	twi.linear.x = speed
    	twi.angular.z = angle
    	
        # sending the message
    	while not rospy.is_shutdown():
    		if counter < freq:
    			self.pub.publish(twi)
    			rospy.sleep(rospy.Duration(duration))
    			counter = counter + 1
    		else:
    			break
        
        # at the end set the values to 0
        # otherwise the car continues moving
        twi.linear.x = 0
        twi.angular.z = 0
        self.pub.publish(twi)
        
    ## This method moves the car once and is used to control it in a high frequency
    #  @param speed The movement speed
    #  @param angle The angular speed of the movement
    def moveOnce(self, speed, angle):
        
        # generating the message
        twi = Twist()
        twi.linear.x = speed
        twi.angular.z = angle
        
        # sending the message once
        self.pub.publish(twi)
        
        # here we do not want to send a msg
        # with 0 movement to stop the car
        # this is done explicitly in the state machine
    
    ## This method does the parking movement after the car is in the right position
    ## Could be used if distance controlled parking is not working
    ## Highly dependent on SOC of battery and on position of the car    
    def parking(self):
    	
        speed = -0.25 #in echt -0.5
        
        #send the message for the first half of the parking movement
        self.moveForTime(speed, -2, 2)
        
        #send the message for the second half of the parking movement    
        self.moveForTime(speed, 2, 2) #in echt 1.5
        
        #avoid rolling against the box in the back    
        self.moveForTime(0.2, 0, 0.3)
    
## This class is the heart of the module
#
#  It receives the data from the laser scanners and provides functions to process the data to extract the
#  positions and orientations of the boxes
class boxEstimator:
    
    ## The constructor
    def __init__(self):
        
        self.frontBoxPositions = list()
        self.frontBoxOrientations = list()
        self.backBoxPositions = list()
        self.backBoxOrientations = list()
        
        self.rangesFront = list()
        self.rangesBack = list()
        
        self.broadcaster = tf.TransformBroadcaster()
        
        self.front_scanner = rospy.Subscriber("/scan", LaserScan, self.callback_frontLaser)
        self.back_scanner = rospy.Subscriber("/scan_back", LaserScan, self.callback_backLaser)
        
        
    ## @var frontBoxPositions
    #  contains the positions of the boxes seen in the front scan
    
    ## @var frontBoxOrientations
    #  contains the orientations of the boxes seen in the front scan
    
    ## @var backBoxPositions
    #  contains the positions of the boxes seen in the back scan
    
    ## @var backBoxOrientations
    #  contains the orientations of the boxes seen in the back scan
    
    ## @var rangesFront
    #  contains the recent ranges of the front laser scanner
    
    ## @var rangesBack
    #  contains the recent ranges of the back laser scanner
    
    ## This method is called when a message from the front scanner is received
    ## It stores the range data to the global list so that other methods have access to the data
    #  @param data The msg of the front scanner 
    def callback_frontLaser(self, data):
        self.rangesFront = data.ranges
        
    ## This method is called when a message from the back scanner is received
    ## It stores the range data to the global list so that other methods have access to the data
    #  @param data The msg of the back scanner    
    def callback_backLaser(self, data):
        self.rangesBack = data.ranges
    
    ## This method can be used for debugging when you want to see the boxes in rviz
    #  @param boxPositions A list containing the positions of the boxes
    #  @param boxOrientations A list containing the Orientations of the boxes
    #  @param isFront A flag to determine where the other params belong to     
    def broadcastBoxesInTf(self, boxPositions, boxOrientations, isFront=True):
          # loop over the box poses and publish them to tf
          for x in range(len(boxPositions)):
            rot = boxOrientations[x]
            trans = np.array((boxPositions[x][0], boxPositions[x][1], 0))
            
            self.broadcaster.sendTransform(trans,
                        rot,
                        rospy.Time.now() + rospy.Duration(0),
                        "box"+str(x),
                        "hokuyo_front_link" if isFront else "hokuyo_back_link")     
    
    ## This method returns the minimum range of the recent front scan data
    def minFront(self):
        return min(self.rangesFront)
    
    ##This method returns the minimum range of the recent back scan data
    def minBack(self):
        return min(self.rangesBack)
    
    ##This method returns a range value which is captured on the mid left of the front scanner
    ##Useful to make sure that the range corresponds to the box in the front whereas the range value of the exact mid might miss the box
    def midLeftFront(self):
        return self.rangesFront[len(self.rangesFront)*3/5]
    
    ##This method returns a range value which is captured on the mid right of the back scanner
    ##Useful to make sure that the range corresponds to the box in the back whereas the range value of the exact mid might miss the box
    def midRightBack(self):
        return self.rangesBack[len(self.rangesBack)*2/5] 
    
    ## This method extracts the boxes from the scan data by:
    ## - grouping points within a certain euclidean distance to segments
    ## - selecting segments with a certain number of points
    ## - fitting lines into these segments
    ## - selecting segments with:
    ##   - 3 lines
    ##   - 2 angles of 90 degree drawn by line 1 and 2 and line 2 and 3
    ##   - 30cm length of line 2 (length of long edge of the box)
    #  @param isFront A flag to determine if the data is from front or back scanner
    #  @param start An index to constrain the start of the scan data
    #  @param end An index to constrain the end of the scan data
    #  @param angular_range The view angle of the scanner
    #  @param data The data of the scanner    
    def estimateBoxesFromScanData(self, isFront, start, end, angular_range, data):
        
        # stores indices where the scan data needs to be separated to form segments
        segList = list()
        segList.append(0)
        
        xCordList = list()
        yCordList = list()
        
        xCordsOfSegments = list()
        yCordsOfSegments = list()
        
        xCordsOfRemainingSegments = list()
        yCordsOfRemainingSegments = list()
        
        lineStartPointList = list()
        lineEndPointList = list()
        
        
        start_angle = -angular_range/2
        angle_step = angular_range/(len(data))
        
        
        L = data
        
        # getting the euclidean coordinates of the scan data
        for x in range(start-1, end-1, 1):
            xCordList.append(L[x]*math.cos(start_angle + x*angle_step))
            yCordList.append(L[x]*math.sin(start_angle + x*angle_step))
        
        # grouping the points within a certain euclidean distance into segments
        # the index where the scan data needs to be split is stored into segList
        for x in range(len(xCordList) - 1):
            if abs(math.sqrt(math.pow(xCordList[x], 2) + math.pow(yCordList[x], 2)) - math.sqrt(math.pow(xCordList[x+1], 2) + math.pow(yCordList[x+1], 2))) > 0.1:
                segList.append(x)
        
        segList.append(len(xCordList) - 1)
        
        # store the segmented points into lists
        for x in range(len(segList) - 1):
            x_array = list()
            y_array = list()
            for y in range(segList[x] + 1, segList[x+1], 1): #TODO +1 entfernen
                x_array.append(xCordList[y])
                y_array.append(yCordList[y])
            xCordsOfSegments.append(x_array)
            yCordsOfSegments.append(y_array)
        
        # select only segments with a certain number of points
        for x in range(len(xCordsOfSegments)):
            if len(xCordsOfSegments[x]) >= 10:
                xCordsOfRemainingSegments.append(xCordsOfSegments[x])
                yCordsOfRemainingSegments.append(yCordsOfSegments[x])



        # fit lines into the remaining segments and obtain a lists with starting points and ending points of the number of lines of each segment
        for x in range(len(xCordsOfRemainingSegments)):
            helper = list()
            for y in range(len(xCordsOfRemainingSegments[x])):
                helper_ = np.array((xCordsOfRemainingSegments[x][y], yCordsOfRemainingSegments[x][y]))
                helper.append(helper_)
            result = self.lineFitter(helper)
            lineStartPointList.append(result[0])
            lineEndPointList.append(result[1])
        
        boxPositions = list()       #reset
        boxOrientations = list()
        
        # process the lines of each remaining segment and select those segments:
        # - only take segments which have 3 lines
        # - line 1 and 2 as line 2 and 3 must draw an angle of about 90 degree
        # - line 2 must be about 30cm long
        # calculate the position and orientation of the selected segments and store them into the class member lists
        # those lists are used by boxLogic to do some logic with the boxes
        for x in range(len(xCordsOfRemainingSegments)):
            if len(lineEndPointList[x]) != 3:
                continue
            flag = 1
            for y in range(len(lineEndPointList[x]) - 1):
                line1 = np.array(lineEndPointList[x][y]) - np.array(lineStartPointList[x][y])
                line2 = np.array(lineEndPointList[x][y+1]) - np.array(lineStartPointList[x][y+1])
           
                angle = math.acos(np.dot(line1, line2) / (np.linalg.norm(line1) * np.linalg.norm(line2)))
                
                if angle < 1.84 and angle > 1.30:
                    flag = flag*1
                else:
                    flag = flag*0
            edge = np.array(lineEndPointList[x][1]) - np.array(lineStartPointList[x][1])
            edge_length = np.linalg.norm(edge)        
            if flag == 1 and (edge_length > 0.28 and edge_length < 0.35):
                helper = list()
                helper.append(lineStartPointList[x][2])
                boxPositions.append(helper)
                
                slope = (lineStartPointList[x][1][1] - lineEndPointList[x][1][1])/(lineStartPointList[x][1][0] - lineEndPointList[x][1][0])
                
                angle = math.atan(slope)
                sign = 0
                if angle < 0:
                    sign = 1
                    
                if sign == 0:
                    test = -math.pi/2
                else:
                    test = math.pi/2
                    
                boxOrientations.append((1)*(angle + test))
        
        if isFront:       
            self.frontBoxPositions = boxPositions
            self.frontBoxOrientations = boxOrientations
        else:
            self.backBoxPositions = boxPositions
            self.backBoxOrientations = boxOrientations
    
    ## This method is used by estimateBoxesFromScanData in order to fit lines into the segments
    ## If the distance of the points to the line from starting point to end point of the segment is not lower than a certain toleranz,
    ## the segment is split, and this function is called recursively again
    ## In the end, this function returns starting and end points of the lines of a segment
    #  @param Seg The points of a segment of the scan data       
    def lineFitter(self, Seg):
        toleranz = 0.05

        a=Seg[0]
        b=Seg[len(Seg) - 1]
        
        counter = 1
        
        bigger_ = list()
        bigger = list()
        
        for x in range(len(Seg)):
            p = Seg[x]
            n = (b-a)/np.linalg.norm(b-a)
            p1q = p - a
            s = a + np.dot(np.dot(n, p1q), n)
            lotDistance = abs(np.linalg.norm(p-s))
            
            
            if lotDistance > toleranz:
                bigger.append(x)
                bigger_.append(lotDistance)      
        
        helper = list()
        
        helper_start = list()
        
        helper_end = list()
        
        if len(bigger) > 0:
            splitPosition = bigger[bigger_.index(max(bigger_))]
            
            dummy = list()
            for x in range(0, splitPosition, 1):
                dummy.append(Seg[x])
            result = self.lineFitter(dummy)
            
            for x in range(len(result[0])):
                helper_start.append(result[0][x])
                helper_end.append(result[1][x])
            
            dummy = list()
            for x in range(splitPosition + 1, len(Seg), 1):
                dummy.append(Seg[x])
            result = self.lineFitter(dummy)
            
            for x in range(len(result[0])):
                helper_start.append(result[0][x])
                helper_end.append(result[1][x])
            
        else:
            helper_start.append(a)
            helper_end.append(b)
                
                
        helper.append(helper_start)
        helper.append(helper_end)        
        return helper
        
                         
    ## This function does some logic with the extracted boxes in order to classify the boxes to box 1 or 2 if there are more than one boxes in the scan
    ## and notify if there is no box in the scan
    ## furthermore the obtained positions and orientations of the boxes are mean values over 3 scans 
    #  @param isFront A flag to determine if we want to use the front scanner or the back scanner
    #  @param onlyOneBoxLeft A flag to disable checking for 2 boxes in a scan   
    def boxLogic(self, isFront, onlyOneBoxLeft=False):
        #if isFront:
            #self.front_scanner = rospy.Subscriber("/scan", LaserScan, self.callback_frontLaser)
        #else:
            #self.back_scanner = rospy.Subscriber("/scan_back", LaserScan, self.callback_backLaser)
        
        counter = 0
        
        pos = list()
        qua = list()
        
        box1position = list()
        box1orientation = list()
        box2position = list()
        box2orientation = list()
        
        while not rospy.is_shutdown():
            
            self.estimateBoxesFromScanData(True, 1, 642, 2.80, self.rangesFront) if isFront else self.estimateBoxesFromScanData(False, 1, 456, 2.80, self.rangesBack)
            
            boxPositions = self.frontBoxPositions if isFront else self.backBoxPositions
            boxOrientations = self.frontBoxOrientations if isFront else self.backBoxOrientations
            
            for x in range(len(boxPositions)):
                distanceToPosition = np.linalg.norm(boxPositions[x][0])
                print distanceToPosition
                if distanceToPosition > 1.0 or onlyOneBoxLeft:
                    box2position.append(boxPositions[x][0])
                    box2orientation.append(boxOrientations[x])
                else:
                    box1position.append(boxPositions[x][0])
                    box1orientation.append(boxOrientations[x])
            if len(box1position) > 3 and len(box2position) > 3:
                mean1position = np.array((0,0,0))
                mean2position = np.array((0,0,0))
                
                mean1orientation = 0
                mean2orientation = 0
                
                for x in range(len(box1position)):
                    mean1position = mean1position + np.array((box1position[x][0], box1position[x][1], 0))
                    mean1orientation = mean1orientation + box1orientation[x]
                    
                for x in range(len(box2position)):
                    mean2position = mean2position + np.array((box2position[x][0], box2position[x][1], 0))
                    mean2orientation = mean2orientation + box2orientation[x]
                    
            
                q1 = tf.transformations.quaternion_from_euler(0,0,mean1orientation/len(box1position))
                q2 = tf.transformations.quaternion_from_euler(0,0,mean2orientation/len(box2position))
                
                p1 = mean1position/len(box1position)
                p2 = mean2position/len(box2position)
            
                pos.append(p1)
                pos.append(p2)
                
                
                qua.append(q1)
                qua.append(q2)
                
                break
                
            elif len(box1position) == 0 and len(box2position) > 3:
                mean2position = np.array((0,0,0))
                mean2orientation = 0
                
                    
                for x in range(len(box2position)):
                    mean2position = mean2position + np.array((box2position[x][0], box2position[x][1], 0))
                    mean2orientation = mean2orientation + box2orientation[x]
                    
            
                q2 = tf.transformations.quaternion_from_euler(0,0,mean2orientation/len(box2position))
                
                p2 = mean2position/len(box2position)
            
                pos.append(p2)
                
                
                qua.append(q2)
                
                break
            elif counter == 8:
                print '100'
                pos.append(0)
                qua.append(0)
                break
            counter = counter + 1
            rospy.sleep(0.05) #war 0.1
        
        #if isFront:   
            #self.front_scanner.unregister()
        #else:
            #self.back_scanner.unregister()
            
        return pos, qua
    

        
                

## The main method
if __name__ == '__main__':
    # creating the node
    rospy.init_node('parkingNode', anonymous=True)
    
    # creating instances of controller and boxEstimator
    c = controller()
    b = boxEstimator()
    rospy.sleep(2)
    
    # setting the speed and time for a movement
    speed = 0.1
    time = 0.3
    
    # the state machine
    # STEP 1: check gap, move forward until box 1 vanishes and only box 2 is left in the front scan
    counter = 0
    while not rospy.is_shutdown():
        pos, qua = b.boxLogic(True, False)

        if len(pos) > 1:
    
            print 'gap:'
            gap = np.linalg.norm(pos[0] - pos[1])
            print gap
            if gap < 1.05:
                print 'gap to small'
            else:
                c.moveOnce(speed, 0)
            
            counter = 0
            
        elif len(pos) == 1 and not type(pos[0]) is int:
            counter = counter + 1
            print counter
            if counter > 2:
                print 'step 2 reached'
                break
        
        elif pos[0] == 0:
            print 'error'
            #c.moveForTime(-0.5, 0, 0.3)
        rospy.sleep(0)
    
    # STEP 2: move forward and control orientation of the car in respect to box 2 until box 2 vanisches in the front scan  
    counter = 0
    while not rospy.is_shutdown():
		pos, qua = b.boxLogic(True, True)

		if len(pos) > 1:
	
			print 'distance:'
			print np.linalg.norm(pos[0] - pos[1])
			#c.moveForTime(speed, 0)
            
			
		elif len(pos) == 1 and not type(pos[0]) is int:
			
			print 'step 2'
			angle = tf.transformations.euler_from_quaternion(qua[0])
			print angle
			
			goalDistanceDeviation = pos[0][1] - 0.55
			
			modifiedAngle = angle[2]
			
			print goalDistanceDeviation
			
			if goalDistanceDeviation > 0.05:
			  modifiedAngle = 0.15
			elif goalDistanceDeviation < -0.05:
			  modifiedAngle =  -0.15
			
			c.moveOnce(speed, modifiedAngle)
			counter = 0
		
		elif pos[0] == 0:
		  counter = counter + 1
		  print counter
		  if counter > 2:
			    print 'step3 reached'
			    break

		rospy.sleep(0)
        
    # STEP 3: switch to back scan, move forward and control orientation of the car in respect to box 1 and stop after a certain distance to box 1 is reached    
    while not rospy.is_shutdown():
		pos, qua = b.boxLogic(False, True)
		if len(pos) > 1:
	
			print 'received 2 boxes'  #not possible because onlyOneBoxLeft is set to true
			
		elif len(pos) == 1 and not type(pos[0]) is int:
			
			print 'step 3'
			angle = tf.transformations.euler_from_quaternion(qua[0])
			print angle
            #c.moveOnce(speed, angle[2])
			#c.moveForTime(speed, angle[2], time)
			c.moveOnce(speed, angle[2])
            
			print pos
			if pos[0][0] > 0.85:
				print 'step 4 reached'
				print pos[0][0]
				break
		
		elif pos[0] == 0:
			print 'step3 error'

		rospy.sleep(0)
    
    # STEP 4: do parking
    c.moveOnce(-0.1, 0)
    #rospy.sleep(0)
    c.moveOnce(0, 0)
    #rospy.sleep(0)
    
    
    
    
    
    #c.parking()
    speed = -0.1 #in echt -0.5
    while not rospy.is_shutdown():
        pos, qua = b.boxLogic(False, True)
        if len(pos) == 1 and not type(pos[0]) is int:
            c.moveOnce(speed, -2)
            b.broadcastBoxesInTf(pos, qua, False)
            #c.moveOnce(0, 0)
            orientation = tf.transformations.euler_from_quaternion(qua[0])
            print orientation
            if orientation[2] > 0.65:
                break
        else:
            break
        rospy.sleep(0.05)	
    
    c.moveOnce(-0.1, 0)
    #rospy.sleep(0)
    c.moveOnce(0, 0)
    #rospy.sleep(0)
    
    while not rospy.is_shutdown():
        distance = b.minBack()
        print distance
        c.moveOnce(speed, 2)
        if distance < 0.18:
            break
        rospy.sleep(0.05)
    
    c.moveForTime(0.1, 0, 0.1)
    #rospy.sleep(0)
    c.moveOnce(0, 0)
    #rospy.sleep(0)
        
    while not rospy.is_shutdown():
        distance = b.minFront()
        print distance
        c.moveOnce(-speed, 2)
        if distance < 0.15:
            break
        rospy.sleep(0.05)
    
    c.moveForTime(-0.1, 0, 0.1)
    #rospy.sleep(0)
    c.moveOnce(0, 0)
    #rospy.sleep(0)
    
    while not rospy.is_shutdown():
        distance = b.minBack()
        print distance
        c.moveOnce(speed, 2)
        if distance < 0.15:
            break
        rospy.sleep(0.05)
        
    c.moveForTime(0.1, 0, 0.1)
    #rospy.sleep(0)
    c.moveOnce(0, 0)
    #rospy.sleep(0)
        
    while not rospy.is_shutdown():
        distance = b.minFront()
        print distance
        c.moveOnce(-speed, 2)
        if distance < 0.15:
            break
        rospy.sleep(0.05)
    
    c.moveForTime(-0.1, 0, 0.1)
    rospy.sleep(0)
    c.moveForTime(0, 0, 0.1)
    rospy.sleep(0)    


    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
