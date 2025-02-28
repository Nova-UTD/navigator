"""
Package: intersection_manager
File: intersection_manager_node.py
Author: Pranav Boyapati

Node to determine the intersection type and plan behavior accordingly
"""

import rclpy
from rclpy.node import Node
from navigator_msgs.msg import TrafficLightDetection
from nav_msgs.msg import Path
from navigator_msgs.msg import CarlaSpeedometer
from nav_msgs.msg import OccupancyGrid
import math
import time
from navigator_msgs.msg import RoadSignsDetection
from navigator_msgs.msg import AllLaneDetections
from navigator_msgs.msg import IntersectionBehavior

class IntersectionManager(Node):
    def __init__(self):
        super().__init__('intersection_manager')
        
        #Create subscriptions
        self.traffic_light_sub = self.create_subscription(TrafficLightDetection, '/traffic_light/detections', self.trafficLightCallback, 1)
        self.road_sign_sub = self.create_subscription(RoadSignsDetection, '/road_signs/detections', self.roadSignsCallback, 1)
        self.route_sub = self.create_subscription(Path, '/planning/path', self.routeCallback, 1)
        self.speed_sub = self.create_subscription(CarlaSpeedometer, '/speed', self.speedCallback, 1)
        self.current_occupancy_sub = self.create_subscription(OccupancyGrid, '/grid/occupancy/current', self.occupancyCallback, 1)
        self.lane_type_detector_sub = self.create_subscription(AllLaneDetections, '/lane_types/detections', self.laneTypeCallback, 1)

        #Create publisher
        self.behavior_publisher = self.create_publisher(IntersectionBehavior, '/intersection', 10)

        #Create timer for calling navigate_intersection function
        self.create_timer(0.001, self.navigate_intersection)

        #Create variables to store info from subscriptions
        self.traffic_light_status = None
        self.traffic_light_height = None
        self.route = None
        self.behavior_through_intersection = None
        self.speed = None
        self.occupancy_grid = None
        self.occupancy_grid_width = None
        self.occupancy_grid_height = None
        self.occupancy_grid_resolution = None
        self.current_lane_type = None
        self.road_sign_type = None
        self.road_sign_width = None
        self.road_sign_height = None
        self.num_lanes_on_road = None
        self.current_lane = None
        self.all_lanes_available = None
        self.numlanesleftofcurrent = None

        self.numTimesStraightChecked = None
        self.numTimesLeftChecked = None
        self.numTimesRightChecked = None


    def laneTypeCallback(self, msg: AllLaneDetections):
        self.num_lanes_on_road = msg.lane_detections[-1].totallanecount
        self.current_lane = msg.lane_detections[-1].currentlane
        self.all_lanes_available = msg.lane_detections[-1].alllanes
        self.lanes_to_left_of_current = msg.lane_detections[-1].numlanesleftofcurrent


    def occupancyCallback(self, msg: OccupancyGrid):
        self.occupancy_grid = msg.data
        self.occupancy_grid_width = msg.info.width
        self.occupancy_grid_resolution = msg.info.resolution
        self.occupancy_grid_height = msg.info.height


    def trafficLightCallback(self, msg: TrafficLightDetection):
        self.traffic_light_status = msg.traffic_lights[-1].label
        self.traffic_light_height = msg.traffic_lights[-1].height


    def roadSignsCallback(self, msg: RoadSignsDetection): 
        self.road_sign_type = msg.road_signs[-1].label
        self.road_sign_width = msg.road_signs[-1].width
        self.road_sign_height = msg.road_signs[-1].height
    
    
    def routeCallback(self, msg: Path):
        self.route = msg.poses

        current_pose_x = self.route[0].pose.position.x
        for i in range(10, 110, 1):
            future_pose_x = self.route[i].pose.position.x
            difference_x = future_pose_x - current_pose_x

            if (difference_x > 0):
                self.behavior_through_intersection = "RightTurn"
            elif (difference_x < 0):
                self.behavior_through_intersection = "LeftTurn"
            else:
                self.behavior_through_intersection = "Straight"


    def speedCallback(self, msg: CarlaSpeedometer):
        self.speed = msg.speed


    def classify_intersection_type(self):
        if (self.traffic_light_status != None):
            self.traffic_light_status = None
            return "TrafficLight"
        elif (self.road_sign_type == "stop"):
            self.road_sign_type = None
            return "StopSign"
        elif (self.road_sign_type == "warning"):
            self.road_sign_type = None
            roundabout = True
            distanceOffset = (90 + (self.num_lanes_on_road * 3.5)) / self.occupancy_grid_resolution * self.occupancy_grid_width
            for i in range(0, 5):
                boxToCheck = distanceOffset + (i * (self.occupancy_grid_width + (0.25 / self.occupancy_grid_resolution)))
                boxToCheckOccupancy = self.occupancy_grid[boxToCheck]
                if (boxToCheckOccupancy == 0):
                    roundabout = False
                    break
            if (roundabout):
                return "Roundabout"
        else:
            return None


    def navigate_traffic_light_intersection(self):
        status_message = None

        if (self.traffic_light_status == 0):
            speedFPS = self.speed * 5280 / 60 / 60
            timeToStop = speedFPS / 15
            brakingDistance = 0.5 * 15 * timeToStop * timeToStop

            distanceTointersection = ((36 / self.traffic_light_height) / 12) - 100

            if (distanceTointersection > brakingDistance):
                status_message = "KeepDriving"
            else:
                status_message = "BrakeAndStop"
        
        elif (self.traffic_light_status == 1):
            speedFPS = self.speed * 5280 / 60 / 60
            timeToStop = speedFPS / 15
            minBrakingDistance = 0.5 * 15 * timeToStop * timeToStop

            distanceTointersection = ((36 / self.traffic_light_height) / 12) - 100

            if (distanceTointersection > minBrakingDistance):
                status_message = "BrakeAndStop"
            else:
                status_message = "KeepDriving"
        
        elif (self.traffic_light_status == 2):
            status_message = "KeepDriving"

        return status_message


    def navigate_stop_sign_intersection(self):
        status_message = None
        
        if (self.speed != 0):
            speedFPS = self.speed * 5280 / 60 / 60
            timeToStop = speedFPS / 15
            brakingDistance = 0.5 * 15 * timeToStop * timeToStop

            distanceTointersection = 1 - (24.57 * math.log(self.road_sign_width / 84))

            if (distanceTointersection > brakingDistance):
                status_message = "KeepDriving"
            else:
                status_message = "BrakeAndStop"
        
        elif (self.speed == 0):
            iterations = 0
            countCarsBefore = 0
            isLeftOccupied = None
            isRightOccupied = None
            isStraightOccupied = None

            if (iterations == 0):
                numStraightLanes = 0
                for lane in self.all_lanes_available:
                    if (lane == "Straight"):
                        numStraightLanes += 1
                distCheckOccupancyLeft = ((numStraightLanes + self.numlanesleftofcurrent) * 3) + 0.5
                xCoorCheckLeft = math.ceil(distCheckOccupancyLeft / self.occupancy_grid_resolution)
                for i in range(1, len(self.all_lanes_available) + 1):
                    posToCheck = self.occupancy_grid[(self.occupancy_grid_width * 3 / self.occupancy_grid_resolution * i) + xCoorCheckLeft]
                    if (posToCheck == 1):
                        isLeftOccupied = True
                        countCarsBefore += 1
                        break

                distCheckOccupancyRight = ((len(self.all_lanes_available) - self.numlanesleftofcurrent) * 3) + 0.5
                xCoorCheckRight = math.ceil(distCheckOccupancyRight / self.occupancy_grid_resolution)
                for i in range(1, len(self.all_lanes_available) + 1):
                    posToCheck = self.occupancy_grid[(self.occupancy_grid_width * 3 / self.occupancy_grid_resolution * i) + xCoorCheckRight]
                    if (posToCheck == 1):
                        isRightOccupied = True
                        countCarsBefore += 1
                        break

                distCheckOccupancyForward = (len(self.all_lanes_available) * 3) + 0.5
                yCoorCheckForward = math.ceil(distCheckOccupancyForward / self.occupancy_grid_resolution)
                for i in range(1, len(self.all_lanes_available) + 1):
                    posToCheck = self.occupancy_grid[(self.occupancy_grid_width * self.occupancy_grid_height) - (len(self.all_lanes_available) + 1 * self.occupancy_grid_width * 3 / self.occupancy_grid_resolution) + yCoorCheckForward]
                    if (posToCheck == 1):
                        isStraightOccupied = True
                        countCarsBefore += 1
                        break

                iterations += 1

            if (isLeftOccupied):
                self.numTimesLeftChecked += 1
                numStraightLanes = 0
                for lane in self.all_lanes_available:
                    if (lane == "Straight"):
                        numStraightLanes += 1
                distCheckOccupancyLeft = ((numStraightLanes + self.numlanesleftofcurrent) * 3) + 0.5
                xCoorCheckLeft = math.ceil(distCheckOccupancyLeft / self.occupancy_grid_resolution) + 5
                for i in range(1, len(self.all_lanes_available) + 1):
                    posToCheck = self.occupancy_grid[(self.occupancy_grid_width * 3 / self.occupancy_grid_resolution * i) + xCoorCheckLeft]
                    if (posToCheck == 1):
                        break
                if (i == len(self.all_lanes_available)):
                    isLeftOccupied = False
                    countCarsBefore -= 1

            if (isRightOccupied):
                self.numTimesRightChecked += 1
                distCheckOccupancyRight = ((len(self.all_lanes_available) - self.numlanesleftofcurrent) * 3) + 0.5
                xCoorCheckRight = math.ceil(distCheckOccupancyRight / self.occupancy_grid_resolution) - 5
                for i in range(1, len(self.all_lanes_available) + 1):
                    posToCheck = self.occupancy_grid[(self.occupancy_grid_width * 3 / self.occupancy_grid_resolution * i) + xCoorCheckRight]
                    if (posToCheck == 1):
                        break
                if (i == len(self.all_lanes_available)):
                    isRightOccupied = False
                    countCarsBefore -= 1
            
            if (isStraightOccupied):
                self.numTimesStraightChecked += 1
                distCheckOccupancyForward = (len(self.all_lanes_available) * 3) + 0.5
                yCoorCheckForward = math.ceil(distCheckOccupancyForward / self.occupancy_grid_resolution)
                for i in range(1, len(self.all_lanes_available) + 1):
                    posToCheck = self.occupancy_grid[(self.occupancy_grid_width * self.occupancy_grid_height) - (len(self.all_lanes_available) * self.occupancy_grid_width * 3 / self.occupancy_grid_resolution) + yCoorCheckForward]
                    if (posToCheck == 1):
                        break
                if (i == len(self.all_lanes_available)):
                    isStraightOccupied = False
                    countCarsBefore -= 1

            if (self.numTimesLeftChecked >= 2):
                countCarsBefore -= 1
                isLeftOccupied = False
                self.numTimesLeftChecked = 0
            if (self.numTimesRightChecked >= 2):
                countCarsBefore -= 1
                isRightOccupied = False
                self.numTimesRightChecked = 0
            if (self.numTimesStraightChecked >= 2):
                countCarsBefore -= 1
                isStraightOccupied = False
                self.numTimesStraightChecked = 0

            if (countCarsBefore == 0):
                time.sleep(3)
                status_message = "ProceedThroughIntersection"
                iterations = 0
                self.numTimesLeftChecked = 0
                self.numTimesRightChecked = 0
                self.numTimesStraightChecked = 0
            else:
                status_message = "WaitForOtherCars"

        return status_message


    def navigate_roundabout(self):
        status_message = None
        
        if (self.speed != 0):
            speedFPS = self.speed * 5280 / 60 / 60
            timeToStop = (speedFPS / 15) - 1.4
            brakingDistance = 0.5 * 15 * timeToStop * timeToStop

            distanceTointersection = 1 - (24.57 * math.log(self.road_sign_width / 68))

            if (distanceTointersection > brakingDistance):
                status_message = "KeepDriving"
            else:
                status_message = "BrakeAndStop"
        
        if (self.speed == 0):
            laneCount = self.num_lanes_on_road

            if (laneCount == 1):
                innerStartPoint = (self.occupancy_grid_width / 2) - (25 / self.occupancy_grid_resolution)
                innerEndPoint = 8.63305 / self.occupancy_grid_resolution
                outerStartPoint = self.occupancy_grid_height - (17 / self.occupancy_grid_resolution)

                for i in range(outerStartPoint, self.occupancy_grid_height):
                    innerStartAdjuster = (i - outerStartPoint) * 18 / (self.occupancy_grid_height - outerStartPoint) / self.occupancy_grid_resolution
                    innerEndAdjuster = (i - outerStartPoint) * 9.69725 / 9.27529 / self.occupancy_grid_resolution
                    for j in range(innerStartPoint + innerStartAdjuster, innerEndPoint + innerEndAdjuster):
                        if (j <= (self.occupancy_grid_width / 2)):
                            pointToCheck = self.occupancy_grid[(i * self.occupancy_grid_width) + j]
                            if (pointToCheck == 1):
                                return "WaitForOtherCars"
                        else:
                            break
                
                status_message = "ProceedThroughIntersection"

            else:
                innerStartPoint = (self.occupancy_grid_width / 2) - (33 / self.occupancy_grid_resolution)
                innerEndPoint = 17.78263 / self.occupancy_grid_resolution
                outerStartPoint = self.occupancy_grid_height - (17 / self.occupancy_grid_resolution)

                for i in range(outerStartPoint, self.occupancy_grid_height):
                    innerStartAdjuster = (i - outerStartPoint) * 22 / (self.occupancy_grid_height - outerStartPoint) / self.occupancy_grid_resolution
                    innerEndAdjuster = (i - outerStartPoint) * 10.50164 / 6.80339 / self.occupancy_grid_resolution
                    for j in range(innerStartPoint + innerStartAdjuster, innerEndPoint + innerEndAdjuster):
                        if (j <= (self.occupancy_grid_width / 2)):
                            pointToCheck = self.occupancy_grid[(i * self.occupancy_grid_width) + j]
                            if (pointToCheck == 1):
                                return "WaitForOtherCars"
                        else:
                            break
                
                status_message = "ProceedThroughIntersection"

        return status_message

    
    #this function is the main control structure
    def navigate_intersection(self):
        controls_message = None

        intersection_type = self.classify_intersection_type()

        if (intersection_type == "TrafficLight"):
            controls_message = self.navigate_traffic_light_intersection()
        elif (intersection_type == "StopSign"):
            controls_message = self.navigate_stop_sign_intersection()
        elif (intersection_type == "Roundabout"):
            controls_message = self.navigate_roundabout()

        intersection_behavior_message = IntersectionBehavior()
        intersection_behavior_message.header.stamp = self.get_clock().now().to_msg()
        intersection_behavior_message.header.frame_id = "camera_frame"

        if (controls_message == "KeepDriving" or controls_message == "ProceedThroughIntersection"):
            intersection_behavior_message.action = "Proceed"
        elif (controls_message == "BrakeAndStop" or controls_message == "WaitForOtherCars"):
            intersection_behavior_message.action = "Wait"
        
        if (intersection_behavior_message.action != ""):
            self.behavior_publisher.publish(intersection_behavior_message)


def main(args=None):
    rclpy.init(args=args)
    intersection_manager = IntersectionManager()
    rclpy.spin(intersection_manager)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
