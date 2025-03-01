#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "assignment_1/obstacle_msg.h"
#include <cmath>
#include <vector>

// Class for representing a point in Cartesian coordinates
class CartesianPoint {
private:
    float x_;
    float y_;
public:
    CartesianPoint(float x, float y){
 		x_=x; 
		y_=y;
	}
    float x() const { return x_; }
    float y() const { return y_; }
};

// Global vectors for storing obstacle centers
std::vector<float> obstacle_x_centers;
std::vector<float> obstacle_y_centers;

// Function to verify if all points in a block are on or close to a given circle
bool isWithinCircle(const std::vector<CartesianPoint>& points, const CartesianPoint& center, float radius) {
    int discardedCount = 0;
    const float tolerance = 0.02; // Allowed deviation from the radius
    const int maxDiscarded = 1;

    for (const auto& point : points) {
        float distance = std::hypot(center.x() - point.x(), center.y() - point.y());
        if (distance < radius - tolerance || distance > radius + tolerance) {
            if (++discardedCount > maxDiscarded) {
                return false; // Too many points outside the circle
            }
        }
    }
    return true; // All points are acceptable
}

// Function to determine if a circle can be inscribed within a block of points
void detectCircle(const std::vector<CartesianPoint>& points, float minRadius, float maxRadius) {
    if (points.size() < 3) return; // Need at least 3 points

    CartesianPoint first = points.front();
    CartesianPoint middle = points[points.size() / 2];
    CartesianPoint last = points.back();

    // Calculate midpoints
    CartesianPoint mid1((first.x() + middle.x()) / 2, (first.y() + middle.y()) / 2);
    CartesianPoint mid2((middle.x() + last.x()) / 2, (middle.y() + last.y()) / 2);

    // Calculate slopes
    float slope1 = (middle.y() - first.y()) / (middle.x() - first.x());
    float slope2 = (last.y() - middle.y()) / (last.x() - middle.x());

    // Adjust for vertical slopes
    if (std::isinf(slope1)) slope1 = -slope2;
    else if (std::isinf(slope2)) slope2 = -slope1;
    else {
        slope1 = -1 / slope1;
        slope2 = -1 / slope2;
    }

    // Solve for the intersection point
    float x_center = ((mid2.y() - mid1.y()) - (slope2 * mid2.x() - slope1 * mid1.x())) / (slope1 - slope2);
    float y_center = slope1 * (x_center - mid1.x()) + mid1.y();

    // Calculate radius
    float radius = std::hypot(x_center - first.x(), y_center - first.y());

    // Validate the radius
    if (radius >= minRadius && radius <= maxRadius) {
        CartesianPoint center(x_center, y_center);
        
        // Check for concavity
        if (std::hypot(middle.x(), middle.y()) < std::min(std::hypot(first.x(), first.y()), std::hypot(last.x(), last.y()))) {
            return; // Skip concave circles
        }
        
        // Validate if points are on the circle
        if (isWithinCircle(points, center, radius)) {
            ROS_INFO("Detected obstacle at [%f, %f]", center.x(), center.y());
            obstacle_x_centers.push_back(center.x());
            obstacle_y_centers.push_back(center.y());
        }
    }
}

// Callback function for processing LaserScan messages
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    int numPoints = static_cast<int>((scan->angle_max - scan->angle_min) / scan->angle_increment);
    std::vector<CartesianPoint> pointCloud;
    std::vector<std::vector<CartesianPoint>> pointBlocks(numPoints);
    const int blindSpotMargin = 20;
    const float gapThreshold = 0.1;
    const float minRadius = 0.17;
    const float maxRadius = 0.19;
    
    obstacle_x_centers.clear();
    obstacle_y_centers.clear();

    // Populate points from LaserScan ranges
    int blockIndex = 0;
    for (int i = blindSpotMargin; i < numPoints - blindSpotMargin - 1; ++i) {
        if (scan->ranges[i] > 0 && scan->ranges[i] <= scan->range_max) {
            float x = scan->ranges[i] * cos(scan->angle_min + i * scan->angle_increment);
            float y = scan->ranges[i] * sin(scan->angle_min + i * scan->angle_increment);
            CartesianPoint point(x, y);
            pointBlocks[blockIndex].push_back(point);
            if (std::abs(scan->ranges[i] - scan->ranges[i + 1]) >= gapThreshold) {
                blockIndex++;
            }
        }
    }

    // Process each block of points
    for (int i = 0; i <= blockIndex; ++i) {
        if (pointBlocks[i].size() >= 3) {
            detectCircle(pointBlocks[i], minRadius, maxRadius);
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "laser_scanner_node");
    ros::NodeHandle nh;
    
    ros::Subscriber laserSubscriber = nh.subscribe("/scan_raw", 1000, laserScanCallback);
    ros::Publisher obstaclePublisher = nh.advertise<assignment_1::obstacle_msg>("scanned_obstacles", 1000);
    
    ros::Rate loopRate(10);
    while (ros::ok()) {
        assignment_1::obstacle_msg msg;
        msg.obstacles_x = obstacle_x_centers;
        msg.obstacles_y = obstacle_y_centers;
        obstaclePublisher.publish(msg);
        ros::spinOnce();
        loopRate.sleep();
    }
    
    return 0;
}
