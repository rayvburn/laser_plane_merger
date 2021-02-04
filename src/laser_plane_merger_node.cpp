#include <ros/ros.h>
#include <laser_plane_merger/LaserPlaneMerger.hpp>

int main(int argc, char **argv) {
	ros::init(argc, argv, "laser_plane_merger");
	LaserPlaneMerger laser_plane_merger;

	ros::spin();
	return 0;
}
