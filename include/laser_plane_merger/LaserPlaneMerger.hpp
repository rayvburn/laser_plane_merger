/*
 * LaserPlaneMerger.hpp
 *
 *  Created on: Feb 4, 2021
 *      Author: rayvburn
 */

#ifndef INCLUDE_LASER_PLANE_MERGER_LASERPLANEMERGER_HPP_
#define INCLUDE_LASER_PLANE_MERGER_LASERPLANEMERGER_HPP_

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/LaserScan.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>

#include <mutex>

class LaserPlaneMerger {
public:
	LaserPlaneMerger();
	virtual ~LaserPlaneMerger() = default;

private:
	void scansCallback(
		const sensor_msgs::LaserScanConstPtr &scan_main,
		const sensor_msgs::LaserScanConstPtr &scan_aux
	);
	geometry_msgs::Point32 findGlobalPosition(
		const geometry_msgs::TransformStamped &pose_ref,
		const double &range,
		const double &angle
	);
	double findScanRange(
		const geometry_msgs::TransformStamped &pose_ref,
		geometry_msgs::Point32 &pos_obs,
		const double &angle
	);
	unsigned int findAngleIndex(
		const double &angle,
		const double &angle_min,
		const double &angle_max,
		const double &angle_inc
	);

	ros::NodeHandle nh_;

	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;
	tf2_ros::TransformBroadcaster tf_broadcaster_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicy;
	message_filters::Subscriber<sensor_msgs::LaserScan>* sub_scan_main_ptr_;
	message_filters::Subscriber<sensor_msgs::LaserScan>* sub_scan_aux_ptr_;
	message_filters::Synchronizer<SyncPolicy>* sync_ptr_;

	ros::Publisher pub_scan_;
	ros::Publisher pub_marker_;
};

#endif /* INCLUDE_LASER_PLANE_MERGER_LASERPLANEMERGER_HPP_ */
