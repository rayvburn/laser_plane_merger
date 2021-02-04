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

#include <mutex>

class LaserPlaneMerger {
public:
	LaserPlaneMerger();
	virtual ~LaserPlaneMerger() = default;

private:
	void scansCallback(const sensor_msgs::LaserScanConstPtr &scan_main, const sensor_msgs::LaserScanConstPtr &scan_aux);
	ros::NodeHandle nh_;

	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicy;
	message_filters::Subscriber<sensor_msgs::LaserScan>* sub_scan_main_ptr_;
	message_filters::Subscriber<sensor_msgs::LaserScan>* sub_scan_aux_ptr_;
	message_filters::Synchronizer<SyncPolicy>* sync_ptr_;

	ros::Publisher pub_scan_;
};

#endif /* INCLUDE_LASER_PLANE_MERGER_LASERPLANEMERGER_HPP_ */
