/*
 * LaserPlaneMerger.cpp
 *
 *  Created on: Feb 4, 2021
 *      Author: rayvburn
 */

#include <laser_plane_merger/LaserPlaneMerger.hpp>

LaserPlaneMerger::LaserPlaneMerger():
	tf_listener_(tf_buffer_),
	sub_scan_main_ptr_(nullptr),
	sub_scan_aux_ptr_(nullptr),
	sync_ptr_(nullptr) {
	std::string scan_main_topic_name;
	std::string scan_aux_topic_name;
	std::string scan_merged_topic_name;
	nh_.param<std::string>("scan_main_topic_name", scan_main_topic_name, "/scan1");
	nh_.param<std::string>("scan_aux_topic_name", scan_aux_topic_name, "/scan2");
	nh_.param<std::string>("scan_merged_topic_name", scan_merged_topic_name, "/scan_merged");

	sub_scan_main_ptr_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan_main_topic_name, 10);
	sub_scan_aux_ptr_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan_aux_topic_name, 10);

	sync_ptr_ = new message_filters::Synchronizer<SyncPolicy>(
		SyncPolicy(10),
		*sub_scan_main_ptr_,
		*sub_scan_aux_ptr_
	);

	sync_ptr_->registerCallback(std::bind(
		&LaserPlaneMerger::scansCallback,
		this,
		std::placeholders::_1,
		std::placeholders::_2)
	);

	pub_scan_ = nh_.advertise<sensor_msgs::LaserScan>(scan_merged_topic_name, 5);
}

void LaserPlaneMerger::scansCallback(
	const sensor_msgs::LaserScanConstPtr &scan_main,
	const sensor_msgs::LaserScanConstPtr &scan_aux
) {
	sensor_msgs::LaserScan scan_merged;
	pub_scan_.publish(scan_merged);
	std::cout << "scansCallback" << std::endl;
}
