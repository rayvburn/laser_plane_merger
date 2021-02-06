/*
 * LaserPlaneMerger.cpp
 *
 *  Created on: Feb 4, 2021
 *      Author: rayvburn
 */

#include <laser_plane_merger/LaserPlaneMerger.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <algorithm>
#include <tf/tf.h>

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
	pub_marker_ = nh_.advertise<visualization_msgs::Marker>("/laser_plane_merger_marker", 5);
}

void LaserPlaneMerger::scansCallback(
	const sensor_msgs::LaserScanConstPtr &scan_main,
	const sensor_msgs::LaserScanConstPtr &scan_aux
) {
	visualization_msgs::Marker marker;
	marker.pose.orientation.x = 0.0f;
	marker.pose.orientation.y = 0.0f;
	marker.pose.orientation.z = 0.0f;
	marker.pose.orientation.w = 1.0f;

	marker.scale.x = 0.01;
	marker.scale.y = 0.01;
	marker.scale.z = 0.01;

	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();

	marker.ns = "laser_plane_merger";
	marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::POINTS;
	marker.id = 1;

	marker.color.a = 1.0f;
	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;

	// try to transform to another coordinate system
	geometry_msgs::TransformStamped transform;
	try {
		transform = tf_buffer_.lookupTransform(
			scan_main->header.frame_id,
			scan_aux->header.frame_id,
			ros::Time::now(),
			ros::Duration(1.0)
		);
	} catch (tf2::TransformException &e) {
		ROS_WARN("exception: %s", e.what());
		return;
	}

	geometry_msgs::TransformStamped transform_glob_aux_pose;
	try {
		transform_glob_aux_pose = tf_buffer_.lookupTransform(
			"map", // scan_aux->header.frame_id,
			scan_aux->header.frame_id, // "map",
			ros::Time::now(),
			ros::Duration(1.0)
		);
	} catch (tf2::TransformException &e) {
		ROS_WARN("exception: %s", e.what());
		return;
	}

	geometry_msgs::TransformStamped transform_glob_main_pose;
	try {
		transform_glob_main_pose = tf_buffer_.lookupTransform(
			scan_main->header.frame_id,
			"map",
			ros::Time::now(),
			ros::Duration(1.0)
		);
	} catch (tf2::TransformException &e) {
		ROS_WARN("exception: %s", e.what());
		return;
	}

	sensor_msgs::LaserScan scan_merged;
	scan_merged.header.stamp = ros::Time::now();
	scan_merged.angle_increment = scan_aux->angle_increment;
	scan_merged.angle_min = scan_aux->angle_min;
	scan_merged.angle_max = scan_aux->angle_max;
	scan_merged.range_min = scan_aux->range_min;
	scan_merged.range_max = scan_aux->range_max;
	scan_merged.scan_time = scan_aux->scan_time;
	scan_merged.time_increment = scan_aux->time_increment;
	scan_merged.intensities = scan_aux->intensities;
	// TEMP: publish rgbd scan transformed into the first scan's frame
	scan_merged.header.frame_id = scan_main->header.frame_id;
	for (const auto& range: scan_aux->ranges) {
		scan_merged.ranges.push_back(range);
	}

	float angle = scan_merged.angle_min;
	// vector of obstacle points global positions
	std::vector<geometry_msgs::Point32> pos_obs_aux_global;
	for (const auto& range: scan_aux->ranges) {
		// find obstacle position (auxiliary scan) in global coordinate system
		pos_obs_aux_global.push_back(findGlobalPosition(transform_glob_aux_pose, range, angle));
//		printf("[global position] before - position_ref: %2.5f, %2.5f, %2.5f, orientation: %2.5f, %2.5f, %2.5f, %2.5f, range: %3.6f, angle: %3.6f\r\n",
//			transform_glob_aux_pose.transform.translation.x,
//			transform_glob_aux_pose.transform.translation.y,
//			transform_glob_aux_pose.transform.translation.z,
//			transform_glob_aux_pose.transform.rotation.x,
//			transform_glob_aux_pose.transform.rotation.y,
//			transform_glob_aux_pose.transform.rotation.z,
//			transform_glob_aux_pose.transform.rotation.w,
//			range,
//			angle
//		);
		angle += scan_merged.angle_increment;
	}
//	printf("\r\n");

	// try to match auxiliary scan's obstacle positions to main scan's angle
	angle = scan_main->angle_min;
	// define a tf between scan frame and global coordinate system as KDL vector
	KDL::Vector vector_global_cs_scan(
		transform_glob_main_pose.transform.translation.x,
		transform_glob_main_pose.transform.translation.y,
		transform_glob_main_pose.transform.translation.z
	);


	int i = 0;
	//while (angle <= scan_main->angle_max) {
	for (const auto& pos: pos_obs_aux_global) {
		if (std::isinf(pos.x) || std::isinf(pos.y) || std::isinf(pos.z)) {
			continue;
		}
		geometry_msgs::Point pt;
		pt.x = pos.x;
		pt.y = pos.y;
		pt.z = pos.z;

		marker.points.push_back(pt);

		//std::cout << i++ << ") x = " << pos.x << " y = " << pos.y << " z = " << pos.z << std::endl;

		/*
		KDL::Vector scan_to_pos(
			pos.x - vector_global_cs_scan.x(),
			pos.y - vector_global_cs_scan.y(),
			pos.z - vector_global_cs_scan.z()
		);
		// use normalized
		double angle = std::atan2(scan_to_pos.y(), scan_to_pos.x());
		unsigned int idx = findAngleIndex(angle, scan_main->angle_min, scan_main->angle_max, scan_main->angle_increment);
		std::cout << "angle: " << angle << "  index: " << idx << std::endl;
		// angle += scan_main->angle_increment;
		*/
	}
	//std::cout << "===" << std::endl;

	pub_scan_.publish(scan_merged);
	pub_marker_.publish(marker);
}

geometry_msgs::Point32 LaserPlaneMerger::findGlobalPosition(
	const geometry_msgs::TransformStamped &pose_ref,
	const double &range,
	const double &angle) {
	// http://faculty.salina.k-state.edu/tim/robotics_sg/Pose/pointTrans2d.html#applying-transformations-to-a-point
	// "homogeneous transformation matrix defines rotation followed by translation in the original coordinate frame"
	tf::Transform tf_ref(
		tf::Quaternion(
			pose_ref.transform.rotation.x,
			pose_ref.transform.rotation.y,
			pose_ref.transform.rotation.z,
			pose_ref.transform.rotation.w),
		tf::Vector3(
			pose_ref.transform.translation.x,
			pose_ref.transform.translation.y,
			pose_ref.transform.translation.z)
	);

	auto copy_tf = pose_ref;
	copy_tf.child_frame_id = "testtt";
	tf_broadcaster_.sendTransform(copy_tf);

	// V1
	tf::Quaternion quat_range;
	quat_range.setRPY(0.0, 0.0, angle);
	tf::Transform tf_range_rot(
		tf::Quaternion(quat_range),
		tf::Vector3(0.0, 0.0, 0.0)
	);

	tf::Quaternion quat_tr;
	quat_tr.setRPY(0.0, 0.0, 0.0);
	tf::Transform tf_range(
		tf::Quaternion(quat_tr),
		tf::Vector3(range, 0.0, 0.0)
	);

	tf::Transform tf = tf_ref * tf_range_rot * tf_range;

//	// V2
//	tf::Quaternion quat_tr;
//	quat_tr.setRPY(0.0, 0.0, 0.0);
//	tf::Transform tf_range(
//		tf::Quaternion(quat_tr),
//		tf::Vector3(range, 0.0, 0.0)
//	);
//
//	tf::Transform tf = tf_ref * tf_range;

	geometry_msgs::Point32 pos_global;
	pos_global.x = tf.getOrigin().getX();
	pos_global.y = tf.getOrigin().getY();
	pos_global.z = tf.getOrigin().getZ();
	return pos_global;

	/*
	// tf2::doTransform(person.pose, person.pose, transform);

	KDL::Rotation orient_range;
	orient_range = orient_range.RPY(0.0, 0.0, angle);

	KDL::Rotation orient_pose;
	orient_pose = orient_pose.Quaternion(
		pose_ref.transform.rotation.x,
		pose_ref.transform.rotation.y,
		pose_ref.transform.rotation.z,
		pose_ref.transform.rotation.w
	);

	// V1
	// KDL::Rotation orient_result(orient_pose * orient_range);
	// V2
	double roll1, roll2;
	double pitch1, pitch2;
	double yaw1, yaw2;
	orient_range.GetRPY(roll1, pitch1, yaw1);
//	printf("orient_range: roll = %3.5f, pitch = %3.5f, yaw = %3.5f\r\n",
//		roll1,
//		pitch1,
//		yaw1
//	);
	orient_pose.GetRPY(roll2, pitch2, yaw2);
//	printf("orient_pose: roll = %3.5f, pitch = %3.5f, yaw = %3.5f\r\n",
//		roll2,
//		pitch2,
//		yaw2
//	);
	KDL::Rotation orient_result;
	orient_result = orient_result.RPY(roll1 + roll2, pitch1 + pitch2, yaw1 + yaw2);
//	printf("\r\n");

	// ranges are expressed as lengths relative to X-axis direction
	KDL::Vector vector_range(range, 0.0, 0.0);
	vector_range = orient_result * vector_range;

	KDL::Vector vector_global;
	vector_global = KDL::Vector(
		pose_ref.transform.translation.x,
		pose_ref.transform.translation.y,
		pose_ref.transform.translation.z)
		+ vector_range;

	geometry_msgs::Point32 pos_global;
	pos_global.x = vector_global.x();
	pos_global.y = vector_global.y();
	pos_global.z = vector_global.z();

	return pos_global;
	*/
}

double LaserPlaneMerger::findScanRange(
	const geometry_msgs::TransformStamped &pose_ref,
	geometry_msgs::Point32 &pos_obs,
	const double &angle) {

}

unsigned int LaserPlaneMerger::findAngleIndex(
	const double &angle,
	const double &angle_min,
	const double &angle_max,
	const double &angle_inc) {
	double range = angle_max - angle_min;
	unsigned int indexes = range / angle_inc;
	return static_cast<unsigned int>((angle / range) * indexes);
}
