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

// NOTE: free function used for sorting
bool sortByAngle (
	const LaserPlaneMerger::ObstacleScanFrame &a,
	const LaserPlaneMerger::ObstacleScanFrame &b
);

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
	// try to transform to another coordinate system
//	geometry_msgs::TransformStamped transform;
//	try {
//		transform = tf_buffer_.lookupTransform(
//			scan_aux->header.frame_id,
//			scan_main->header.frame_id,
//			ros::Time::now(),
//			ros::Duration(1.0)
//		);
//	} catch (tf2::TransformException &e) {
//		ROS_WARN("exception: %s", e.what());
//		return;
//	}

	geometry_msgs::TransformStamped transform_glob_aux_pose;
	try {
		transform_glob_aux_pose = tf_buffer_.lookupTransform(
			"map",
			scan_aux->header.frame_id,
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
			"map",
			scan_main->header.frame_id,
			ros::Time::now(),
			ros::Duration(1.0)
		);
	} catch (tf2::TransformException &e) {
		ROS_WARN("exception: %s", e.what());
		return;
	}

	// vector of obstacle points global positions:
	// - retrieved from main scan
	auto pos_obs_main_global = computeGlobalPositions(transform_glob_main_pose, *scan_main);
	// - retrieved from auxiliary scan
	auto pos_obs_aux_global = computeGlobalPositions(transform_glob_aux_pose, *scan_aux);

	// find height of resultant scan
	double height = 0.0;
	if (!pos_obs_main_global.empty()) {
		for (const auto &pos: pos_obs_main_global) {
			// find a valid value
			if (std::isinf(pos.x) || std::isinf(pos.y) || std::isinf(pos.z)) {
				continue;
			}
			height = pos.z;
			break;
		}
	}
	// set equal height of scans
	int same_height_occurences = 0;
	for (auto& pos : pos_obs_aux_global) {
		if (pos.z == height) {
			if (same_height_occurences++ >= 5) {
				// abort as it seems to be unnecessary
				ROS_INFO_ONCE("Aborting height equalization. Height is: %4.5f (this is printed only once)", height);
				break;
			}
			continue;
		}
		pos.z = height;
	}

	// publish
	publishGlobalPositionsVisualization(pos_obs_main_global, "scan_main", 0);
	publishGlobalPositionsVisualization(pos_obs_aux_global, "scan_aux", 1);

	// vector of tuples, tuple consists of:
	// - position
	// - double (angle of vector connecting `scan_pos` (MAIN!) with the `pos`)
	// - bool (flag indicating that the obstacle was detected in main scan)
	std::vector<ObstacleScanFrame> pos_obs_global;
	for (auto& pos : pos_obs_main_global) {
		pos_obs_global.push_back(
			ObstacleScanFrame(
				pos,
				computeAngle(transform_glob_main_pose, pos),
				computeEuclideanDistance(transform_glob_main_pose, pos),
				true
			)
		);
	}
	for (auto& pos : pos_obs_aux_global) {
		pos_obs_global.push_back(
			ObstacleScanFrame(
				pos,
				computeAngle(transform_glob_main_pose, pos),
				computeEuclideanDistance(transform_glob_main_pose, pos),
				false
			)
		);
	}
	// sort by vector direction angles
	sort(pos_obs_global.begin(), pos_obs_global.end(), sortByAngle);

	// stores resultant vector of obstacles included in merged scan (compared to pos_obs_global, it has
	// "doubled" obstacles erased
	//std::vector<geometry_msgs::Point32> pos_obs_global_final;
	std::vector<ObstacleScanFrame*> pos_obs_global_final;

	for (auto it = pos_obs_global.begin(); it != pos_obs_global.end(); it++) {
		// evaluate whether obstacle in "main scan" is not detected (Inf) or further than the corresponding
		// obstacle detected in the "auxiliary scan"
		// NOTE: scans are matched heuristically

		bool is_main_scan = it->is_main;

		printf("%ld) angle: %4.5f, is_main: %d",
			it - pos_obs_global.begin(),
			it->angle,
			is_main_scan
		);

		// abort further actions if it's not a main scan
		if (!is_main_scan) {
			printf("\r\n");
			continue;
		}

		// find current "main_scan"'s neighbours (that are main scan's obstacles!)
		auto neighbour_main_up = it;
		auto neighbour_main_down = it;

		// do not perform search if this is first element of the vector
		if (it != pos_obs_global.begin()) {
			while (--neighbour_main_up != pos_obs_global.begin()) {
				// evaluate whether it's obstacle detected in "main scan"
				if (neighbour_main_up->is_main == true) {
					break;
				}
			}
		}
		// do not perform search if this is last element of the vector
		if (it != pos_obs_global.end()) {
			while (++neighbour_main_down != pos_obs_global.end()) {
				// evaluate whether it's obstacle detected in "main scan"
				if (neighbour_main_down->is_main == true) {
					break;
				}
			}
		}

		printf(" | n-bours - up: %ld, down: %ld",
			//it - neighbour_main_up,
			//neighbour_main_down - it
			neighbour_main_up - pos_obs_global.begin(),
			neighbour_main_down - pos_obs_global.begin()
		);

		// check, whether there is an "auxiliary scan" between the current one and the "down" (bottom)
		// neighbour (we are looking from top to bottom)
		int distance_to_bottom_neighbour = neighbour_main_down - it;
		int distance_to_upper_neighbour = it - neighbour_main_up;

		// i.e. the bottom "main" neighbour is separated from the current "scan" with an "auxiliary" one
		if (distance_to_upper_neighbour == 0) {
			// 0 is reserved for .begin() `iterator`
			pos_obs_global_final.push_back(&(*it));
			printf("  -  0 dist to upper neighbour!\r\n");
			continue;
		}
		if (distance_to_upper_neighbour >= 2) {
			// TODO
			//
			// previous main scan and the current one are separated with an auxiliary scan -
			// therefore main scans' data must be compared with auxiliary one's.
			//
//			// check, whether the previous data was taken from `main_scan`
//			if (pos_obs_global_final.back()->is_main) {
//				// it's from main, so must decide, whether to chose an auxiliary scan data
//				// or stick with the currently investigated data (related to the main scan)
//				// NOTE: decision criteria is a distance to the obstacle
//				pos_obs_global_final.push_back(chooseClosest(&(*it), &(*(it-1))));
//				printf("\r\n");
//				continue;
//			}

			// V2
			// check whether the previous data was taken from the `main` scan
			// because the auxiliary scan's angle was closer to the current `scan`,
			// compared to the upper neighbour's
			double angle_distance_current = std::fabs(it->angle - (it-1)->angle);
			double angle_distance_neighbour = std::fabs(neighbour_main_up->angle - (it-1)->angle);
			printf(" | UPP angDist - cur: %4.5f, upp n-bour: %4.5f",
				angle_distance_current,
				angle_distance_neighbour
			);
			if (angle_distance_current <= angle_distance_neighbour) {
				// select closest from current (main) and previous (auxiliary)
				pos_obs_global_final.push_back(chooseClosest(&(*it), &(*(it-1))));
				//printf(" SLCTD!\r\n");
				printf(" %ld/%ld!\r\n",
					(it-1) - pos_obs_global.begin(),
					it - pos_obs_global.begin()
				);
				continue;
			}
			// requires check of the bottom neighbourhood
			printf(" **REQ BOT!** ");
			printf("D:%d", distance_to_bottom_neighbour);
		}
		if (distance_to_bottom_neighbour >= 2) {
			// must check, whether the auxiliary scan's angle is closer to the current instance or the next one
			double angle_distance_current = std::fabs(it->angle - (it+1)->angle);
			double angle_distance_neighbour = std::fabs(neighbour_main_down->angle - (it+1)->angle);
			printf(" | BOT angDist - cur: %4.5f, bot n-bour: %4.5f",
				angle_distance_current,
				angle_distance_neighbour
			);
			if (angle_distance_current <= angle_distance_neighbour) {
				// take the current (main) scan as the bottom neighbour is closer to the
				// auxiliary scan
				// NOTE: auxiliary scan will be considered in the next iteration (for the next range)
				pos_obs_global_final.push_back(chooseClosest(&(*it), &(*(it+1))));
				//printf(" CURR!\r\n");
				printf(" %ld/%ld!\r\n",
					it - pos_obs_global.begin(),
					(it+1) - pos_obs_global.begin()
				);
				continue;
			} /* else {
				// currently: it
				// it + 1 is an auxiliary scan data
				// it + 2 is a main scan data (neighbour_main_down)
				pos_obs_global_final.push_back(chooseClosest(&(*neighbour_main_down), &(*(it+1))));
			}
			*/
			pos_obs_global_final.push_back(&(*it));
			printf(" %ld! NXT\r\n", it - pos_obs_global.begin());
			continue;
		}

		if (distance_to_bottom_neighbour == 1 || distance_to_upper_neighbour == 1) {
			pos_obs_global_final.push_back(&(*it));
			//printf("  -  1 dist to upper OR bottom neighbour!\r\n");
			printf(" / SEL: %ld!\r\n", it - pos_obs_global.begin());
			continue;
		}
	}

	printf("FINAL: total %ld, main: %ld, aux: %ld  /  diff: %ld\r\n\n\n",
		pos_obs_global_final.size(),
		pos_obs_main_global.size(),
		pos_obs_aux_global.size(),
		pos_obs_main_global.size() - pos_obs_aux_global.size()
	);

	// final evaluation
	sensor_msgs::LaserScan scan_merged;
	scan_merged.header.stamp = ros::Time::now();
	scan_merged.header.frame_id = scan_main->header.frame_id;
	scan_merged.angle_increment = scan_main->angle_increment;
	scan_merged.angle_min = scan_main->angle_min;
	scan_merged.angle_max = scan_main->angle_max;
	scan_merged.range_min = scan_main->range_min;
	scan_merged.range_max = scan_main->range_max;
	scan_merged.scan_time = scan_main->scan_time;
	scan_merged.time_increment = scan_main->time_increment;
	scan_merged.intensities = scan_main->intensities;

	for (const auto& obstacle : pos_obs_global_final) {
		// TODO: final refinement (angle_increment matching vs distance)
		scan_merged.ranges.push_back(obstacle->distance);
	}

	pub_scan_.publish(scan_merged);
}

// NOTE: free function
// SOURCE: https://www.geeksforgeeks.org/sorting-vector-of-pairs-in-c-set-1-sort-by-first-and-second/
// Driver function to sort the vector elements by second element of pairs
bool sortByAngle (
	const LaserPlaneMerger::ObstacleScanFrame &a,
	const LaserPlaneMerger::ObstacleScanFrame &b
) {
    return (a.angle < b.angle);
}

std::vector<geometry_msgs::Point32> LaserPlaneMerger::computeGlobalPositions(
	const geometry_msgs::TransformStamped &pose_ref,
	const sensor_msgs::LaserScan &scan
) {
	std::vector<geometry_msgs::Point32> vector;
	float angle = scan.angle_min;
	// vector of obstacle points global positions
	for (const auto& range: scan.ranges) {
		// find obstacle position (auxiliary scan) in global coordinate system
		vector.push_back(findGlobalPosition(pose_ref, range, angle));
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
		angle += scan.angle_increment;
	}
//	printf("\r\n");
	return vector;
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

void LaserPlaneMerger::publishGlobalPositionsVisualization(
	const std::vector<geometry_msgs::Point32> &positions,
	const std::string &ns,
	const int &id
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

	marker.ns = ns;
	marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::POINTS;
	marker.id = id;

	marker.color.a = 1.0f;
	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;

	for (const auto& pos: positions) {
		if (std::isinf(pos.x) || std::isinf(pos.y) || std::isinf(pos.z)) {
			continue;
		}
		geometry_msgs::Point pt;
		pt.x = pos.x;
		pt.y = pos.y;
		pt.z = pos.z;

		marker.points.push_back(pt);
	}

	pub_marker_.publish(marker);
}

double LaserPlaneMerger::computeEuclideanDistance(
		const geometry_msgs::TransformStamped &pose_ref,
		const geometry_msgs::Point32 &position
) {
	return std::sqrt(std::pow(position.x - pose_ref.transform.translation.x, 2)
		+ std::pow(position.y - pose_ref.transform.translation.y, 2)
		+ std::pow(position.z - pose_ref.transform.translation.z, 2)
	);
}

double LaserPlaneMerger::computeAngle(
	const geometry_msgs::TransformStamped &pose_ref,
	const geometry_msgs::Point32 &position
) {
	tf::Vector3 v(
		position.x - pose_ref.transform.translation.x,
		position.y - pose_ref.transform.translation.y,
		position.z - pose_ref.transform.translation.z
	);
	// v.angle(tf::Vector3(1.0, 0.0, 0.0));
	double angle = std::atan2(v.getY(), v.getX());
	/*
	printf("[computeAngle] pos_ref: %2.5f, %2.5f, %2.5f | pos: %2.5f, %2.5f, %2.5f | diff: %2.5f, %2.5f, %2.5f | angle: %3.6f\r\n",
		position.x,
		position.y,
		position.z,
		pose_ref.transform.translation.x,
		pose_ref.transform.translation.y,
		pose_ref.transform.translation.z,
		position.x - pose_ref.transform.translation.x,
		position.y - pose_ref.transform.translation.y,
		position.z - pose_ref.transform.translation.z,
		angle
	);
	*/

	return angle;
}

LaserPlaneMerger::ObstacleScanFrame* LaserPlaneMerger::chooseClosest(
	ObstacleScanFrame* main,
	ObstacleScanFrame* aux
) {
	// is inf?
	return (aux->distance < main->distance) ? aux : main;
}
