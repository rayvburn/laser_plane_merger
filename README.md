# laser_plane_merger
ROS node that allows to merge two `sensor_msgs/LaserScan`s, projecting them onto a single plane.

First scan is a main one (`main_scan`), the second one is an auxiliary one (`aux_scan`). Merged scan contains ranges projected onto the main scan's plane.

The merging algorithm uses main scan's points as a reference. The merged scan contains only these points from the auxiliary scan that are closer to the sensor frame, compared to the main one.

Sensor model is considered to compare main scan's readings with the closest readings from the auxiliary scan.

A typical application is to set `main_scan` as a LiDAR scan, whereas the second one as a LaserScan retrieved from a RGBD camera (converted from PCL).

![Separate LaserScans for LiDAR and RGBD camera](doc/tiago_office_laser_rgbd.png)

![Merged scans, projected onto single plane](doc/tiago_office_scans_merged.png)

Screenshots retrieved from PAL's TiAGo robot simulation.

# Run
The package was tested (so far) under Ubuntu 16.04 and ROS Kinetic.

Node can be run with:

    rosrun laser_plane_merger laser_plane_merger_node scan1:=scan scan2:=rgbd_scan

## Subscribed topics
* `/scan1` ([sensor_msgs/LaserScan](http://docs.ros.org/en/kinetic/api/sensor_msgs/html/msg/LaserScan.html))
* `/scan2` ([sensor_msgs/LaserScan](http://docs.ros.org/en/kinetic/api/sensor_msgs/html/msg/LaserScan.html))
* `/tf` ([tf/tfMessage](http://docs.ros.org/en/kinetic/api/tf/html/msg/tfMessage.html))
* `/tf_static` ([tf2_msgs/TFMessage](http://docs.ros.org/en/kinetic/api/tf2_msgs/html/msg/TFMessage.html))

The node uses [`message_filters::Subscriber`](http://docs.ros.org/en/kinetic/api/message_filters/html/c++/classmessage__filters_1_1Subscriber.html)s with [`ApproximateTime`](http://wiki.ros.org/message_filters/ApproximateTime) synchronization policy so the resultant scan is computed with slower scan's (main or auxiliary) frequency.

## Published topics
* `/scan_merged` ([sensor_msgs/LaserScan](http://docs.ros.org/en/kinetic/api/sensor_msgs/html/msg/LaserScan.html))
* `/laser_plane_merger/marker` ([visualization_msgs/Marker](http://docs.ros.org/en/kinetic/api/visualization_msgs/html/msg/Marker.html))

## Parameters
* ~*scan_main_topic_name* (string, default: "/scan1")
* ~*scan_aux_topic_name* (string, default: "/scan2")
* ~*scan_merged_topic_name* (string, default: "/scan_merged")
* ~*global_frame_name* (string, default: "map")

Parameters default values can easily be remapped as noted in [Run](https://github.com/rayvburn/laser_plane_merger/blob/main/README.md#run) section or with a `launch` file.

# Merging algorithm
Scan with a higher resolution must be selected as a `main` one if one wants to merge 2 different scans.

It comes from the fact that the algorithm tries to compare obstacle elements found in main scan to elements found in auxiliary scan (if scanning ray goes in the same direction from reference position) and selects closest obstacle element from both scans.

What's more, the resultant `LaserScan` inherits all features (angle range, angle increment) from the `main` scan.

![algorithm explained](https://plantuml.gitlab-static.net/png/U9pjLijkt30ClFih25-wCR0_q4ERKx-WjoPOq39j2vMb1IcdJR--fEnzPVj2s-IKqnhMS3Wpr4g-bOoSvobKrQQvWUts7THjoYE8moY0pFw1khhwqSi5xTAqdpD1Pbth4qy2FQS9xgAXzHad4iW9Xf1Q3CiB9VoxkikUwz8-YSy-gRJKGshLaWkaH1HeefYLECsn0n_XvE9CmR-mB7g4CepaXp5RRHu9c6GE6MCkE6L1vn9tFWvcoH0JAfCjlf756tpyK5ULxw4-D1UjU_0oO_3VqAHf4NycjlSoj-BOjzIzhuqlLc1QFcMUwHxg_Tm6B-Enu2UMdbglZONDYDdIgG2X6q-bQw5IW6D23K-qeSip4pYjzvBDOkhV6cHxu6-QGuUckP1onhTLLEUPt0ehcu6f9wRew2WHDbV1tLYvRHJTbV74VQmuakk2Kls2-_SaVS112NpilCFYDO-OhmrzGO6EihgXicLiuhkTIT_jYlYweY2q3gr7VRv_3VoP0rUElgfUKtxEbDS3CN66zl69Tuo37MeR56UpYCED7dX_Zbse9ymwEfo_-k2H7vVpTKNVHRLw7YJgWUt6bCHakg2_h4BlufMnJOkYbZIHyqIBlfTjk9X44PhQS76FKBbSpb9QH9IikikDi4VFOjh_2yygYVMY7ipM0L1kvwV6HEsM419hptqgizEYizGKeOLDi_xL9k91ap10qnJHBzEfsDAFdggwrHrcdq3V0GDpAXq0)
