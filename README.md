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

# Merging algorithm
TBD (plantuml)
