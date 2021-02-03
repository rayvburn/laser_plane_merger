# laser_plane_merger
Merge multiple LaserScans projecting them onto a single plane


First scan is a main one, all points that it carries are placed inside the merged scan.

Second scan is a "auxiliary" one. The merged scan contains only these points (projected onto first scan's plane) from second scan that are not present in the first one (sensor model is considered).

A typical application is to set first scan as a laser scan, whereas the second one as a LaserScan-converted PCL (from RGBD camera).
