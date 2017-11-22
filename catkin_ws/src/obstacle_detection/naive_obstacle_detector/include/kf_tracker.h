#include <iostream>
#include <string.h>
#include <fstream>
#include <algorithm>
#include <iterator>
#include "kf_tracker/featureDetection.h"
#include "kf_tracker/CKalmanFilter.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include "opencv2/video/tracking.hpp"
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
 #include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <limits>
#include <utility>
#include <pcl/registration/correspondence_estimation.h>

//Constants for KF
float dvx = 0.01f; // 1.0
float dvy = 0.01f; // 1.0
float dx = 1.0f;
float dy = 1.0f;

// Process Noise Covariance Matrix Q
// [ Ex 0  0    0 0    0 ]
// [ 0  Ey 0    0 0    0 ]
// [ 0  0  Ev_x 0 0    0 ]
// [ 0  0  0    1 Ev_y 0 ]
//// [ 0  0  0    0 1    Ew ]
//// [ 0  0  0    0 0    Eh ]
float sigmaP = 0.01;
float sigmaQ = 0.1;


// calculate euclidean distance of two points
double euclidean_distance(geometry_msgs::Point& p1, geometry_msgs::Point& p2)
{
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

std::pair<int,int> findIndexOfMin(std::vector<std::vector<float> > distMat)
{
    std::pair<int,int>minIndex;
    float minEl=std::numeric_limits<float>::max();
    for (int i=0; i<distMat.size();i++)
        for(int j=0;j<distMat.at(0).size();j++)
        {
            if( distMat[i][j]<minEl)
            {
                minEl=distMat[i][j];
                minIndex=std::make_pair(i,j);
            }
        }
    return minIndex;
}
