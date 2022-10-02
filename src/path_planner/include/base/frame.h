#pragma once
#include <string>
#include <vector>
#include <math.h>  
#include <ctime>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <ros_viz_tools/ros_viz_tools.h>

#include "planner/astar.h"

#include "base/plannerBase.h"
#include "base/smootherBase.h"
#include "data_struct/data_struct.h"

#include "tool/KDTree.hpp"
#include "tool/tool.h"
#include "tool/corridor.h"


using namespace std;
using namespace Car;
using namespace Eigen;


class frame
{
public:
    frame();

    bool planReady();
    bool planReady_noWayPoints();
    bool Plan();
    bool Smooth();
    void marker_Vis();
    void LoopAction();

    ros::NodeHandle n_;
    ros_viz_tools::RosVizTools marker_;
    string marker_frame_id_;

    State start_;
    State end_;
    vector<State> wayPoints_;

    grid_map::GridMap grid_map_;
    ros::Publisher map_publisher_;
    ros::Subscriber wayPoint_sub_;
    ros::Subscriber start_sub_;
    ros::Subscriber end_sub_;
    cv::Mat img_src_;

    corridor Corridor_;

    double resolution_;
    unsigned char OCCUPY_;
    unsigned char FREE_;

    vector<State> referPath_;
    vector<State> optPath_;

    plannerBase* plannerB_;
    smootherBase* smoothB_;
    
    pointVec obstaclePoint_;
    KDTree obstacleTree_;
    MatrixXi map_;

    bool startStateFlag_;
    bool endStateFlag_;
    bool wayPointsFlag_;

private:
    
    void wayPointCb(const geometry_msgs::PointStampedConstPtr &p);
    void startCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &start);
    void goalCb(const geometry_msgs::PoseStampedConstPtr &goal);

    void initParameter();
    void initImgInfo();
    void initGridMap();
    void initROS_Setting();
    void loadMap();
};

