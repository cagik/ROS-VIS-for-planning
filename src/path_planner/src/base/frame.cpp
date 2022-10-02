#include "base/frame.h"

frame::frame()
{   
    n_ = ros::NodeHandle("~");
    initParameter();
    initImgInfo();
    initGridMap();
    initROS_Setting();
    loadMap();
    
    plannerB_ = new planner::Astar;
}

bool frame::Plan(){
    plannerB_->plan(start_, end_, map_, obstacleTree_, &referPath_);
    startStateFlag_ = false;
}

bool frame::Smooth(){

}

bool frame::planReady()
{
    return startStateFlag_ && endStateFlag_ && wayPointsFlag_;
}

bool frame::planReady_noWayPoints()
{
    return startStateFlag_ && endStateFlag_;
}

void frame::marker_Vis()
{
    int marker_id = 0;

    marker_.clear();

    if(wayPoints_.size() >= 2)
    {
        const auto &p1 = wayPoints_[wayPoints_.size() - 2];
        const auto &p2 = wayPoints_.back();
        if (Tool::distance(p1, p2) <= 0.001) 
        {
            wayPoints_.clear();
        }
    }

    visualization_msgs::Marker wayPoints_marker = marker_.newSphereList(1, "reference point", marker_id++, ros_viz_tools::RED, marker_frame_id_);
        for(size_t i = 0; i != wayPoints_.size(); i++){
            geometry_msgs::Point p;
            p.x = wayPoints_[i].x;
            p.y = wayPoints_[i].y;
            p.z = 1.0;
            wayPoints_marker.points.push_back(p);
        }
    marker_.append(wayPoints_marker);

    geometry_msgs::Vector3 scale;
    scale.x = 2.0;
    scale.y = 0.3;
    scale.z = 0.3;
    geometry_msgs::Pose start_pose;
    start_pose.position.x = start_.x;
    start_pose.position.y = start_.y;
    start_pose.position.z = 1.0;
    auto start_quat = tf::createQuaternionFromYaw(start_.heading);
    start_pose.orientation.x = start_quat.x();
    start_pose.orientation.y = start_quat.y();
    start_pose.orientation.z = start_quat.z();
    start_pose.orientation.w = start_quat.w();
    visualization_msgs::Marker start_marker = marker_.newArrow(scale, start_pose, "start point", marker_id++, ros_viz_tools::CYAN, marker_frame_id_);
    marker_.append(start_marker);
    geometry_msgs::Pose end_pose;
    end_pose.position.x = end_.x;
    end_pose.position.y = end_.y;
    end_pose.position.z = 1.0;
    auto end_quat = tf::createQuaternionFromYaw(end_.heading);
    end_pose.orientation.x = end_quat.x();
    end_pose.orientation.y = end_quat.y();
    end_pose.orientation.z = end_quat.z();
    end_pose.orientation.w = end_quat.w();
    visualization_msgs::Marker end_marker = marker_.newArrow(scale, end_pose, "end point", marker_id++, ros_viz_tools::CYAN, marker_frame_id_);
    marker_.append(end_marker);


    ros_viz_tools::ColorRGBA path_color;
    path_color.r = 1.0;
    path_color.g = 0.0;
    path_color.b = 0.0;

    visualization_msgs::Marker refer_path_marker = marker_.newLineStrip(0.2, "ref path", marker_id++, path_color, marker_frame_id_);
    for(size_t i = 0; i < referPath_.size() ; ++i)
    {
        geometry_msgs::Point p;
        p.x = referPath_[i].x;
        p.y = referPath_[i].y;
        p.z = 1.0;
        refer_path_marker.points.push_back(p);
        path_color.a = 1;
        refer_path_marker.colors.emplace_back(path_color);
    }
    marker_.append(refer_path_marker);

    path_color.r = 0.063;
    path_color.g = 0.305;
    path_color.b = 0.545;
    visualization_msgs::Marker opt_path_marker = marker_.newLineStrip(0.5, "opt path", marker_id++, path_color, marker_frame_id_);
    for(size_t i = 0; i < optPath_.size(); ++i)
    {
        geometry_msgs::Point p;
        p.x = optPath_[i].x;
        p.y = optPath_[i].y;
        p.z = 1.0;
        opt_path_marker.points.push_back(p);
        path_color.a = 1;
        opt_path_marker.colors.emplace_back(path_color);
    }
    marker_.append(opt_path_marker); 

    path_color.r = 0.1;
    path_color.g = 0.1;
    path_color.b = 0.8;
    for(size_t i = 0; i < Corridor_.vis_boxes.size(); i++)
    {
        visualization_msgs::Marker corridor_marker = marker_.newLineStrip(1, "corridor path", marker_id++, path_color, marker_frame_id_);
        for(size_t j = 0; j <= Corridor_.vis_boxes[i].size() ; ++j)
        {
            geometry_msgs::Point p;
            if(j == 0){
                p.x = Corridor_.vis_boxes[i][0];
                p.y = Corridor_.vis_boxes[i][1];
                p.z = 1.0;
            }
            if(j == 1){
                p.x = Corridor_.vis_boxes[i][0];
                p.y = Corridor_.vis_boxes[i][3];
                p.z = 1.0;
            }
            if(j == 2){
                p.x = Corridor_.vis_boxes[i][2];
                p.y = Corridor_.vis_boxes[i][3];
                p.z = 1.0;
            }
            if(j == 3){
                p.x = Corridor_.vis_boxes[i][2];
                p.y = Corridor_.vis_boxes[i][1];
                p.z = 1.0;
            }
            if(j == 4){
                p.x = Corridor_.vis_boxes[i][0];
                p.y = Corridor_.vis_boxes[i][1];
                p.z = 1.0;
            }
            corridor_marker.points.push_back(p);
            path_color.a = 1;
            corridor_marker.colors.emplace_back(path_color);
        }
        marker_.append(corridor_marker); 
    }

}

void frame::LoopAction(){
    ros::Time time = ros::Time::now();
    grid_map_.setTimestamp(time.toNSec());
    nav_msgs::OccupancyGrid message;
    grid_map::GridMapRosConverter::toOccupancyGrid(grid_map_, "obstacle", FREE_, OCCUPY_, message);
    map_publisher_.publish(message);
    marker_.publish();
}

void frame::wayPointCb(const geometry_msgs::PointStampedConstPtr &p) {
    State wayPoint;
    wayPoint.x = p->point.x;
    wayPoint.y = p->point.y;
    wayPoints_.emplace_back(wayPoint);
    cout << "wpState x: " << wayPoint.x << "  y:" << wayPoint.y << endl;
    cout << "state: " << map_(int(wayPoint.x), int(wayPoint.y)) << endl;
    wayPointsFlag_ = true;
}


void frame::startCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &start) {
    start_.x = start->pose.pose.position.x;
    start_.y = start->pose.pose.position.y;
    start_.heading = tf::getYaw(start->pose.pose.orientation);
    startStateFlag_ = true;
    cout << "startState x: " << start_.x << "  y:" << start_.y << endl;
    std::cout << "get initial state." << std::endl;
}


void frame::goalCb(const geometry_msgs::PoseStampedConstPtr &goal) {
    end_.x = goal->pose.position.x;
    end_.y = goal->pose.position.y;
    end_.heading = tf::getYaw(goal->pose.orientation);
    endStateFlag_ = true;
    std::cout << "get the goal." << std::endl;
}

void frame::initParameter()
{
    resolution_ = 1;
    OCCUPY_ = 0;
    FREE_ = 255;
    startStateFlag_ = false;
    endStateFlag_ = false;
    wayPointsFlag_ = false;
}

void frame::initImgInfo()
{
    string image_dir = ros::package::getPath("path_planner");
    string image_file = "gridmap.png";
    image_dir.append("/" + image_file);
    img_src_ = cv::imread(image_dir, CV_8UC1);
}

void frame::initGridMap()
{
    grid_map_ = grid_map::GridMap(std::vector<std::string>{"obstacle"});
    grid_map::Position p(0.5 * img_src_.rows - 0.5, 0.5 * img_src_.cols - 0.5);
    grid_map::GridMapCvConverter::initializeFromImage(img_src_, resolution_, grid_map_, p);
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(img_src_, "obstacle", grid_map_, OCCUPY_, FREE_, 1);
    grid_map_.setFrameId("/map");
}

void frame::initROS_Setting()
{
    map_publisher_ = n_.advertise<nav_msgs::OccupancyGrid>("grid_map", 1, true);
    wayPoint_sub_ = n_.subscribe("/clicked_point", 1, &frame::wayPointCb, this);
    start_sub_ = n_.subscribe("/initialpose", 1, &frame::startCb, this);
    end_sub_ = n_.subscribe("/move_base_simple/goal", 1, &frame::goalCb,this);
    marker_ = ros_viz_tools::RosVizTools(n_, "markers");
    marker_frame_id_ = "/map";
}

void frame::loadMap()
{
    int width = img_src_.cols;
    int height = img_src_.rows;
    map_ = MatrixXi(width, height);



    threshold(img_src_.clone(), img_src_, 0, 255, cv::THRESH_OTSU);

    obstaclePoint_.clear();


    for(int y = 0; y < height; y++)
    {
        for(int x = 0; x < width; x++)
        {
            if(img_src_.at<uchar>(y, x) == 0)
            {
                map_(width-1-y, height-1-x) = obstacle;
                point_t tmp = {double(width-1-y), double(height-1-x)};
                //map_(x, y) = obstacle;
                //point_t tmp = {double(x), double(y)};
                obstaclePoint_.push_back(tmp);
            }
            else
            {   
                map_(width-1-y, height-1-x) = freeGrid;
                //map_(x, y) = freeGrid;
            }
        }
    }

    // cv::Mat img_tmp = cv::Mat::zeros(cv::Size(width, height), CV_8UC1);
    // int step = 0;
    // for (int i = 0; i < width; i++)
    // {
    //     for (int j = 0; j < height; j++)
    //     {
    //         if(map_(i, j) == obstacle){
    //             img_tmp.at<uchar>(i, j) = 0;
    //         }
    //         else{
    //             img_tmp.at<uchar>(i, j) = 255;
    //         }
    //     }
        
    // }

    // cv::imwrite("/home/cavata/workspace/pathPlannerFrame/tmp.png", img_tmp);

    vector<pair<int, int>> bound64;
    bound64.resize(49);
    int x,y;
    x = -3;
    y = -3;
    for(int i = 0; i < bound64.size(); i++){
        if(x == 0 && y ==0){
            x++;
            continue;
        }
        if(x > 3){
            x = -3;
            y++;
        }
        bound64[i] = {x, y};
        x++;
    }

    for(size_t i = 0; i < obstaclePoint_.size(); i++){
        for (size_t j = 0; j < bound64.size(); j++)
        {
            int bound_x = obstaclePoint_[i][0] + bound64[j].first;
            int bound_y = obstaclePoint_[i][1] + bound64[j].second;
            if(bound_x >= 0 && bound_x < height && bound_y >= 0 && bound_y < width)
            {
                map_(bound_x, bound_y) = obstacle;
            } 
        }
    }

    obstaclePoint_.clear();

    for(size_t i = 0; i < map_.rows(); i++){
        for(size_t j = 0; j < map_.cols(); j++){
            if(map_(i, j) == obstacle){
                point_t tmp = {double(i), double(j)};                
                obstaclePoint_.push_back(tmp);
            }
        }
    }


    KDTree tmpTree(obstaclePoint_);
    obstacleTree_ = tmpTree;
}