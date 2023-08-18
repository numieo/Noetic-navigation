/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*         Mike Phillips (put the planner in its own thread)
*********************************************************************/
#include <move_base/move_base.h>
#include <move_base_msgs/RecoveryStatus.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace move_base {

  MoveBase::MoveBase(tf2_ros::Buffer& tf) :
    tf_(tf),
    as_(NULL),
    planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
    recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
    planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
    runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false) {

    as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", [this](auto& goal){ executeCb(goal); }, false);

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    recovery_trigger_ = PLANNING_R;

    //get some parameters that will be global to the move base node
    std::string global_planner, local_planner;
    private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
    private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
    private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));
    private_nh.param("planner_frequency", planner_frequency_, 0.0);
    private_nh.param("controller_frequency", controller_frequency_, 20.0);
    private_nh.param("planner_patience", planner_patience_, 5.0);
    private_nh.param("controller_patience", controller_patience_, 15.0);
    private_nh.param("max_planning_retries", max_planning_retries_, -1);  // disabled by default

    private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
    private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

    // parameters of make_plan service
    private_nh.param("make_plan_clear_costmap", make_plan_clear_costmap_, true);
    private_nh.param("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_, true);

    //set up plan triple buffer
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

    //set up the planner's thread
    planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

    //for commanding the base
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );

    ros::NodeHandle action_nh("move_base");
    action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
    recovery_status_pub_= action_nh.advertise<move_base_msgs::RecoveryStatus>("recovery_status", 1);

    //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
    //they won't get any useful information back about its status, but this is useful for tools
    //like nav_view and rviz
    ros::NodeHandle simple_nh("move_base_simple");
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, [this](auto& goal){ goalCB(goal); });

    //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
    private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
    private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
    private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
    private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

    private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
    private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
    private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

    //initialize the global planner
    try {       // 尝试初始化全局规划器
      // 通过bgp_loader_（插件加载器）的createInstance方法创建一个全局规划器的实例
      planner_ = bgp_loader_.createInstance(global_planner);
      // bgp_loader_.getName(global_planner)初始化全局规划器
      // planner_costmap_ros_ 是一个代价地图的ROS包装类，将其传递给规划器以便规划器能够与其交互
      planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
    } 
    // 捕获如果全局规划器初始化失败
    catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }

    //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    // 创建一个代价地图的ROS包装类实例，处理坐标系之间的变换
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    // 将局部规划器的代价地图暂停，因为在初始化阶段不需要即时的代价地图更新
    controller_costmap_ros_->pause();

    //create a local planner
    try {   // 尝试初始化局部规划器， 创建一个局部规划器的实例
      tc_ = blp_loader_.createInstance(local_planner);
      ROS_INFO("Created local_planner %s", local_planner.c_str());
      // 初始化局部规划器
      // blp_loader_.getName(local_planner) 获取局部规划器的名称。
      // &tf_ 是一个TransformListener的指针，用于处理坐标系之间的变换。
      // controller_costmap_ros_ 是局部规划器使用的代价地图的ROS包装类
      tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
      exit(1);
    }

    // Start actively updating costmaps based on sensor data
    planner_costmap_ros_->start();
    controller_costmap_ros_->start();

    //advertise a service for getting a plan
    make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

    //advertise a service for clearing the costmaps
    clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps initially");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }

    //load any user specified recovery behaviors, and if that fails load the defaults
    if(!loadRecoveryBehaviors(private_nh)){
      loadDefaultRecoveryBehaviors();
    }

    //initially, we'll need to make a plan
    state_ = PLANNING;

    //we'll start executing recovery behaviors at the beginning of our list
    recovery_index_ = 0;

    //we're all set up now so we can start the action server
    as_->start();

    dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = [this](auto& config, auto level){ reconfigureCB(config, level); };
    dsrv_->setCallback(cb);
  }

  void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level){
    // 创建了一个scoped_lock对象，使用互斥锁（configuration_mutex_）来保证在配置参数时不会有多个线程同时进行操作
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    //The first time we're called, we just want to make sure we have the
    //original configuration
    // 这个条件判断语句检查是否是第一次进入这个回调函数。如果是第一次进入，则进行初始配置，
    // 将当前的配置参数存储为初始参数，并标记setup_为true
    if(!setup_)
    {
      last_config_ = config;
      default_config_ = config;
      setup_ = true;
      return;
    }

    if(config.restore_defaults) {     // 检查是否需要恢复默认配置
      config = default_config_;
      //if someone sets restore defaults on the parameter server, prevent looping
      // 表示已经完成了恢复默认配置的操作
      config.restore_defaults = false;
    }

    if(planner_frequency_ != config.planner_frequency)
    {
      planner_frequency_ = config.planner_frequency;
      p_freq_change_ = true;
    }

    if(controller_frequency_ != config.controller_frequency)
    {
      controller_frequency_ = config.controller_frequency;
      c_freq_change_ = true;
    }

    planner_patience_ = config.planner_patience;
    controller_patience_ = config.controller_patience;
    max_planning_retries_ = config.max_planning_retries;
    conservative_reset_dist_ = config.conservative_reset_dist;

    recovery_behavior_enabled_ = config.recovery_behavior_enabled;
    clearing_rotation_allowed_ = config.clearing_rotation_allowed;
    shutdown_costmaps_ = config.shutdown_costmaps;

    oscillation_timeout_ = config.oscillation_timeout;
    oscillation_distance_ = config.oscillation_distance;
    // 处理全局规划器（BaseGlobalPlanner）的重新配置
    // 检查当前传入的全局规划器名称是否与上一次配置中的名称不同，以判断是否需要更换全局规划器
    if(config.base_global_planner != last_config_.base_global_planner) {
      // 这行代码将当前的全局规划器指针（planner_）保存到old_planner中，以便在出现初始化错误时恢复
      boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
      //initialize the global planner
      // 表示正在加载新的全局规划器
      ROS_INFO("Loading global planner %s", config.base_global_planner.c_str());
      try {
        // 创建一个新的全局规划器实例，使用传入的全局规划器名称
        planner_ = bgp_loader_.createInstance(config.base_global_planner);
        // 创建一个独占的互斥锁对象，锁定planner_mutex_，确保在初始化新规划器过程中不会有其他线程访问
        // wait for the current planner to finish planning
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        // 用于清除之前规划器生成的路径，以确保新规划器的路径是从一个干净状态开始的
        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        // 重置一些状态信息
        resetState();
        // 调用新规划器的initialize方法，初始化新的全局规划器，传入全局规划器的名称和代价地图信息
        planner_->initialize(bgp_loader_.getName(config.base_global_planner), planner_costmap_ros_);
        // 解锁互斥锁，允许其他线程访问
        lock.unlock();
      } 
      // 处理如果初始化新规划器失败的情况 
      catch (const pluginlib::PluginlibException& ex) {
        // 指示新规划器的创建失败
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_global_planner.c_str(), ex.what());
        planner_ = old_planner;  // 如果新规划器初始化失败，将之前保存的旧规划器指针赋值回planner_，恢复原有的规划器
        // 将配置参数中的全局规划器名称恢复为上一次配置中的名称
        config.base_global_planner = last_config_.base_global_planner;
      }
    }
    // 检查当前传入的局部规划器名称是否与上一次配置中的名称不同，以判断是否需要更换局部规划器
    if(config.base_local_planner != last_config_.base_local_planner){
      // 将当前的局部规划器指针（tc_）保存到old_planner中，以便在出现初始化错误时恢复
      boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
      //create a local planner
      // 尝试加载并初始化新的局部规划器
      try {
        // 使用插件加载器blp_loader_创建一个新的局部规划器实例，使用传入的局部规划器名称
        tc_ = blp_loader_.createInstance(config.base_local_planner);
        // Clean up before initializing the new planner
        // 清除之前规划器生成的路径，以确保新规划器的路径是从一个干净状态开始的
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        // 调用新规划器的initialize方法，初始化新的局部规划器，传入局部规划器的名称、TransformListener和代价地图信息
        tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
      } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_local_planner.c_str(), ex.what());
        // 如果新规划器初始化失败，将之前保存的旧规划器指针赋值回tc_，恢复原有的规划器
        tc_ = old_planner;
        // 将配置参数中的局部规划器名称恢复为上一次配置中的名称，以保持一致性
        config.base_local_planner = last_config_.base_local_planner;
      }
    }

    make_plan_clear_costmap_ = config.make_plan_clear_costmap;
    make_plan_add_unreachable_goal_ = config.make_plan_add_unreachable_goal;
    // 将当前的配置参数存储为上一次的配置，以便下次对比
    last_config_ = config;
  }
  // 处理目标（goal）回调和清除代价地图窗口的操作       // goal 机器人的目标姿态
  void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
    // 使用ROS的ROS_DEBUG_NAMED宏输出一个调试级别的消息，表示正在处理目标回调，并将姿态信息包装到行动消息中，然后重新发送给服务器
    ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::MoveBaseActionGoal action_goal;  // 存储行动目标
    action_goal.header.stamp = ros::Time::now();     // 当前时间
    action_goal.goal.target_pose = *goal;    // 将传入的目标姿态指针内容复制到action_goal的目标姿态字段中

    action_goal_pub_.publish(action_goal);   // 将目标姿态信息作为行动目标发布出去
  }
  // 清除代价地图中的一个矩形区域   // size_x size_y 矩形区域的宽度和高度
  void MoveBase::clearCostmapWindows(double size_x, double size_y){
    geometry_msgs::PoseStamped global_pose;
  
    //clear the planner's costmap
    getRobotPose(global_pose, planner_costmap_ros_);
    // 获取规划器的代价地图中机器人的姿态信息，并将其存储在global_pose中
    std::vector<geometry_msgs::Point> clear_poly;
    // 每个顶点的坐标被计算并添加到清除多边形的顶点集中。这是代价地图管理中常用的操作，
    // 用于在机器人移动或者路径规划发生变化时，清除地图上的部分信息
    double x = global_pose.pose.position.x;
    double y = global_pose.pose.position.y;
    // 创建一个名为pt的geometry_msgs::Point类型的实例，用于表示一个点的坐标
    geometry_msgs::Point pt;

    pt.x = x - size_x / 2;    // 计算矩形区域左下角点的x坐标
    pt.y = y - size_y / 2;    // 计算矩形区域左下角点的y坐标
    clear_poly.push_back(pt); // 将计算得到的点的坐标添加到清除多边形的顶点中
    // 右下角
    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);
    // 右上角
    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);
    // 左上角
    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);
    // 获取规划器的代价地图，然后使用setConvexPolygonCost方法设置多边形区域内的代价值为costmap_2d::FREE_SPACE（表示空闲区域）
    planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

    //clear the controller's costmap
    // 获取控制器的代价地图中机器人的姿态信息
    getRobotPose(global_pose, controller_costmap_ros_);
    // 清空用于存储清除多边形顶点的向量
    clear_poly.clear();
    x = global_pose.pose.position.x;    // 获取机器人姿态的x坐标
    y = global_pose.pose.position.y;    // 获取机器人姿态的y坐标

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);
    // 获取控制器的代价地图，然后使用setConvexPolygonCost方法设置多边形区域内的代价值为costmap_2d::FREE_SPACE（表示空闲区域）
    controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
  }

  bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
    //clear the costmaps
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
    controller_costmap_ros_->resetLayers();

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
    planner_costmap_ros_->resetLayers();
    return true;
  }

  // 定义了一个名为planService的函数，用于处理获取规划路径的服务
  bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp){
    // 检查行动服务器（as_）是否处于活动状态（已在执行中），如果是，表示机器人正在执行行动，无法为外部用户制定计划
    if(as_->isActive()){
      // 指示move_base必须处于非活动状态才能为外部用户制定计划
      ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
      // 表示计划的请求处理失败
      return false;
    }
    //make sure we have a costmap for our planner
    // 检查是否有有效的规划器代价地图
    if(planner_costmap_ros_ == NULL){
      ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
      return false;
    }
    // 存储计划的起始姿态
    geometry_msgs::PoseStamped start;
    //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
    // 如果请求中的起始姿态的frame_id为空，表示用户没有指定起始姿态，这时将使用机器人的当前姿态作为起始姿态
    if(req.start.header.frame_id.empty())
    {
        // 存储机器人的全局姿态
        geometry_msgs::PoseStamped global_pose;
        // 尝试获取机器人的全局姿态信息
        if(!getRobotPose(global_pose, planner_costmap_ros_)){
          ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
          // 表示计划的请求处理失败
          return false;
        }
        // 将请求中指定的起始姿态赋值给start
        start = global_pose;
    }
    else
    {
        // 将请求中指定的起始姿态赋值给start
        start = req.start;
    }
    // 这行代码检查make_plan_clear_costmap_标志是否为真，表示是否需要在制定计划前清除代价地图中的一部分窗口
    if (make_plan_clear_costmap_) {
      //update the copy of the costmap the planner uses
      // 清除代价地图中的一部分窗口。窗口的大小是由两倍的clearing_radius_决定的
      clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);
    }

    //first try to make a plan to the exact desired goal
    // 创建一个存储全局规划路径的std::vector<geometry_msgs::PoseStamped>类型的实例global_plan
    std::vector<geometry_msgs::PoseStamped> global_plan;
    // 尝试使用全局规划器生成一个计划路径，从起始点start到目标点req.goal。如果生成路径失败（返回false）或者路径为空，则执行下面的代码块
    if(!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty()){
      // 输出一条调试级别的消息，指示在确切目标点（req.goal）生成规划路径失败，系统将尝试在容差范围内寻找一个可行的目标
      ROS_DEBUG_NAMED("move_base","Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance",
          req.goal.pose.position.x, req.goal.pose.position.y);

      //search outwards for a feasible goal within the specified tolerance
      geometry_msgs::PoseStamped p; // 存储待搜索的目标姿态
      p = req.goal;                 // 将请求中的目标姿态（req.goal）赋值给p，即将目标姿态作为搜索的起始点
      bool found_legal = false;     // 创建一个布尔变量found_legal，用于指示是否找到合法的目标点
      float resolution = planner_costmap_ros_->getCostmap()->getResolution(); // 获取代价地图的分辨率
      float search_increment = resolution*3.0;    // 设置搜索的增量值，初始值为分辨率的3倍
      // 如果请求中的容差值（req.tolerance）大于0且小于搜索增量值，则将搜索增量值设为容差值
      if(req.tolerance > 0.0 && req.tolerance < search_increment) search_increment = req.tolerance;
      // 使用嵌套的循环结构，尝试在目标点的周围搜索一个可行的目标点
      // max_offset 控制搜索的最大偏移量
      for(float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment) {
        // y_offset 控制目标点在y轴方向的偏移量
        for(float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment) {
          // x_offset 控制目标点在x轴方向的偏移量
          for(float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment) {

            //don't search again inside the current outer layer
            // 首先检查不要在当前外层范围内重新搜索，然后在目标点的周围进行搜索
            if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) continue;

            //search to both sides of the desired goal
            // 两个循环遍历y_mult和x_mult，它们分别为-1和1，用于在目标点两侧搜索
            for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) {

              //if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
              if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;

              for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) {
                if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;
                // 计算搜索的目标点的位置
                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;
                // 使用规划器尝试生成从起始点到搜索到的目标点的规划路径, 如果成功生成路径且路径不为空
                if(planner_->makePlan(start, p, global_plan)){
                  if(!global_plan.empty()){
                    // 如果需要添加不可达的原始目标点到全局计划中
                    if (make_plan_add_unreachable_goal_) {
                      //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                      //(the reachable goal should have been added by the global planner)
                      // 将原始目标点添加到全局计划的末尾，以便在局部规划器可以到达的情况下，尽可能地靠近原始目标点
                      global_plan.push_back(req.goal);
                    }

                    found_legal = true;
                    ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                    break;
                  }
                }
                else{
                  // 标记已找到合法目标点，输出调试消息
                  ROS_DEBUG_NAMED("move_base","Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                }
              }
            }
          }
        }
      }
    }

    //copy the plan into a message to send out
    // 调整响应消息的全局规划路径数组大小为与global_plan一致
    resp.plan.poses.resize(global_plan.size());
    // 将全局规划路径复制到响应消息中
    for(unsigned int i = 0; i < global_plan.size(); ++i){
      resp.plan.poses[i] = global_plan[i];
    }
    // 表示规划路径的请求已经成功处理
    return true;
  }

  MoveBase::~MoveBase(){
    recovery_behaviors_.clear();

    delete dsrv_;

    if(as_ != NULL)
      delete as_;

    if(planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;

    if(controller_costmap_ros_ != NULL)
      delete controller_costmap_ros_;

    planner_thread_->interrupt();
    planner_thread_->join();

    delete planner_thread_;

    delete planner_plan_;
    delete latest_plan_;
    delete controller_plan_;

    planner_.reset();
    tc_.reset();
  }
  // 定义了名为makePlan的函数，用于生成规划路径
  bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    // 创建一个名为lock的互斥锁，用于在规划器使用代价地图时锁定代价地图的互斥锁，确保安全地访问代价地图
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

    //make sure to set the plan to be empty initially
    // 清空规划路径数组，确保开始时路径为空
    plan.clear();

    //since this gets called on handle activate
    // 检查是否存在有效的规划器代价地图, 如果代价地图为空（NULL），输出错误消息，指示无法生成全局规划路径
    if(planner_costmap_ros_ == NULL) {
      ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    //get the starting pose of the robot
    // 将获取的机器人姿态作为起始姿态
    geometry_msgs::PoseStamped global_pose;
    if(!getRobotPose(global_pose, planner_costmap_ros_)) {
      ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      return false;
    }
    // 将获取的机器人姿态作为起始姿态
    const geometry_msgs::PoseStamped& start = global_pose;

    //if the planner fails or returns a zero length plan, planning failed
    // 如果规划失败或生成的路径长度为0，返回false
    if(!planner_->makePlan(start, goal, plan) || plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }
    // 表示规划成功
    return true;
  }

  void MoveBase::publishZeroVelocity(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }
  // isQuaternionValid方法用于检查给定的四元数是否适用于导航目标。
  // 首先检查四元数是否包含NaN或infs，然后检查四元数的长度是否接近于零，
  // 最后检查四元数是否使z轴接近垂直。如果任何检查不通过，将输出相应的错误消息，并返回false表示四元数无效。
  // 如果所有检查都通过，将返回true表示四元数有效
  bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion& q){
    //first we need to check if the quaternion has nan's or infs
    // 检查四元数的各个分量是否包含NaN或无穷大
    // 如果任何分量为非有限值，输出错误消息，表示四元数包含NaN或infs，无法作为导航目标
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }
    // 创建一个tf2::Quaternion实例tf_q，使用传入的四元数的分量初始化
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    // 检查四元数的长度是否接近于零
    // 如果四元数的长度的平方小于1e-6，输出错误消息，表示四元数的长度接近于零，无法作为导航目标
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    // 对四元数进行归一化，确保其长度为1
    tf_q.normalize();

    tf2::Vector3 up(0, 0, 1);
    // 计算单位向量up旋转后与自身点积的结果，通过将其旋转应用到up上
    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));
    // 通过dot计算点积，检查旋转后的向量是否与初始向量相似。如果相似度不高，输出错误消息，表示四元数无效，用于导航的四元数必须使z轴接近垂直
    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }
    // 表示给定的四元数有效
    return true;
  }
  // 定义了名为goalToGlobalFrame的函数，用于将给定的目标姿态从目标坐标系转换到全局坐标系
  geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
    // 获取全局规划器代价地图的全局坐标系ID
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    // 这个全局坐标系ID用于目标姿态的转换
    geometry_msgs::PoseStamped goal_pose, global_pose;
    goal_pose = goal_pose_msg;

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    // 将goal_pose的时间戳（header.stamp）设置为当前时间，以确保获取最新的变换
    goal_pose.header.stamp = ros::Time();
    // 尝试将目标姿态消息从目标坐标系转换到全局坐标系
    try{
      tf_.transform(goal_pose_msg, global_pose, global_frame);
    }
    // 如果转换失败，捕获tf2::TransformException异常，输出警告消息，指示无法将目标姿态转换到全局坐标系
    catch(tf2::TransformException& ex){
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
      // 如果转换成功，将转换后的全局坐标系中的目标姿态返回
      return goal_pose_msg;
    }
    // 如果发生转换异常，返回原始的目标姿态消息
    return global_pose;
  }
  // 唤醒规划器线程
  void MoveBase::wakePlanner(const ros::TimerEvent& event)
  {
    // we have slept long enough for rate
    // 唤醒等待在planner_cond_条件变量上的一个规划器线程
    planner_cond_.notify_one();
  }
  // 唤醒等待在planner_cond_条件变量上的一个规划器线程
  void MoveBase::planThread(){
    ROS_DEBUG_NAMED("move_base_plan_thread","Starting planner thread...");
    ros::NodeHandle n; // 用于该线程的ROS通信
    ros::Timer timer;  // 定时唤醒规划器线程
    bool wait_for_wake = false;
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    while(n.ok()){
      //check if we should run the planner (the mutex is locked)
      while(wait_for_wake || !runPlanner_){
        //if we should not be running the planner then suspend this thread
        // 规划器线程正在启动
        ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
        planner_cond_.wait(lock);
        wait_for_wake = false;
      }
      // 记录线程的开始时间，用于计算规划器线程的执行时间
      ros::Time start_time = ros::Time::now();

      //time to plan! get a copy of the goal and unlock the mutex
      // 将规划器目标姿态（planner_goal_）复制给它
      geometry_msgs::PoseStamped temp_goal = planner_goal_;
      // 解锁互斥锁，允许其他线程操作
      lock.unlock();
      // 表示规划器线程正在进行规划
      ROS_DEBUG_NAMED("move_base_plan_thread","Planning...");

      //run planner
      // 清空规划器计划路径
      planner_plan_->clear();
      // 在n.ok()为true（表示ROS节点正常运行）且成功生成规划路径（gotPlan为true)
      bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

      if(gotPlan){
        // 输出调试消息，表示规划器线程正在进行规划
        ROS_DEBUG_NAMED("move_base_plan_thread","Got Plan with %zu points!", planner_plan_->size());
        //pointer swap the plans under mutex (the controller will pull from latest_plan_)
        // 将temp_plan指向的规划路径赋值给planner_plan_
        std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;
        // 重新获取互斥锁以更新指针交换的操作
        lock.lock();
        planner_plan_ = latest_plan_; // 最近一次成功生成规划的时间
        latest_plan_ = temp_plan;   // 最近一次成功生成规划的时间
        last_valid_plan_ = ros::Time::now();
        planning_retries_ = 0;      // 规划重试次数清零
        new_global_plan_ = true;    // 已生成新的全局规划路径
        // 输出调试消息，表示成功生成规划路径，并指示规划路径的点数
        ROS_DEBUG_NAMED("move_base_plan_thread","Generated a plan from the base_global_planner");

        //make sure we only start the controller if we still haven't reached the goal
        // 检查是否应该启动控制器（runPlanner_为true）以确保只有在尚未达到目标时才启动控制器
        // 如果满足条件，将state_设置为CONTROLLING状态
        if(runPlanner_)
          state_ = CONTROLLING;
        // 将runPlanner_设置为false，表示停止规划
        if(planner_frequency_ <= 0)
          runPlanner_ = false;
        // 解锁互斥锁，以允许其他线程操作
        lock.unlock();
      }
      //if we didn't get a plan and we are in the planning state (the robot isn't moving)
      // 在没有成功生成规划路径且当前状态为规划状态（state_为PLANNING）时执行以下步骤
      else if(state_==PLANNING){
        // 未成功生成规划路径
        ROS_DEBUG_NAMED("move_base_plan_thread","No Plan...");
        ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

        //check if we've tried to make a plan for over our time limit or our maximum number of retries
        //issue #496: we stop planning when one of the conditions is true, but if max_planning_retries_
        //is negative (the default), it is just ignored and we have the same behavior as ever
        lock.lock();
        planning_retries_++;
        // 计算尝试规划的结束时间
        // 在互斥锁中检查是否已经尝试规划超过了时间限制或最大重试次数的条件
        if(runPlanner_ &&
           (ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_))){
          //we'll move into our obstacle clearing mode
          state_ = CLEARING;    // 表示进入障碍物清除模式
          runPlanner_ = false;  // 停止规划 // proper solution for issue #523
          publishZeroVelocity();        // 发布零速度，停止机器人运动
          recovery_trigger_ = PLANNING_R; // 触发规划恢复动作
        }

        lock.unlock();
      }

      //take the mutex for the next iteration
      lock.lock();

      //setup sleep interface if needed
      // 需要定时唤醒规划器线程
      if(planner_frequency_ > 0){
        // 需要定时唤醒规划器线程
        ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
        // 计算从当前时间到下一次规划器线程应该唤醒的时间的时间间隔
        if (sleep_time > ros::Duration(0.0)){
          wait_for_wake = true; // 等待唤醒
          // 创建一个定时器timer，在计算出的休眠时间后触发wakePlanner方法
          timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
        }
      }
    }
  }
  // 处理移动基座目标的执行请求
  void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
  {
    // 检查目标姿态的四元数是否有效，即检查是否存在NaN或无穷大的情况。
    // 如果四元数无效，表示目标姿态不合法，将任务标记为失败，并返回错误信息
    if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }
    // 将移动基座目标消息中的局部目标姿态（move_base_goal->target_pose）转换为全局坐标系中的目标姿态，
    // 即将目标姿态转换到规划器使用的全局坐标系中
    geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);

    publishZeroVelocity();  // 停止机器人的运动
    //we have a goal so start the planner
    // 保护下面的操作
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    // 将要规划的目标位置
    planner_goal_ = goal;
    // 允许启动规划器
    runPlanner_ = true;
    // 唤醒规划器线程，以便开始进行规划
    planner_cond_.notify_one();
    // 解锁互斥锁
    lock.unlock();
    // 发布当前目标姿态到名为current_goal的ROS话题，以便其他节点可以获取机器人的当前目标位置
    current_goal_pub_.publish(goal);

    // 控制循环的频率
    ros::Rate r(controller_frequency_);
    // 之前是否已经关闭了代价地图
    if(shutdown_costmaps_){
      // 重新启动之前已关闭的代价地图
      ROS_DEBUG_NAMED("move_base","Starting up costmaps that were shut down previously");
      // 启动之前已关闭的全局规划器的代价地图
      planner_costmap_ros_->start();
      // 启动之前已关闭的控制器的代价地图
      controller_costmap_ros_->start();
    }

    //we want to make sure that we reset the last time we had a valid plan and control
    last_valid_control_ = ros::Time::now();
    last_valid_plan_ = ros::Time::now();
    last_oscillation_reset_ = ros::Time::now();
    planning_retries_ = 0;

    ros::NodeHandle n;
    while(n.ok()) // 条件为ROS节点是否仍然在运行
    {
      if(c_freq_change_)    // 控制器频率是否发生了变化
      {
        // 表示正在设置控制器的频率为新的controller_frequency_
        ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
        // 更新ros::Rate对象r的频率为新的controller_frequency_
        r = ros::Rate(controller_frequency_);
        // 将c_freq_change_设置为false，表示已经处理了控制器频率的变化
        c_freq_change_ = false;
      }

      if(as_->isPreemptRequested()){    // 检查是否收到了取消目标请求
        if(as_->isNewGoalAvailable()){  // 检查是否有新的目标可用
          //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
          // 接受新的目标
          move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();
          // 检查新目标的姿态的四元数是否有效
          if(!isQuaternionValid(new_goal.target_pose.pose.orientation)){
            // 如果新目标的姿态无效，将任务标记为失败并返回错误信息
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
          }
          // 将新目标消息中的局部目标姿态转换为全局坐标系中的目标姿态
          goal = goalToGlobalFrame(new_goal.target_pose);

          //we'll make sure that we reset our state for the next execution cycle
          recovery_index_ = 0;
          state_ = PLANNING;

          //we have a new goal so make sure the planner is awake
          lock.lock();
          planner_goal_ = goal;
          runPlanner_ = true;
          planner_cond_.notify_one();
          lock.unlock();

          //publish the goal point to the visualizer
          ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
          current_goal_pub_.publish(goal);

          //make sure to reset our timeouts and counters
          last_valid_control_ = ros::Time::now();
          last_valid_plan_ = ros::Time::now();
          last_oscillation_reset_ = ros::Time::now();
          planning_retries_ = 0;
        }
        else {
          //if we've been preempted explicitly we need to shut things down
          resetState();

          //notify the ActionServer that we've successfully preempted
          ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
          as_->setPreempted();

          //we'll actually return from execute after preempting
          return;
        }
      }

      //we also want to check if we've changed global frames because we need to transform our goal pose
      if(goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID()){
        goal = goalToGlobalFrame(goal);

        //we want to go back to the planning state for the next execution cycle
        recovery_index_ = 0;
        state_ = PLANNING;

        //we have a new goal so make sure the planner is awake
        lock.lock();
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        //publish the goal point to the visualizer
        ROS_DEBUG_NAMED("move_base","The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
        current_goal_pub_.publish(goal);

        //make sure to reset our timeouts and counters
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;
      }

      //for timing that gives real time even in simulation
      // 获取当前的Wall时间作为循环开始时间
      ros::WallTime start = ros::WallTime::now();

      //the real work on pursuing a goal is done here
      // 执行控制周期，将目标作为参数传递给executeCycle函数，并将其返回的结果存储在done变量中
      bool done = executeCycle(goal);

      //if we're done, then we'll return from execute
      // 如果done为true，表示控制周期已完成，直接从execute函数返回
      if(done)
        return;

      //check if execution of the goal has completed in some way
      // 检查执行目标是否以某种方式完成
      ros::WallDuration t_diff = ros::WallTime::now() - start;
      // 计算控制周期的实际执行时间差
      ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());

      r.sleep();
      //make sure to sleep for the remainder of our cycle time
      // 如果控制周期的实际执行时间超过了期望的控制器频率，且当前状态为CONTROLLING，则输出警告消息，表示控制循环没有达到预期的频率
      if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }

    //wake up the planner thread so that it can exit cleanly
    lock.lock();
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    //if the node is killed then we'll abort and return
    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    return;
  }

  // 计算两个姿态之间的欧几里德距离
  double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    // 用hypot函数计算两个点之间的欧几里德距离，即从p1到p2的距离
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }

  // 执行控制周期的主要逻辑。传入的参数是目标姿态
  bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal){
    // 使用互斥锁锁定配置，确保在控制周期执行期间不会修改配置
    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    //we need to be able to publish velocity commands
    // 创建一个用于存储速度命令的geometry_msgs::Twist消息
    geometry_msgs::Twist cmd_vel;

    //update feedback to correspond to our curent position
    // 获取机器人的全局姿态
    geometry_msgs::PoseStamped global_pose;
    getRobotPose(global_pose, planner_costmap_ros_);
    const geometry_msgs::PoseStamped& current_position = global_pose;

    //push the feedback out
    // 将当前姿态作为基本位置放入反馈消息中
    move_base_msgs::MoveBaseFeedback feedback;
    feedback.base_position = current_position;
    // 通过Action服务器发布反馈消息，包含当前机器人的基本位置
    as_->publishFeedback(feedback);

    //check to see if we've moved far enough to reset our oscillation timeout
    // 检查是否移动了足够远，以重置振荡超时
    // 使用distance函数计算当前姿态与oscillation_pose_之间的距离，如果大于等于oscillation_distance_，则表示机器人已经移动足够远，可以重置振荡超时
    if(distance(current_position, oscillation_pose_) >= oscillation_distance_)
    {
      // 更新上次重置振荡超时的时间为当前时间
      last_oscillation_reset_ = ros::Time::now();
      // 将当前姿态赋值给oscillation_pose_，用于跟踪振荡检测
      oscillation_pose_ = current_position;

      //if our last recovery was caused by oscillation, we want to reset the recovery index
      // 如果上一个恢复操作是由振荡引起的，将恢复索引重置为0
      if(recovery_trigger_ == OSCILLATION_R)
        recovery_index_ = 0;
    }

    //check that the observation buffers for the costmap are current, we don't want to drive blind
    // 检查代价地图的观测缓冲区是否是最新的，以确保机器人具有准确的感知数据
    if(!controller_costmap_ros_->isCurrent()){
      ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
      publishZeroVelocity(); // 发布零速度命令以确保机器人不会运动
      return false;
    }

    //if we have a new plan then grab it and give it to the controller
    // 如果有一个新的全局规划，则获取该规划并传递给控制器
    if(new_global_plan_){
      //make sure to set the new plan flag to false
      // 表示已经处理了新的全局规划
      new_global_plan_ = false;
      // 表示获取了一个新的规划并且正在交换指针
      ROS_DEBUG_NAMED("move_base","Got a new plan...swap pointers");

      //do a pointer swap under mutex
      // 创建一个临时指针来存储控制器规划
      std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;
      // 在互斥锁的保护下执行指针交换，将最新的全局规划指针赋值给控制器规划指针
      boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
      controller_plan_ = latest_plan_;
      latest_plan_ = temp_plan;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base","pointers swapped!");

      // 如果无法将全局规划传递给控制器
      if(!tc_->setPlan(*controller_plan_)){
        //ABORT and SHUTDOWN COSTMAPS
        // 输出错误消息，表示无法将全局规划传递给控制器
        ROS_ERROR("Failed to pass global plan to the controller, aborting.");
        // 调用resetState函数，用于重置导航状态和清理相关数据
        resetState();

        //disable the planner thread
        lock.lock();
        // 设置Action服务器的目标为"失败"状态，并附带错误消息，表示无法将全局规划传递给控制器
        runPlanner_ = false;
        lock.unlock();

        as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
        return true;  // 表示导航执行周期已经结束
      }

      //make sure to reset recovery_index_ since we were able to find a valid plan
      // 如果成功将全局规划传递给控制器，重置恢复索引，因为已经找到有效的规划
      if(recovery_trigger_ == PLANNING_R)
        recovery_index_ = 0;
    }

    //the move_base state machine, handles the control logic for navigation
    // 切换到状态机中的当前状态并执行相应的操作
    switch(state_){
      //if we are in a planning state, then we'll attempt to make a plan
      // 表示当处于规划状态时
      case PLANNING:
        {
          // 锁定互斥锁，以确保在修改runPlanner_标志时不会发生竞态条件
          boost::recursive_mutex::scoped_lock lock(planner_mutex_);
          runPlanner_ = true;         // 表示要开始运行规划器线程
          planner_cond_.notify_one(); // 通知规划器线程可以运行
        }
        ROS_DEBUG_NAMED("move_base","Waiting for plan, in the planning state.");
        break;

      //if we're controlling, we'll attempt to find valid velocity commands
      // 表示当处于控制状态时，输出调试消息，表示当前处于控制状态
      case CONTROLLING:
        ROS_DEBUG_NAMED("move_base","In controlling state.");

        //check to see if we've reached our goal
        if(tc_->isGoalReached()){
          ROS_DEBUG_NAMED("move_base","Goal reached!");
          resetState();

          //disable the planner thread
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
          return true;
        }

        //check for an oscillation condition
        if(oscillation_timeout_ > 0.0 &&
            last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
        {
          publishZeroVelocity();
          state_ = CLEARING;
          recovery_trigger_ = OSCILLATION_R;
        }

        {
         boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));

        if(tc_->computeVelocityCommands(cmd_vel)){
          ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                           cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
          last_valid_control_ = ros::Time::now();
          //make sure that we send the velocity command to the base
          vel_pub_.publish(cmd_vel);
          if(recovery_trigger_ == CONTROLLING_R)
            recovery_index_ = 0;
        }
        else {
          ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
          ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

          //check if we've tried to find a valid control for longer than our time limit
          if(ros::Time::now() > attempt_end){
            //we'll move into our obstacle clearing mode
            publishZeroVelocity();
            state_ = CLEARING;
            recovery_trigger_ = CONTROLLING_R;
          }
          else{
            //otherwise, if we can't find a valid control, we'll go back to planning
            last_valid_plan_ = ros::Time::now();
            planning_retries_ = 0;
            state_ = PLANNING;
            publishZeroVelocity();

            //enable the planner thread in case it isn't running on a clock
            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            runPlanner_ = true;
            planner_cond_.notify_one();
            lock.unlock();
          }
        }
        }

        break;

      //we'll try to clear out space with any user-provided recovery behaviors
      case CLEARING:
        ROS_DEBUG_NAMED("move_base","In clearing/recovery state");
        //we'll invoke whatever recovery behavior we're currently on if they're enabled
        if(recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size()){
          ROS_DEBUG_NAMED("move_base_recovery","Executing behavior %u of %zu", recovery_index_+1, recovery_behaviors_.size());

          move_base_msgs::RecoveryStatus msg;
          msg.pose_stamped = current_position;
          msg.current_recovery_number = recovery_index_;
          msg.total_number_of_recoveries = recovery_behaviors_.size();
          msg.recovery_behavior_name =  recovery_behavior_names_[recovery_index_];

          recovery_status_pub_.publish(msg);

          recovery_behaviors_[recovery_index_]->runBehavior();

          //we at least want to give the robot some time to stop oscillating after executing the behavior
          last_oscillation_reset_ = ros::Time::now();

          //we'll check if the recovery behavior actually worked
          ROS_DEBUG_NAMED("move_base_recovery","Going back to planning state");
          last_valid_plan_ = ros::Time::now();
          planning_retries_ = 0;
          state_ = PLANNING;

          //update the index of the next recovery behavior that we'll try
          recovery_index_++;
        }
        else{
          ROS_DEBUG_NAMED("move_base_recovery","All recovery behaviors have failed, locking the planner and disabling it.");
          //disable the planner thread
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          ROS_DEBUG_NAMED("move_base_recovery","Something should abort after this.");

          if(recovery_trigger_ == CONTROLLING_R){
            ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid control. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == PLANNING_R){
            ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid plan. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == OSCILLATION_R){
            ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Robot is oscillating. Even after executing recovery behaviors.");
          }
          resetState();
          return true;
        }
        break;
      default:
        ROS_ERROR("This case should never be reached, something is wrong, aborting");
        resetState();
        //disable the planner thread
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Reached a case that should not be hit in move_base. This is a bug, please report it.");
        return true;
    }

    //we aren't done yet
    return false;
  }

  bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node){
    XmlRpc::XmlRpcValue behavior_list;
    if(node.getParam("recovery_behaviors", behavior_list)){
      if(behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
        for(int i = 0; i < behavior_list.size(); ++i){
          if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
            if(behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type")){
              //check for recovery behaviors with the same name
              for(int j = i + 1; j < behavior_list.size(); j++){
                if(behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                  if(behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type")){
                    std::string name_i = behavior_list[i]["name"];
                    std::string name_j = behavior_list[j]["name"];
                    if(name_i == name_j){
                      ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.",
                          name_i.c_str());
                      return false;
                    }
                  }
                }
              }
            }
            else{
              ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
              return false;
            }
          }
          else{
            ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                behavior_list[i].getType());
            return false;
          }
        }

        //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
        for(int i = 0; i < behavior_list.size(); ++i){
          try{
            //check if a non fully qualified name has potentially been passed in
            if(!recovery_loader_.isClassAvailable(behavior_list[i]["type"])){
              std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
              for(unsigned int i = 0; i < classes.size(); ++i){
                if(behavior_list[i]["type"] == recovery_loader_.getName(classes[i])){
                  //if we've found a match... we'll get the fully qualified name and break out of the loop
                  ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                      std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                  behavior_list[i]["type"] = classes[i];
                  break;
                }
              }
            }

            boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

            //shouldn't be possible, but it won't hurt to check
            if(behavior.get() == NULL){
              ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
              return false;
            }

            //initialize the recovery behavior with its name
            behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behavior_names_.push_back(behavior_list[i]["name"]);
            recovery_behaviors_.push_back(behavior);
          }
          catch(pluginlib::PluginlibException& ex){
            ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
            return false;
          }
        }
      }
      else{
        ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.",
            behavior_list.getType());
        return false;
      }
    }
    else{
      //if no recovery_behaviors are specified, we'll just load the defaults
      return false;
    }

    //if we've made it here... we've constructed a recovery behavior list successfully
    return true;
  }

  //we'll load our default recovery behaviors here
  void MoveBase::loadDefaultRecoveryBehaviors(){
    recovery_behaviors_.clear();
    try{
      //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
      ros::NodeHandle n("~");
      n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
      n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

      //first, we'll load a recovery behavior to clear the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behavior_names_.push_back("conservative_reset");
      recovery_behaviors_.push_back(cons_clear);

      //next, we'll load a recovery behavior to rotate in place
      boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
      if(clearing_rotation_allowed_){
        rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        recovery_behavior_names_.push_back("rotate_recovery");
        recovery_behaviors_.push_back(rotate);
      }

      //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behavior_names_.push_back("aggressive_reset");
      recovery_behaviors_.push_back(ags_clear);

      //we'll rotate in-place one more time
      if(clearing_rotation_allowed_){
        recovery_behaviors_.push_back(rotate);
        recovery_behavior_names_.push_back("rotate_recovery");
      }
    }
    catch(pluginlib::PluginlibException& ex){
      ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
    }

    return;
  }

  void MoveBase::resetState(){
    // Disable the planner thread
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    runPlanner_ = false;
    lock.unlock();

    // Reset statemachine
    state_ = PLANNING;
    recovery_index_ = 0;
    recovery_trigger_ = PLANNING_R;
    publishZeroVelocity();

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }
  }

  bool MoveBase::getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap)
  {
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = robot_base_frame_;
    robot_pose.header.stamp = ros::Time(); // latest available
    ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

    // get robot pose on the given costmap frame
    try
    {
      tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
    }
    catch (tf2::LookupException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }

    // check if global_pose time stamp is within costmap transform tolerance
    if (!global_pose.header.stamp.isZero() &&
        current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
    {
      ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. " \
                        "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                        current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
      return false;
    }

    return true;
  }
};
