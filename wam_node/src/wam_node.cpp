#include <unistd.h>
#include <math.h>
#include <ctime>
#include <cstdlib>
#include <fstream>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <Eigen/Core>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include "wam_common/CartPointTraj.h"
#include "wam_common/JointTraj.h"
#include "wam_common/ForceTorque.h"
#include "wam_common/GravityComp.h"
#include "wam_common/Hold.h"
#include "wam_common/JointMove.h"
#include "wam_common/PoseMove.h"
#include "wam_common/CartPosMove.h"
#include "wam_common/OrtnMove.h"
#include "wam_common/BHandFingerPos.h"
#include "wam_common/BHandGraspPos.h"
#include "wam_common/BHandSpreadPos.h"
#include "wam_common/BHandFingerVel.h"
#include "wam_common/BHandGraspVel.h"
#include "wam_common/BHandSpreadVel.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "trajectory_msgs/JointTrajectory.h"

#include <barrett/math.h> 
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>
#include <barrett/systems/wam.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/log.h>
#include <barrett/products/force_torque_sensor.h>


#include "sys_time.h"
#include "gains.h"

typedef tf::Quaternion btQuaternion;
typedef tf::Matrix3x3 btMatrix3x3;

using namespace barrett;

static const bool LOG_DATA = true;  
static const int PUBLISH_FREQ = 250; // Default Control Loop / Publishing Frequency
// BHand publishing currently experiencing latency (65-75% of target Hz)
static const int BHAND_PUBLISH_FREQ = 250; // Publishing Frequency for the BarretHand
static const double SPEED = 0.5; // Default Cartesian Velocity
static const double TORQUE_LIM = 3.0;
static const double JP_VEL_LIM = 1.5;
static const double TRANSITION_DURATION = 0.5;  // seconds

//WamNode Class
template<size_t DOF>
  class WamNode
  {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  protected:

    typedef barrett::Hand::jp_type hjp_type;
    typedef boost::tuple<double, jp_type> input_jp_type;
    typedef boost::tuple<double, cp_type> input_cp_type;
    typedef boost::tuple<double, Eigen::Quaterniond> input_quat_type;
    typedef boost::tuple<double, double, jp_type, jp_type, jv_type, jt_type> jpLog_tuple_type;  

    std::vector<input_jp_type>* jpVec;
    math::Spline<jp_type>* jpSpline;
    systems::Callback<double, jp_type>* jpTrajectory;

    std::vector<input_cp_type>* cpVec;
    math::Spline<cp_type>* cpSpline;
    systems::Callback<double, cp_type>* cpTrajectory;

    systems::Wam<DOF>& wam;
    Hand* hand;
    ForceTorqueSensor* ft_sensor;
    hjp_type hjp_init;
    jp_type jp, jp_cmd, jp_home, jp_init;
    cp_type cp_cmd, rt_cv_cmd;
    Eigen::Quaterniond ortn_cmd;
    pose_type pose_cmd;
    systems::Ramp ramp;

    // Data logging
    char tmpLogFile[100];
    char csvLogFile[100];
    systems::TupleGrouper<double, double, jp_type, jp_type, jv_type, jt_type> jpLog_tuple_grouper;
    systems::PeriodicDataLogger<jpLog_tuple_type> * data_logger;
    systems::dTime sys_dtime;

    //Subscribers
    ros::Subscriber cart_traj_sub;
    ros::Subscriber joint_traj_sub;

    //Published Topics
    sensor_msgs::JointState wam_joint_state, bhand_joint_state;
    geometry_msgs::PoseStamped wam_pose;
    wam_common::ForceTorque ft_sensor_state;

    //Publishers
    ros::Publisher wam_joint_state_pub, bhand_joint_state_pub, wam_pose_pub, ft_sensor_pub;

    //Services
    ros::ServiceServer gravity_srv, go_home_srv, go_init_srv, hold_jpos_srv, hold_cpos_srv;
    ros::ServiceServer joint_move_srv, pose_move_srv;
    ros::ServiceServer cart_move_srv, ortn_move_srv, hand_close_srv;
    ros::ServiceServer hand_open_grsp_srv, hand_close_grsp_srv, hand_open_sprd_srv;
    ros::ServiceServer hand_close_sprd_srv, hand_fngr_pos_srv, hand_fngr_vel_srv;
    ros::ServiceServer hand_grsp_pos_srv, hand_grsp_vel_srv, hand_sprd_pos_srv;
    ros::ServiceServer hand_sprd_vel_srv;

  public:
    WamNode(systems::Wam<DOF>& wam_) :
        // wam(wam_), hand(NULL), ramp(NULL, SPEED)
        wam(wam_), hand(NULL), ramp(NULL)
    {
    }
    void
    init(ProductManager& pm);
    void
    close(ProductManager& pm);

    ~WamNode()
    {

    }

    bool
    gravity(wam_common::GravityComp::Request &req, wam_common::GravityComp::Response &res);
    bool
    goHome(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool
    goInit(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool
    holdJPos(wam_common::Hold::Request &req, wam_common::Hold::Response &res);
    bool
    holdCPos(wam_common::Hold::Request &req, wam_common::Hold::Response &res);
    bool
    jointMove(wam_common::JointMove::Request &req, wam_common::JointMove::Response &res);
    bool
    poseMove(wam_common::PoseMove::Request &req, wam_common::PoseMove::Response &res);
    bool
    cartMove(wam_common::CartPosMove::Request &req, wam_common::CartPosMove::Response &res);
    bool
    ortnMove(wam_common::OrtnMove::Request &req, wam_common::OrtnMove::Response &res);
    bool
    handOpenGrasp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool
    handCloseGrasp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool
    handOpenSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool
    handCloseSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool
    handFingerPos(wam_common::BHandFingerPos::Request &req, wam_common::BHandFingerPos::Response &res);
    bool
    handGraspPos(wam_common::BHandGraspPos::Request &req, wam_common::BHandGraspPos::Response &res);
    bool
    handSpreadPos(wam_common::BHandSpreadPos::Request &req, wam_common::BHandSpreadPos::Response &res);
    bool
    handFingerVel(wam_common::BHandFingerVel::Request &req, wam_common::BHandFingerVel::Response &res);
    bool
    handGraspVel(wam_common::BHandGraspVel::Request &req, wam_common::BHandGraspVel::Response &res);
    bool
    handSpreadVel(wam_common::BHandSpreadVel::Request &req, wam_common::BHandSpreadVel::Response &res);
    void
    cartTrajCB(const wam_common::CartPointTraj::ConstPtr& msg);
    void  
    jointTrajCB(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
    void
    publishWam(ProductManager& pm);
    void  
    publishFT(ProductManager& pm);
    void
    publishHand(void);
  };

// Templated Initialization Function
template<size_t DOF>
  void WamNode<DOF>::init(ProductManager& pm)
  {
    ros::NodeHandle n_("wam"); // WAM specific nodehandle
    ros::NodeHandle nh_("bhand"); // BarrettHand specific nodehandle

    pm.getExecutionManager()->startManaging(ramp); //starting ramp manager

    // ft_sensor.update();
    ft_sensor = pm.getForceTorqueSensor();

    // Print and Set Gains
    printGains(wam);

    // Default values
    // double jp_Kp[] = {900., 2500., 600., 500., 50., 50., 8.};
    // double jp_Kd[] = {10., 20., 5., 2., 0.5, 0.5, 0.05};
    // double jp_Ki[] = {2.5, 5, 2, 0.5, 0.5, 0.5, 0.1};

    double jp_Kp[] = {1500., 2500., 600., 500., 50., 50., 50.};
    double jp_Kd[] = {50., 20., 5., 2., 0.5, 0.5, 0.05};
    double jp_Ki[] = {25., 10., 4., 1., 5., 5., 5.};

    setJpGains<DOF>(wam, jp_Kp, jp_Kd, jp_Ki);

    printGains(wam);

    ROS_INFO(" \n %zu-DOF WAM", DOF);

    // Adjust the torque limits to allow for BarrettHand movements at extents
    pm.getSafetyModule()->setTorqueLimit(TORQUE_LIM);
    pm.getSafetyModule()->setVelocityLimit(JP_VEL_LIM);

    // Define initialization position
    jp_init[0] = 0.0;
    jp_init[1] = -1.967;
    jp_init[2] = 0.0;
    jp_init[3] = 2.5;
    jp_init[4] = 0.0;
    jp_init[5] = -0.5;
    jp_init[6] = 0.0;

    usleep(500000);
    wam.moveTo(jp_init);


    if (pm.foundHand()) //Does the following only if a BarrettHand is present
    {
      std::cout << "Barrett Hand" << std::endl;
      hand = pm.getHand();

      usleep(500000);
      hand->initialize();
      // Update state & sensor measurements
      hand->update();

      /* Default: mid-grasp */	
      //hand->open();

      //hjp_init[0] = 1.2;
      //hjp_init[1] = 1.2;
      //hjp_init[2] = 1.2;
      //hjp_init[3] = 0;
      //hand->trapezoidalMove(hjp_init);
	
      /* Push config */
      hand->close(Hand::SPREAD);

      //Hand::jp_type hi = hand->getInnerLinkPosition(); 
      //Hand::jp_type ho = hand->getOuterLinkPosition();
      //for (size_t i = 0; i < 4; i++)
      //  hjp_init[i] = hi[i];
      //for (size_t j = 0; j < 3; j++)
      //  hjp_init[j + 4] = ho[j];

      hjp_init[0] = 0.5;
      hjp_init[1] = 2.4;
      hjp_init[2] = 2.4;
      hjp_init[3] = 3.1415;
      hand->trapezoidalMove(hjp_init);

      //Publishing the following topics only if there is a BarrettHand present
      bhand_joint_state_pub = nh_.advertise < sensor_msgs::JointState > ("joint_states", 1); // bhand/joint_states

      //Advertise the following services only if there is a BarrettHand present
      hand_open_grsp_srv = nh_.advertiseService("open_grasp", &WamNode::handOpenGrasp, this); // bhand/open_grasp
      hand_close_grsp_srv = nh_.advertiseService("close_grasp", &WamNode::handCloseGrasp, this); // bhand/close_grasp
      hand_open_sprd_srv = nh_.advertiseService("open_spread", &WamNode::handOpenSpread, this); // bhand/open_spread
      hand_close_sprd_srv = nh_.advertiseService("close_spread", &WamNode::handCloseSpread, this); // bhand/close_spread
      hand_fngr_pos_srv = nh_.advertiseService("finger_pos", &WamNode::handFingerPos, this); // bhand/finger_pos
      hand_grsp_pos_srv = nh_.advertiseService("grasp_pos", &WamNode::handGraspPos, this); // bhand/grasp_pos
      hand_sprd_pos_srv = nh_.advertiseService("spread_pos", &WamNode::handSpreadPos, this); // bhand/spread_pos
      hand_fngr_vel_srv = nh_.advertiseService("finger_vel", &WamNode::handFingerVel, this); // bhand/finger_vel
      hand_grsp_vel_srv = nh_.advertiseService("grasp_vel", &WamNode::handGraspVel, this); // bhand/grasp_vel
      hand_sprd_vel_srv = nh_.advertiseService("spread_vel", &WamNode::handSpreadVel, this); // bhand/spread_vel

      //Set up the BarrettHand joint state publisher
      // const char* bhand_jnts[] = {"inner_f1", "inner_f2", "inner_f3", "spread", "outer_f1", "outer_f2", "outer_f3"};
      const char* bhand_jnts[] = {"bhand_f1_j1", "bhand_f2_j1", "bhand_f3_j1", "bhand_spread", "bhand_f1_j2", "bhand_f2_j2", "bhand_f3_j2"};
      std::vector < std::string > bhand_joints(bhand_jnts, bhand_jnts + 7);
      bhand_joint_state.name.resize(7);
      bhand_joint_state.name = bhand_joints;
      bhand_joint_state.position.resize(7);
    }

    wam.gravityCompensate(true); // Turning on Gravity Compenstation by Default when starting the WAM Node

    //Setting up WAM joint state publisher
    const char* wam_jnts[] = {"wam_j1", "wam_j2", "wam_j3", "wam_j4", "wam_j5", "wam_j6", "wam_j7"};
    std::vector < std::string > wam_joints(wam_jnts, wam_jnts + 7);
    wam_joint_state.name = wam_joints;
    wam_joint_state.name.resize(DOF);
    wam_joint_state.position.resize(DOF);
    wam_joint_state.velocity.resize(DOF);
    wam_joint_state.effort.resize(DOF);

    //Publishing the following rostopicss
    wam_joint_state_pub = n_.advertise < sensor_msgs::JointState > ("joint_states", 1); // wam/joint_states
    wam_pose_pub = n_.advertise < geometry_msgs::PoseStamped > ("pose", 1); // wam/pose
    ft_sensor_pub = n_.advertise < wam_common::ForceTorque > ("ft_sensor", 1); // wam/ft_sensor

    //Subscribing to the following rostopics
    cart_traj_sub = n_.subscribe("cart_traj_cmd", 1, &WamNode::cartTrajCB, this); 
    joint_traj_sub = n_.subscribe("joint_traj_cmd", 1, &WamNode::jointTrajCB, this); 

    //Advertising the following rosservices
    gravity_srv = n_.advertiseService("gravity_comp", &WamNode::gravity, this); // wam/gravity_comp
    go_home_srv = n_.advertiseService("go_home", &WamNode::goHome, this); // wam/go_home
    go_init_srv = n_.advertiseService("go_init", &WamNode::goInit, this); // wam/go_home
    hold_jpos_srv = n_.advertiseService("hold_joint_pos", &WamNode::holdJPos, this); // wam/hold_joint_pos
    hold_cpos_srv = n_.advertiseService("hold_cart_pos", &WamNode::holdCPos, this); // wam/hold_cart_pos
    joint_move_srv = n_.advertiseService("joint_move", &WamNode::jointMove, this); // wam/joint_move
    pose_move_srv = n_.advertiseService("pose_move", &WamNode::poseMove, this); // wam/pose_move
    cart_move_srv = n_.advertiseService("cart_move", &WamNode::cartMove, this); // wam/cart_pos_move
    ortn_move_srv = n_.advertiseService("ortn_move", &WamNode::ortnMove, this); // wam/ortn_move


    // Data Logging
    if (LOG_DATA) {
      strcpy(tmpLogFile, "/tmp/dataLogXXXXXX");
      strcpy(csvLogFile, "/home/robot/dataLog.csv");
      data_logger = new systems::PeriodicDataLogger<jpLog_tuple_type>( pm.getExecutionManager(),
      new log::RealTimeWriter<jpLog_tuple_type>(tmpLogFile, pm.getExecutionManager()->getPeriod()),
      1);
    } 

  }

template<size_t DOF>
  void WamNode<DOF>::close(ProductManager& pm)
  {
   
    // Data Logging
    if (LOG_DATA) {

      data_logger->closeLog();
      printf("Logging stopped.\n");

      log::Reader<jpLog_tuple_type> lr(tmpLogFile);
   
      if (std:: ifstream(csvLogFile)) {
        std::remove(csvLogFile);
      }

      lr.exportCSV(csvLogFile);
      printf("Output written to %s.\n", csvLogFile);
      std::remove(tmpLogFile);
    }


    wam.moveTo(jp_init);

    if (hand != NULL)
    {
      hand->open(Hand::GRASP, true);
      hand->close(Hand::SPREAD, true);
      hand->idle();
    }

    wam.moveTo(jp_init);
    wam.moveHome();
  
    printf("Please Shift-Idle the WAM...");
    pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
  }


// gravity_comp service callback
template<size_t DOF>
  bool WamNode<DOF>::gravity(wam_common::GravityComp::Request &req, wam_common::GravityComp::Response &res)
  {
    wam.gravityCompensate(req.gravity);
    ROS_INFO("Gravity Compensation Request: %s", (req.gravity) ? "true" : "false");
    return true;
  }

// goHome Function for sending the WAM safely back to its home starting position.
template<size_t DOF>
  bool WamNode<DOF>::goHome(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    ROS_INFO("Returning to Home Position");

    wam.moveTo(jp_init);

    if (hand != NULL)
    {
      hand->open(Hand::GRASP, true);
      hand->close(Hand::SPREAD, true);
    }
    
    wam.moveHome();

    return true;
  }

template<size_t DOF>
  bool WamNode<DOF>::goInit(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    ROS_INFO("Returning to Home Position");

    wam.moveTo(jp_init);

    return true;
  }

//Function to hold WAM Joint Positions
template<size_t DOF>
  bool WamNode<DOF>::holdJPos(wam_common::Hold::Request &req, wam_common::Hold::Response &res)
  {
    ROS_INFO("Joint Position Hold request: %s", (req.hold) ? "true" : "false");

    if (req.hold)
      wam.moveTo(wam.getJointPositions());
    else
      wam.idle();
    return true;
  }

//Function to hold WAM end effector Cartesian Position
template<size_t DOF>
  bool WamNode<DOF>::holdCPos(wam_common::Hold::Request &req, wam_common::Hold::Response &res)
  {
    ROS_INFO("Cartesian Position Hold request: %s", (req.hold) ? "true" : "false");

    if (req.hold)
      wam.moveTo(wam.getToolPosition());
    else
      wam.idle();
    return true;
  }

//Function to command a joint space move to the WAM
/*** Blocking ****/
template<size_t DOF>
  bool WamNode<DOF>::jointMove(wam_common::JointMove::Request &req, wam_common::JointMove::Response &res)
  {
    if (req.joints.size() != DOF)
    {
      ROS_INFO("Request Failed: %zu-DOF request received, must be %zu-DOF", req.joints.size(), DOF);
      return false;
    }
    ROS_INFO("Moving Robot to Commanded Joint Pose");
    for (size_t i = 0; i < DOF; i++)
      jp_cmd[i] = req.joints[i];
    wam.moveTo(jp_cmd, true);

    return true;
  }

//Function to command a pose move to the WAM
template<size_t DOF>
  bool WamNode<DOF>::poseMove(wam_common::PoseMove::Request &req, wam_common::PoseMove::Response &res)
  {
    ROS_INFO("Moving Robot to Commanded Pose");

    cp_cmd[0] = req.pose.position.x;
    cp_cmd[1] = req.pose.position.y;
    cp_cmd[2] = req.pose.position.z;
    ortn_cmd.x() = req.pose.orientation.x;
    ortn_cmd.y() = req.pose.orientation.y;
    ortn_cmd.z() = req.pose.orientation.z;
    ortn_cmd.w() = req.pose.orientation.w;

    pose_cmd = boost::make_tuple(cp_cmd, ortn_cmd);
    /* Sasha: test this. */
    //wam.moveTo(pose_cmd, false); //(TODO:KM Update Libbarrett API for Pose Moves)
    ROS_INFO("Pose Commands for WAM not yet supported by API");
    return false;
  }

//Function to command a cartesian move to the WAM
template<size_t DOF>
  bool WamNode<DOF>::cartMove(wam_common::CartPosMove::Request &req, wam_common::CartPosMove::Response &res)
  {
    ROS_INFO("Moving Robot to Commanded Cartesian Position");

    for (int i = 0; i < 3; i++)
      cp_cmd[i] = req.position[i];
    wam.moveTo(cp_cmd, false);
    return true;
  }

//Function to command an orientation move to the WAM
template<size_t DOF>
  bool WamNode<DOF>::ortnMove(wam_common::OrtnMove::Request &req, wam_common::OrtnMove::Response &res)
  {
    ROS_INFO("Moving Robot to Commanded End Effector Orientation");

    ortn_cmd.x() = req.orientation[0];
    ortn_cmd.y() = req.orientation[1];
    ortn_cmd.z() = req.orientation[2];
    ortn_cmd.w() = req.orientation[3];

    wam.moveTo(ortn_cmd, false);
    return true;
  }

//Function to open the BarrettHand Grasp
template<size_t DOF>
  bool WamNode<DOF>::handOpenGrasp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    ROS_INFO("Opening the BarrettHand Grasp");
    hand->open(Hand::GRASP, false);
    return true;
  }

//Function to close the BarrettHand Grasp
template<size_t DOF>
  bool WamNode<DOF>::handCloseGrasp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    ROS_INFO("Closing the BarrettHand Grasp");
    hand->close(Hand::GRASP, false);
    return true;
  }

//Function to open the BarrettHand Spread
template<size_t DOF>
  bool WamNode<DOF>::handOpenSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    ROS_INFO("Opening the BarrettHand Spread");
    hand->open(Hand::SPREAD, false);
    return true;
  }

//Function to close the BarrettHand Spread
template<size_t DOF>
  bool WamNode<DOF>::handCloseSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    ROS_INFO("Closing the BarrettHand Spread");
    hand->close(Hand::SPREAD, false);
    return true;
  }

//Function to control a BarrettHand Finger Position
template<size_t DOF>
  bool WamNode<DOF>::handFingerPos(wam_common::BHandFingerPos::Request &req, wam_common::BHandFingerPos::Response &res)
  {
    ROS_INFO("Moving BarrettHand to Finger Positions: %.3f, %.3f, %.3f radians", req.radians[0], req.radians[1],
             req.radians[2]);
    hand->trapezoidalMove(Hand::jp_type(req.radians[0], req.radians[1], req.radians[2], 0.0), Hand::GRASP, false);
    return true;
  }

//Function to control the BarrettHand Grasp Position
template<size_t DOF>
  bool WamNode<DOF>::handGraspPos(wam_common::BHandGraspPos::Request &req, wam_common::BHandGraspPos::Response &res)
  {
    ROS_INFO("Moving BarrettHand Grasp: %.3f radians", req.radians);
    hand->trapezoidalMove(Hand::jp_type(req.radians), Hand::GRASP, false);
    return true;
  }

//Function to control the BarrettHand Spread Position
template<size_t DOF>
  bool WamNode<DOF>::handSpreadPos(wam_common::BHandSpreadPos::Request &req, wam_common::BHandSpreadPos::Response &res)
  {
    ROS_INFO("Moving BarrettHand Spread: %.3f radians", req.radians);
    hand->trapezoidalMove(Hand::jp_type(req.radians), Hand::SPREAD, false);
    return true;
  }

//Function to control a BarrettHand Finger Velocity
template<size_t DOF>
  bool WamNode<DOF>::handFingerVel(wam_common::BHandFingerVel::Request &req, wam_common::BHandFingerVel::Response &res)
  {
    ROS_INFO("Moving BarrettHand Finger Velocities: %.3f, %.3f, %.3f m/s", req.velocity[0], req.velocity[1],
             req.velocity[2]);
    hand->velocityMove(Hand::jv_type(req.velocity[0], req.velocity[1], req.velocity[2], 0.0), Hand::GRASP);
    return true;
  }

//Function to control a BarrettHand Grasp Velocity
template<size_t DOF>
  bool WamNode<DOF>::handGraspVel(wam_common::BHandGraspVel::Request &req, wam_common::BHandGraspVel::Response &res)
  {
    ROS_INFO("Moving BarrettHand Grasp: %.3f m/s", req.velocity);
    hand->velocityMove(Hand::jv_type(req.velocity), Hand::GRASP);
    return true;
  }

//Function to control a BarrettHand Spread Velocity
template<size_t DOF>
  bool WamNode<DOF>::handSpreadVel(wam_common::BHandSpreadVel::Request &req, wam_common::BHandSpreadVel::Response &res)
  {
    ROS_INFO("Moving BarrettHand Spread: %.3f m/s", req.velocity);
    usleep(5000);
    hand->velocityMove(Hand::jv_type(req.velocity), Hand::SPREAD);
    return true;
  }

//Callback function for joint Time-Parameterized Position Messages
/*** Non-Blocking ***/
template<size_t DOF>
  void WamNode<DOF>::jointTrajCB(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
  {

    // double t_start = highResolutionSystemTime();
    // std::cout << "Start time: " << t_start << std::endl;

    size_t n_pts = msg->points.size();
    size_t i_start = 0;

    jpVec = new std::vector<input_jp_type>;
    input_jp_type jpSamp;

    ROS_INFO("\n");
    ROS_INFO("Joint Traj. message received.");
    ROS_INFO("Time now: %15.5f",ros::Time::now().toSec());
    ROS_INFO("Message time: %15.5f", msg->header.stamp.toSec());
    ROS_INFO("Time diff: %15.5f \n", ros::Time::now().toSec() - msg->header.stamp.toSec());

    // Skip traj points that are behind in time
    // *** Make sure both pc's are synched (chrony) ***
    // for (size_t i = 0; i < n_pts; i++) {

    //   // Stamp set as planning start time by publisher (see piper interface)
    //   ros::Time point_time = msg->header.stamp + msg->points[i].time_from_start;

    //   ROS_INFO("Time now: %15.5f",ros::Time::now().toSec());
    //   ROS_INFO("plan_start time: %15.5f", msg->header.stamp.toSec());
    //   ROS_INFO("point %d time from start: %15.5f \n", int(i), msg->points[i].time_from_start.toSec());

    //   if ( point_time.toSec() < ros::Time::now().toSec() ) 
    //   {
    //     i_start=i+1;
    //     ROS_INFO_STREAM("Skipping traj. point: "<<i);
    //   } 
    //   else 
    //   {
    //     break;
    //   }
    // }

    // Check start pos. matches current
    //  jp_type jp_curr = wam.getJointPositions();
    // for (size_t j = 0; j < DOF; j++)
    // {
    //   if ( std::abs(jp_curr[j] - msg->points[i_start].positions[j]) > 1.0e-02) 
    //   {
    //     ROS_ERROR("Starting joint positions do not match! Dropping trajectory msg.");
    //     return;
    //   }
    // }   

    jp_type jp_curr = wam.getJointPositions();

    for (size_t i = i_start; i < n_pts; i++)
    {   
      boost::get<0>(jpSamp) = msg->points[i].time_from_start.toSec();
      
      if (i==i_start) {
        //Replace starting point with current pos.    
        for (size_t j = 0; j < DOF; j++) {
          boost::get<1>(jpSamp)[j] = jp_curr[j];
        }

        /********* DEBUG: Fix last 6 DOFs **********/
        // // boost::get<1>(jpSamp)[1] = -1.91256 ;
        // boost::get<1>(jpSamp)[2] =  1.47136 ;
        // boost::get<1>(jpSamp)[3] =  1.2343 ;
        // boost::get<1>(jpSamp)[4] =  0.300391 ;
        // boost::get<1>(jpSamp)[5] =  1.13839 ;
        // boost::get<1>(jpSamp)[6] =  -1.61063 ;
        /******************************************/ 
      }
      else {
        for (size_t j = 0; j < DOF; j++) {
          boost::get<1>(jpSamp)[j] = msg->points[i].positions[j];
        } 

        /********* DEBUG: Fix last 6 DOFs **********/
        // // boost::get<1>(jpSamp)[1] = -1.91256 ;
        // boost::get<1>(jpSamp)[2] =  1.47136 ;
        // boost::get<1>(jpSamp)[3] =  1.2343 ;
        // boost::get<1>(jpSamp)[4] =  0.300391 ;
        // boost::get<1>(jpSamp)[5] =  1.13839 ;
        // boost::get<1>(jpSamp)[6] =  -1.61063 ;
        /******************************************/ 
      }

      jpVec->push_back(jpSamp);
    }

    jpSpline = new math::Spline<jp_type>(*jpVec);
    jpTrajectory = new systems::Callback<double, jp_type>(boost::ref(*jpSpline));
    systems::forceConnect(ramp.output, jpTrajectory->input);

    // Data Logging
    if (LOG_DATA) {
      systems::forceConnect(ramp.output, sys_dtime.input);

      systems::forceConnect(sys_dtime.output, jpLog_tuple_grouper.template getInput<0>());
      systems::forceConnect(ramp.output, jpLog_tuple_grouper.template getInput<1>());
      systems::forceConnect(jpTrajectory->output, jpLog_tuple_grouper.template getInput<2>());
      systems::forceConnect(wam.jpOutput, jpLog_tuple_grouper.template getInput<3>());
      systems::forceConnect(wam.jvOutput, jpLog_tuple_grouper.template getInput<4>());
      systems::forceConnect(wam.jtSum.output, jpLog_tuple_grouper.template getInput<5>());

      systems::forceConnect(jpLog_tuple_grouper.output, data_logger->input);
    }

    ramp.stop();
    ramp.reset();
    ramp.setOutput(jpSpline->initialS() );
    wam.trackReferenceSignal(jpTrajectory->output);
    
    ROS_INFO("Time diff. to ramp: %15.5f \n", ros::Time::now().toSec() - msg->header.stamp.toSec());
   
    ramp.start();
    // ramp.smoothStart(0.001);

    // double del_time = highResolutionSystemTime() - t_start;
    // std::cout << "Exec. time: " << del_time << std::endl;

  }


//Callback function for Cartesian Time-Parameterized Position Messages
template<size_t DOF>
  void WamNode<DOF>::cartTrajCB(const wam_common::CartPointTraj::ConstPtr& msg)
  {

    assert(msg->time.size()==msg->x.size());
    assert(msg->time.size()==msg->y.size());
    assert(msg->time.size()==msg->z.size());

    size_t n_pts = msg->time.size();

    cpVec = new std::vector<input_cp_type>;
    input_cp_type cpSamp;

    for (size_t i = 0; i < n_pts; i++)
    {     
      boost::get<0>(cpSamp) = msg->time[i];
      boost::get<1>(cpSamp) << msg->x[i], msg->y[i], msg->z[i];
      cpVec->push_back(cpSamp);
    }

    cpSpline = new math::Spline<cp_type>(*cpVec);
    cpTrajectory = new systems::Callback<double, cp_type>(boost::ref(*cpSpline));
    systems::forceConnect(ramp.output, cpTrajectory->input);

    ramp.stop();
    ramp.reset();
    ramp.setOutput(cpSpline->initialS());
    wam.trackReferenceSignal(cpTrajectory->output);

    ramp.start();
    // ramp.smoothStart(TRANSITION_DURATION);
    // ramp.smoothStart(0.0001);
    
    while (cpTrajectory->input.getValue() < cpSpline->finalS()) {
      usleep(100000);
    }

  }

//Function to update the WAM publisher
template<size_t DOF>
  void WamNode<DOF>::publishWam(ProductManager& pm)
  {
    //Current values to be published
    jp_type jp = wam.getJointPositions();
    jt_type jt = wam.getJointTorques();
    jv_type jv = wam.getJointVelocities();
    cp_type cp_pub = wam.getToolPosition();
    Eigen::Quaterniond to_pub = wam.getToolOrientation();

    //publishing sensor_msgs/JointState to wam/joint_states
    for (size_t i = 0; i < DOF; i++)
    {
      wam_joint_state.position[i] = jp[i];
      wam_joint_state.velocity[i] = jv[i];
      wam_joint_state.effort[i] = jt[i];
    }
    wam_joint_state.header.stamp = ros::Time::now();
    wam_joint_state_pub.publish(wam_joint_state);

    //publishing geometry_msgs/PoseStamed to wam/pose
    wam_pose.header.stamp = ros::Time::now();
    wam_pose.pose.position.x = cp_pub[0];
    wam_pose.pose.position.y = cp_pub[1];
    wam_pose.pose.position.z = cp_pub[2];
    wam_pose.pose.orientation.w = to_pub.w();
    wam_pose.pose.orientation.x = to_pub.x();
    wam_pose.pose.orientation.y = to_pub.y();
    wam_pose.pose.orientation.z = to_pub.z();
    wam_pose_pub.publish(wam_pose);
  }

  //Function to update the WAM publisher
template<size_t DOF>
  void WamNode<DOF>::publishFT(ProductManager& pm)
  {

    ft_sensor->update(true);
    ft_sensor->updateAccel(true);

    //Current values to be published
    cf_type cf = ft_sensor->getForce();
    ct_type ct = ft_sensor->getTorque();
    ca_type ca = ft_sensor->getAccel(); 

    //publishing sensor_msgs/JointState to wam/force_torque
    ft_sensor_state.force.x = cf[0];
    ft_sensor_state.force.y = cf[1];  
    ft_sensor_state.force.z = cf[2];
    ft_sensor_state.torque.x = ct[0];
    ft_sensor_state.torque.y = ct[1];
    ft_sensor_state.torque.z = ct[2];
    ft_sensor_state.accel.x = ca[0];
    ft_sensor_state.accel.y = ca[1];
    ft_sensor_state.accel.z = ca[2];

    ft_sensor_state.header.stamp = ros::Time::now();
    ft_sensor_pub.publish(ft_sensor_state);
  }


//Function to update the real-time control loops
template<size_t DOF>
  void WamNode<DOF>::publishHand() //systems::PeriodicDataLogger<debug_tuple>& logger
  {
    while (ros::ok())
    {
      hand->update(); // Update the hand sensors
      Hand::jp_type hi = hand->getInnerLinkPosition(); // get finger positions information
      Hand::jp_type ho = hand->getOuterLinkPosition();
      for (size_t i = 0; i < 4; i++) // Save finger positions
        bhand_joint_state.position[i] = hi[i];
      for (size_t j = 0; j < 3; j++)
        bhand_joint_state.position[j + 4] = ho[j];
      bhand_joint_state.header.stamp = ros::Time::now(); // Set the timestamp
      bhand_joint_state_pub.publish(bhand_joint_state); // Publish the BarrettHand joint states
      btsleep(1.0 / BHAND_PUBLISH_FREQ); // Sleep according to the specified publishing frequency
    }
  }


//wam_main Function
template<size_t DOF>
  int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam)
  {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    ros::init(argc, argv, "wam_node");
    WamNode<DOF> wam_node(wam);
    wam_node.init(pm);
    ros::Rate pub_rate(PUBLISH_FREQ);

    if (pm.getHand())
      boost::thread handPubThread(&WamNode<DOF>::publishHand, &wam_node);
    
    while (ros::ok() && pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE)
    {
      try
      {
        ros::spinOnce();
        wam_node.publishWam(pm);
        wam_node.publishFT(pm);
        pub_rate.sleep();
      }
      catch (const std::exception &e)
      {
        ROS_ERROR_STREAM(e.what());
      }
    }

    // Safe close
    wam_node.close(pm);
    return 0;
  } 
