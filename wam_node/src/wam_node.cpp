/*
 Copyright 2012 Barrett Technology <support@barrett.com>

 This file is part of barrett-ros-pkg.

 This version of barrett-ros-pkg is free software: you can redistribute it
 and/or modify it under the terms of the GNU General Public License as
 published by the Free Software Foundation, either version 3 of the
 License, or (at your option) any later version.

 This version of barrett-ros-pkg is distributed in the hope that it will be
 useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License along
 with this version of barrett-ros-pkg.  If not, see
 <http://www.gnu.org/licenses/>.

 Barrett Technology holds all copyrights on barrett-ros-pkg. As the sole
 copyright holder, Barrett reserves the right to release future versions
 of barrett-ros-pkg under a different license.

 File: wam_node.cpp
 Date: 5 June, 2012
 Author: Kyle Maroney
 */

/*Made catkin compatible and newer ROS friendly by Benjamin Blumer */
#include <unistd.h>
#include <math.h>

#include <boost/thread.hpp> // BarrettHand threading
#include <boost/bind.hpp>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include "wam_common/RTJointPos.h"
#include "wam_common/RTJointVel.h"
#include "wam_common/RTCartPos.h"
#include "wam_common/RTCartVel.h"
#include "wam_common/RTOrtnPos.h"
#include "wam_common/RTOrtnVel.h"
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

#include <barrett/math.h> 
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>
#include <barrett/systems/wam.h>
#include <barrett/detail/stl_utils.h>

typedef tf::Quaternion btQuaternion;
typedef tf::Matrix3x3 btMatrix3x3;

using namespace barrett;

static const int PUBLISH_FREQ = 250; // Default Control Loop / Publishing Frequency
static const int BHAND_PUBLISH_FREQ = 5; // Publishing Frequency for the BarretHand
static const double SPEED = 0.03; // Default Cartesian Velocity
static const double TORQUE_LIM = 3.0;
static const double JP_VEL_LIM = 1.5;

//Creating a templated multiplier for our real-time computation
template<typename T1, typename T2, typename OutputType>
  class Multiplier : public systems::System, public systems::SingleOutput<OutputType>
  {
  public:
    Input<T1> input1;
  public:
    Input<T2> input2;

  public:
    Multiplier(std::string sysName = "Multiplier") :
        systems::System(sysName), systems::SingleOutput<OutputType>(this), input1(this), input2(this)
    {
    }
    virtual ~Multiplier()
    {
      mandatoryCleanUp();
    }

  protected:
    OutputType data;
    virtual void operate()
    {
      data = input1.getValue() * input2.getValue();
      this->outputValue->setData(&data);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(Multiplier);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

//Creating a templated converter from Roll, Pitch, Yaw to Quaternion for real-time computation
class ToQuaternion : public systems::SingleIO<math::Vector<3>::type, Eigen::Quaterniond>
{
public:
  Eigen::Quaterniond outputQuat;

public:
  ToQuaternion(std::string sysName = "ToQuaternion") :
      systems::SingleIO<math::Vector<3>::type, Eigen::Quaterniond>(sysName)
  {
  }
  virtual ~ToQuaternion()
  {
    mandatoryCleanUp();
  }

protected:
  btQuaternion q;
  virtual void operate()
  {
    const math::Vector<3>::type &inputRPY = input.getValue();
    q.setEulerZYX(inputRPY[2], inputRPY[1], inputRPY[0]);
    outputQuat.x() = q.getX();
    outputQuat.y() = q.getY();
    outputQuat.z() = q.getZ();
    outputQuat.w() = q.getW();
    this->outputValue->setData(&outputQuat);
  }

private:
  DISALLOW_COPY_AND_ASSIGN(ToQuaternion);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//Simple Function for converting Quaternion to RPY
math::Vector<3>::type toRPY(Eigen::Quaterniond inquat)
{
  math::Vector<3>::type newRPY;
  btQuaternion q(inquat.x(), inquat.y(), inquat.z(), inquat.w());
  btMatrix3x3(q).getEulerZYX(newRPY[2], newRPY[1], newRPY[0]);
  return newRPY;
}

//WamNode Class
template<size_t DOF>
  class WamNode
  {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
  protected:
    bool cart_vel_status, ortn_vel_status, jnt_vel_status;
    bool jnt_pos_status, cart_pos_status, ortn_pos_status, new_rt_cmd;
    double cart_vel_mag, ortn_vel_mag;
    systems::Wam<DOF>& wam;
    Hand* hand;
    jp_type jp, jp_cmd, jp_home, jp_init;
    jp_type rt_jp_cmd, rt_jp_rl;
    jv_type rt_jv_cmd;
    cp_type cp_cmd, rt_cv_cmd;
    cp_type rt_cp_cmd, rt_cp_rl;
    Eigen::Quaterniond ortn_cmd, rt_op_cmd, rt_op_rl;
    pose_type pose_cmd;
    math::Vector<3>::type rt_ortn_cmd;
    systems::ExposedOutput<Eigen::Quaterniond> orientationSetPoint, current_ortn;
    systems::ExposedOutput<cp_type> cart_dir, current_cart_pos, cp_track;
    systems::ExposedOutput<math::Vector<3>::type> rpy_cmd, current_rpy_ortn;
    systems::ExposedOutput<jv_type> jv_track;
    systems::ExposedOutput<jp_type> jp_track;
    systems::TupleGrouper<cp_type, Eigen::Quaterniond> rt_pose_cmd;
    systems::Summer<cp_type> cart_pos_sum;
    systems::Summer<math::Vector<3>::type> ortn_cmd_sum;
    systems::Ramp ramp;
    systems::RateLimiter<jp_type> jp_rl;
    systems::RateLimiter<cp_type> cp_rl;
    Multiplier<double, cp_type, cp_type> mult_linear;
    Multiplier<double, math::Vector<3>::type, math::Vector<3>::type> mult_angular;
    ToQuaternion to_quat, to_quat_print;
    Eigen::Quaterniond ortn_print;
    ros::Time last_cart_vel_msg_time, last_ortn_vel_msg_time, last_jnt_vel_msg_time;
    ros::Time last_jnt_pos_msg_time, last_cart_pos_msg_time, last_ortn_pos_msg_time;
    ros::Duration rt_msg_timeout;

    //Subscribed Topics
    wam_common::RTCartVel cart_vel_cmd;
    wam_common::RTOrtnVel ortn_vel_cmd;

    //Subscribers
    ros::Subscriber cart_vel_sub;
    ros::Subscriber ortn_vel_sub;
    ros::Subscriber jnt_vel_sub;
    ros::Subscriber jnt_pos_sub;
    ros::Subscriber cart_pos_sub;
    ros::Subscriber ortn_pos_sub;

    //Published Topics
    sensor_msgs::JointState wam_joint_state, bhand_joint_state;
    geometry_msgs::PoseStamped wam_pose;

    //Publishers
    ros::Publisher wam_joint_state_pub, bhand_joint_state_pub, wam_pose_pub;

    //Services
    ros::ServiceServer gravity_srv, go_home_srv, hold_jpos_srv, hold_cpos_srv;
    ros::ServiceServer hold_ortn_srv, joint_move_srv, pose_move_srv;
    ros::ServiceServer cart_move_srv, ortn_move_srv, hand_close_srv;
    ros::ServiceServer hand_open_grsp_srv, hand_close_grsp_srv, hand_open_sprd_srv;
    ros::ServiceServer hand_close_sprd_srv, hand_fngr_pos_srv, hand_fngr_vel_srv;
    ros::ServiceServer hand_grsp_pos_srv, hand_grsp_vel_srv, hand_sprd_pos_srv;
    ros::ServiceServer hand_sprd_vel_srv;

  public:
    WamNode(systems::Wam<DOF>& wam_) :
        wam(wam_), hand(NULL), ramp(NULL, SPEED)
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
    holdJPos(wam_common::Hold::Request &req, wam_common::Hold::Response &res);
    bool
    holdCPos(wam_common::Hold::Request &req, wam_common::Hold::Response &res);
    bool
    holdOrtn(wam_common::Hold::Request &req, wam_common::Hold::Response &res);
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
    cartVelCB(const wam_common::RTCartVel::ConstPtr& msg);
    void
    ortnVelCB(const wam_common::RTOrtnVel::ConstPtr& msg);
    void
    jntVelCB(const wam_common::RTJointVel::ConstPtr& msg);
    void
    jntPosCB(const wam_common::RTJointPos::ConstPtr& msg);
    void
    cartPosCB(const wam_common::RTCartPos::ConstPtr& msg);
    void
    publishWam(ProductManager& pm);
    void
    publishHand(void);
    void
    updateRT(ProductManager& pm);
  };

// Templated Initialization Function
template<size_t DOF>
  void WamNode<DOF>::init(ProductManager& pm)
  {
    ros::NodeHandle n_("wam"); // WAM specific nodehandle
    ros::NodeHandle nh_("bhand"); // BarrettHand specific nodehandle

    //Setting up real-time command timeouts and initial values
    cart_vel_status = false; //Bool for determining cartesian velocity real-time state
    ortn_vel_status = false; //Bool for determining orientation velocity real-time state
    new_rt_cmd = false; //Bool for determining if a new real-time message was received
    rt_msg_timeout.fromSec(0.3); //rt_status will be determined false if rt message is not received in specified time
    cart_vel_mag = SPEED; //Setting default cartesian velocity magnitude to SPEED
    ortn_vel_mag = SPEED;
    pm.getExecutionManager()->startManaging(ramp); //starting ramp manager

    ROS_INFO(" \n %zu-DOF WAM", DOF);
    // jp_home = wam.getJointPositions();

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

      // Move j3 in order to give room for hand initialization
      // jp_type jp_init = wam.getJointPositions();
      // jp_init[3] -= 0.35;

      usleep(500000);
      hand->initialize();
      hand->update();

      //Publishing the following topics only if there is a BarrettHand present
      bhand_joint_state_pub = nh_.advertise < sensor_msgs::JointState > ("joint_states", 1); // bhand/joint_states

      //Advertise the following services only if there is a BarrettHand present
      hand_open_grsp_srv = nh_.advertiseService("open_grasp", &WamNode<DOF>::handOpenGrasp, this); // bhand/open_grasp
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
      const char* bhand_jnts[] = {"inner_f1", "inner_f2", "inner_f3", "spread", "outer_f1", "outer_f2", "outer_f3"};
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

    //Publishing the following rostopics
    wam_joint_state_pub = n_.advertise < sensor_msgs::JointState > ("joint_states", 1); // wam/joint_states
    wam_pose_pub = n_.advertise < geometry_msgs::PoseStamped > ("pose", 1); // wam/pose

    //Subscribing to the following rostopics
    cart_vel_sub = n_.subscribe("cart_vel_cmd", 1, &WamNode::cartVelCB, this); // wam/cart_vel_cmd
    ortn_vel_sub = n_.subscribe("ortn_vel_cmd", 1, &WamNode::ortnVelCB, this); // wam/ortn_vel_cmd
    jnt_vel_sub = n_.subscribe("jnt_vel_cmd", 1, &WamNode::jntVelCB, this); // wam/jnt_vel_cmd
    jnt_pos_sub = n_.subscribe("jnt_pos_cmd", 1, &WamNode::jntPosCB, this); // wam/jnt_pos_cmd
    cart_pos_sub = n_.subscribe("cart_pos_cmd", 1, &WamNode::cartPosCB, this); // wam/cart_pos_cmd

    //Advertising the following rosservices
    gravity_srv = n_.advertiseService("gravity_comp", &WamNode::gravity, this); // wam/gravity_comp
    go_home_srv = n_.advertiseService("go_home", &WamNode::goHome, this); // wam/go_home
    hold_jpos_srv = n_.advertiseService("hold_joint_pos", &WamNode::holdJPos, this); // wam/hold_joint_pos
    hold_cpos_srv = n_.advertiseService("hold_cart_pos", &WamNode::holdCPos, this); // wam/hold_cart_pos
    hold_ortn_srv = n_.advertiseService("hold_ortn", &WamNode::holdOrtn, this); // wam/hold_ortn
    joint_move_srv = n_.advertiseService("joint_move", &WamNode::jointMove, this); // wam/joint_move
    pose_move_srv = n_.advertiseService("pose_move", &WamNode::poseMove, this); // wam/pose_move
    cart_move_srv = n_.advertiseService("cart_move", &WamNode::cartMove, this); // wam/cart_pos_move
    ortn_move_srv = n_.advertiseService("ortn_move", &WamNode::ortnMove, this); // wam/ortn_move

  }

template<size_t DOF>
  void WamNode<DOF>::close(ProductManager& pm)
  {
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

    if (hand != NULL)
    {
      hand->open(Hand::GRASP, true);
      hand->close(Hand::SPREAD, true);
    }
    
    wam.moveTo(jp_init);
    wam.moveHome();

    // for (size_t i = 0; i < DOF; i++)
    //   jp_cmd[i] = 0.0;
    
    // wam.moveTo(jp_cmd, true);
    // jp_home[3] -= 0.3;
    // wam.moveTo(jp_home, true);
    // jp_home[3] += 0.3;
    // wam.moveTo(jp_home, true);

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

//Function to hold WAM end effector Orientation
template<size_t DOF>
  bool WamNode<DOF>::holdOrtn(wam_common::Hold::Request &req, wam_common::Hold::Response &res)
  {
    ROS_INFO("Orientation Hold request: %s", (req.hold) ? "true" : "false");

    if (req.hold)
    {
      orientationSetPoint.setValue(wam.getToolOrientation());
      wam.trackReferenceSignal(orientationSetPoint.output);
    }
    else
      wam.idle();
    return true;
  }

//Function to command a joint space move to the WAM
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
    wam.moveTo(jp_cmd, false);
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

//Callback function for RT Cartesian Velocity messages
template<size_t DOF>
  void WamNode<DOF>::cartVelCB(const wam_common::RTCartVel::ConstPtr& msg)
  {
    if (cart_vel_status)
    {
      for (size_t i = 0; i < 3; i++)
        rt_cv_cmd[i] = msg->direction[i];
      new_rt_cmd = true;
      if (msg->magnitude != 0)
        cart_vel_mag = msg->magnitude;
    }
    last_cart_vel_msg_time = ros::Time::now();

  }

//Callback function for RT Orientation Velocity Messages
template<size_t DOF>
  void WamNode<DOF>::ortnVelCB(const wam_common::RTOrtnVel::ConstPtr& msg)
  {
    if (ortn_vel_status)
    {
      for (size_t i = 0; i < 3; i++)
        rt_ortn_cmd[i] = msg->angular[i];
      new_rt_cmd = true;
      if (msg->magnitude != 0)
        ortn_vel_mag = msg->magnitude;
    }
    last_ortn_vel_msg_time = ros::Time::now();
  }

//Callback function for RT Joint Velocity Messages
template<size_t DOF>
  void WamNode<DOF>::jntVelCB(const wam_common::RTJointVel::ConstPtr& msg)
  {
    if (msg->velocities.size() != DOF)
    {
      ROS_INFO("Commanded Joint Velocities != DOF of WAM");
      return;
    }
    if (jnt_vel_status)
    {
      for (size_t i = 0; i < DOF; i++)
        rt_jv_cmd[i] = msg->velocities[i];
      new_rt_cmd = true;
    }
    last_jnt_vel_msg_time = ros::Time::now();
  }

//Callback function for RT Joint Position Messages
template<size_t DOF>
  void WamNode<DOF>::jntPosCB(const wam_common::RTJointPos::ConstPtr& msg)
  {
    if (msg->joints.size() != DOF)
    {
      ROS_INFO("Commanded Joint Positions != DOF of WAM");
      return;
    }
    if (jnt_pos_status)
    {
      for (size_t i = 0; i < DOF; i++)
      {
        rt_jp_cmd[i] = msg->joints[i];
        rt_jp_rl[i] = msg->rate_limits[i];
      }
      new_rt_cmd = true;
    }
    last_jnt_pos_msg_time = ros::Time::now();
  }

//Callback function for RT Cartesian Position Messages
template<size_t DOF>
  void WamNode<DOF>::cartPosCB(const wam_common::RTCartPos::ConstPtr& msg)
  {
    if (cart_pos_status)
    {
      for (size_t i = 0; i < 3; i++)
      {
        rt_cp_cmd[i] = msg->position[i];
        rt_cp_rl[i] = msg->rate_limits[i];
      }
      new_rt_cmd = true;
    }
    last_cart_pos_msg_time = ros::Time::now();
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

//Function to update the real-time control loops
template<size_t DOF>
  void WamNode<DOF>::updateRT(ProductManager& pm) //systems::PeriodicDataLogger<debug_tuple>& logger
  {
    //Real-Time Cartesian Velocity Control Portion
    if (last_cart_vel_msg_time + rt_msg_timeout > ros::Time::now()) // checking if a cartesian velocity message has been published and if it is within timeout
    {
      if (!cart_vel_status)
      {
        cart_dir.setValue(cp_type(0.0, 0.0, 0.0)); // zeroing the cartesian direction
        current_cart_pos.setValue(wam.getToolPosition()); // Initializing the cartesian position
        current_ortn.setValue(wam.getToolOrientation()); // Initializing the orientation
        systems::forceConnect(ramp.output, mult_linear.input1); // connecting the ramp to multiplier
        systems::forceConnect(cart_dir.output, mult_linear.input2); // connecting the direction to the multiplier
        systems::forceConnect(mult_linear.output, cart_pos_sum.getInput(0)); // adding the output of the multiplier
        systems::forceConnect(current_cart_pos.output, cart_pos_sum.getInput(1)); // with the starting cartesian position offset
        systems::forceConnect(cart_pos_sum.output, rt_pose_cmd.getInput<0>()); // saving summed position as new commanded pose.position
        systems::forceConnect(current_ortn.output, rt_pose_cmd.getInput<1>()); // saving the original orientation to the pose.orientation
        ramp.setSlope(cart_vel_mag); // setting the slope to the commanded magnitude
        ramp.stop(); // ramp is stopped on startup
        ramp.setOutput(0.0); // ramp is re-zeroed on startup
        ramp.start(); // start the ramp
        wam.trackReferenceSignal(rt_pose_cmd.output); // command WAM to track the RT commanded (500 Hz) updated pose
      }
      else if (new_rt_cmd)
      {
        ramp.reset(); // reset the ramp to 0
        ramp.setSlope(cart_vel_mag);
        cart_dir.setValue(rt_cv_cmd); // set our cartesian direction to subscribed command
        current_cart_pos.setValue(wam.tpoTpController.referenceInput.getValue()); // updating the current position to the actual low level commanded value
      }
      cart_vel_status = true;
      new_rt_cmd = false;
    }

    //Real-Time Angular Velocity Control Portion
    else if (last_ortn_vel_msg_time + rt_msg_timeout > ros::Time::now()) // checking if a orientation velocity message has been published and if it is within timeout
    {
      if (!ortn_vel_status)
      {
        rpy_cmd.setValue(math::Vector<3>::type(0.0, 0.0, 0.0)); // zeroing the rpy command
        current_cart_pos.setValue(wam.getToolPosition()); // Initializing the cartesian position
        current_rpy_ortn.setValue(toRPY(wam.getToolOrientation())); // Initializing the orientation

        systems::forceConnect(ramp.output, mult_angular.input1); // connecting the ramp to multiplier
        systems::forceConnect(rpy_cmd.output, mult_angular.input2); // connecting the rpy command to the multiplier
        systems::forceConnect(mult_angular.output, ortn_cmd_sum.getInput(0)); // adding the output of the multiplier
        systems::forceConnect(current_rpy_ortn.output, ortn_cmd_sum.getInput(1)); // with the starting rpy orientation offset
        systems::forceConnect(ortn_cmd_sum.output, to_quat.input);
        systems::forceConnect(current_cart_pos.output, rt_pose_cmd.getInput<0>()); // saving the original position to the pose.position
        systems::forceConnect(to_quat.output, rt_pose_cmd.getInput<1>()); // saving the summed and converted new quaternion commmand as the pose.orientation
        ramp.setSlope(ortn_vel_mag); // setting the slope to the commanded magnitude
        ramp.stop(); // ramp is stopped on startup
        ramp.setOutput(0.0); // ramp is re-zeroed on startup
        ramp.start(); // start the ramp
        wam.trackReferenceSignal(rt_pose_cmd.output); // command the WAM to track the RT commanded up to (500 Hz) cartesian velocity
      }
      else if (new_rt_cmd)
      {
        ramp.reset(); // reset the ramp to 0
        ramp.setSlope(ortn_vel_mag); // updating the commanded angular velocity magnitude
        rpy_cmd.setValue(rt_ortn_cmd); // set our angular rpy command to subscribed command
        current_rpy_ortn.setValue(toRPY(wam.tpoToController.referenceInput.getValue())); // updating the current orientation to the actual low level commanded value
      }
      ortn_vel_status = true;
      new_rt_cmd = false;
    }

    //Real-Time Joint Velocity Control Portion
    else if (last_jnt_vel_msg_time + rt_msg_timeout > ros::Time::now()) // checking if a joint velocity message has been published and if it is within timeout
    {
      if (!jnt_vel_status)
      {
        jv_type jv_start;
        for (size_t i = 0; i < DOF; i++)
          jv_start[i] = 0.0;
        jv_track.setValue(jv_start); // zeroing the joint velocity command
        wam.trackReferenceSignal(jv_track.output); // command the WAM to track the RT commanded up to (500 Hz) joint velocities
      }
      else if (new_rt_cmd)
      {
        jv_track.setValue(rt_jv_cmd); // set our joint velocity to subscribed command
      }
      jnt_vel_status = true;
      new_rt_cmd = false;
    }

    //Real-Time Joint Position Control Portion
    else if (last_jnt_pos_msg_time + rt_msg_timeout > ros::Time::now()) // checking if a joint position message has been published and if it is within timeout
    {
      if (!jnt_pos_status)
      {
        jp_type jp_start = wam.getJointPositions();
        jp_track.setValue(jp_start); // setting initial the joint position command
        jp_rl.setLimit(rt_jp_rl);
        systems::forceConnect(jp_track.output, jp_rl.input);
        wam.trackReferenceSignal(jp_rl.output); // command the WAM to track the RT commanded up to (500 Hz) joint positions
      }
      else if (new_rt_cmd)
      {
        jp_track.setValue(rt_jp_cmd); // set our joint position to subscribed command
        jp_rl.setLimit(rt_jp_rl); // set our rate limit to subscribed rate to control the rate of the moves
      }
      jnt_pos_status = true;
      new_rt_cmd = false;
    }

    //Real-Time Cartesian Position Control Portion
    else if (last_cart_pos_msg_time + rt_msg_timeout > ros::Time::now()) // checking if a cartesian position message has been published and if it is within timeout
    {
      if (!cart_pos_status)
      {
        cp_track.setValue(wam.getToolPosition());
        current_ortn.setValue(wam.getToolOrientation()); // Initializing the orientation
        cp_rl.setLimit(rt_cp_rl);
        systems::forceConnect(cp_track.output, cp_rl.input);
        systems::forceConnect(cp_rl.output, rt_pose_cmd.getInput<0>()); // saving the rate limited cartesian position command to the pose.position
        systems::forceConnect(current_ortn.output, rt_pose_cmd.getInput<1>()); // saving the original orientation to the pose.orientation
        wam.trackReferenceSignal(rt_pose_cmd.output); //Commanding the WAM to track the real-time pose command.
      }
      else if (new_rt_cmd)
      {
        cp_track.setValue(rt_cp_cmd); // Set our cartesian positions to subscribed command
        cp_rl.setLimit(rt_cp_rl); // Updating the rate limit to subscribed rate to control the rate of the moves
      }
      cart_pos_status = true;
      new_rt_cmd = false;
    }

    //If we fall out of 'Real-Time', hold joint positions
    else if (cart_vel_status | ortn_vel_status | jnt_vel_status | jnt_pos_status | cart_pos_status)
    {
      wam.moveTo(wam.getJointPositions()); // Holds current joint positions upon a RT message timeout
      cart_vel_status = ortn_vel_status = jnt_vel_status = jnt_pos_status = cart_pos_status = ortn_pos_status = false;
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
        wam_node.updateRT(pm);
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
