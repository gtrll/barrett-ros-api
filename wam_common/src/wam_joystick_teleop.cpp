#include <math.h>
#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Joy.h"
#include "std_srvs/Empty.h"

#include "wam_msgs/RTCartVel.h"
#include "wam_msgs/RTOrtnVel.h"
#include "wam_srvs/GravityComp.h"
#include "wam_srvs/Hold.h"
#include "wam_srvs/BHandGraspVel.h"
#include "wam_srvs/BHandSpreadVel.h"

const int CNTRL_FREQ = 50; // Frequency at which we will publish our control messages.

//WamTeleop Class
class WamTeleop
{
public:
  ros::NodeHandle n_, nw_, nh_; // NodeHandles for publishing / subscribing on topics "/... & /wam/..." & "/bhand/..."

  // Boolean statuses for commanded states
  bool pose_pubbed, grsp_publish, sprd_publish;
  bool cart_publish, ortn_publish, home_publish;
  bool hold_publish, home_st, hold_st, ortn_mode;
  
  // Integer status for BarrettHand commanded state
  int bh_cmd_st;

  //variables to describe buttons on Joystick and their assignments
  int deadman_btn, guardian_deadman_btn;
  int gpr_open_btn, gpr_close_btn;
  int sprd_open_btn, sprd_close_btn;
  int ortn_btn, home_btn, hold_btn;
  int axis_x, axis_y, axis_z;
  int axis_r, axis_p, axis_yaw;

  //variables to describe velocity commands
  double max_grsp_vel, max_sprd_vel;
  double cart_mag, ortn_mag;
  double req_xdir, req_ydir, req_zdir;
  double req_rdir, req_pdir, req_yawdir;
  double bh_grsp_vel, bh_sprd_vel;

  //Subscribers
  ros::Subscriber joy_sub;

  //Services
  wam_srvs::BHandGraspVel grasp_vel;
  wam_srvs::BHandSpreadVel spread_vel;
  wam_srvs::Hold hold;
  std_srvs::Empty go_home;

  //Service Clients
  ros::ServiceClient grasp_vel_srv, spread_vel_srv, go_home_srv;
  ros::ServiceClient hold_srv;

  //Messages
  wam_msgs::RTCartVel cart_vel;
  wam_msgs::RTOrtnVel ortn_vel;

  //Publishers
  ros::Publisher cart_vel_pub, ortn_vel_pub;

  WamTeleop() :
      nw_("wam"), nh_("bhand") // Name our nodehandle "wam" to preceed our messages/services
  {
  }

  void init();
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
  void update();

  ~WamTeleop()
  {
  }
};

// WAM Teleoperation Initialization Function
void WamTeleop::init()
{
  // We will check the parameter server for WAM Teleoperation configuration variables
  n_.param("deadman_button", deadman_btn, 10);
  n_.param("guardian_deadman_button", guardian_deadman_btn, 11); 
  n_.param("gripper_open_button", gpr_open_btn, 12);
  n_.param("gripper_close_button", gpr_close_btn, 14);
  n_.param("spread_open_button", sprd_open_btn, 13);
  n_.param("spread_close_button", sprd_close_btn, 15);
  n_.param("orientation_control_button", ortn_btn, 8);
  n_.param("go_home_button", home_btn, 0);
  n_.param("hold_joints_button", hold_btn, 3);
  n_.param("grasp_max_velocity", max_grsp_vel, 1.0);
  n_.param("spread_max_velocity", max_sprd_vel, 1.0);
  n_.param("cartesian_magnitude", cart_mag, 0.05);
  n_.param("orientation_magnitude", ortn_mag, 1.0);
  n_.param("cartesian_x_axis", axis_x, 3);
  n_.param("cartesian_y_axis", axis_y, 2);
  n_.param("cartesian_x_axis", axis_z, 1);
  n_.param("orientation_roll_axis", axis_r, 3);
  n_.param("orientation_pitch_axis", axis_p, 2);
  n_.param("orientation_yaw_axis", axis_yaw, 1);

  hold.request.hold = false; // Default Start for joint hold command is false
  cart_publish = ortn_publish = false; // Setting publisher states to false 
  bh_cmd_st = 0;// Initializing BarrettHand Command State to Zero

  //Subscribers
  joy_sub = n_.subscribe < sensor_msgs::Joy > ("joy", 1, &WamTeleop::joyCallback, this); // /joy

  //Service Clients
  grasp_vel_srv = nh_.serviceClient<wam_srvs::BHandGraspVel>("grasp_vel");      // /bhand/grasp_vel
  spread_vel_srv = nh_.serviceClient<wam_srvs::BHandSpreadVel>("spread_vel");   // /bhand/spread_vel
  go_home_srv = nw_.serviceClient<std_srvs::Empty>("go_home");                  // /wam/go_home
  hold_srv = nw_.serviceClient<wam_srvs::Hold>("hold_joint_pos");               // /wam/hold_joint_pos

  //Publishers
  cart_vel_pub = nw_.advertise<wam_msgs::RTCartVel>("cart_vel_cmd", 1);         // /wam/cart_vel_cmd
  ortn_vel_pub = nw_.advertise<wam_msgs::RTOrtnVel>("ortn_vel_cmd", 1);         // /wam/ortn_vel_cmd
}

void WamTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  //Set our publishing states back to false for new commands
  grsp_publish = sprd_publish = cart_publish = ortn_publish = home_publish = hold_publish = ortn_mode = false; 
 
  //Return with no deadman pressed or without first pose published yet.
  if (!joy_msg->buttons[deadman_btn] | joy_msg->buttons[guardian_deadman_btn]) 
    return;

  if (joy_msg->buttons[ortn_btn]) //Checking our Orientation button state 
    ortn_mode = true;

  // Gripper Grasp Command
  if(joy_msg->buttons[gpr_open_btn]) //Checking to see if gripper open button is being pressed
  {
    bh_grsp_vel = -max_grsp_vel; // Set the velocity for the gripper command
    grsp_publish = true; // set grasp publish state
  }
  else if(joy_msg->buttons[gpr_close_btn]) //Checking to see if gripper close button is being pressed
  {
    bh_grsp_vel = max_grsp_vel; // Set the velocity for the gripper command
    grsp_publish = true; // set grasp publish state
  }

  // Gripper Spread Command
  if (joy_msg->buttons[sprd_open_btn])
  {
    bh_sprd_vel = -max_sprd_vel;
    sprd_publish = true;
  }
  else if (joy_msg->buttons[sprd_close_btn])
  {
    bh_sprd_vel = max_sprd_vel;
    sprd_publish = true;
  }
  
  // Go Home Command
  if (joy_msg->buttons[home_btn] && !home_st) // Checking to see if go home button is pressed and if it was pressed last callback loop
    home_publish = true; // set true only when button is first pressed down
  home_st = joy_msg->buttons[home_btn]; // setting the state for the next loop

  // Hold Joints Command
  if (joy_msg->buttons[hold_btn] && !hold_st)// Checking to see if hold position button is pressed and if it was pressed last callback loop
    hold_publish = true;// set true only when button is first pressed down
  hold_st = joy_msg->buttons[hold_btn];// setting the state for the next callback loop


  //Cartesian Velocity Portion
  if ((joy_msg->axes[axis_x] > 0.25 || joy_msg->axes[axis_x] < -0.25) && !ortn_mode)
  {
    req_xdir = joy_msg->axes[axis_x];
    cart_publish = true;
  }
  else
    req_xdir = 0.0;
  if ((joy_msg->axes[axis_y] > 0.25 || joy_msg->axes[axis_y] < -0.25) && !ortn_mode)
  {
    req_ydir = joy_msg->axes[axis_y];
    cart_publish = true;
  }
  else
    req_ydir = 0.0;
  if ((joy_msg->axes[axis_z] > 0.25 || joy_msg->axes[axis_z] < -0.25) && !ortn_mode)
  {
    req_zdir = joy_msg->axes[axis_z];
    cart_publish = true;
  }
  else
    req_zdir = 0.0;

  //RPY Velocity Portion
  if ((joy_msg->axes[axis_r] > 0.25 || joy_msg->axes[axis_r] < -0.25) && ortn_mode)
  {
    req_rdir = -joy_msg->axes[axis_r];
    ortn_publish = true;
  }
  else
    req_rdir = 0.0;
  if ((joy_msg->axes[axis_y] > 0.25 || joy_msg->axes[axis_y] < -0.25) && ortn_mode)
  {
    req_pdir = -joy_msg->axes[axis_p];
    ortn_publish = true;
  }
  else
    req_pdir = 0.0;
  if ((joy_msg->axes[axis_z] > 0.25 || joy_msg->axes[axis_z] < -0.25) && ortn_mode)
  {
    req_yawdir = joy_msg->axes[axis_yaw];
    ortn_publish = true;
  }
  else
    req_yawdir = 0.0;

}

// Function for updating the commands and publishing
void WamTeleop::update()
{
  //Check our publish hand states and act accordingly
  if (grsp_publish && bh_cmd_st == 0 && !sprd_publish && !cart_publish && !ortn_publish) // Only if grasp publish state is set
  {
    grasp_vel.request.velocity = bh_grsp_vel; // set grasp velocity to commanded
    grasp_vel_srv.call(grasp_vel); // call grasp velocity service
    bh_cmd_st = 1; // set the BarrettHand commanded state to signify grasp command
  }
  else if (sprd_publish && bh_cmd_st == 0 && !grsp_publish && !cart_publish && !ortn_publish)  // only if spread publish state is set
  {
    spread_vel.request.velocity = bh_sprd_vel; // set spread velocity to commanded
    spread_vel_srv.call(spread_vel); // call spread velocity service
    bh_cmd_st = 2; // set the BarrettHand commanded state to signify spread command
  }
  else if (bh_cmd_st != 0 && !grsp_publish && !sprd_publish && !cart_publish && !ortn_publish) // only if nothing published and last bhand state !=0
  {
    if (bh_cmd_st == 1) // if BarrettHand state is in grasp mode
    {
      grasp_vel.request.velocity = 0.0; // Zero the velocity
      grasp_vel_srv.call(grasp_vel); // Command zero velocity to grasp
    }
    if (bh_cmd_st == 2) // if Barretthand state is in spread mode
    {
      spread_vel.request.velocity = 0.0; // Zero the velocity 
      spread_vel_srv.call(spread_vel); // Command zero velocity to spread
    }
    bh_cmd_st = 0; // set the BarrettHand state to no mode (0)
  }

  //Check our publish hold joint position state and act accordingly
  if (hold_publish && !cart_publish && !ortn_publish && !grsp_publish && !sprd_publish && !home_publish) // if only hold_publish state is set
  {
    if (!hold.request.hold) // Check previous holding state
      hold.request.hold = true; // Setting the hold request to true;
    else
      hold.request.hold = false; // Setting the hold request to false
    hold_srv.call(hold); // Call the commanded hold state
  }
  
  //Check our publish go home state and act accordingly
  if(home_publish && !hold_publish && !cart_publish && !grsp_publish && !sprd_publish && !ortn_publish) // if only home_publish state is set
    go_home_srv.call(go_home); // Command WAM to go home

  //Check our published cartesian velocity state and act accordingly
  if(cart_publish && !ortn_publish && !grsp_publish && !sprd_publish && !home_publish && !hold_publish) // if only cart_publish state is set
  {
     cart_vel.direction[0] = req_xdir;
     cart_vel.direction[1] = req_ydir;
     cart_vel.direction[2] = req_zdir;
     cart_vel.magnitude = cart_mag;
     cart_vel_pub.publish(cart_vel);
  }
  
  //Check our published orientation velocity state and act accordingly
  if(ortn_publish && !cart_publish && !grsp_publish && !sprd_publish && !home_publish && !hold_publish) // if only cart_publish state is set
  {
     ortn_vel.angular[0] = req_rdir;
     ortn_vel.angular[1] = req_pdir;
     ortn_vel.angular[2] = req_yawdir;
     ortn_vel.magnitude = ortn_mag;
     ortn_vel_pub.publish(ortn_vel);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wam_teleop"); // Start our wam_node and name it "wam_teleop"
  WamTeleop wam_teleop; // Declare a member of WamTeleop "wam_teleop"
  wam_teleop.init(); // Initialize our teleoperation

  ros::Rate pub_rate(CNTRL_FREQ); // Setting the publishing rate to CNTRL_FREQ (50Hz by default)

  while (wam_teleop.n_.ok()) // Looping at the specified frequency while our ros node is ok
  {
    ros::spinOnce();
    wam_teleop.update(); // Update and send commands to the WAM
    pub_rate.sleep();
  }
  return 0;
}
