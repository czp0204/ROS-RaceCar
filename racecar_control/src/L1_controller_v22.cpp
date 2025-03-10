#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>//new
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <unistd.h>
#include <time.h>



#define PI 3.14159265358979
double last_eta = 0;

void delay_msec(int msec)
{ 
	clock_t now = clock();
	while(clock()-now < msec);
}

/********************/
/* CLASS DEFINITION */
/********************/
class L1Controller {
public:
  L1Controller();
  void initMarker();
  bool isForwardWayPt(const geometry_msgs::Point &wayPt,
                      const geometry_msgs::Pose &carPose);
  bool isWayPtAwayFromLfwDist(const geometry_msgs::Point &wayPt,
                              const geometry_msgs::Point &car_pos);
  double getYawFromPose(const geometry_msgs::Pose &carPose);
  double getPitchFromPose(const sensor_msgs::Imu &carPose);//new
  double getEta(const geometry_msgs::Pose &carPose);
  double getCar2GoalDist();
  double getL1Distance(const double &_Vcmd);
  double getSteeringAngle(double eta);
  double getGasInput(const float &current_v);
  geometry_msgs::Point
  get_odom_car2WayPtVec(const geometry_msgs::Pose &carPose);

private:
  ros::NodeHandle n_;
  ros::Subscriber odom_sub, path_sub, goal_sub,imu_sub;
  ros::Publisher pub_, marker_pub;
  ros::Timer timer1, timer2;
  tf::TransformListener tf_listener;
  

  visualization_msgs::Marker points, line_strip, goal_circle;
  geometry_msgs::Twist cmd_vel;
  geometry_msgs::Point odom_goal_pos;
  sensor_msgs::Imu imudata;
  nav_msgs::Odometry odom;
  nav_msgs::Path map_path, odom_path;

  double L, Lfw, Lrv, Vcmd, lfw, lrv, steering, u, v,a1,a2,Pitch,L1,X1;
  double Gas_gain, baseAngle, Angle_gain, D_steer, goalRadius;
  double controller_freq, baseSpeed;
  bool foundForwardPt, goal_received, goal_reached;

  void odomCB(const nav_msgs::Odometry::ConstPtr &odomMsg);
  void pathCB(const nav_msgs::Path::ConstPtr &pathMsg);
  void goalCB(const geometry_msgs::PoseStamped::ConstPtr &goalMsg);
  void imuCB(const sensor_msgs::Imu::ConstPtr &imuMsg);
  void goalReachingCB(const ros::TimerEvent &);
  void controlLoopCB(const ros::TimerEvent &);

}; // end of class

L1Controller::L1Controller() {
  // Private parameters handler
  ros::NodeHandle pn("~");

  // Car parameter
  pn.param("L", L, 0.335);
  pn.param("Lrv", Lrv, 10.0);
  pn.param("Vcmd", Vcmd, 1.0);
  pn.param("lfw", lfw, 0.13);
  pn.param("lrv", lrv, 10.0);
  pn.param("a1", a1, 0.25);
  pn.param("a2", a2, 0.2);
  pn.param("L1", L1, 0.65);
  pn.param("X1", X1, -20.0);
  // Controller parameter
  pn.param("controller_freq", controller_freq, 40.0);
  pn.param("AngleGain", Angle_gain, -5.2);//5.2
  pn.param("D_steer", D_steer, -0.0);
  pn.param("GasGain", Gas_gain, 1.0);
  pn.param("baseSpeed", baseSpeed, 1.35);//1.35
  pn.param("baseAngle", baseAngle, 0.0);

  // Publishers and Subscribers
  odom_sub = n_.subscribe("/odometry/filtered", 1, &L1Controller::odomCB, this);
  // path_sub = n_.subscribe("/move_base/NavfnROS/plan", 1,
  // &L1Controller::pathCB, this);
  path_sub = n_.subscribe("/move_base/TebLocalPlannerROS/local_plan", 1,
                          &L1Controller::pathCB, this);
  // path_sub = n_.subscribe("/move_base/GlobalPlanner/plan", 1,
  //                          &L1Controller::pathCB, this);
  goal_sub =
      n_.subscribe("/move_base_simple/goal", 1, &L1Controller::goalCB, this);
  marker_pub = n_.advertise<visualization_msgs::Marker>("car_path", 10);
  pub_ = n_.advertise<geometry_msgs::Twist>("/car/cmd_vel", 1);
  // imu_sub = n_.subscribe("/imu_data", 1, &L1Controller::imuCB, this);
  imu_sub = n_.subscribe("/imu_data", 1, &L1Controller::imuCB, this);
  // Timer
  timer1 = n_.createTimer(ros::Duration((1.0) / controller_freq),
                          &L1Controller::controlLoopCB,
                          this); // Duration(0.05) -> 20Hz
  timer2 = n_.createTimer(ros::Duration((0.5) / controller_freq),
                          &L1Controller::goalReachingCB,
                          this); // Duration(0.05) -> 20Hz

  // Init variables
  Lfw = goalRadius = getL1Distance(Vcmd);
  foundForwardPt = false;
  goal_received = false;
  goal_reached = false;
  cmd_vel.linear.x = 0; // 1500 for stop
  cmd_vel.angular.z = baseAngle;

  // Show info
  ROS_INFO("[param] baseSpeed: %f", baseSpeed);
  ROS_INFO("[param] baseAngle: %f", baseAngle);
  ROS_INFO("[param] AngleGain: %f", Angle_gain);
  ROS_INFO("[param] D_steer: %f", D_steer);
  ROS_INFO("[param] Vcmd: %f", Vcmd);
  ROS_INFO("[param] Lfw: %f", Lfw);

  // Visualization Marker Settings
  initMarker();
}

void L1Controller::initMarker() {
  points.header.frame_id = line_strip.header.frame_id =
      goal_circle.header.frame_id = "odom";
  points.ns = line_strip.ns = goal_circle.ns = "Markers";
  points.action = line_strip.action = goal_circle.action =
      visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w =
      goal_circle.pose.orientation.w = 1.0;
  points.id = 0;
  line_strip.id = 1;
  goal_circle.id = 2;

  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  goal_circle.type = visualization_msgs::Marker::CYLINDER;
  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.2;
  points.scale.y = 0.2;

  // LINE_STRIP markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.1;

  goal_circle.scale.x = goalRadius;
  goal_circle.scale.y = goalRadius;
  goal_circle.scale.z = 0.1;

  // Points are green
  points.color.g = 1.0f;
  points.color.a = 1.0;

  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  // goal_circle is yellow
  goal_circle.color.r = 1.0;
  goal_circle.color.g = 1.0;
  goal_circle.color.b = 0.0;
  goal_circle.color.a = 0.5;
}

void L1Controller::odomCB(const nav_msgs::Odometry::ConstPtr &odomMsg) {
  odom = *odomMsg;
}

void L1Controller::pathCB(const nav_msgs::Path::ConstPtr &pathMsg) {
  map_path = *pathMsg;
}

void L1Controller::imuCB(const sensor_msgs::Imu::ConstPtr &imuMsg) {
  geometry_msgs::Twist carVel = odom.twist.twist;
  imudata = *imuMsg;
  Pitch = getPitchFromPose(imudata) * 180.0 / PI ;
  // if (!goal_reached) {
  //   if(Pitch * 180 / PI < -1)
  // cmd_vel.linear.x = -1.0 * baseSpeed;}

  // ROS_INFO("\nPitch= %.2f",Pitch);
}

void L1Controller::goalCB(const geometry_msgs::PoseStamped::ConstPtr &goalMsg) {
  try {
    geometry_msgs::PoseStamped odom_goal;
    tf_listener.transformPose("odom", ros::Time(0), *goalMsg, "map", odom_goal);
    odom_goal_pos = odom_goal.pose.position;
    goal_received = true;
    goal_reached = false;

    /*Draw Goal on RVIZ*/
    goal_circle.pose = odom_goal.pose;
    marker_pub.publish(goal_circle);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}
//siyuanshu欧拉角RPY分别代表Roll（滚转角），Pitch（俯仰角），Yaw（偏航角），分别对应绕XYZ轴旋转
//旋转的正方向是，从XYZ轴的箭头方向看过去，顺时针为正，逆时针为负。
double L1Controller::getYawFromPose(const geometry_msgs::Pose &carPose) {
  float x = carPose.orientation.x;
  float y = carPose.orientation.y;
  float z = carPose.orientation.z;
  float w = carPose.orientation.w;

  double tmp, yaw;
  tf::Quaternion q(x, y, z, w);
  tf::Matrix3x3 quaternion(q);
  quaternion.getRPY(tmp, tmp, yaw);

  return yaw;
}
double L1Controller::getPitchFromPose(const sensor_msgs::Imu &carPose) {
  float x = carPose.orientation.x;
  float y = carPose.orientation.y;
  float z = carPose.orientation.z;
  float w = carPose.orientation.w;

  double roll,pitch, yaw;
  tf::Quaternion q(x, y, z, w);
  tf::Matrix3x3 quaternion(q);
  quaternion.getRPY(roll, pitch, yaw);

  return pitch;
}
//   double L1Controller::getPitchFromPose(const geometry_msgs::Pose &carPose) {
//   float x = carPose.orientation.x;
//   float y = carPose.orientation.y;
//   float z = carPose.orientation.z;
//   float w = carPose.orientation.w;

//   double tmp, Pitch,tmp2;
//   tf::Quaternion q(x, y, z, w);
//   tf::Matrix3x3 quaternion(q);
//   quaternion.getRPY(tmp, Pitch, tmp2);

//   return Pitch;
// }

bool L1Controller::isForwardWayPt(const geometry_msgs::Point &wayPt,
                                  const geometry_msgs::Pose &carPose) {
  float car2wayPt_x = wayPt.x - carPose.position.x;
  float car2wayPt_y = wayPt.y - carPose.position.y;
  double car_theta = getYawFromPose(carPose);

  float car_car2wayPt_x =
      cos(car_theta) * car2wayPt_x + sin(car_theta) * car2wayPt_y;
  float car_car2wayPt_y =
      -sin(car_theta) * car2wayPt_x + cos(car_theta) * car2wayPt_y;

  if (car_car2wayPt_x > 0) /*is Forward WayPt*/
    return true;
  else
    return false;
}


bool L1Controller::isWayPtAwayFromLfwDist(const geometry_msgs::Point &wayPt,
                                          const geometry_msgs::Point &car_pos) {
  double dx = wayPt.x - car_pos.x;
  double dy = wayPt.y - car_pos.y;
  double dist = sqrt(dx * dx + dy * dy);

  if (dist < Lfw)
    return false;
  else if (dist >= Lfw)
    return true;
}

geometry_msgs::Point
L1Controller::get_odom_car2WayPtVec(const geometry_msgs::Pose &carPose) {
  geometry_msgs::Point carPose_pos = carPose.position;
  double carPose_yaw = getYawFromPose(carPose);
  geometry_msgs::Point forwardPt;
  geometry_msgs::Point odom_car2WayPtVec;
  foundForwardPt = false;

  if (!goal_reached) {
    for (int i = 0; i < map_path.poses.size(); i++) {
      geometry_msgs::PoseStamped odom_path_pose = map_path.poses[i];
      try {
        /* tf_listener.transformPose("odom", ros::Time(0) , map_path_pose, "map"
         * ,odom_path_pose);*/
        geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
        bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt, carPose);

        if (_isForwardWayPt || 1) {
          bool _isWayPtAwayFromLfwDist =
              isWayPtAwayFromLfwDist(odom_path_wayPt, carPose_pos);
          if (_isWayPtAwayFromLfwDist) {
            forwardPt = odom_path_wayPt;
            foundForwardPt = true;
            break;
          }
        }
      } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }
    }

  } else if (goal_reached) {
    forwardPt = odom_goal_pos;
    foundForwardPt = false;
    // ROS_INFO("goal REACHED!");
  }

  /*Visualized Target Point on RVIZ*/
  /*Clear former target point Marker*/
  points.points.clear();
  line_strip.points.clear();

  if (foundForwardPt && !goal_reached) {
    points.points.push_back(carPose_pos);
    points.points.push_back(forwardPt);
    line_strip.points.push_back(carPose_pos);
    line_strip.points.push_back(forwardPt);
  }

  marker_pub.publish(points);
  marker_pub.publish(line_strip);

  odom_car2WayPtVec.x = cos(carPose_yaw) * (forwardPt.x - carPose_pos.x) +
                        sin(carPose_yaw) * (forwardPt.y - carPose_pos.y);
  odom_car2WayPtVec.y = -sin(carPose_yaw) * (forwardPt.x - carPose_pos.x) +
                        cos(carPose_yaw) * (forwardPt.y - carPose_pos.y);
  return odom_car2WayPtVec;
}

double L1Controller::getEta(const geometry_msgs::Pose &carPose) {
  geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);

  double eta = atan2(odom_car2WayPtVec.y, odom_car2WayPtVec.x);
  return eta;
}

double L1Controller::getCar2GoalDist() {
  geometry_msgs::Point car_pose = odom.pose.pose.position;
  double car2goal_x = odom_goal_pos.x - car_pose.x;
  double car2goal_y = odom_goal_pos.y - car_pose.y;

  double dist2goal = sqrt(car2goal_x * car2goal_x + car2goal_y * car2goal_y);

  return dist2goal;
}

double L1Controller::getL1Distance(const double &_Vcmd) {
  geometry_msgs::Twist carVel = odom.twist.twist;
  
  //L1 = a1 * carVel.linear.x + a2;
  L1 = 0.55 ;
  return L1;
}

double L1Controller::getSteeringAngle(double eta) {
  geometry_msgs::Twist carVel = odom.twist.twist;
  Lfw = getL1Distance(carVel.linear.x);
  double steeringAnge =
      -atan2((L * sin(eta)), (Lfw / 2 + lfw * cos(eta))) * (180.0 / PI);
  return steeringAnge;
}

double L1Controller::getGasInput(const float &current_v) {
  double u = (Vcmd - current_v) * Gas_gain;
  return u;
}

void L1Controller::goalReachingCB(const ros::TimerEvent &) {

  if (goal_received) {
    double car2goal_dist = getCar2GoalDist();
    if (car2goal_dist < goalRadius) {
      goal_reached = true;
      goal_received = false;
      ROS_INFO("Goal Reached !");
    }
  }
}

void L1Controller::controlLoopCB(const ros::TimerEvent &) 
{

  geometry_msgs::Pose carPose = odom.pose.pose;
  geometry_msgs::Twist carVel = odom.twist.twist;
 // sensor_msgs::Imu imudata =  odom.Imu.
  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = baseAngle;

  if (goal_received) {
    /*Estimate Steering Angle*/
    double eta = getEta(carPose);
  //  double fz = getPitchFromPose(carPose)* 180.0 / PI;
    double car_theta = getYawFromPose(carPose)*180.0 / PI;

    if (foundForwardPt) {
      cmd_vel.angular.z =
          baseAngle + getSteeringAngle(eta) * Angle_gain +
          (getSteeringAngle(eta) - getSteeringAngle(last_eta)) * D_steer;
      last_eta = eta;

      /*Estimate Gas Input*/
      if (!goal_reached) {
        if( Pitch * 180 / PI < X1)
        {// { clock_t now =clock();
        //   while(clock()-now < 1000)
          cmd_vel.linear.x = -1.0 * baseSpeed;
          sleep(0.1);
          clock_t now1 =clock();
          while(clock()-now1 < 500)
          cmd_vel.linear.x = 0.5 * baseSpeed;
          // Angle_gain = 2.0;
        }
        else{
         cmd_vel.linear.x = baseSpeed;
        }
        ROS_INFO("\nLfw = %.2f", Lfw);
        ROS_INFO("\nNOWspeed = %.2f\nSteering angle = %.5f", cmd_vel.linear.x,
                 cmd_vel.angular.z);
      //  ROS_INFO("\nFZ = %.2f",fz);
        ROS_INFO("\ncar_Yaw = %.2f",car_theta);
        ROS_INFO("\nPitch = %.2f",Pitch);
      }
    }
  }
  pub_.publish(cmd_vel);
}

/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv) {
  // Initiate ROS
  ros::init(argc, argv, "L1Controller_v2");
  L1Controller controller;
  ros::spin();
  return 0;
}
