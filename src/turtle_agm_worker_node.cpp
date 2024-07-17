#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <agm_msgs/WebComm.h>
#include <string>
#include <iostream>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <turtlesim/Pose.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int factory_complete_status;
const float pi = 3.14159265358979323846;

void factory_complete_Callback(const std_msgs::Int64::ConstPtr &msg)
{
  factory_complete_status = msg->data;
}

class Robot_Class
{

public:
  std::string key;
  std::string move_base;
  std::string name;
  ros::NodeHandle n;
  agm_msgs::WebComm job;
  geometry_msgs::Twist control_command;
  ros::Publisher cmd_vel;
  ros::Subscriber subscriber_pose;
  turtlesim::Pose pose;
  int robot_connected;
  float goal_x;
  float goal_y;

  void agm_comm();
  void connect_robot();
  void move(float posX, float posY, float posZ, float orientX, float orientY, float orientZ, float orientW);
  void poseCallback(const turtlesim::Pose::ConstPtr &msg);
  bool robot_at_goal();
};

void Robot_Class::agm_comm()
{
  ros::ServiceClient agmClient = n.serviceClient<agm_msgs::WebComm>("/web_comm");
  job.request.key = key;
  agmClient.call(job);
}

void Robot_Class::poseCallback(const turtlesim::Pose::ConstPtr &msg)
{
  // populate pose for movement
  pose.x = msg->x;
  pose.y = msg->y;
  pose.theta = msg->theta;

  // broadcast TF
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = name;
  transformStamped.transform.translation.x = msg->x;
  transformStamped.transform.translation.y = msg->y;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);
}

void Robot_Class::connect_robot()
{
  subscriber_pose = n.subscribe<turtlesim::Pose>("/" + name + "/pose", 5, &Robot_Class::poseCallback, this);
  cmd_vel = n.advertise<geometry_msgs::Twist>(name + "/cmd_vel", 10);

  // initiate the values of the control command to zero where needed
  control_command.linear.y = 0.0;
  control_command.linear.z = 0.0;
  control_command.angular.x = 0.0;
  control_command.angular.y = 0.0;
  robot_connected = 1;
}

void Robot_Class::move(float posX, float posY, float posZ, float orientX, float orientY, float orientZ, float orientW)
{
  cout << "Trying to move to X: " + to_string(posX) + " Y: " + to_string(posY) << endl;
  goal_x = posX;
  goal_y = posY;
  cout << "Current position X: " + to_string(pose.x) + " Y: " + to_string(pose.y) << endl;
  // tell the action client that we want to spin a thread by default

  float angle = atan2(posY - pose.y, posX - pose.x) - pose.theta;
  // ensure angle is between -pi and pi
  if (angle < -pi)
  {
    angle += 2 * pi;
  }
  else if (angle > pi)
  {
    angle -= 2 * pi;
  }

  // angular velocity
  control_command.angular.z = 5 * (angle);
  float adaptive_control = 1.0;

  // control to enable sharp turns and staighter paths
  if (abs(control_command.angular.z) < .5)
  {
    adaptive_control = .5;
  }
  else
  {
    adaptive_control = abs(control_command.angular.z);
  }

  // linear velocity
  control_command.linear.x = .5 / adaptive_control * sqrt(pow(posX - pose.x, 2) + pow(posY - pose.y, 2));

  cmd_vel.publish(control_command);
};

bool Robot_Class::robot_at_goal()
{
  // check if we are at the goal within tolerance
  float dx, dy, tolerance = 0.1;
  dx = abs(pose.x - goal_x);
  dy = abs(pose.y - goal_y);
  if (dx <= tolerance && dy <= tolerance)
  {
    // if we are at the goal move to next goal
    return true;
  }
  else
  {
    return false;
  }
}

void output(int in)
{
  cout << to_string(in) << endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtle_agm_worker_node");
  ros::NodeHandle n;
  ros::Subscriber factory_complete = n.subscribe("factory_complete", 100, factory_complete_Callback);
  Robot_Class robot;
  int robot_moved_source = 0;
  int robot_moved_dest = 0;
  int counter = 0;
  int check = 0;

  if (argc > 1)
  {
    robot.key = argv[1];
    robot.move_base = argv[2];
    robot.name = argv[3];
    robot.robot_connected = 0;
  }
  else
  {
    cout << "No key defined for the robot interface" << endl;
  }

  // find next job
  robot.job.request.function = "START";
  // source coordinates
  float sPosX, sPosY, sPosZ, sOrientX, sOrientY, sOrientZ, sOrientW;
  float dPosX, dPosY, dPosZ, dOrientX, dOrientY, dOrientZ, dOrientW;

  // destination coordinates

  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    string job = robot.job.request.function;
    int status = robot.job.response.status;
    check = check + 1;
    cout << to_string(check) + " check" << endl;
    cout << to_string(counter) + " counter" << endl;
    if (factory_complete_status == 1)
    {
      // let's connect to the robot
      if (robot.robot_connected == 0)
      {
        robot.connect_robot();
      }

      if (job == "START" && counter > 100)
      {
        cout << job << " bob" << endl;
        cout << to_string(check) + " second check" << endl;
        // ready for a new job
        robot.job.request.function = "NEXTJOB";
        robot.job.request.location = "";
        robot_moved_source = 0;
        robot_moved_dest = 0;
        counter = 0;
        robot.agm_comm();
      }
      else if (job == "NEXTJOB" && status == 1)
      {
        // we have a new job to be activated
        cout << "Found next job and activating" << endl;
        // source
        sPosX = robot.job.response.sourcePosX;
        sPosY = robot.job.response.sourcePosY;
        sPosZ = robot.job.response.sourcePosZ;
        sOrientX = robot.job.response.sourceOrientX;
        sOrientY = robot.job.response.sourceOrientY;
        sOrientZ = robot.job.response.sourceOrientZ;
        sOrientW = robot.job.response.sourceOrientW;
        // destination
        dPosX = robot.job.response.destPosX;
        dPosY = robot.job.response.destPosY;
        dPosZ = robot.job.response.destPosZ;
        dOrientX = robot.job.response.destOrientX;
        dOrientY = robot.job.response.destOrientY;
        dOrientZ = robot.job.response.destOrientZ;
        dOrientW = robot.job.response.destOrientW;

        robot.job.request.function = "ACTIVATEJOB";
        robot.job.request.location = "";
        robot.agm_comm();
      }
      else if (job == "ACTIVATEJOB" && status == 1)
      {
        robot.goal_x = sPosX;
        robot.goal_y = sPosY;
        // MOVING ROBOT
        if (!robot.robot_at_goal() && !robot_moved_source)
        {
          robot.move(sPosX, sPosY, sPosZ, sOrientX, sOrientY, sOrientZ, sOrientW);
        }
        else
        {
          // move to source
          cout << "Moving to source" << endl;
          robot.job.request.function = "MOVEWORKER";
          robot.job.request.location = "source";
          cout << sPosX << " " << sPosY << " " << sPosZ << " " << sOrientX << " " << sOrientY << " " << sOrientZ << " " << sOrientW << endl;

          if (robot.job.request.function == "ERROR")
          {
            cout << "We have an error moving" << endl;
          }
          else
          {
            robot_moved_source = 1;
            robot.agm_comm();
          }
        }
      }
      else if (job == "MOVEWORKER" && status == 1)
      {
        // either TAKEPART or LOADPART depending on location
        if (robot.job.request.location == "source")
        {
          cout << "Taking part" << endl;
          robot.job.request.function = "TAKEPART";
        }
        else
        {
          cout << "Loading workstation" << endl;
          robot.job.request.function = "LOADPART";
        }
        robot.job.request.location = "";
        robot.agm_comm();
      }
      else if (job == "TAKEPART" && status == 1)
      {
        robot.goal_x = dPosX;
        robot.goal_y = dPosY;
        // MOVE ROBOT
        if (!robot.robot_at_goal() && !robot_moved_dest)
        {
          robot.move(dPosX, dPosY, dPosZ, dOrientX, dOrientY, dOrientZ, dOrientW);
        }
        else
        {
          // move to destination station
          cout << "Moving to destination" << endl;
          robot.job.request.function = "MOVEWORKER";
          robot.job.request.location = "destination";
          cout << dPosX << " " << dPosY << " " << dPosZ << " " << dOrientX << " " << dOrientY << " " << dOrientZ << " " << dOrientW << endl;

          if (robot.job.request.function == "ERROR")
          {
            cout << "We have an error moving" << endl;
          }
          else
          {
            robot_moved_dest = 1;
            robot.agm_comm();
          }
        }
        //
      }
      else if (job == "LOADPART" && status == 1)
      {
        // archive job
        cout << "Archiving job" << endl;
        robot.job.request.function = "ARCHIVEJOB";
        robot.job.request.location = "";
        robot_moved_source = 0;
        robot_moved_dest = 0;
        robot.agm_comm();
      }
      else if (job == "ARCHIVEJOB" && status == 1)
      {
        // start over
        robot.job.request.function = "START";
        robot.job.request.location = "";
        robot_moved_source = 0;
        robot_moved_dest = 0;
        cout << "Start again" << endl;
      }
      else
      {
        // error
        if (robot.job.request.function != "ERROR")
        {
          cout << "reset" << endl;
          cout << job << endl;
          cout << status << endl;
          cout << robot.job.response.name << endl;
          // start over
          robot.job.request.function = "START";
          robot.job.request.location = "";
        }

        if (status == 10001 || status == 10002 || status == 10003 || status == 10004 || status == 10005 || (status == 0 && job == "NEXTJOB"))
        {
          // there wasn't a job to do, reset and ask again
          robot.job.request.function = "START";
        }
      }
    }
    else
    {
      cout << "Factory not ready" << endl;
    }
    counter += 1;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}