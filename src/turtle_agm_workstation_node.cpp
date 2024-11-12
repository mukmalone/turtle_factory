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
#include <turtlesim/SetPen.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int factory_complete_status;
const float pi = 3.14159265358979323846;

void factory_complete_Callback(const std_msgs::Int64::ConstPtr &msg)
{
  factory_complete_status = msg->data;
}

class Workstation_Class
{

public:
  std::string key;
  std::string owner;
  std::string move_base;
  std::string name;
  ros::NodeHandle n;
  agm_msgs::WebComm job;
  geometry_msgs::Twist control_command;
  ros::Publisher cmd_vel;
  ros::Subscriber subscriber_pose;
  ros::ServiceClient pen;
  turtlesim::Pose pose;
  turtlesim::SetPen pen_state;
  int workstation_connected;

  float goal_x;
  float goal_y;
  // this goal is for the station
  bool at_goal;
  // this goal is for marking the part in or out of station
  bool at_goal2;
  // this goal is to park workstation
  bool at_park;

  void agm_comm();
  void connect_workstation();
  void move(float posX, float posY, float posZ, float orientX, float orientY, float orientZ, float orientW);
  void poseCallback(const turtlesim::Pose::ConstPtr &msg);
  void workstation_at_goal();
};

void Workstation_Class::agm_comm()
{
  ros::ServiceClient agmClient = n.serviceClient<agm_msgs::WebComm>("/web_comm" + name);
  job.request.key = key;
  job.request.owner = owner;
  agmClient.call(job);
}

void Workstation_Class::poseCallback(const turtlesim::Pose::ConstPtr &msg)
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

void Workstation_Class::connect_workstation()
{
  subscriber_pose = n.subscribe<turtlesim::Pose>("/" + name + "/pose", 5, &Workstation_Class::poseCallback, this);
  cmd_vel = n.advertise<geometry_msgs::Twist>("/" + name + "/cmd_vel", 10);
  cout<<"name: " <<name<<endl;

  // initiate the values of the control command to zero where needed
  control_command.linear.y = 0.0;
  control_command.linear.z = 0.0;
  control_command.angular.x = 0.0;
  control_command.angular.y = 0.0;
  // re-enable pen with a different color
  pen_state.request.r = 0;
  pen_state.request.g = 0;
  pen_state.request.b = 0;
  pen_state.request.off = 1;
  pen_state.request.width = 2;
  pen = n.serviceClient<turtlesim::SetPen>("/" + name + "/set_pen");
  pen.call(pen_state);
  workstation_connected = 1;
}

void Workstation_Class::move(float posX, float posY, float posZ, float orientX, float orientY, float orientZ, float orientW)
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

void Workstation_Class::workstation_at_goal()
{
  // check if we are at the goal within tolerance
  float dx, dy, tolerance = 0.01;
  dx = abs(pose.x - goal_x);
  dy = abs(pose.y - goal_y);

  if (dx <= tolerance && dy <= tolerance)
  {
    if (!at_goal && !at_goal2 && !at_park)
    {
      at_goal = true;
      at_goal2 = false;
      at_park = false;
    }
    else if (at_goal && !at_goal2 && !at_park)
    {
      at_goal = true;
      at_goal2 = true;
      at_park = false;
    }
    else if (at_goal && at_goal2 && !at_park)
    {
      at_goal = true;
      at_goal2 = true;
      at_park = true;
    }
  }
}

void output(int in)
{
  cout << to_string(in) << endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtle_agm_workstation_node");
  ros::NodeHandle n;
  ros::Subscriber factory_complete = n.subscribe("factory_complete", 100, factory_complete_Callback);
  Workstation_Class workstation;
  int workstation_completed = 0;
  int counter = 0;
  int check = 0;

  if (argc > 1)
  {
    workstation.key = argv[1];
    workstation.move_base = argv[2];
    workstation.name = argv[3];
    workstation.owner = argv[4];
    workstation.workstation_connected = 0;
  }
  else
  {
    cout << "No key defined for the workstation interface" << endl;
  }

  // find next job
  workstation.job.request.function = "START";
  // workstation coordinates
  float sPosX, sPosY, sPosZ, sOrientX, sOrientY, sOrientZ, sOrientW;
  // tools and programs and poNum
  string tools, program, poNum;

  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    string job = workstation.job.request.function;
    int status = workstation.job.response.status;
    check = check + 1;
    if (factory_complete_status == 1)
    {
      // let's connect to the workstation
      if (workstation.workstation_connected == 0)
      {
        workstation.connect_workstation();
        workstation.at_goal = false;
        workstation.at_goal2 = false;
        workstation.at_park = false;
      }

      if (job == "START" && counter > 200)
      {
        // ready for a new job
        workstation.job.request.function = "WORKSTATIONSTATUS";
        workstation.job.request.location = "";
        workstation.job.request.poNum = "";
        workstation.job.request.stationStatus = "";
        workstation.job.request.stepStatus = "";
        workstation_completed = 0;
        workstation.at_goal = false;
        workstation.at_goal2 = false;
        workstation.at_park = false;
        counter = 0;
        workstation.agm_comm();
      }
      else if (job == "WORKSTATIONSTATUS" && status == 1)
      {
        // we have a new job to be activated
        cout << "Found next job and activating" << endl;
        // source
        sPosX = workstation.job.response.sourcePosX;
        sPosY = workstation.job.response.sourcePosY;
        sPosZ = workstation.job.response.sourcePosZ;
        sOrientX = workstation.job.response.sourceOrientX;
        sOrientY = workstation.job.response.sourceOrientY;
        sOrientZ = workstation.job.response.sourceOrientZ;
        sOrientW = workstation.job.response.sourceOrientW;
        // tools
        tools = workstation.job.response.tools;
        // program
        program = workstation.job.response.program;
        //poNum
        poNum = workstation.job.response.poNum;

        // setup goal
        if (!workstation.at_goal && !workstation.at_goal2 && !workstation.at_park)
        {
          workstation.goal_x = sPosX;
          workstation.goal_y = sPosY;
        }
        else if (workstation.at_goal && !workstation.at_goal2 && !workstation.at_park)
        {
          workstation.goal_x = sPosX;
          workstation.goal_y = sPosY + 0.4;
          workstation.pen_state.request.r = 0;
          workstation.pen_state.request.g = 0;
          workstation.pen_state.request.b = 255;
          workstation.pen_state.request.off = 0;
          workstation.pen_state.request.width = 8;
          workstation.pen.call(workstation.pen_state);
        }
        else if (workstation.at_goal && workstation.at_goal2 && !workstation.at_park)
        {
          workstation.goal_x = sPosX - 0.6;
          workstation.goal_y = sPosY;
          workstation.pen_state.request.r = 0;
          workstation.pen_state.request.g = 0;
          workstation.pen_state.request.b = 0;
          workstation.pen_state.request.off = 1;
          workstation.pen.call(workstation.pen_state);
        }

        // MOVING workstation
        workstation.workstation_at_goal();
        if ((!workstation.at_goal || !workstation.at_goal2 || !workstation.at_park) && !workstation_completed)
        {
          workstation.move(workstation.goal_x, workstation.goal_y, sPosZ, sOrientX, sOrientY, sOrientZ, sOrientW);
        }
        else
        {
          // move to workstation and perfom action
          workstation.job.request.function = "WORKSTATIONCOMPLETE";
          workstation.job.request.location = "";
          workstation.job.request.poNum = poNum;
          workstation.job.request.stationStatus = "WAITING";
          workstation.job.request.stepStatus = "PASS";

          if (workstation.job.request.function == "ERROR")
          {
            cout << "We have an error moving" << endl;
          }
          else
          {
            workstation.pen_state.request.off = 1;
            workstation.pen.call(workstation.pen_state);
            workstation_completed= 1;
            workstation.at_goal = false;
            workstation.at_goal2 = false;
            workstation.at_park = false;
            workstation.agm_comm();
          }
        }
      }
      
      else if (job == "WORKSTATIONCOMPLETE" && status == 1)
      {
        // start over
         workstation.job.request.function = "START";
        workstation.job.request.location = "";
        workstation.job.request.poNum = "";
        workstation.job.request.stationStatus = "";
        workstation.job.request.stepStatus = "";
        workstation_completed = 0;
        cout << "Start again" << endl;
      }
      else
      {
        // error
        if (workstation.job.request.function != "ERROR")
        {
          workstation.job.request.function = "START";
          workstation.job.request.location = "";
        }

        if (status == 10001 || status == 10002 || status == 10003 || status == 10004 || status == 10005 || status == 10006 || status == 10007 || status == 10008 || status == 10009 || (status == 0 && job == "NEXTJOB"))
        {
          // there wasn't a job to do, reset and ask again
          workstation.job.request.function = "START";
        }
      }
    }
    else
    {
      //cout << "Factory not ready" << endl;
    }
    counter += 1;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}