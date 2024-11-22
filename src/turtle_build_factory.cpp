// Author: Michael Muldoon
// email: michael.muldoon.home@gmail.com
// license: Apache 2.0
// Comment: This node builds the factory by placing a circle at the coordinates specified in the launch file and launching the appropriate number of
//          turtle workers

#include <ros/ros.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <turtlesim/Kill.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Pose.h>
#include <turtle_factory/NextGoal.h>
#include <std_msgs/Int64.h>
#include <ros/console.h>

const float pi = 3.14159265358979323846;

using namespace std;

class Robot_Class
{
public:
    string robot_name;
    int robot_num;
    ros::NodeHandle n;
    ros::Subscriber subscriber_pose;
    turtlesim::Pose pose;
    ros::Publisher cmd_vel;
    geometry_msgs::Twist control_command;
    float goal_x;
    float goal_y;

    void move_robot();

    void spawn_robot();

    void poseCallback(const turtlesim::Pose::ConstPtr &msg);

    void get_goal();

    bool robot_at_goal();
};

void Robot_Class::move_robot()
{
    float angle = atan2(goal_y - pose.y, goal_x - pose.x) - pose.theta;
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
    control_command.linear.x = .5 / adaptive_control * sqrt(pow(goal_x - pose.x, 2) + pow(goal_y - pose.y, 2));

    cmd_vel.publish(control_command);
}

void Robot_Class::spawn_robot()
{
    // This class will spawn the turtle and turn-off the pen
    turtlesim::Spawn turtle;
    turtle.request.x = 1;
    turtle.request.y = 10.0;
    turtle.request.theta = 1.57;
    turtle.request.name = robot_name;
    ros::ServiceClient spawn_turtle = n.serviceClient<turtlesim::Spawn>("/spawn");
    spawn_turtle.call(turtle);

    turtlesim::SetPen pen_state;
    pen_state.request.off = 1;
    ros::ServiceClient pen = n.serviceClient<turtlesim::SetPen>("/" + robot_name + "/set_pen");
    pen.call(pen_state);

    subscriber_pose = n.subscribe<turtlesim::Pose>("/" + robot_name + "/pose", 5, &Robot_Class::poseCallback, this);
    cmd_vel = n.advertise<geometry_msgs::Twist>(robot_name + "/cmd_vel", 10);

    // initiate the values of the control command to zero where needed
    control_command.linear.y = 0.0;
    control_command.linear.z = 0.0;
    control_command.angular.x = 0.0;
    control_command.angular.y = 0.0;
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
    transformStamped.child_frame_id = robot_name;
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

void Robot_Class::get_goal()
{
    ros::ServiceClient goalClient = n.serviceClient<turtle_factory::NextGoal>("/next_goal");
    turtle_factory::NextGoal nextGoal;
    nextGoal.request.x = pose.x;
    nextGoal.request.y = pose.y;
    nextGoal.request.theta = pose.theta;
    goalClient.call(nextGoal);
    goal_x = nextGoal.response.x;
    goal_y = nextGoal.response.y;
}

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

int main(int argc, char **argv)
{
    // simulation instance
    string sim_namespace;
    if (argc > 1)
    {
        sim_namespace = argv[1];
    }
    else
    {
        sim_namespace = "";
    }

    ros::init(argc, argv, "turtle_build_factory");
    ros::NodeHandle n;

    // wait for spawn service so we know turtlesim is up
    ros::service::waitForService("/spawn");

    // kill turtle1 the default turtle
    turtlesim::Kill turtle_to_kill;
    turtle_to_kill.request.name = "turtle1";
    ros::ServiceClient kill = n.serviceClient<turtlesim::Kill>("/kill");
    kill.call(turtle_to_kill);

    // Declare a variable to store the turtle's name who will draw the factory
    string turtle_name;
    // Try to get the parameter from the parameter server
    if (n.getParam("turtle_name", turtle_name))
    {
        cout << "Turtle name: " + turtle_name << endl;
    }
    else
    {
        turtle_name = "turtle1"; // Default name if not set
    }

    // This class will spawn the turtle and turn-off the pen
    turtlesim::Spawn turtle;
    turtle.request.x = 1.0;
    turtle.request.y = 1.0;
    turtle.request.theta = 1.57;
    turtle.request.name = turtle_name;
    ros::ServiceClient spawn_turtle = n.serviceClient<turtlesim::Spawn>("/spawn");
    spawn_turtle.call(turtle);

    ros::ServiceClient move_abs = n.serviceClient<turtlesim::TeleportAbsolute>("/" + turtle_name + "/teleport_absolute");
    ros::ServiceClient pen = n.serviceClient<turtlesim::SetPen>("/" + turtle_name + "/set_pen");
    ros::Publisher control_pub = n.advertise<geometry_msgs::Twist>("" + turtle_name + "/cmd_vel", 10);
    ros::Publisher factory_complete_pub = n.advertise<std_msgs::Int64>("factory_complete", 10);

    // setup number of workstations and workers
    int num_workers, num_workstations;
    int worker_start_num, workstation_start_num;
    string worker_names = "";
    string workstation_names = "";
    // workers
    n.getParam("/worker_names", worker_names);
    n.getParam("/num_workers", num_workers);
    n.getParam("/worker_start_num", worker_start_num);
    Robot_Class workers[num_workers];

    // workstations
    n.getParam("/workstation_names", workstation_names);
    n.getParam("/num_workstations", num_workstations);
    n.getParam("/workstation_start_num", workstation_start_num);
    Robot_Class workstations[num_workers];

    // drawing parameters
    int num_buffers, num_workstations_per_row, num_rows_workstations;
    n.getParam("/num_buffers", num_buffers);
    n.getParam("/num_workstations_per_row", num_workstations_per_row);
    n.getParam("/num_rows_workstations", num_rows_workstations);

    // broadcast factory is not yet completed
    std_msgs::Int64 factory_complete;
    int executed = 0;
    ros::Rate loop_rate(20);
    while (ros::ok())
    {

        while (executed == 0)
        {
            factory_complete.data = 0;
            factory_complete_pub.publish(factory_complete);

            // set pen color and turn off to teleport to location
            turtlesim::SetPen pen_state;
            pen_state.request.off = 1;
            pen_state.request.width = 3;

            if (pen.call(pen_state))
            {
                turtlesim::TeleportAbsolute coordinates;
                // setup infeed and outfeed buffers
                for (int j = 0; j < 2; j++)
                {
                    // j=0 is for infeed & outfeed setup
                    // j=1 is for workstation setup
                    int draw_loop_size, row_loop_size;
                    if (j == 0)
                    {
                        draw_loop_size = num_buffers;
                        row_loop_size = 1;
                    }
                    else
                    {
                        draw_loop_size = num_workstations_per_row;
                        row_loop_size = num_rows_workstations;
                    }

                    for (int k = 0; k < row_loop_size; k++)
                    {
                        for (int i = 0; i < draw_loop_size; i++)
                        {
                            // teleport to the target location
                            float origin_x, origin_y;

                            // setup the parameters
                            if (i == 0 && j == 0)
                            {
                                n.getParam("/infeed_origin_x", origin_x);
                                n.getParam("/infeed_origin_y", origin_y);
                                pen_state.request.r = 255;
                                pen_state.request.g = 255;
                                pen_state.request.b = 255;
                            }
                            else if (i == 1 && j == 0)
                            {
                                n.getParam("/outfeed_origin_x", origin_x);
                                n.getParam("/outfeed_origin_y", origin_y);
                                pen_state.request.r = 0;
                                pen_state.request.g = 0;
                                pen_state.request.b = 0;
                            }
                            else if (j == 1)
                            {
                                origin_x = 1 + (i * (10.0 / num_workstations_per_row));
                                origin_y = 2 + (k * (8.0 / num_rows_workstations));
                                pen_state.request.r = 0;
                                pen_state.request.g = 255;
                                pen_state.request.b = 0;
                            }

                            coordinates.request.x = origin_x;
                            coordinates.request.y = origin_y;
                            coordinates.request.theta = 0.0;
                            move_abs.call(coordinates);

                            // turn on the pen and draw a circle
                            pen_state.request.off = 0;
                            pen.call(pen_state);

                            ros::Rate loop_rate(1);
                            int cnt = 0;
                            // this will draw the circle
                            while (cnt != 2)
                            {
                                geometry_msgs::Twist control_command;
                                control_command.linear.x = 2.5;
                                control_command.linear.y = 0.0;
                                control_command.linear.z = 0.0;
                                control_command.angular.x = 0.0;
                                control_command.angular.y = 0.0;
                                control_command.angular.z = 12.0;

                                control_pub.publish(control_command);
                                ros::spinOnce();
                                loop_rate.sleep();
                                ++cnt;
                            }
                            // turn off pen
                            pen_state.request.off = 1;
                            pen.call(pen_state);
                            factory_complete_pub.publish(factory_complete);
                        }
                    }
                }

                // turn off pen and teleport to starting location
                pen_state.request.off = 1;
                pen.call(pen_state);

                // get the search step size and place 1st turutle at the start
                float turtle_origin_xy;
                n.getParam("/turtle_origin_xy", turtle_origin_xy);
                coordinates.request.x = turtle_origin_xy;
                coordinates.request.y = turtle_origin_xy;
                coordinates.request.theta = 1.57;
                move_abs.call(coordinates);
                // re-enable pen with a different color
                pen_state.request.r = 0;
                pen_state.request.g = 0;
                pen_state.request.b = 0;
                pen_state.request.off = 0;
                pen_state.request.width = 2;
                pen.call(pen_state);

                turtle_to_kill.request.name = turtle_name;
                ros::ServiceClient kill = n.serviceClient<turtlesim::Kill>("/kill");
                kill.call(turtle_to_kill);

                // setup turtlebots workers
                for (int i = 0; i < num_workers; i++)
                {
                    if ((i + 1 + (worker_start_num)) < 10)
                    {
                        workers[i].robot_name = worker_names + "0" + to_string(i + 1 + worker_start_num);
                    }
                    else
                    {
                        workers[i].robot_name = worker_names + to_string(i + 1 + worker_start_num);
                    }
                    workers[i].robot_num = i + 1;
                    workers[i].spawn_robot();
                    factory_complete_pub.publish(factory_complete);
                }

                // setup turtlebots workstations
                for (int i = 0; i < num_workstations; i++)
                {
                    if ((i + 1 + (workstation_start_num)) < 10)
                    {
                        workstations[i].robot_name = workstation_names + "0" + to_string(i + 1 + workstation_start_num);
                    }
                    else
                    {
                        workstations[i].robot_name = workstation_names + to_string(i + 1 + workstation_start_num);
                    }
                    workstations[i].robot_num = i + 1;
                    workstations[i].spawn_robot();
                    factory_complete_pub.publish(factory_complete);
                }

                ++executed;
                // get ready to broadcast the factory is complete
                factory_complete.data = 1;
            }
        }

        factory_complete_pub.publish(factory_complete);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}