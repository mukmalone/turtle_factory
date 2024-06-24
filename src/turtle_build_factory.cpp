//Author: Michael Muldoon
//email: michael.muldoon.home@gmail.com
//license: Apache 2.0
//Comment: This node builds the factory by placing a circle at the coordinates specified in the launch file

#include <ros/ros.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

int main (int argc, char **argv)
{
    ros::init(argc, argv, "turtle_build_factory");
    ros::NodeHandle n;
    
    ros::ServiceClient move_abs = n.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
    ros::ServiceClient pen = n.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
    ros::Publisher control_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    ros::Publisher search_pub = n.advertise<std_msgs::String>("start_search", 10);
   

    int executed = 0;

    while (executed==0){
        //set pen color and turn off to teleport to location
        turtlesim::SetPen pen_state;        
        pen_state.request.off = 1;
        pen_state.request.width = 3;

        int num_buffers, num_workstations, num_rows_workstations;
        n.getParam("/num_buffers", num_buffers);
        n.getParam("/num_workstations", num_workstations);
        n.getParam("/num_rows_workstations", num_rows_workstations);

        if(pen.call(pen_state)){
            turtlesim::TeleportAbsolute coordinates;
            //setup infeed and outfeed buffers
            for(int j=0; j<2;j++){
                //j=0 is for infeed & outfeed setup
                //j=1 is for workstation setup
                int draw_loop_size, row_loop_size;
                if(j==0){
                    draw_loop_size = num_buffers;
                    row_loop_size = 1;
                } else {
                    draw_loop_size = num_workstations;
                    row_loop_size = num_rows_workstations;
                }

                for(int k = 0; k < row_loop_size; k++){
                    for (int i = 0; i < draw_loop_size; i++)
                    {
                        //teleport to the target location
                        float origin_x, origin_y;

                        //setup the parameters
                        if (i==0 && j==0){
                            n.getParam("/infeed_origin_x", origin_x);
                            n.getParam("/infeed_origin_y", origin_y);
                            pen_state.request.r = 255;
                            pen_state.request.g = 255;
                            pen_state.request.b = 255;
                        } else if (i==1 && j==0) {
                            n.getParam("/outfeed_origin_x", origin_x);
                            n.getParam("/outfeed_origin_y", origin_y);
                            pen_state.request.r = 0;
                            pen_state.request.g = 0;
                            pen_state.request.b = 0;
                        } else if (j==1) {
                            origin_x = 1 + (i * (10.0 / num_workstations));
                            origin_y = 2 + (k * (8.0 / num_rows_workstations));
                            pen_state.request.r = 0;
                            pen_state.request.g = 255;
                            pen_state.request.b = 0;
                        }

                        coordinates.request.x = origin_x;
                        coordinates.request.y = origin_y;
                        coordinates.request.theta = 0.0;
                        move_abs.call(coordinates);
                        
                        //turn on the pen and draw a circle
                        pen_state.request.off = 0;
                        pen.call(pen_state);

                        ros::Rate loop_rate(1);
                        int cnt = 0;
                        //this will draw the circle
                        while (cnt != 1)
                        {
                            geometry_msgs::Twist control_command;
                            control_command.linear.x = 2.5;
                            control_command.linear.y = 0.0;
                            control_command.linear.z = 0.0;
                            control_command.angular.x = 0.0;
                            control_command.angular.y = 0.0;
                            control_command.angular.z = 10.0;

                            control_pub.publish(control_command);
                            ros::spinOnce();
                            loop_rate.sleep();
                            ++cnt;
                        }
                        //turn off pen 
                        pen_state.request.off = 1;
                        pen.call(pen_state);
                    }
                }
            }
            
            

            //turn off pen and teleport to starting location
            pen_state.request.off = 1;
            pen.call(pen_state);

            //get the search step size and place turutle at the start
            float search_step_size;
            n.getParam("/search_step_size", search_step_size);
            coordinates.request.x = search_step_size;
            coordinates.request.y = search_step_size;
            coordinates.request.theta = 1.57;
            move_abs.call(coordinates);
            //re-enable pen with a different color
            pen_state.request.r = 0;
            pen_state.request.g = 0;
            pen_state.request.b = 0;
            pen_state.request.off = 0;
            pen_state.request.width = 2;
            pen.call(pen_state);
            ++executed;
        }
       else {
            ROS_WARN("Waiting for turtlesim service to start");
        }
    }
    //the board is setup and we will publish the search is ready to begin
    ros::Rate loop_rate(2);
    std_msgs::String start_search;
    start_search.data="START_SEARCH";
    while(ros::ok()){
        search_pub.publish(start_search);
        ros::spinOnce();
        loop_rate.sleep();
    }
    

    return 0;
}