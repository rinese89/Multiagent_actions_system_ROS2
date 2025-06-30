#include <functional>
#include <chrono>
#include <string>
#include <math.h>
#include <memory>
#include <thread>

#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> 
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

// reference msgs
#include "controller_pkg/msg/reference.hpp"

//data msgs
#include "std_msgs/msg/float64_multi_array.hpp"

// tf2 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "tb3_actions/action/controller_commands.hpp"

//NAMESPACES
std::string namespace_1 = "tb3_1";


class ControllerFollowerServer : public rclcpp::Node
{

    public: ControllerFollowerServer() : Node("control_follower_server_node")
    {

        //----------------------- Publishers --------------------

        std::string tb3_1_cmd_vel_topic = namespace_1 + "/cmd_vel";
        tb31_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(tb3_1_cmd_vel_topic,10);

        std::string tb3_1_data_topic = namespace_1 + "/data";
        tb31_data_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(tb3_1_data_topic,10);

        // ---------------------- Subscribers Robots Topics--------------------

        std::string aruco_topic = namespace_1 + "/aruco_pose";
            aruco_pose_sub_=this->create_subscription<geometry_msgs::msg::PoseStamped>(
                aruco_topic,10,std::bind(&ControllerFollowerServer::aruco_callback,this,std::placeholders::_1));

        std::string odom_topic_1 = namespace_1 + "/relative_odom";
            odom_1_sub_=this->create_subscription<nav_msgs::msg::Odometry>(
                odom_topic_1,10,std::bind(&ControllerFollowerServer::odom_1_callback,this,std::placeholders::_1));
        
            
        //-------------- Node info --------------------------------
        /*
        auto topics = this->get_topic_names_and_types();
        for (const auto & topic : topics)
        {
          const std::string & topic_name = topic.first;
        
          auto pubs = this->get_publishers_info_by_topic(topic_name);
          for (const auto & pub : pubs)
          {
            RCLCPP_INFO(this->get_logger(), "  %s publica en %s (tipo: %s)", 
              pub.node_name().c_str(), topic_name.c_str(), pub.topic_type().c_str());
          }
        
          auto subs = this->get_subscriptions_info_by_topic(topic_name);
          for (const auto & sub : subs)
          {
            RCLCPP_INFO(this->get_logger(), "  %s se suscribe a %s (tipo: %s)", 
              sub.node_name().c_str(), topic_name.c_str(), sub.topic_type().c_str());
          }
        }
                */

        // ------------------------- Threads -----------------------------

        //security_th_ = std::thread(&ControllerFollowerServer::security_control,this);
        //security_th_.detach();

        // ---------- Action callbacks --------------------

        control_server_ = rclcpp_action::create_server<tb3_actions::action::ControllerCommands>(
            this,"controller_tb31_command",
            std::bind(&ControllerFollowerServer::handle_goal_callback,this,std::placeholders::_1,std::placeholders::_2),
            std::bind(&ControllerFollowerServer::handle_cancel_callback,this,std::placeholders::_1),
            std::bind(&ControllerFollowerServer::handle_accepted_callback,this,std::placeholders::_1));
        
            RCLCPP_INFO(this->get_logger(),"Actions initilized");
    }

    private:

    rclcpp_action::GoalResponse handle_goal_callback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const tb3_actions::action::ControllerCommands::Goal> goal)
    {
        
        (void)uuid;
        RCLCPP_INFO(this->get_logger(),"Goal received to robot: %s Task: %s",goal->robot.c_str(), goal->status_rq.c_str());

        if(goal->robot == "rob1")
        {
            if(goal->status_rq=="start_control")
            {
                RCLCPP_INFO(this->get_logger(),"Goal Accepted");
                RCLCPP_INFO(this->get_logger(),"Prepare to start_control trajectory");
               return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;       
            }
            else if(goal->status_rq=="reset_to_initial_position")
            {
                RCLCPP_INFO(this->get_logger(),"Goal Accepted");
                RCLCPP_INFO(this->get_logger(),"Prepare to stop control");
               return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;       
            }
            else if(goal->status_rq=="recovery")
            {
                RCLCPP_INFO(this->get_logger(),"Goal Accepted");
                RCLCPP_INFO(this->get_logger(),"Prepare to recovery mode");
               return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;       
            }
            else if(goal->status_rq=="reset_odom")
            {
                RCLCPP_INFO(this->get_logger(),"Goal Accepted");
                RCLCPP_INFO(this->get_logger(),"Prepare to stop control");
               return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;       
            }
            else
            {
                RCLCPP_INFO(this->get_logger(),"Does not a correct goal -> rejected");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),"Goal Rejected from follower server");
            return rclcpp_action::GoalResponse::REJECT;
        }
    }        

    rclcpp_action::CancelResponse handle_cancel_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tb3_actions::action::ControllerCommands>> goal_handle)
    {

        if(goal_handle->is_executing())
            RCLCPP_INFO(this->get_logger(), "Goal still executing");
        
        cancel_goal_=true;
        control_status_rob1_=false;

        RCLCPP_INFO(this->get_logger(), "Cancel request received");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tb3_actions::action::ControllerCommands>> goal_handle)
    {

        const auto goal = goal_handle->get_goal();
        RCLCPP_INFO(this->get_logger(),"Goal to robot: %s Task: %s. Prepared to EXECUTING",goal->robot.c_str(), goal->status_rq.c_str());

        if (goal->robot == "rob1")
        {
            if(goal->status_rq == "start_control")
            {
                std::thread{std::bind(&ControllerFollowerServer::execute_control_rob1,this,std::placeholders::_1),goal_handle}.detach();
            }
            else if(goal->status_rq == "reset_to_initial_position")
            {
                std::thread{std::bind(&ControllerFollowerServer::reset_to_initial_position,this,std::placeholders::_1),goal_handle}.detach();
            }
            else if(goal->status_rq == "recovery")
            {
                std::thread{std::bind(&ControllerFollowerServer::recovery,this,std::placeholders::_1),goal_handle}.detach();
            }
            else if(goal->status_rq == "reset_odom")
            {
                std::thread{std::bind(&ControllerFollowerServer::reset_odom,this,std::placeholders::_1),goal_handle}.detach();
            }
            else
                RCLCPP_INFO(this->get_logger(),"Goal error");  
        }
        else
            RCLCPP_INFO(this->get_logger(),"Goal for rob1 rejected from follower server");
    }

    void execute_control_rob1(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tb3_actions::action::ControllerCommands>> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<tb3_actions::action::ControllerCommands::Result>();
        auto feedback = std::make_shared<tb3_actions::action::ControllerCommands::Feedback>();

        //while(!aruco_flag_)
        //{
        //    RCLCPP_INFO_ONCE(this->get_logger(),"Waiting to catch the aruco marker");
        //}
        
        RCLCPP_INFO_ONCE(this->get_logger(),"Marker recieve, starting control");

        control_status_rob1_=true;  
        double Klin = 0.6;
        double Krot = 0.8;  
        double error_d, error_a;
        
        rclcpp::Rate rate(10); 

        while(control_status_rob1_)
        {
            if(abs(theta_aruco_)<0.9)
            {
                double distance =  sqrt(x_aruco_*x_aruco_+y_aruco_*y_aruco_);
                error_d = distance - fix_distance_;
                error_a = atan2(y_aruco_,x_aruco_);

                geometry_msgs::msg::Twist cmd_msg_rob1;

                 if(error_d > 0.05 || error_d < -0.05  || error_a > 0.05 || error_a < -0.05)
                {
                    cmd_msg_rob1.linear.x = Klin * error_d;
                    cmd_msg_rob1.angular.z = Krot * error_a;
                }

                else
                {
                    cmd_msg_rob1.angular.z = 0.0;
                    cmd_msg_rob1.linear.x = 0.0;
                }
                feedback->robot = "rob1";
                goal_handle->publish_feedback(feedback);
                tb31_cmd_vel_->publish(cmd_msg_rob1);
            }

            else
            {
                RCLCPP_INFO_ONCE(this->get_logger(),"Aruco signal lost");
                RCLCPP_INFO(this->get_logger(),"Last aruco angle catched: %.5f",theta_aruco_);
                control_status_rob1_ = false;
            }
            std_msgs::msg::Float64MultiArray data_msg_rob1;
            data_msg_rob1.data = {x_rob_1,y_rob_1, theta_rob_1,error_d,error_a,x_aruco_,y_aruco_,theta_aruco_};
            tb31_data_ -> publish(data_msg_rob1);
            rate.sleep(); 
        }
        geometry_msgs::msg::Twist cmd_msg_rob1;
        cmd_msg_rob1.angular.z = 0.0;
        cmd_msg_rob1.linear.x = 0.0;
        tb31_cmd_vel_->publish(cmd_msg_rob1);

        if(cancel_goal_)
        {
            RCLCPP_INFO(this->get_logger(),"Stop Control. REASON: Goal canceled");
            result->robot = "rob1";
            result->status_rs="Stop Control. REASON: Goal canceled";
            goal_handle->succeed(result);
        }
        else
        {
            result->robot = "rob1";
            result->status_rs="aruco_lost";
            goal_handle->succeed(result);
            aruco_flag_=false;  
        }
    }

    void recovery(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tb3_actions::action::ControllerCommands>> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<tb3_actions::action::ControllerCommands::Result>();
        auto feedback = std::make_shared<tb3_actions::action::ControllerCommands::Feedback>();
        
        RCLCPP_INFO(this->get_logger(),"Inside function recovery mode");

        double x_rob0 = goal->x_pos;
        double y_rob0 = goal->y_pos;
        double theta_rob0 = goal->theta;

        RCLCPP_INFO(this->get_logger(),"x_rob0: %.5f",x_rob0);
        RCLCPP_INFO(this->get_logger(),"y_rob0: %.5f",y_rob0);
        RCLCPP_INFO(this->get_logger(),"theta_rob0: %.5f",theta_rob0);

        tf2::Matrix3x3 transformation_matrix(
            cos(theta_rob0), -sin(theta_rob0), 0.0,
           sin(theta_rob0), cos(theta_rob0), 0.0,
            0.0, 0.0, 1.0
        );

        tf2::Vector3 goal_from_rob0 = {-0.7, 0.0, 0.0};

        // Rotation
        tf2::Vector3 goal_rot = transformation_matrix * goal_from_rob0;

        //Translation
        double goal_x = goal_rot[0] + x_rob0;
        double goal_y = goal_rot[1] + y_rob0; 
        double goal_theta = theta_rob0;

        RCLCPP_INFO(this->get_logger(),"goal_x: %.5f",goal_x);
        RCLCPP_INFO(this->get_logger(),"goal_y: %.5f",goal_y);
        RCLCPP_INFO(this->get_logger(),"goal_theta: %.5f",goal_theta);

        RCLCPP_INFO(this->get_logger(),"x_rob1: %.5f",x_rob_1);
        RCLCPP_INFO(this->get_logger(),"y_rob1: %.5f",y_rob_1);
        RCLCPP_INFO(this->get_logger(),"theta_rob1: %.5f",theta_rob_1);

        double mu = 0.5*((goal_x - x_rob_1)*cos(theta_rob_1)+ (goal_y - y_rob_1)*sin(theta_rob_1))/((goal_y - y_rob_1)*cos(theta_rob_1) - (goal_x - x_rob_1)*sin(theta_rob_1));
        double x_c = ((x_rob_1 + goal_x)/2) + mu*(y_rob_1 - goal_y);
        double y_c = ((y_rob_1 + goal_y)/2) + mu*(goal_x - x_rob_1);
        double dif_x = x_rob_1 - x_c;
        double dif_y = y_rob_1 - y_c;
        double r = hypot(dif_x,dif_y);
        double arc = atan2((goal_y - y_c),(goal_x - x_c)) - atan2((y_rob_1 - y_c),(x_rob_1 - x_c));

        // Asegura que arc esté en [-π, π]
        //arc = std::fmod(arc + M_PI, 2*M_PI) - M_PI;
        
        double max_vel = 0.2;

        double dt = (r+b)*abs(arc)/max_vel;

        double v_rob = r*abs(arc)/dt;
        double w_rob = arc/dt;

        RCLCPP_INFO(this->get_logger(),"mu: %.5f",mu);
        RCLCPP_INFO(this->get_logger(),"x_c: %.5f, y_c: %.5f",x_c,y_c);
        RCLCPP_INFO(this->get_logger(),"arc: %.5f",arc);
        RCLCPP_INFO(this->get_logger(),"dt: %.5f",dt);
        RCLCPP_INFO(this->get_logger(),"r: %.5f",r);

        
        geometry_msgs::msg::Twist cmd_vel_msg;
        rclcpp::Time init_time = this->get_clock()->now();
        double max_error;
        rclcpp::Rate rate(10); 
        while(!recovery_flag_ && !cancel_goal_)
        {
            rclcpp::Time time = this->get_clock()->now();
            rclcpp::Duration diff = time - init_time;
            double recovery_time = diff.seconds()+diff.nanoseconds()*1e-9;
            double dx = goal_x - x_rob_1;
            double dy = goal_y - y_rob_1;
            double error_d = hypot(dx, dy);
            double error_a = goal_theta - theta_rob_1;
            //RCLCPP_INFO(this->get_logger(),"Recovery Time: %.5f",recovery_time);

            if(recovery_time < 5.0)
            {
                max_error = 0.05;
                RCLCPP_INFO_ONCE(this->get_logger(),"Max Error: %.5f",max_error);
            }
            
            else if (recovery_time < 7.0)
            {
                max_error = 0.1;
                RCLCPP_INFO_ONCE(this->get_logger(),"Max Error: %.5f",max_error);

            }
            else if(recovery_time < 12.0)
            {
                max_error = 0.15;
                RCLCPP_INFO_ONCE(this->get_logger(),"Max Error: %.5f",max_error);

            }
            else if(recovery_time < 15.0)
            {
                max_error = 0.25;
                RCLCPP_INFO_ONCE(this->get_logger(),"Max Error: %.5f",max_error);

            }
            else if(recovery_time < 20.0)
            {
                max_error = 0.35;
                RCLCPP_INFO_ONCE(this->get_logger(),"Max Error: %.5f",max_error);

            }

            if (abs(error_d) > max_error)
            {
                cmd_vel_msg.linear.x = v_rob;
                cmd_vel_msg.angular.z = w_rob;
            }
            else if(abs(error_a)> 0.05)
            {
                cmd_vel_msg.linear.x = 0.0;
                cmd_vel_msg.angular.z = 0.5 * error_a;
                RCLCPP_INFO_ONCE(this->get_logger(),"Error angle: %.5f",error_a);
            }
            else
            {
                cmd_vel_msg.linear.x = 0.0;
                cmd_vel_msg.angular.z = 0.0;
                recovery_flag_ = true;
                RCLCPP_INFO(this->get_logger(),"Achived recovery position");
                RCLCPP_INFO(this->get_logger(),"x_rob1: %.5f",x_rob_1);
                RCLCPP_INFO(this->get_logger(),"y_rob1: %.5f",y_rob_1);
                RCLCPP_INFO(this->get_logger(),"theta_rob1: %.5f",theta_rob_1);
            }
            /*
            double dx = goal_x - x_rob_1;
            double dy = goal_y - y_rob_1;
            //RCLCPP_INFO(this->get_logger(),"X_odom: %.5f",x_rob_1);
            //RCLCPP_INFO(this->get_logger(),"Y_odom: %.5f",y_rob_1);
            //RCLCPP_INFO(this->get_logger(),"Theta: %.5f",theta_rob_1);
            //RCLCPP_INFO(this->get_logger(),"Dx: %.5f",dx);
            //RCLCPP_INFO(this->get_logger(),"Dy: %.5f",dy);

            //if(theta_rob_1 > M_PI)
            //theta_rob_1 = theta_rob_1 - 2*M_PI;

            double error_d = hypot(dx, dy);
            double angle_to_goal = atan2(dy, dx);
            double angle_diff = angle_to_goal - theta_rob_1;
            RCLCPP_INFO_ONCE(this->get_logger(),"Angle: %.5f",angle_diff);
            double error_a = goal_theta - theta_rob_1;

            geometry_msgs::msg::Twist cmd_vel_msg;

            if (abs(error_d) > 0.1)
            {
                cmd_vel_msg.linear.x = 0.12 * error_d;
                cmd_vel_msg.angular.z = 0.5 * angle_diff;
                RCLCPP_INFO(this->get_logger(),"Error_distance: %.5f",error_d);
                RCLCPP_INFO(this->get_logger(),"Angle: %.5f",angle_diff);
            }
            else if(abs(error_a)> 0.05)
            {
                cmd_vel_msg.linear.x = 0.0;
                cmd_vel_msg.angular.z = 0.5 * error_a;
                RCLCPP_INFO_ONCE(this->get_logger(),"Error angle: %.5f",error_a);
            }
            else
            {
                cmd_vel_msg.linear.x = 0.0;
                cmd_vel_msg.angular.z = 0.0;
                recovery_flag_ = true;
                RCLCPP_INFO(this->get_logger(),"Achived recovery position");
                RCLCPP_INFO(this->get_logger(),"x_rob1: %.5f",x_rob_1);
                RCLCPP_INFO(this->get_logger(),"y_rob1: %.5f",y_rob_1);
                RCLCPP_INFO(this->get_logger(),"theta_rob1: %.5f",theta_rob_1);
            }
            */
            feedback->robot = "rob1";
            goal_handle->publish_feedback(feedback);
            tb31_cmd_vel_->publish(cmd_vel_msg);

            std_msgs::msg::Float64MultiArray data_msg_rob1;
            data_msg_rob1.data = {x_rob_1,y_rob_1, theta_rob_1,error_d,error_a,x_aruco_,y_aruco_,theta_aruco_,recovery_time};
            tb31_data_ -> publish(data_msg_rob1);
            
            rate.sleep(); 
        }

        if(cancel_goal_)
        {
            RCLCPP_INFO(this->get_logger(),"Stop Control. REASON: Goal canceled");
            result->robot = "rob1";
            result->status_rs="Stop Control. REASON: Goal canceled";
            goal_handle->succeed(result); 
        }
        else
        {
           recovery_flag_ = false;
           result->robot = "rob1";
           result->status_rs="recovery_succeed";
           goal_handle->succeed(result);    
        }
    }


    void reset_to_initial_position(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tb3_actions::action::ControllerCommands>> goal_handle){

        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<tb3_actions::action::ControllerCommands::Result>();
        auto feedback = std::make_shared<tb3_actions::action::ControllerCommands::Feedback>();

        RCLCPP_INFO(this->get_logger(),"Goal receive in stop control to robot %s, Task: %s", goal->robot.c_str(),goal->status_rq.c_str());
        double goal_x = -0.8;
        double goal_y = 0.0; 
        double goal_theta = 0.0;

        RCLCPP_INFO(this->get_logger(),"goal_x: %.5f",goal_x);
        RCLCPP_INFO(this->get_logger(),"goal_y: %.5f",goal_y);
        RCLCPP_INFO(this->get_logger(),"goal_theta: %.5f",goal_theta);

        RCLCPP_INFO(this->get_logger(),"Initial x_rob_1: %.5f",x_rob_1);
        RCLCPP_INFO(this->get_logger(),"Initial y_rob_1: %.5f",y_rob_1);
        RCLCPP_INFO(this->get_logger(),"Initial theta_rob_1: %.5f",theta_rob_1);

        rclcpp::Rate rate(10); 
        while(!reset_flag_)
        {
            
            double dx = goal_x - x_rob_1;
            double dy = goal_y - y_rob_1;
            //RCLCPP_INFO(this->get_logger(),"X_odom: %.5f",x_rob_1);
            //RCLCPP_INFO(this->get_logger(),"Y_odom: %.5f",y_rob_1);
            //RCLCPP_INFO(this->get_logger(),"Theta: %.5f",theta_rob_1);
            //RCLCPP_INFO(this->get_logger(),"Dx: %.5f",dx);
            //RCLCPP_INFO(this->get_logger(),"Dy: %.5f",dy);

            //if(theta_rob_1 > M_PI)
            //theta_rob_1 = theta_rob_1 - 2*M_PI;

            double error_d = hypot(dx, dy);
            double angle_to_goal = atan2(dy, dx);
            double angle_diff = angle_to_goal - theta_rob_1;
            RCLCPP_INFO_ONCE(this->get_logger(),"Angle: %.5f",angle_diff);
            double error_a = goal_theta - theta_rob_1;

            geometry_msgs::msg::Twist cmd_vel_msg;

            if (abs(error_d) > 0.1)
            {
                if (angle_diff < -5)
                {
                    angle_diff = angle_diff + 2*M_PI;
                }
                else if (angle_diff > 5)
                {
                    angle_diff = angle_diff - 2*M_PI;
                }

                if(error_d > 1.0)
                {
                    Kd = 0.09;
                }
                else
                {
                    Kd = 0.12;
                }

                cmd_vel_msg.linear.x = Kd * error_d;
                cmd_vel_msg.angular.z = 0.5 * angle_diff;
                RCLCPP_INFO(this->get_logger(),"Error_distance: %.5f",error_d);
                RCLCPP_INFO(this->get_logger(),"Angle: %.5f",angle_diff);
            }
            else if(abs(error_a)> 0.05)
            {
                cmd_vel_msg.linear.x = 0.0;
                cmd_vel_msg.angular.z = 0.5 * error_a;
                RCLCPP_INFO_ONCE(this->get_logger(),"Error angle: %.5f",error_a);
            }
            else
            {
                cmd_vel_msg.linear.x = 0.0;
                cmd_vel_msg.angular.z = 0.0;
                reset_flag_ = true;
                RCLCPP_INFO(this->get_logger(),"Achived reset position");
                RCLCPP_INFO(this->get_logger(),"Final x_rob1: %.5f",x_rob_1);
                RCLCPP_INFO(this->get_logger(),"Final y_rob1: %.5f",y_rob_1);
                RCLCPP_INFO(this->get_logger(),"Final theta_rob1: %.5f",theta_rob_1);
            }
            //feedback->robot = "rob0";
            //goal_handle->publish_feedback(feedback);
            tb31_cmd_vel_->publish(cmd_vel_msg);
            rate.sleep(); 
        }

           reset_flag_ = false;
           result->robot = "rob1";
           result->status_rs="reset_succeed";
           goal_handle->succeed(result);    
    }

    void reset_odom(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tb3_actions::action::ControllerCommands>> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<tb3_actions::action::ControllerCommands::Result>();

        RCLCPP_INFO(this->get_logger(),"Reseting odometry");

        x0_odom_1 = x_odom_1;
        y0_odom_1 = y_odom_1;
        theta0_odom_1 = theta_odom_1;

        result->robot = "rob1";
        result->status_rs="Odometry reseted succeed";
        goal_handle->succeed(result);
    }

    //------------------------- Callbacks Subscriptions functions -----------------------------------------------------

    void aruco_callback(const geometry_msgs::msg::PoseStamped::SharedPtr aruco_msg)
    {
        aruco_flag_ = true;
        time_aruco_ = aruco_msg->header.stamp.sec+ aruco_msg->header.stamp.nanosec*1e-9;

        tf2::Matrix3x3 transformation_matrix(
            0.0, 0.0, 1.0,
           -1.0, 0.0, 0.0,
            0.0, -1.0, 0.0
        );

        tf2::Vector3 tvec(
            aruco_msg->pose.position.x,
            aruco_msg->pose.position.y,
            aruco_msg->pose.position.z
        );

        tf2::Vector3 new_pose = transformation_matrix * tvec;
        x_aruco_ = new_pose.x();
        y_aruco_ = new_pose.y();

        // Conversión de cuaternión a RPY
        tf2::Quaternion q(
            aruco_msg->pose.orientation.x,
            aruco_msg->pose.orientation.y,
            aruco_msg->pose.orientation.z,
            aruco_msg->pose.orientation.w
        );

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        if (roll < 0)
            theta_aruco_ = -(M_PI + roll);
        else
            theta_aruco_ = -(roll - M_PI);
        
        //RCLCPP_INFO(this->get_logger(),"Aruc_1: Time: %.5f, x: %.5f, y: %.5f, theta: %.5f",time_aruco_,x_aruco_,y_aruco_,theta_aruco_);
    }

    void odom_1_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
        time_odom_1 = odom_msg->header.stamp.sec + odom_msg->header.stamp.nanosec*1e-9;
        x_odom_1 = odom_msg->pose.pose.position.x - x0_odom_1 - x_init_odom_1;
        y_odom_1 = odom_msg->pose.pose.position.y - y0_odom_1 - y_init_odom_1;

        //xp_odom_1 = odom_msg->twist.twist.linear.x;
        //yp_odom_1 = odom_msg->twist.twist.linear.y;
        
        tf2::Quaternion q(odom_msg->pose.pose.orientation.x,
                            odom_msg->pose.pose.orientation.y,
                            odom_msg->pose.pose.orientation.z,
                            odom_msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll,pitch,yaw;
        m.getRPY(roll,pitch,yaw);

        theta_odom_1 = yaw - theta0_odom_1 -theta_init_odom_1;

        //-- Fusion---
        x_rob_1 = x_odom_1;
        y_rob_1 = y_odom_1;
        theta_rob_1 = theta_odom_1;

        //RCLCPP_INFO(this->get_logger(),"Odom_1_odom: x: %.5f, y: %.5f, theta: %.5f", x_odom_1,y_odom_1,theta_odom_1);
    }

    //------------------------- Security function -----------------------------------------------------

    rclcpp_action::Server<tb3_actions::action::ControllerCommands>::SharedPtr control_server_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_1_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr tb31_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tb31_data_;


    bool control_status_rob1_ = false;
    bool aruco_flag_ = false;
    bool recovery_flag_ = false;
    bool reset_flag_ = false;
    bool cancel_goal_ = false;

    //--------------- TB3_1 -------------------------------- 
    double x_rob_1,y_rob_1,theta_rob_1;
    double time_odom_1,x_odom_1,y_odom_1,theta_odom_1,xp_odom_1,yp_odom_1;
    double time_aruco_,x_aruco_,y_aruco_,theta_aruco_;
    double x0_odom_1 = 0.0;
    double y0_odom_1 = 0.0;
    double theta0_odom_1 = 0.0;
    double x_init_odom_1 = 0.0;
    double y_init_odom_1 = 0.0;
    double theta_init_odom_1 = 0.0;

    //------------- Controller -----------------------------
    double b = (0.16/2);
    double fix_distance_ = 0.3;
    double Kd;
};


int main(int argc, char* argv[]){

    rclcpp::init(argc,argv);

    ControllerFollowerServer::SharedPtr node;
    node = std::make_shared<ControllerFollowerServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}