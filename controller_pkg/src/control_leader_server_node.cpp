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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "tb3_actions/action/controller_commands.hpp"

//NAMESPACES
std::string namespace_0 = "tb3_0";


class ControllerLeaderServer : public rclcpp::Node{

    public: ControllerLeaderServer() : Node("control_leader_server_node"),tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_){

        //callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Opciones de suscripción usando el grupo
        //rclcpp::SubscriptionOptions options;
        //options.callback_group = callback_group_;

        //----------------------- Publishers ---------------------
        std::string tb3_0_cmd_vel_topic = namespace_0 + "/cmd_vel";
        tb30_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(tb3_0_cmd_vel_topic,10);

        std::string tb3_0_data_topic = namespace_0 + "/data";
        tb30_data_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(tb3_0_data_topic,10);

        tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&ControllerLeaderServer::get_transform, this));

        // ---------------------- Subscribers Robots Topics--------------------

        /*
        std::string amcl_topic = namespace_0 + "/amcl_pose";
        amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            amcl_topic,10,std::bind(&ControllerLeaderServer::amcl_callback,this,std::placeholders::_1));

        std::string odom_topic_0 = namespace_0 + "/last_pose_tf";
        odom_0_sub_=this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_0,10,std::bind(&ControllerLeaderServer::odom_0_callback,this,std::placeholders::_1));

        // ---------------------- Subscribers Robots Topics--------------------
        std::string reference_topic = "/reference";
        reference_sub_ = this->create_subscription<controller_pkg::msg::Reference>(
           reference_topic,10,std::bind(&ControllerLeaderServer::reference_callback,this,std::placeholders::_1));
            
        //-------------- Node info --------------------------------
        
        
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

        std::string amcl_topic = namespace_0 + "/amcl_pose";
        amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            amcl_topic, rclcpp::QoS(10), std::bind(&ControllerLeaderServer::amcl_callback, this, std::placeholders::_1));

        //std::string odom_topic_0 = namespace_0 + "/last_pose_tf";
        //odom_0_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //    odom_topic_0, rclcpp::QoS(10), std::bind(&ControllerLeaderServer::odom_0_callback, this, std::placeholders::_1));

        std::string odom_topic_0 = namespace_0 + "/odom";
        odom_0_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_0, rclcpp::QoS(10), std::bind(&ControllerLeaderServer::odom_0_callback, this, std::placeholders::_1));

        std::string reference_topic = "/reference";
        reference_sub_ = this->create_subscription<controller_pkg::msg::Reference>(
            reference_topic, rclcpp::QoS(10), std::bind(&ControllerLeaderServer::reference_callback, this, std::placeholders::_1));

        // ---------- Action callbacks --------------------

        control_server_ = rclcpp_action::create_server<tb3_actions::action::ControllerCommands>(
            this,"controller_tb30_command",
            std::bind(&ControllerLeaderServer::handle_goal_callback,this,std::placeholders::_1,std::placeholders::_2),
            std::bind(&ControllerLeaderServer::handle_cancel_callback,this,std::placeholders::_1),
            std::bind(&ControllerLeaderServer::handle_accepted_callback,this,std::placeholders::_1));
        
            RCLCPP_INFO(this->get_logger(),"Actions initilized");
    }

    private:

    rclcpp_action::GoalResponse handle_goal_callback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const tb3_actions::action::ControllerCommands::Goal> goal)
    {
        
        (void)uuid;
        RCLCPP_INFO(this->get_logger(),"Goal received to robot: %s Task: %s",goal->robot.c_str(), goal->status_rq.c_str());

            if(goal->status_rq=="start_control")
            {
                RCLCPP_INFO(this->get_logger(),"Goal Accepted");
                RCLCPP_INFO(this->get_logger(),"Prepare to start_control trajectory");
               return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;       
            }
            else if(goal->status_rq=="reset_to_initial_position"){
                RCLCPP_INFO(this->get_logger(),"Goal Accepted");
                RCLCPP_INFO(this->get_logger(),"Prepare to stop control");
               return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;       
            }
            else if(goal->status_rq=="reset_odom"){
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

    rclcpp_action::CancelResponse handle_cancel_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tb3_actions::action::ControllerCommands>> goal_handle)
    {

        control_status_rob0_ = false;
        cancel_goal_=true;
        if(goal_handle->is_executing())
            RCLCPP_INFO(this->get_logger(), "Goal still executing");
        

        RCLCPP_INFO(this->get_logger(), "Cancel request received");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tb3_actions::action::ControllerCommands>> goal_handle)
    {

        const auto goal = goal_handle->get_goal();
        RCLCPP_INFO(this->get_logger(),"Goal to robot: %s Task: %s. Prepared to EXECUTING",goal->robot.c_str(), goal->status_rq.c_str());

        if (goal->robot == "rob0")
        {
            if(goal->status_rq == "start_control")
            {
                std::thread{std::bind(&ControllerLeaderServer::execute_control_rob0,this,std::placeholders::_1),goal_handle}.detach();
            }
            else if(goal->status_rq == "reset_to_initial_position")
            {
                std::thread{std::bind(&ControllerLeaderServer::reset_to_initial_position,this,std::placeholders::_1),goal_handle}.detach();
            }
            else if(goal->status_rq == "reset_odom")
            {
                std::thread{std::bind(&ControllerLeaderServer::reset_odom,this,std::placeholders::_1),goal_handle}.detach();
            }
            else
                RCLCPP_INFO(this->get_logger(),"Goal error");  
        } 
    }

    void execute_control_rob0(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tb3_actions::action::ControllerCommands>> goal_handle){

        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<tb3_actions::action::ControllerCommands::Result>();
        auto feedback = std::make_shared<tb3_actions::action::ControllerCommands::Feedback>();

        control_status_rob0_=true;
        rclcpp::Rate rate(20); 

        RCLCPP_INFO(this->get_logger(),"Goal receive in execute start_control function: Robot: %s Task: %s", goal->robot.c_str(), goal->status_rq.c_str());
        if(goal->status_rq=="start_control")
        {   
            while(control_status_rob0_)
            {
                RCLCPP_INFO_ONCE(this->get_logger(),"Executing Controller");    
                
                double x_inc_odom = cos(theta_last_odom)*(x_odom_0-x_last_odom) + sin(theta_last_odom)*(y_odom_0-y_last_odom);
                double y_inc_odom = -sin(theta_last_odom)*(x_odom_0-x_last_odom) + cos(theta_last_odom)*(y_odom_0-y_last_odom);
                double theta_inc_odom = theta_odom_0 - theta_last_odom;

                x_rob_0 = cos(theta_last_tf)*x_inc_odom-sin(theta_last_tf)*y_inc_odom + x_last_tf;
                y_rob_0 = sin(theta_last_tf)*x_inc_odom+cos(theta_last_tf)*y_inc_odom + y_last_tf;
                theta_rob_0 = theta_inc_odom+theta_last_tf;
                
                double xe_rob = x_rob_0 + e*cos(theta_rob_0);
                double ye_rob = y_rob_0 + e*sin(theta_rob_0);
                
                double x_error = x_ref_ - xe_rob;
                double y_error = y_ref_ - ye_rob;

                double x_vel = xp_ref_ + k*x_error;
                double y_vel = yp_ref_ + k*y_error;

                double wL=(1/e)*((e*cos(theta_rob_0)+b*sin(theta_rob_0))*x_vel + (e*sin(theta_rob_0)-b*cos(theta_rob_0))*y_vel);
                double wR=(1/e)*((e*cos(theta_rob_0)-b*sin(theta_rob_0))*x_vel + (e*sin(theta_rob_0)+b*cos(theta_rob_0))*y_vel);

                double v = (wR+wL)/2;
                double w = (wR-wL)/(2*b);

                geometry_msgs::msg::Twist cmd_msg_rob0;
                std_msgs::msg::Float64MultiArray data_msg_rob0;

                cmd_msg_rob0.linear.x = v;
                cmd_msg_rob0.angular.z = w;

                data_msg_rob0.data = {t_,x_rob_0,y_rob_0, theta_rob_0,x_ref,y_ref,xp_ref,yp_ref,x_vel, y_vel, x_error, y_error, wL, wR,v,w,xe_rob,ye_rob};

                tb30_cmd_vel_->publish(cmd_msg_rob0);
                tb30_data_ -> publish(data_msg_rob0);

                feedback->robot = "rob0";
                feedback->x_pos = x_rob_0;
                feedback->y_pos = y_rob_0;
                feedback->theta = theta_rob_0;
                goal_handle->publish_feedback(feedback);
                
                rate.sleep(); 
            }

            RCLCPP_INFO(this->get_logger(),"x_feed: %.5f",x_rob_0);
            RCLCPP_INFO(this->get_logger(),"y_feed: %.5f",y_rob_0);
            RCLCPP_INFO(this->get_logger(),"theta_feed: %.5f",theta_rob_0);
            
            geometry_msgs::msg::Twist cmd_msg_rob0;
            cmd_msg_rob0.angular.z = 0.0;
            cmd_msg_rob0.linear.x = 0.0;
            tb30_cmd_vel_->publish(cmd_msg_rob0);
            
            if(cancel_goal_)
            {
            result->robot = "rob0";
            result->status_rs = "Stop Control. REASON: Goal canceled";
            RCLCPP_INFO(this->get_logger(),"Stop Control. REASON: Goal canceled");
            goal_handle->succeed(result);
            }
            else
            {
            result->robot = "rob0";
            result->status_rs="Control goal ROB 0 cancel. REASON: waiting to follower robot";
            RCLCPP_INFO(this->get_logger(),"Control goal ROB 0 cancel. REASON: Waiting to follower robot");
            goal_handle->succeed(result);
            }
        }
    }

    void reset_to_initial_position(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tb3_actions::action::ControllerCommands>> goal_handle){

        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<tb3_actions::action::ControllerCommands::Result>();
        auto feedback = std::make_shared<tb3_actions::action::ControllerCommands::Feedback>();

        RCLCPP_INFO(this->get_logger(),"Goal receive in stop control to robot %s, Task: %s", goal->robot.c_str(),goal->status_rq.c_str());
        double goal_x = 0.0;
        double goal_y = 0.0; 
        double goal_theta = 0.0;

        RCLCPP_INFO(this->get_logger(),"goal_x: %.5f",goal_x);
        RCLCPP_INFO(this->get_logger(),"goal_y: %.5f",goal_y);
        RCLCPP_INFO(this->get_logger(),"goal_theta: %.5f",goal_theta);

        RCLCPP_INFO(this->get_logger(),"Initial x_rob_0: %.5f",x_rob_0);
        RCLCPP_INFO(this->get_logger(),"Initial y_rob_0: %.5f",y_rob_0);
        RCLCPP_INFO(this->get_logger(),"Initial theta_rob_0: %.5f",theta_rob_0);

        rclcpp::Rate rate(100); 
        while(!reset_flag_)
        {
            double dx = goal_x - x_rob_0;
            double dy = goal_y - y_rob_0;
            //RCLCPP_INFO(this->get_logger(),"X_odom: %.5f",x_rob_1);
            //RCLCPP_INFO(this->get_logger(),"Y_odom: %.5f",y_rob_1);
            //RCLCPP_INFO(this->get_logger(),"Theta: %.5f",theta_rob_1);
            //RCLCPP_INFO(this->get_logger(),"Dx: %.5f",dx);
            //RCLCPP_INFO(this->get_logger(),"Dy: %.5f",dy);

            //if(theta_rob_1 > M_PI)
            //theta_rob_1 = theta_rob_1 - 2*M_PI;

            double error_d = hypot(dx, dy);
            double angle_to_goal = atan2(dy, dx);
            double angle_diff = angle_to_goal - theta_rob_0;
            RCLCPP_INFO_ONCE(this->get_logger(),"Angle: %.5f",angle_diff);
            double error_a = goal_theta - theta_rob_0;

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
                RCLCPP_INFO(this->get_logger(),"Final x_rob0: %.5f",x_rob_0);
                RCLCPP_INFO(this->get_logger(),"Final y_rob0: %.5f",y_rob_0);
                RCLCPP_INFO(this->get_logger(),"Final theta_rob0: %.5f",theta_rob_0);
            }
            //feedback->robot = "rob0";
            //goal_handle->publish_feedback(feedback);
            tb30_cmd_vel_->publish(cmd_vel_msg);
            rate.sleep(); 
        }

           reset_flag_ = false;
           result->robot = "rob0";
           result->status_rs="reset_succeed";
           goal_handle->succeed(result);    
    }

    void get_transform()
    {
        const std::string target_frame = "map";
        const std::string source_frame = "tb3_0/base_footprint";
        const rclcpp::Duration timeout = rclcpp::Duration::from_seconds(0.1);

        geometry_msgs::msg::TransformStamped transform_stamped;

        if (tf_buffer_.canTransform(target_frame, source_frame, this->now(), tf2::durationFromSec(0.5))) {
            try {

                transform_stamped = tf_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
                nav_msgs::msg::Odometry last_pose_tf;
                last_pose_tf.header.stamp = this->now();
                last_pose_tf.header.frame_id = target_frame;
                last_pose_tf.child_frame_id = source_frame;

                x_last_tf = transform_stamped.transform.translation.x;
                y_last_tf = transform_stamped.transform.translation.y;
                tf2::Quaternion q(transform_stamped.transform.rotation.x,
                                    transform_stamped.transform.rotation.y,
                                    transform_stamped.transform.rotation.z,
                                    transform_stamped.transform.rotation.w);
                tf2::Matrix3x3 m(q);
                double roll,pitch,yaw;
                m.getRPY(roll,pitch,yaw);
                theta_last_tf = yaw;

                x_last_odom = x_odom_0;
                y_last_odom = y_odom_0;
                theta_last_odom = theta_odom_0;
            } 
            catch (tf2::TransformException &ex) {
              RCLCPP_WARN(this->get_logger(), "Excepción al obtener la transformación: %s", ex.what());
            }
        } 
        else 
        {
        RCLCPP_WARN(this->get_logger(), "Transformación no disponible entre %s y %s en %f segundos",
                target_frame.c_str(), source_frame.c_str(), timeout.seconds());
        }
    }
    

    void reset_odom(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tb3_actions::action::ControllerCommands>> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<tb3_actions::action::ControllerCommands::Result>();

        RCLCPP_INFO(this->get_logger(),"Reseting odometry");

        x0_odom_0 = x_odom_0;
        y0_odom_0 = y_odom_0;
        theta0_odom_0 = theta_odom_0;

        //resul->robot = "rob1";
        result->status_rs="Odometry reseted succeed";
        goal_handle->succeed(result);
    }

    //------------------------- Callbacks Subscriptions functions -----------------------------------------------------
    
    void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr amcl_msg)
    {

        time_amcl = amcl_msg->header.stamp.sec + amcl_msg->header.stamp.nanosec*1e-9;
        x_amcl = amcl_msg->pose.pose.position.x;
        y_amcl = amcl_msg->pose.pose.position.y;

        geometry_msgs::msg::Quaternion q_msg = amcl_msg->pose.pose.orientation;

        tf2::Quaternion q_amcl;
        tf2::fromMsg(q_msg,q_amcl);
        tf2::Matrix3x3 m(q_amcl);
        double roll,pitch,yaw;
        m.getRPY(roll,pitch,yaw);

        theta_amcl = yaw;

        //x_rob_0 = x_amcl;
        //y_rob_0 = y_amcl;
        //theta_rob_0 = theta_amcl;
        
        //RCLCPP_INFO(this->get_logger(),"AMCL x: %.5f, y: %.5f, theta: %.5f", x_amcl,y_amcl,theta_amcl);


        //RCLCPP_INFO(this->get_logger(), "Callback AMCL thread: %ld", std::hash<std::thread::id>{}(std::this_thread::get_id()));
        }

    void odom_0_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
        
        //if(!init_odom){
        //    time_odom_0 = odom_msg->header.stamp.sec + odom_msg->header.stamp.nanosec*1e-9;
        //    x0_odom_0 = odom_msg->pose.pose.position.x;
        //    y0_odom_0 = odom_msg->pose.pose.position.y;
        //
        //    tf2::Quaternion q(odom_msg->pose.pose.orientation.x,
        //                        odom_msg->pose.pose.orientation.y,
        //                        odom_msg->pose.pose.orientation.z,
        //                        odom_msg->pose.pose.orientation.w);
        //    tf2::Matrix3x3 m(q);
        //    double roll,pitch,yaw;
        //    m.getRPY(roll,pitch,yaw);
//
        //    theta0_odom_0 = yaw;
        //    init_odom = true;
        //}
        
        time_odom_0 = odom_msg->header.stamp.sec + odom_msg->header.stamp.nanosec*1e-9;
        x_odom_0 = odom_msg->pose.pose.position.x - x0_odom_0;
        y_odom_0 = odom_msg->pose.pose.position.y - y0_odom_0;
        
        tf2::Quaternion q(odom_msg->pose.pose.orientation.x,
                            odom_msg->pose.pose.orientation.y,
                            odom_msg->pose.pose.orientation.z,
                            odom_msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll,pitch,yaw;
        m.getRPY(roll,pitch,yaw);

        theta_odom_0 = yaw - theta0_odom_0;        

    
        
        //RCLCPP_INFO(this->get_logger(), "Callback Odom thread: %ld", std::hash<std::thread::id>{}(std::this_thread::get_id()));
       // RCLCPP_INFO(this->get_logger(),"Odom x: %.5f, y: %.5f, theta: %.5f", x_odom_0,y_odom_0,theta_odom_0);
    }

    void reference_callback(const controller_pkg::msg::Reference::SharedPtr ref_msg)
    {
        x_ref = ref_msg->pose.position.x;
        y_ref = ref_msg->pose.position.y; 
        tf2::Quaternion q(
            ref_msg->pose.orientation.x,
            ref_msg->pose.orientation.y,
            ref_msg->pose.orientation.z,
            ref_msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll,pitch,yaw;
        m.getRPY(roll,pitch,yaw);

        theta_ref = yaw;
        //thetap_ref_ = ref_msg->velocity.angular.z;

        xp_ref=ref_msg->velocity.linear.x;
		yp_ref=ref_msg->velocity.linear.y;
        double thetap_ref = ref_msg->velocity.angular.z;

        x_ref_ = x_ref+e*cos(theta_ref);
        y_ref_ = y_ref+e*sin(theta_ref);
        theta_ref_ = theta_ref;

        xp_ref_ = xp_ref-e*sin(theta_ref_)*thetap_ref;
        yp_ref_ = yp_ref+e*cos(theta_ref_)*thetap_ref;
        
       // RCLCPP_INFO(this->get_logger(),"Refe x: %.5f, y: %.5f, theta: %.5f",x_ref_,y_ref_,theta_ref_);

        //RCLCPP_INFO(this->get_logger(), "Callback Reference thread: %ld", std::hash<std::thread::id>{}(std::this_thread::get_id()));
    }


    rclcpp_action::Server<tb3_actions::action::ControllerCommands>::SharedPtr control_server_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_0_sub_;

    rclcpp::Subscription<controller_pkg::msg::Reference>::SharedPtr reference_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr tb30_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tb30_data_;

    //rclcpp::CallbackGroup::SharedPtr callback_group_;


    bool control_status_rob0_ = false;
    bool cancel_goal_ = false;
    bool reset_flag_ = false;
    bool init_odom = false;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    
    //--------------- TB3_0 -------------------------------- 
    double x_rob_0,y_rob_0,theta_rob_0;
    double time_amcl,x_amcl,y_amcl,theta_amcl;
    double time_odom_0,x_odom_0,y_odom_0,theta_odom_0,xp_odom_0,yp_odom_0;
    double x0_odom_0 = 0.0;
    double y0_odom_0 = 0.0;
    double theta0_odom_0 = 0.0;
    double x_last_odom, y_last_odom,theta_last_odom;
    double x_last_tf,y_last_tf,theta_last_tf;

    //---------------- REF ---------------------------------
    double x_ref_, y_ref_,theta_ref_,xp_ref_,yp_ref_,thetap_ref_,t_;
    double x_ref, y_ref,theta_ref,xp_ref,yp_ref;

    //------------- Controller -----------------------------
    double b = (0.16/2);
    double e = 0.1;
    double k = 0.4;
    double v_max = 0.25;
    double w_max = 1.8;
    double Kd;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

};


int main(int argc, char* argv[]){

    rclcpp::init(argc,argv);

    ControllerLeaderServer::SharedPtr node;
    node = std::make_shared<ControllerLeaderServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}