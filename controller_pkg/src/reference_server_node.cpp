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

// tf2 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "tb3_actions/action/reference_commands.hpp"

// reference msgs & marker visualization
#include "controller_pkg/msg/reference.hpp"
#include "visualization_msgs/msg/marker.hpp"

//data msgs
#include "std_msgs/msg/float64_multi_array.hpp"

//NAMESPACES
std::string namespace_0 = "tb3_0";
std::string namespace_1 = "tb3_1";


class ReferenceServer : public rclcpp::Node{

    public: ReferenceServer() : Node("reference_server_node")
    {

        //----------------------- Publishers ---------------------
        
        //tb30_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
        
        ref_pub = this->create_publisher<controller_pkg::msg::Reference>("reference", 10);
        marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("reference_marker",10);

        data_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("data_ref",10);
            
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

        //security_th_ = std::thread(&ReferenceServer::security_control,this);
        //security_th_.detach();

        // ---------- Action callbacks --------------------

        reference_server_ = rclcpp_action::create_server<tb3_actions::action::ReferenceCommands>(
            this,"reference_command",
            std::bind(&ReferenceServer::handle_goal_callback,this,std::placeholders::_1,std::placeholders::_2),
            std::bind(&ReferenceServer::handle_cancel_callback,this,std::placeholders::_1),
            std::bind(&ReferenceServer::handle_accepted_callback,this,std::placeholders::_1));
        
            RCLCPP_INFO(this->get_logger(),"Actions initilized");

    }

    private:

    rclcpp_action::GoalResponse handle_goal_callback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const tb3_actions::action::ReferenceCommands::Goal> goal){
        
        (void)uuid;
        RCLCPP_INFO(this->get_logger(),"Goal status received: %s",goal->ref_status_rq.c_str());

        if(goal->ref_status_rq=="start_reference"){
            RCLCPP_INFO(this->get_logger(),"Goal Accepted");
            RCLCPP_INFO(this->get_logger(),"Prepare to start reference trajectory");
           return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;       
        }
        else if(goal->ref_status_rq=="stop_reference"){
            RCLCPP_INFO(this->get_logger(),"Goal Accepted");
            RCLCPP_INFO(this->get_logger(),"Prepare to stop reference");
           return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;       
        }
        else if(goal->ref_status_rq=="reset_reference"){
            RCLCPP_INFO(this->get_logger(),"Goal Accepted");
            RCLCPP_INFO(this->get_logger(),"Prepare to stop reference");
           return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;       
        }
        else
        RCLCPP_INFO(this->get_logger(),"Does not a correct goal -> rejected");
        return rclcpp_action::GoalResponse::REJECT;
    }        

    rclcpp_action::CancelResponse handle_cancel_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tb3_actions::action::ReferenceCommands>> goal_handle)
    {
        ref_status_=false;        
        cancel_goal_=true;

        if(goal_handle->is_executing())
            RCLCPP_INFO(this->get_logger(), "Goal still executing");
        

        RCLCPP_INFO(this->get_logger(), "Cancel request received");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tb3_actions::action::ReferenceCommands>> goal_handle){

        RCLCPP_INFO(this->get_logger(),"Goal accepted: EXECUTING");
        const auto goal = goal_handle->get_goal();
        RCLCPP_INFO(this->get_logger(),"Goal receive: %s", goal->ref_status_rq.c_str());

        if(goal->ref_status_rq == "start_reference"){
            std::thread{std::bind(&ReferenceServer::execute_reference,this,std::placeholders::_1),goal_handle}.detach();
        }
        else if(goal->ref_status_rq == "stop_reference"){
            std::thread{std::bind(&ReferenceServer::stop_reference,this,std::placeholders::_1),goal_handle}.detach();
        }
        else if(goal->ref_status_rq == "reset_reference"){
            std::thread{std::bind(&ReferenceServer::reset_reference,this,std::placeholders::_1),goal_handle}.detach();
        }
        else
            RCLCPP_INFO(this->get_logger(),"Goal error");    
    }

    void execute_reference(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tb3_actions::action::ReferenceCommands>> goal_handle){

        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<tb3_actions::action::ReferenceCommands::Result>();
        auto feedback = std::make_shared<tb3_actions::action::ReferenceCommands::Feedback>();

        init_time = this->get_clock()->now();
        ref_status_ = true;
        restarting_time_ = goal->time;
        RCLCPP_INFO(this->get_logger(),"Restarting time: %.5f",restarting_time_);

        RCLCPP_INFO(this->get_logger(),"Goal receive in execute start_reference function: %s", goal->ref_status_rq.c_str());
        if(goal->ref_status_rq=="start_reference")
        {
            rclcpp::Rate rate(10);
            while(ref_status_)
            {
                RCLCPP_INFO_ONCE(this->get_logger(),"Executing Reference");
                
                rclcpp::Time time = this->get_clock()->now();
                rclcpp::Duration diff = time - init_time;

                t = diff.seconds()+diff.nanoseconds()*1e-9 + restarting_time_; 

                //TODO: Implement the reference to compute x, y and theta and their velocities
                /*
                double A = 0.5;
                double w = 2 * M_PI/150;
                

                double x0 = -0.5;
                double y0 = 0.0;
                */
                // Angulo de compensación de la trayectoria de referencia
                // el vector apuntará al eje x
                 

                double Ax = 2.0;
                double Ay = 1.0;
                double w = 2 * M_PI/300;
                

                double x0 = 0.0;
                double y0 = 0.0;
                double theta_comp = -atan2(2*Ay,Ax);

                // Definimos la posición trayectoria
                 x = x0 + Ax*sin(w*t);
                 y = y0 + Ay*sin(2*w*t);

                // Calculamos la velocidad
                xp = Ax * cos(w*t)*w;
                yp = Ay * 2* cos(2*w*t)*w;

                double xpp = -Ax * sin(w*t)*w*w;
                double ypp = -Ay * 4 *sin(w*t)*w*w;

                // Reorientamos con el ángulo theta_comp
                double c = cos(theta_comp);
                double s = sin(theta_comp);
                x = c*x-s*y + x0;
                y = s*x+c*y + y0;
                xp = c*xp-s*yp;
                yp = s*xp+c*yp;
                xpp = c*xpp-s*ypp;
                ypp = s*xpp+c*ypp;

                theta = atan2(yp,xp);
                thetap = (ypp * xp - xpp * yp) / (pow(xp,2) + pow(yp,2));
                feedback->time = t;
                goal_handle->publish_feedback(feedback);

                std_msgs::msg::Float64MultiArray data_msg;
                data_msg.data = {x,y,theta,xp,yp,thetap,t};
                data_ -> publish(data_msg);
                
                //Pto descentrado de la referencia
                //x = x + e*cos(theta);
                //y = y + e*sin(theta); 
                
                /*
                x = x0 + A*sin(w*t);
                y = y0 + A*cos(w*t);
                xp = A*cos(w*t)*w;
                yp = A*-sin(w*t)*w;
                double xpp = A*-sin(w*t)*w*w;
                double ypp = A*-cos(w*t)*w*w;
                theta = atan2(yp,xp);
                thetap = (ypp * xp - xpp * yp) / (pow(xp,2) + pow(yp,2));
                 */

                //Publicamos la referencia
                publish_reference(x,y,theta,xp,yp,thetap);
                
                //Publicamos la transformacion
                reference_marker(x,y,theta);   
                rate.sleep();   
            }

            if(cancel_goal_)
            {
            RCLCPP_INFO(this->get_logger(),"Stop Reference. REASON: Goal canceled");
            result->ref_status_rs="Stop Reference. REASON: Goal canceled";
            goal_handle->succeed(result);
            }
            else
            {
            RCLCPP_INFO(this->get_logger(),"Reference goal succeed");
            result->ref_status_rs="Reference goal succeed";
            goal_handle->succeed(result);
            }
        }
    }

    void stop_reference(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tb3_actions::action::ReferenceCommands>> goal_handle){

        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<tb3_actions::action::ReferenceCommands::Result>();
        auto feedback = std::make_shared<tb3_actions::action::ReferenceCommands::Feedback>();

        RCLCPP_INFO(this->get_logger(),"Goal receive in stop control function: %s", goal->ref_status_rq.c_str());
        if(goal->ref_status_rq=="stop_reference")
        {
            feedback->time = t;
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(this->get_logger(),"Stopping");
            ref_status_ = false;

            result->ref_status_rs="Stop Reference succeed";
            goal_handle->succeed(result);
        }
    }

    void reset_reference(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tb3_actions::action::ReferenceCommands>> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<tb3_actions::action::ReferenceCommands::Result>();

        RCLCPP_INFO(this->get_logger(),"Reseting odometry");

        

        result->ref_status_rs="Odometry reseted succeed";
        goal_handle->succeed(result);
    }

    void publish_reference(double x, double y, double theta, double xp, double yp, double w)
    {
        tf2::Quaternion q;
        double roll=0.0;
        double pitch=0.0;
        q.setRPY(roll,pitch,theta);

        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = x;
        pose_msg.position.y = y;
        pose_msg.orientation.x = q.x();
        pose_msg.orientation.y = q.y();
        pose_msg.orientation.z = q.z();
        pose_msg.orientation.w = q.w();
        
        geometry_msgs::msg::Twist vel_msg;
        vel_msg.linear.x = xp;
        vel_msg.linear.y = yp;
        vel_msg.angular.z = w;

        controller_pkg::msg::Reference ref_msg;
        ref_msg.pose = pose_msg;
        ref_msg.velocity = vel_msg;

        //Publicamos
        ref_pub->publish(ref_msg);
    }

    void reference_marker(double x, double y, double theta)
    {
        geometry_msgs::msg::Point start_tf, end_tf;
        start_tf.x = x;
        start_tf.y = y;
        start_tf.z = 0.0;
        
        end_tf.x = x+0.25*cos(theta);
        end_tf.y = y+0.25*sin(theta);
        end_tf.z = 0;       

        visualization_msgs::msg::Marker marker_msg;
        marker_msg.header.stamp = this->get_clock()->now();  
        marker_msg.header.frame_id = "map";
        marker_msg.points.push_back(start_tf);
        marker_msg.points.push_back(end_tf); 
        marker_msg.ns = "marker_arrow";
        marker_msg.id = 0;
        marker_msg.type = visualization_msgs::msg::Marker::ARROW;
        marker_msg.action = visualization_msgs::msg::Marker::ADD;
        marker_msg.scale.x = 0.05;
        marker_msg.scale.y = 0.1;
        marker_msg.scale.z = 0.1;
        marker_msg.color.a = 1.0;
        marker_msg.color.r = 0.0;
        marker_msg.color.g = 1.0;
        marker_msg.color.b = 0.0;        

        marker_pub->publish(marker_msg);

    }

    //------------------------- Security function -----------------------------------------------------

    /*
    void security_control()
    {
        while(rclcpp::ok()){

            if(altitude_>1.2 && altitude_<=1.5){
                RCLCPP_INFO(this->get_logger(),"Be careful, MAX altitude exceeded: %.5f",altitude_);
            }
            else if(altitude_< 0.1 && !is_taking_off_ && !is_landing_){
                RCLCPP_INFO(this->get_logger(),"Be careful, MIN altitude exceeded: %.5f",altitude_);
            }
            else if(altitude_>1.5){
                RCLCPP_INFO(this->get_logger(),"CANCEL GOALS: MAX altitude dangerous: %.5f",altitude_);
                danger_flag_=true;
                sendCmd((char*)"CONTROL",0,0,0,0);
                sendCmd((char*)"LANDING",0,0,0,0);
            }
            else
                danger_flag_=false;

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
     
    
    void drone_cmd_vel_callback(const std::shared_ptr<geometry_msgs::msg::Twist> drone_cmd_vel_msg){

        if (drone_commands_callback_flag_ && !danger_flag_){
            
            sendCmd((char*)"CONTROL", drone_cmd_vel_msg->linear.z, drone_cmd_vel_msg->linear.x,
                                    drone_cmd_vel_msg->linear.y, drone_cmd_vel_msg->linear.z); // Throttle, Yaw, Pitch, Roll
                                    
            RCLCPP_INFO(this->get_logger(),"Drone_cmd_vel_callback: Throttle: %.5f, Yaw: %.5f, Pitch: %.5f, Roll: %.5f",drone_cmd_vel_msg->linear.z,drone_cmd_vel_msg->linear.x,drone_cmd_vel_msg->linear.y,drone_cmd_vel_msg->angular.z);
        }
    }
    */ 

    rclcpp_action::Server<tb3_actions::action::ReferenceCommands>::SharedPtr reference_server_;

    //std::thread security_th_;
    rclcpp::Time init_time;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<controller_pkg::msg::Reference>::SharedPtr ref_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr data_;

    bool ref_status_ = false;
    bool cancel_goal_= false;

    double x=0.0;
    double y=0.0;
    double theta=0.0;
    double xp=0.0;
    double yp=0.0;
    double thetap=0.0;
    double t;
    double restarting_time_ = 0.0;
};


int main(int argc, char* argv[]){

    rclcpp::init(argc,argv);

    ReferenceServer::SharedPtr node;
    node = std::make_shared<ReferenceServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}