#include <functional>
#include <chrono>
#include <string>
#include <math.h>
#include <memory>
#include <thread>

// ros client library
#include <rclcpp/rclcpp.hpp>

// tf2 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// lifecycle
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// actions
#include "rclcpp_action/rclcpp_action.hpp"
#include "tb3_actions/action/controller_commands.hpp"
#include "tb3_actions/action/reference_commands.hpp"


using namespace std::placeholders;

class ControllerClient : public rclcpp_lifecycle::LifecycleNode{

    public: ControllerClient() : LifecycleNode("controller_lc_client_node"){

        RCLCPP_INFO(get_logger(),"Uncofigured state");
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State & state)
    {
        // Primer estado del nodo. Se crean las acciones

        RCLCPP_INFO(get_logger(),"On_configure()");
        RCLCPP_INFO(this->get_logger(),"Last state id: %d, label: %s,",state.id(),state.label().c_str());

       //-------- Controller Action Client -------------------------
        
        tb30_control_client_ = rclcpp_action::create_client<tb3_actions::action::ControllerCommands>(this,"controller_tb30_command");
        tb31_control_client_ = rclcpp_action::create_client<tb3_actions::action::ControllerCommands>(this,"controller_tb31_command");

        //-------- To recive controller action callbacks ------------
        send_goal_control_options_.goal_response_callback = std::bind(&ControllerClient::goal_response_control_callback,this,_1);
        send_goal_control_options_.feedback_callback = std::bind(&ControllerClient::feedback_control_callback,this,_1,_2);
        send_goal_control_options_.result_callback = std::bind(&ControllerClient::result_control_callback,this,_1);


        //-------- Reference Action Client -------------------------
        
        tb3_reference_client_ = rclcpp_action::create_client<tb3_actions::action::ReferenceCommands>(this,"reference_command");

        //-------- To recive reference action callbacks ------------
        send_goal_reference_options_.goal_response_callback = std::bind(&ControllerClient::goal_response_reference_callback,this,_1);
        send_goal_reference_options_.feedback_callback = std::bind(&ControllerClient::feedback_reference_callback,this,_1,_2);
        send_goal_reference_options_.result_callback = std::bind(&ControllerClient::result_reference_callback,this,_1);

        
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & state)
    { 
        // Segundo estado del nodo: comprobamos que los nodos de control y de la rederencia estań disponible
        // Si es así, enviámos los objetivos correspondientes

        RCLCPP_INFO(get_logger(),"On_activate()");
        RCLCPP_INFO(this->get_logger(),"Last state id: %d, label: %s,",state.id(),state.label().c_str());    

        //--------------------- Control ------------------------------

        while(!tb30_control_client_->wait_for_action_server(std::chrono::seconds(2)))
        {
            RCLCPP_WARN(this->get_logger(), "Trying to connect, waiting to control sever activation");
        }

        // Con este flag indicamos al sistema que hemos comenzado y se van a enviar los objetivos a los servidores
        deactivate_flag_ = false;

        //--------- ROB 1 -----------
        
        tb3_actions::action::ControllerCommands::Goal goal_control_rob1_msg;
        goal_control_rob1_msg.robot = "rob1";
        goal_control_rob1_msg.status_rq = "start_control";
        std::thread{std::bind(&ControllerClient::send_control_goal,this,std::placeholders::_1),goal_control_rob1_msg}.detach();
        RCLCPP_INFO(get_logger(),"Start control ROB 1");


        //--------- ROB 0 -----------

        tb3_actions::action::ControllerCommands::Goal goal_control_rob0_msg;
        goal_control_rob0_msg.robot = "rob0";
        goal_control_rob0_msg.status_rq = "start_control";
        std::thread{std::bind(&ControllerClient::send_control_goal,this,std::placeholders::_1),goal_control_rob0_msg}.detach();
        RCLCPP_INFO(get_logger(),"Start control ROB 0");

        //--------------------- Reference ------------------------------

        while(!tb3_reference_client_->wait_for_action_server(std::chrono::seconds(2)))
        {
            RCLCPP_WARN(this->get_logger(), "Trying to connect, waiting reference activation");
        }

        tb3_actions::action::ReferenceCommands::Goal goal_ref_msg;
        goal_ref_msg.ref_status_rq = "start_reference";
        goal_ref_msg.time = ref_time_;
        std::thread{std::bind(&ControllerClient::send_reference_goal,this,std::placeholders::_1),goal_ref_msg}.detach();
        RCLCPP_INFO(get_logger(),"Start reference trajectory");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & state)
    {
        // Tercer estado del sistema. Cambiamos el valor del flag activado en el estado anterior.
        // Esto produce una cancelación de todos los objetivos que había activos en el sistema. 

        RCLCPP_INFO(this->get_logger(),"Last state id: %d, label: %s,",state.id(),state.label().c_str());    

        RCLCPP_INFO(get_logger(),"on_deactivate() called");

        deactivate_flag_ = true;

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &state)
    {
        // Regresamos los robots a la posición inicial para que estén operativos para una nueva prueba.
        // Para ello se envían los objetivos correspondientes.

        RCLCPP_INFO(get_logger(),"on_cleanup() called");
        RCLCPP_INFO(this->get_logger(),"Last state id: %d, label: %s,",state.id(),state.label().c_str());   

        tb3_actions::action::ControllerCommands::Goal goal_rob0_msg;
        goal_rob0_msg.robot = "rob0";
        goal_rob0_msg.status_rq = "reset_to_initial_position";
        std::thread{std::bind(&ControllerClient::send_control_goal,this,std::placeholders::_1),goal_rob0_msg}.detach();

        tb3_actions::action::ControllerCommands::Goal goal_rob1_msg;
        goal_rob1_msg.robot = "rob1";
        goal_rob1_msg.status_rq = "reset_to_initial_position";
        std::thread{std::bind(&ControllerClient::send_control_goal,this,std::placeholders::_1),goal_rob1_msg}.detach();

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &state)
    {
        // Finalizamos el nodo.

        RCLCPP_INFO(get_logger(),"on_shutdown() called");
        RCLCPP_INFO(this->get_logger(),"Last state id: %d, label: %s,",state.id(),state.label().c_str());    

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    //------------------------------------------------------------------------------------------------------------
    //------------------------- Handle CONTROL Server Response --------------------------------------------------
    //------------------------------------------------------------------------------------------------------------

    void goal_response_control_callback(rclcpp_action::ClientGoalHandle<tb3_actions::action::ControllerCommands>::SharedPtr goal_handle)
    {

        auto goal_status = goal_handle->get_status();

        std::string status_str;
        switch (goal_status) 
        {
            case rclcpp_action::GoalStatus::STATUS_ACCEPTED:
                status_str = "ACCEPTED";
                break;
            case rclcpp_action::GoalStatus::STATUS_EXECUTING:
                status_str = "EXECUTING";
                break;
            case rclcpp_action::GoalStatus::STATUS_CANCELING:
                status_str = "CANCELING";
                break;
            case rclcpp_action::GoalStatus::STATUS_SUCCEEDED:
                status_str = "SUCCEEDED";
                break;
            case rclcpp_action::GoalStatus::STATUS_CANCELED:
                status_str = "CANCELED";
                break;
            case rclcpp_action::GoalStatus::STATUS_ABORTED:
                status_str = "ABORTED";
                break;
            case rclcpp_action::GoalStatus::STATUS_UNKNOWN:
            default:
                status_str = "UNKNOWN";
                break;
        }

        RCLCPP_INFO(this->get_logger(), "Goal status: %s", status_str.c_str());

        if(!goal_handle)
            RCLCPP_INFO(this->get_logger(),"Goal rejected by server");
        else
            RCLCPP_INFO(this->get_logger(),"Goal accepted by server, waiting for result");
    }

    void feedback_control_callback(rclcpp_action::ClientGoalHandle<tb3_actions::action::ControllerCommands>::SharedPtr goal_handle, const std::shared_ptr<const tb3_actions::action::ControllerCommands::Feedback> feedback)
    {        
        if(feedback->robot == "rob0")
        {
            int rob_0 = 0;
            RCLCPP_INFO_ONCE(this->get_logger(),"Recieving feedback from ROBOT LEADER");
            x_rob_leader_= feedback->x_pos;
            y_rob_leader_= feedback->y_pos;
            theta_rob_leader_= feedback->theta;

            if((aruco_stop_flag_ || deactivate_flag_) && goal_active_rob0_)
            {
                RCLCPP_INFO(this->get_logger(),"Cancelling ROB_0 controlller in pose:");
                RCLCPP_INFO(this->get_logger(),"x_rob0: %.5f",x_rob_leader_);
                RCLCPP_INFO(this->get_logger(),"y_rob0: %.5f",y_rob_leader_);
                RCLCPP_INFO(this->get_logger(),"theta_rob0: %.5f",theta_rob_leader_);
                cancel_control_goal_th_ = std::thread(&ControllerClient::cancel_control_goal,this,goal_handle,rob_0);
                cancel_control_goal_th_.detach();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        else if(feedback->robot == "rob1")
        {
            int rob_1 = 1;
            RCLCPP_INFO_ONCE(this->get_logger(),"Recieving feedback from ROBOT FOLLOWER");
            if(deactivate_flag_ && goal_active_rob1_)
            {
                RCLCPP_INFO(this->get_logger(),"Cancelling ROB_1 controlller");
                cancel_control_goal_th_ = std::thread(&ControllerClient::cancel_control_goal,this,goal_handle,rob_1);
                cancel_control_goal_th_.detach();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }        
    }

    void result_control_callback(const rclcpp_action::ClientGoalHandle<tb3_actions::action::ControllerCommands>::WrappedResult & result)
    {
        if(result.result->robot == "rob1")
        {
            if(result.result->status_rs == "aruco_lost")
            {
                goal_active_rob1_ = false;
                aruco_stop_flag_ = true;

                RCLCPP_INFO(this->get_logger(),"Not found aruco marker");
                RCLCPP_INFO(get_logger(),"Starting recovery mode");

                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                tb3_actions::action::ControllerCommands::Goal goal_control_rob1_msg;
                goal_control_rob1_msg.robot = "rob1";
                goal_control_rob1_msg.status_rq = "recovery";
                goal_control_rob1_msg.x_pos = x_rob_leader_;
                goal_control_rob1_msg.y_pos = y_rob_leader_;
                goal_control_rob1_msg.theta = theta_rob_leader_;
                std::thread{std::bind(&ControllerClient::send_control_goal,this,std::placeholders::_1),goal_control_rob1_msg}.detach();
                
                RCLCPP_INFO(get_logger(),"Sending RECOVERY goal to ROB1");
            }
            
            else if(result.result->status_rs == "recovery_succeed")
            {
                goal_active_rob1_ = false;
                aruco_stop_flag_ = false;

                while(goal_active_ref_ || goal_active_rob0_ || goal_active_rob1_)
                {
                    RCLCPP_WARN(this->get_logger(),"TRYING TO SEND A NEW GOAL WITH ACTIVE GOALS YET");
                }

                RCLCPP_INFO(this->get_logger(),"Aruco marker found: restarting leader-follower system");

                tb3_actions::action::ControllerCommands::Goal goal_control_rob1_msg;
                goal_control_rob1_msg.robot = "rob1";
                goal_control_rob1_msg.status_rq = "start_control";
                std::thread{std::bind(&ControllerClient::send_control_goal,this,std::placeholders::_1),goal_control_rob1_msg}.detach();

                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                tb3_actions::action::ControllerCommands::Goal goal_control_rob0_msg;
                goal_control_rob0_msg.robot = "rob0";
                goal_control_rob0_msg.status_rq = "start_control";
                std::thread{std::bind(&ControllerClient::send_control_goal,this,std::placeholders::_1),goal_control_rob0_msg}.detach();
                
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                tb3_actions::action::ReferenceCommands::Goal goal_ref_msg;
                goal_ref_msg.ref_status_rq = "start_reference";
                goal_ref_msg.time = ref_time_;
                std::thread{std::bind(&ControllerClient::send_reference_goal,this,std::placeholders::_1),goal_ref_msg}.detach();
            }

            std::string result_recive = result.result->status_rs;
            RCLCPP_INFO(this->get_logger(),"Result received from ROB 1: %s",result_recive.c_str());
        }

        else if(result.result->robot == "rob0" && goal_active_rob0_)
        {
            std::string result_recive = result.result->status_rs;
            RCLCPP_INFO(this->get_logger(),"Result received from ROB 0: %s",result_recive.c_str());
            goal_active_rob0_=false;
        }
        else
        {
            std::string result_recive = result.result->status_rs;
            std::string robot_recive = result.result->robot;
            RCLCPP_INFO(this->get_logger(),"Result received from %s: %s",result.result->robot.c_str(),result_recive.c_str());
        }
    }

    // ----------------------- Send & Cancel ROBOTS Goals --------------------------------------------------------

    void send_control_goal(tb3_actions::action::ControllerCommands::Goal goal_msg)
    {                
        if(goal_msg.robot == "rob0")
        {
            goal_active_rob0_=true;
            RCLCPP_INFO(this->get_logger(),"Sending Goal: %s ,to robot %s",goal_msg.status_rq.c_str(),goal_msg.robot.c_str());
            tb30_control_client_->async_send_goal(goal_msg,send_goal_control_options_);

        }
        else if (goal_msg.robot == "rob1")
        {
            goal_active_rob1_=true;
            RCLCPP_INFO(this->get_logger(),"Sending Goal: %s ,to robot %s",goal_msg.status_rq.c_str(),goal_msg.robot.c_str());
            tb31_control_client_->async_send_goal(goal_msg,send_goal_control_options_);
        }
    }

    void cancel_control_goal(rclcpp_action::ClientGoalHandle<tb3_actions::action::ControllerCommands>::SharedPtr goal_handle, int rob)
    {
        if(goal_active_rob0_ && rob == 0)
        {
            RCLCPP_WARN(this->get_logger(),"Trying to cancel the goal from Robot LEADER");
            tb30_control_client_->async_cancel_goal(goal_handle);
            goal_active_rob0_=false;
        }
        else if(goal_active_rob1_ && rob == 1)
        {
            RCLCPP_WARN(this->get_logger(),"Trying to cancel the goal from Robot FOLLOWER");
            tb31_control_client_->async_cancel_goal(goal_handle);
            goal_active_rob1_=false;
        }
    }

    //------------------------------------------------------------------------------------------------------------
    //------------------------- Handle RFERENCE Server Response --------------------------------------------------
    //------------------------------------------------------------------------------------------------------------

    void goal_response_reference_callback(rclcpp_action::ClientGoalHandle<tb3_actions::action::ReferenceCommands>::SharedPtr goal_handle)
    {
        auto goal_status = goal_handle->get_status();

        std::string status_str;
        switch (goal_status) 
        {
            case rclcpp_action::GoalStatus::STATUS_ACCEPTED:
                status_str = "ACCEPTED";
                break;
            case rclcpp_action::GoalStatus::STATUS_EXECUTING:
                status_str = "EXECUTING";
                break;
            case rclcpp_action::GoalStatus::STATUS_CANCELING:
                status_str = "CANCELING";
                break;
            case rclcpp_action::GoalStatus::STATUS_SUCCEEDED:
                status_str = "SUCCEEDED";
                break;
            case rclcpp_action::GoalStatus::STATUS_CANCELED:
                status_str = "CANCELED";
                break;
            case rclcpp_action::GoalStatus::STATUS_ABORTED:
                status_str = "ABORTED";
                break;
            case rclcpp_action::GoalStatus::STATUS_UNKNOWN:
            default:
                status_str = "UNKNOWN";
                break;
        }

        RCLCPP_INFO(this->get_logger(), "Goal status: %s", status_str.c_str());

        if(!goal_handle)
            RCLCPP_INFO(this->get_logger(),"Goal rejected by reference server");
        else
            RCLCPP_INFO(this->get_logger(),"Goal accepted by reference server, waiting for result");
    }

    void feedback_reference_callback(rclcpp_action::ClientGoalHandle<tb3_actions::action::ReferenceCommands>::SharedPtr goal_handle, const std::shared_ptr<const tb3_actions::action::ReferenceCommands::Feedback> feedback)
    {
        RCLCPP_INFO_ONCE(this->get_logger(),"Recieving feedback from Reference");
        ref_time_= feedback->time;
        
        if((aruco_stop_flag_ || deactivate_flag_) && goal_active_ref_)
        {   
            RCLCPP_INFO(this->get_logger(),"Cancelling Reference at time: %.5f",ref_time_);
            cancel_ref_goal_th_ = std::thread(&ControllerClient::cancel_reference_goal,this,goal_handle);
            cancel_ref_goal_th_.detach();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }       
    }

    void result_reference_callback(const rclcpp_action::ClientGoalHandle<tb3_actions::action::ReferenceCommands>::WrappedResult & result)
    {
        std::string result_recive = result.result->ref_status_rs;
        RCLCPP_INFO(this->get_logger(),"Result received from reference: %s",result_recive.c_str());
        goal_active_ref_=false;
    }

    // ----------------------- Send & Cancel REFERENCE Goal --------------------------------------------------------

    void send_reference_goal(tb3_actions::action::ReferenceCommands::Goal goal_msg)
    {
    
        RCLCPP_INFO(this->get_logger(),"Sending Goal to Reference");
        tb3_reference_client_->async_send_goal(goal_msg,send_goal_reference_options_);
        goal_active_ref_=true;

        if(goal_msg.ref_status_rq == "start_reference")
        {
            stop_ref_flag_= false;
        }
    }

    void cancel_reference_goal(rclcpp_action::ClientGoalHandle<tb3_actions::action::ReferenceCommands>::SharedPtr goal_handle)
    {
        if(goal_active_ref_)
        {
            RCLCPP_WARN(this->get_logger(),"Trying to cancel the Reference goal");
            tb3_reference_client_->async_cancel_goal(goal_handle);
            goal_active_ref_=false;
        }
    }

    rclcpp_action::Client<tb3_actions::action::ControllerCommands>::SharedPtr tb30_control_client_;
    rclcpp_action::Client<tb3_actions::action::ControllerCommands>::SharedPtr tb31_control_client_;
    rclcpp_action::Client<tb3_actions::action::ControllerCommands>::SendGoalOptions send_goal_control_options_;

    rclcpp_action::Client<tb3_actions::action::ReferenceCommands>::SharedPtr tb3_reference_client_;
    rclcpp_action::Client<tb3_actions::action::ReferenceCommands>::SendGoalOptions send_goal_reference_options_;

    std::thread send_control_goal_th;
    std::thread cancel_control_goal_th_;
    std::thread send_ref_goal_th;
    std::thread cancel_ref_goal_th_;

    bool goal_active_rob0_=false;
    bool goal_active_rob1_=false;
    bool goal_active_ref_ = false;
    bool aruco_stop_flag_=false;
    bool stop_ref_flag_= false;
    bool cancel_ROB0_goal_ = false;
    bool deactivate_flag_ = false;

    double x_rob_leader_ = 0.0;
    double y_rob_leader_ = 0.0;
    double theta_rob_leader_ = 0.0;
    double ref_time_ = 0.0;

};

int main(int argc, char* argv[]){

    rclcpp::init(argc,argv);

    ControllerClient::SharedPtr node;
    node = std::make_shared<ControllerClient>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();

    return 0;
}