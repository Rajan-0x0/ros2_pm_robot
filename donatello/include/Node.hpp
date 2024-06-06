#ifndef DONATELLO_NODE_HPP
#define DONATELLO_NODE_HPP

#if defined(SKILLSET_DEBUG_MODE)
#include "turtle_skillset/NodeDebug.hpp"
#define SKILLSET_NODE turtle_skillset::TurtleNodeDebug
#else
#include "turtle_skillset/Node.hpp"
#define SKILLSET_NODE turtle_skillset::TurtleNode
#endif

using namespace std::chrono_literals;

#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/teleport_relative.hpp"

#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class DonatelloNode : public SKILLSET_NODE {

	public:
	    DonatelloNode() : SKILLSET_NODE("donatello_node", "best turtle ever") {
	    
		// -------------------- turtlesim Topics --------------------

		turtlesim_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
		    "/donatello/pose", 10, [this](turtlesim::msg::Pose::UniquePtr msg)
		    { this->pose_callback_(std::move(msg)); });

		turtlesim_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
		    "/donatello/cmd_vel", 10, [this](geometry_msgs::msg::Twist::UniquePtr msg)
		    { this->cmd_vel_callback_(std::move(msg)); });

		// ------------------ turtlesim teleport_relative ------------------

		teleport_relative_ = this->create_client<turtlesim::srv::TeleportRelative>("/donatello/teleport_relative");

		// -------------------- Skill MoveForward --------------------

		move_forward_distance_ = 0.0;
		move_forward_timer_ = this->create_wall_timer(1s, [this]()
		                                              { this->skill_move_forward_callback_(); });
		move_forward_timer_->cancel();

		// -------------------- Skill RotateAngle --------------------
		
		rotate_angle_ = 0.0;
		rotate_angle_timer_ = this->create_wall_timer(1s, [this]()
		                                              { this->skill_rotate_angle_callback_(); });
		rotate_angle_timer_->cancel();
		
		
		
		
		
		
		// -------------------- Skill MoveIncircle --------------------  
		    circle_remaining_distance_ = 0.0;
		    move_in_circle_timer_ = this->create_wall_timer(1s, [this]()
				                                   { this->skill_move_in_circle_callback_(); });
		    move_in_circle_timer_->cancel();       
		
	    }
	    
	    
	    

	    //-------------------- Event Hook --------------------
	    // void event_authority_to_skill_hook();
	    // void event_authority_to_teleop_hook();

	    //-------------------- Skill Hook --------------------
	    // bool skill_move_forward_validate_hook();
	    // void skill_move_forward_start_hook();
	    void skill_move_forward_on_start();
	    void skill_move_forward_invariant_has_authority_hook();
	    void skill_move_forward_interrupt_hook();    

	    // bool skill_rotate_angle_validate_hook();
	    // void skill_rotate_angle_start_hook();
	    void skill_rotate_angle_on_start();
	    void skill_rotate_angle_invariant_has_authority_hook();
	    void skill_rotate_angle_interrupt_hook();    
	    
	    
		

	    // bool skill_move_in_circle_validate_hook();
	    // void skill_move_in_circle_start_hook();
	    void skill_move_in_circle_on_start();
	    void skill_move_in_circle_invariant_has_authority_hook();
	    void skill_move_in_circle_interrupt_hook();   
	     

	private:
	    void pose_callback_(turtlesim::msg::Pose::UniquePtr msg);
	    void cmd_vel_callback_(geometry_msgs::msg::Twist::UniquePtr msg);
	    void teleport_(double linera, double angular);
	    void teleport_callback_(rclcpp::Client<turtlesim::srv::TeleportRelative>::SharedFutureWithRequest future);

	    void skill_move_forward_callback_();
	    void skill_rotate_angle_callback_();
	    
	    
	    
	    void skill_move_in_circle_callback_();


	private:
	    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtlesim_pose_sub_;
	    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr turtlesim_cmd_vel_sub_;
	    rclcpp::Client<turtlesim::srv::TeleportRelative>::SharedPtr teleport_relative_;

	    double move_forward_distance_;
	    rclcpp::TimerBase::SharedPtr move_forward_timer_; 

	    double rotate_angle_;
	    rclcpp::TimerBase::SharedPtr rotate_angle_timer_;
	    
	    
	    
	    
	    double move_in_circle_distance_;
	    rclcpp::TimerBase::SharedPtr move_in_circle_timer_;
	    
	    double circle_remaining_distance_;
	    rclcpp::TimerBase::SharedPtr circle_remaining_timer_;           
};
#endif

