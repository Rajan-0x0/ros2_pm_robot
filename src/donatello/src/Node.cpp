#include "Node.hpp"

// ---------------------------------------- Turtlesime Topics ----------------------------------------

	void DonatelloNode::pose_callback_(turtlesim::msg::Pose::UniquePtr msg) {
	    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "turtlesim_pose_callback");

	    geometry_msgs::msg::Pose2D data;
	    data.x = msg->x;
	    data.y = msg->y;
	    data.theta = msg->theta;

	    geometry_msgs::msg::Pose2D old = this->get_data_pose().value;
	    if (old.x != data.x || old.y != data.y || old.theta != data.theta)
	    {
		this->set_data_pose(data);
	    }
	}

	void DonatelloNode::cmd_vel_callback_(geometry_msgs::msg::Twist::UniquePtr msg) {
	    (void)msg;
	    this->event_authority_to_teleop();
	}

// ---------------------------------------- Turtlesim Teleport ----------------------------------------

	void DonatelloNode::teleport_(double linear, double angular) {
	    auto request = std::make_shared<turtlesim::srv::TeleportRelative::Request>();
	    request->linear = linear;
	    request->angular = angular;

	    while (!this->teleport_relative_->wait_for_service(1s))
	    {
		if (!rclcpp::ok())
		{
		    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
		    return;
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
	    }

	    auto result = this->teleport_relative_->async_send_request(request, [this]	(rclcpp::Client<turtlesim::srv::TeleportRelative>::SharedFutureWithRequest future)
		                                                       { this->teleport_callback_(future); });
	}
	

	void DonatelloNode::teleport_callback_(rclcpp::Client<turtlesim::srv::TeleportRelative>::SharedFutureWithRequest future) {
	    (void)future;
	}

// ---------------------------------------- Skill MoveForward ----------------------------------------

	void DonatelloNode::skill_move_forward_on_start() {
	    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "skill_move_forward_on_start");

	    auto input = this->skill_move_forward_input();
	    this->move_forward_distance_ = input->distance;

	    this->move_forward_timer_->reset();
	}
	

	void DonatelloNode::skill_move_forward_callback_() {
	    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "skill_move_forward_timer");
	    auto input = this->skill_move_forward_input();

	    this->teleport_(input->speed, 0.0);
	    move_forward_distance_ -= input->speed;
	    if (move_forward_distance_ <= 0.0)
	    {
		move_forward_timer_->cancel();
		this->skill_move_forward_success_completed();
	    }
	}
	

	void DonatelloNode::skill_move_forward_invariant_has_authority_hook() {
	    move_forward_timer_->cancel();
	}
	

	void DonatelloNode::skill_move_forward_interrupt_hook() {
	    move_forward_timer_->cancel();
	}
	

// ---------------------------------------- Skill RotateAngle ----------------------------------------

	void DonatelloNode::skill_rotate_angle_on_start() {
	    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "skill_rotate_angle_on_start");

	    auto input = this->skill_rotate_angle_input();
	    this->rotate_angle_ = input->angle;

	    this->rotate_angle_timer_->reset();
	}
	
	

	void DonatelloNode::skill_rotate_angle_callback_() {
	    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "skill_rotate_angle_timer");
	    auto input = this->skill_rotate_angle_input();

	    if (input->angle >= 0)
	    {
		this->teleport_(0.0, input->speed);
		rotate_angle_ -= input->speed;
		if (rotate_angle_ <= 0.0)
		{
		    rotate_angle_timer_->cancel();
		    this->skill_rotate_angle_success_completed();
		}
	    }
	    else
	    {
		this->teleport_(0.0, -input->speed);
		rotate_angle_ += input->speed;
		if (rotate_angle_ >= 0.0)
		{
		    rotate_angle_timer_->cancel();
		    this->skill_rotate_angle_success_completed();
		}
	    }
	}
	
	

	void DonatelloNode::skill_rotate_angle_invariant_has_authority_hook() {
	    rotate_angle_timer_->cancel();
	}
	
	

	void DonatelloNode::skill_rotate_angle_interrupt_hook() {
	    rotate_angle_timer_->cancel();
	}








// ---------------------------------------- Skill MoveInCircle ----------------------------------------

	void DonatelloNode::skill_move_in_circle_on_start() {
	    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "skill_move_in_circle_on_start");

	    auto input = this->skill_move_in_circle_input();
	    double circumference = 2 * M_PI * input->radius;

	    circle_remaining_distance_ = circumference;

	    this->move_in_circle_timer_->reset();
	}
	
	
	void DonatelloNode::skill_move_in_circle_callback_() {
	    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "skill_move_in_circle_timer");
	    auto input = this->skill_move_in_circle_input();

	    double radius = input->radius;
	    double angular_speed = input->speed;
	    double linear_speed = angular_speed * radius; //v = ω*r Lineare Geschwindigkeit, Winkelgeschwindigkeit

	    this->teleport_(linear_speed, angular_speed);

	    this->circle_remaining_distance_ -= linear_speed;
	    if (circle_remaining_distance_ <= 0.0)
	    {
		move_in_circle_timer_->cancel();
		this->skill_move_in_circle_success_completed();
	    }
	}



	void DonatelloNode::skill_move_in_circle_invariant_has_authority_hook() {
	    move_in_circle_timer_->cancel();
	}


	void DonatelloNode::skill_move_in_circle_interrupt_hook() {
	    move_in_circle_timer_->cancel();
	}

