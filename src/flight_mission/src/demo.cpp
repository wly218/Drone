#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/waypoints.hpp"
#include <chrono>
#include <iostream>
#include <string>
#include <array>

/**
 * |-------------------------------------------------------|
 * |		无人机携带物块从起飞点起飞，识别二维码中的内容，随后	|
 * |	后飞行到障碍物处并顺时针绕障碍物飞行。接着按照二维码中两个	 |
 * |	图片类别将自身携带的物块投放到对应图片靶以及带有信号灯的特	 |
 * |	殊靶上。投放完成后穿越位置不定的圆环，穿越完成后根据二维码	 |
 * |	中降落点的左右信息，选取正确的降落点降落。				  |
 * |	【飞行高度不得低于1.2米，悬停投放不得高于0.8米】		  |
 * |-------------------------------------------------------|
 * |	1、起飞												|
 * |	2、识别二维码										 |			
 * |	3、定点识图【使用静态数组确定航点】						 |
 * |	4、投放激光靶										 |
 * |	5、定点识图【使用静态数组确定航点】						 |
 * |	6、抵达降落点										 |
 * |	7、降落												|
 * |-------------------------------------------------------|
 */

using base_interfaces_demo::msg::Waypoints;
using namespace std::chrono_literals;

enum class DroneState
{
	IDLE,	  				// 空闲状态【初始状态】
	TAKEOFF,  				// 起飞状态【飞行到指定高度】
	QR_RECOGNITION,	  		// 识别二维码【通过设定航点飞行】
	PICTURE_RECOGNITION,	// 图像识别
	DROP_BOX,				// 投放物块
	LASER_TARGET,			// 激光靶识别
	PASSING_CIRCLE,			// 钻圈
	RTL,					// 返航
	LANDING,  				// 降落状态【原地降落】
	EMERGENCY 				// 紧急状态【暂且不知道怎么设置，留到后期优化设置吧】
};

class State_Machine:public rclcpp::Node
{
public:
	State_Machine():Node("my_state_machine"),currentState(DroneState::IDLE),currentPoint(0)
	{
		count_ = 0;
		recognized_picture = 0;
		remaining_targets = 3;
		publisher_ = this->create_publisher<Waypoints>("target_position",10);
		timer_ = this->create_wall_timer(1s,std::bind(&State_Machine::update,this));
	}

	DroneState getCurrentState() const;
	void setEmergency();
	void update();

private:
	DroneState currentState;
	int16_t currentPoint;
	int16_t count_;
	int16_t remaining_targets;
	int16_t recognized_picture;
	static std::array<std::array<double, 3>, 9> waypoints;
	Waypoints p;
	rclcpp::Publisher<Waypoints>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

	void handleTakeoff();
	void handleQR();
	void handlePicture();
	void handleLaserTarget();
	void handlePassingCircle();
	void handleRtl();
	void handleDrop();
	void handleLanding();
	void handleEmergency();
};

std::array<std::array<double, 3>, 9> State_Machine::waypoints = {{
    {0.0,0.0,-1.6},  // take-off point
    {1.8,0.0,-1.6},  // QR point
    {1.8,1.6,-1.6}, // picture1 point
    {3.6,1.6,-1.6}, // picture2 point
    {6.0,1.0,-1.6}, // laser-target point
    {3.6,-1.6,-1.6},  // picture3 point
    {1.8,-1.6,-1.6},  // picture4 point
    {0.0,1.6,-1.6}, // left-landing point
    {0.0,-1.6,-1.6}   // right-landing point
	//---------------------------------------以下是实机的航点坐标
    // {0.0,0.0,-1.6},  // take-off point
    // {1.8,0.0,-1.6},  // QR point
    // {1.8,-1.6,-1.6}, // picture1 point
    // {3.6,-1.6,-1.6}, // picture2 point
    // {6.0,-1.0,-1.6}, // laser-target point
    // {3.6,1.6,-1.6},  // picture3 point
    // {1.8,1.6,-1.6},  // picture4 point
    // {0.0,-1.6,-1.6}, // left-landing point
    // {0.0,1.6,-1.6}   // right-landing point
}};

/**
* @brief Get the state of the current state machine
*/
DroneState State_Machine::getCurrentState() const
{
	return currentState;
}

/**
* @brief Setting up the state of emergency
*/
void State_Machine::setEmergency()
{
	currentState = DroneState::EMERGENCY;
}

/**
* @brief Update the state of the state machine
*/
void State_Machine::update()
{
	switch (currentState)
	{
	case DroneState::IDLE:
		RCLCPP_INFO(this->get_logger(),"Ready to fly!");
		currentState = DroneState::TAKEOFF;
		break;
	case DroneState::TAKEOFF:
		handleTakeoff();
		break;
	case DroneState::QR_RECOGNITION:
		handleQR();
		break;
	case DroneState::PICTURE_RECOGNITION:
		handlePicture();
		break;
	case DroneState::DROP_BOX:
		handleDrop();
		break;
	case DroneState::LASER_TARGET:
		handleLaserTarget();
		break;
	case DroneState::PASSING_CIRCLE:
		handlePassingCircle();
		break;
	case DroneState::RTL:
		handleRtl();
		break;
	case DroneState::LANDING:
		handleLanding();
		break;
	case DroneState::EMERGENCY:
		handleEmergency();
		break;
	}
}

/*
	For the time being, the state switching logic is implemented using a counting (timing) method, 
	later and localization will be integrated to identify whether a task is completed or not.
*/

/**
* @brief Take off from the original standby.
*/
void State_Machine::handleTakeoff()
{
	RCLCPP_INFO(this->get_logger(),"Take off!");
	p.position_x = waypoints[currentPoint][1];
	p.position_y = waypoints[currentPoint][0];
	p.position_z = waypoints[currentPoint][2];
	publisher_ ->publish(p);
	count_++;
	if(count_ >= 5)
	{
		count_ = 0;
		currentPoint++;
		currentState = DroneState::QR_RECOGNITION;
	}
}

/**
* @brief Recognize the QR code to get the image to be recognized 
		and the information of the landing site
*/
void State_Machine::handleQR()
{
	RCLCPP_INFO(this->get_logger(),"Recognizing QR!");
	p.position_x = waypoints[currentPoint][1];
	p.position_y = waypoints[currentPoint][0];
	p.position_z = waypoints[currentPoint][2];
	publisher_ ->publish(p);
	count_++;
	if(count_ >= 5)
	{
		count_ = 0;
		currentPoint++;
		currentState = DroneState::PICTURE_RECOGNITION;
	}
}

/**
* @brief Recognize the images in turn
*/
void State_Machine::handlePicture()
{
	RCLCPP_INFO(this->get_logger(),"Recognizing pictures");
	p.position_x = waypoints[currentPoint][1];
	p.position_y = waypoints[currentPoint][0];
	p.position_z = waypoints[currentPoint][2];
	publisher_ ->publish(p);
	count_++;
	if(count_ >= 5)
	{
		count_ = 0;
		currentPoint++;
		recognized_picture++;
		if(recognized_picture == 2)
		{
			currentState = DroneState::LASER_TARGET;
		}
		else if(recognized_picture == 4)
		{
			currentState = DroneState::RTL;
		}
	}
}

/**
* @brief Recognize the laser target and dropping objects
*/
void State_Machine::handleLaserTarget()
{
	RCLCPP_INFO(this->get_logger(),"Recognizing the laser target!");
	p.position_x = waypoints[currentPoint][1];
	p.position_y = waypoints[currentPoint][0];
	p.position_z = waypoints[currentPoint][2];
	publisher_ ->publish(p);
	count_++;
	if(count_ >= 5)
	{
		count_ = 0;
		currentPoint++;
		currentState = DroneState::PICTURE_RECOGNITION;
	}
}

void State_Machine::handleDrop()
{}

void State_Machine::handlePassingCircle()
{}

void State_Machine::handleRtl()
{
	RCLCPP_INFO(this->get_logger(),"Ready to rtl!");
	p.position_x = waypoints[currentPoint][1];
	p.position_y = waypoints[currentPoint][0];
	p.position_z = waypoints[currentPoint][2];
	publisher_ ->publish(p);
	count_++;
	if(count_ >= 5)
	{
		count_ = 0;
		currentState = DroneState::LANDING;
	}
}

void State_Machine::handleLanding()
{
	RCLCPP_INFO(this->get_logger(),"Ready to rtl!");
	p.position_x = waypoints[currentPoint][1];
	p.position_y = waypoints[currentPoint][0];
	p.position_z = waypoints[currentPoint][2] + 1.6;
	publisher_ ->publish(p);
	count_++;
	if(count_ >= 5)
	{
		count_ = 0;
	}
}

void State_Machine::handleEmergency()
{}

int main(int argc,char** argv)
{
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<State_Machine>());
	rclcpp::shutdown();
	return 0;
}