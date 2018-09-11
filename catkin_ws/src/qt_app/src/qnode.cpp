/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/qt_app/qnode.hpp"

#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_P 0x70
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_app {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown();
      ros::waitForShutdown();
    }
	wait();
}

int kfd = 0;
struct termios cooked, raw;

bool QNode::init() {
	ros::init(init_argc,init_argv,"qt_app");
	if ( ! ros::master::check() ) {
		return false;
	}
    ros::start();
	ros::NodeHandle n;
	// Add your ros communications here.
    chatter_subscriber = n.subscribe("qt_app", 1000, &QNode::readMsg, this);
	start();
	return true;
}

bool QNode::init(const std::string &master_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	ros::init(remappings,"qt_app");
	if ( ! ros::master::check() ) {
		return false;
	}
    ros::start();
	ros::NodeHandle n;
	// Add your ros communications here.
    chatter_subscriber = n.subscribe("qt_app", 1000, &QNode::readMsg, this);
	start();
	return true;
}

void QNode::readMsg(const std_msgs::String &message_holder){
    log(Info, message_holder.data);
}

void QNode::run() {
    ros::NodeHandle n;
    chatter_subscriber = n.subscribe("qt_app", 1000, &QNode::readMsg, this);
    ros::spin();
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace qt_app
