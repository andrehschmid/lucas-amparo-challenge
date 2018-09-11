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
      ros::shutdown(); // explicitly needed since we use ros::start();
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
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
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
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;

    char c;
    bool dirty = false;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to translate model.");
    puts("Use WASD keys to rotate model.");
    puts("Use [1,4] keys to change model.");
    puts("Use p key to take a snapshot.");

    while ( ros::ok() || c == KEYCODE_Q) {
        if(read(kfd, &c, 1) < 0){
            perror("read():");
            exit(-1);
        }

        std_msgs::String msg;
        std::stringstream ss;

        switch(c){
            //Rotation
            case KEYCODE_LEFT:
                ss << "rotate left";
                dirty = true;
            break;
            case KEYCODE_RIGHT:
                ss << "rotate right";
                dirty = true;
            break;
            case KEYCODE_UP:
                ss << "rotate up";
                dirty = true;
            break;
            case KEYCODE_DOWN:
                ss << "rotate down";
                dirty = true;
            break;
            //Translation
            case KEYCODE_W:
                ss << "translate up";
                dirty = true;
            break;
            case KEYCODE_A:
                ss << "translate left";
                dirty = true;
            break;
            case KEYCODE_S:
                ss << "translate down";
                dirty = true;
            break;
            case KEYCODE_D:
                ss << "translate right";
                dirty = true;
            break;
            //Change Model
            case KEYCODE_1:
                ss << "model torus";
                dirty = true;
            break;
            case KEYCODE_2:
                ss << "model cylinder";
                dirty = true;
            break;
            case KEYCODE_3:
                ss << "model cuboid";
                dirty = true;
            break;
            case KEYCODE_4:
                ss << "model sphere";
                dirty = true;
            break;
            //Snapshot
            case KEYCODE_P:
                ss << "snapshot";
                dirty = true;
        }
        msg.data = ss.str();
        if(dirty == true){
            chatter_publisher.publish(msg);
            log(Info,std::string("I sent: ")+msg.data);
            dirty = false;
        }
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
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
