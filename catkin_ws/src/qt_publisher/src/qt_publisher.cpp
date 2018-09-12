#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>

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
#define KEYCODE_Y 0x79
#define KEYCODE_U 0x75
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34

int main(int argc, char **argv){
    ros::init(argc, argv, "qt_publisher");
    int kfd = 0;
    struct termios cooked, raw;
    ros::Publisher chatter_publisher;

    ros::start();
    ros::NodeHandle n;

    chatter_publisher = n.advertise<std_msgs::String>("qt_app", 1000);

    ros::Rate loop_rate(1);

    char c = 0x62;
    //Starting with B as value
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
    puts("Use y/u keys to zoom in and out, respectively.");
    puts("Use q key to close the application.");

    while (c != KEYCODE_Q) {
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
	    //Zoom
	    case KEYCODE_Y:
	        ss << "zoom in";
	        dirty = true;
	    break;
	    case KEYCODE_U:
		ss << "zoom out";
		dirty = true;
	    break;
	    //Snapshot
	    case KEYCODE_P:
	        ss << "snapshot";
	        dirty = true;
	    break;
	    //Exit
	    case KEYCODE_Q:
		ss << "exit";
		dirty = true;
	    break;
	}

	if(dirty == true){
	    //Sending message
	    msg.data = ss.str();
	    chatter_publisher.publish(msg);
	    std::cerr << std::string("I recieved: ")+msg.data+std::string("\n");
	    ros::spinOnce();
	    dirty = false;
	}
    }
    std::cout << "Ros shutdown" << std::endl;
    return 0;
}
