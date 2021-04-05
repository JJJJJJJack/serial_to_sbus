#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>
#include "serial/serial.h"

#include <time.h>

#include "sensor_msgs/Joy.h"

#define CHANNEL_MAP_A 1.599340478
#define CHANNEL_MAP_B -1406.57873

#define CHANNEL_ROLL 0
#define CHANNEL_PITCH 1
#define CHANNEL_YAW 3
#define CHANNEL_THROTTLE 2
#define CHANNEL_FLIGHT_MODE 7

using namespace std;

bool joy_control_ready = false;
sensor_msgs::Joy joy_control;

//void goal_callback(const geometry_msgs::PoseStamped& message){
//  goal << message.pose.position.x, message.pose.position.y, message.pose.position.z;
//}

void joystick_command_callback(const sensor_msgs::Joy& message){
  joy_control_ready = true;
  joy_control = message;
}

void form_channel_data(uint8_t* data, uint16_t* channel){
  data[0] = 0x0F;
  int channel_parity = 0;
  int channel_number = 0;
  uint16_t channel_value = 0;
  uint8_t checkXOR = data[1];
  for(int i = 1; i < 33; i++){
    channel_parity = i % 2;
    channel_number = (i - 1) / 2;
    if(channel_parity == 0){
      // lower byte
      data[i] = channel_value & 0x00FF;
    }else{
      // higher byte
      int aa = channel[channel_number];
      channel_value = (uint16_t)floor(CHANNEL_MAP_A * aa + CHANNEL_MAP_B);
      data[i] = ((channel_value & 0xFF00) >> 8) & 0x00FF;
    }
    if(i > 1)
      checkXOR ^= data[i];
  }
  data[33] = 0x00;
  data[34] = checkXOR ^ data[33];
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial_to_sbus");
  ros::NodeHandle n;
	
  ros::Rate loop_rate(100);
  
  ros::Subscriber sub_joy_control  = n.subscribe("/joy_control", 1, joystick_command_callback);
  //ros::Subscriber sub_quad_vel   = n.subscribe("velocity", 1, quad_vel_callback);
  //ros::Subscriber sub_obstacle   = n.subscribe("/vicon/Block/Block", 1, obstacle_callback);
  //ros::Subscriber sub_goal       = n.subscribe("goal", 1, goal_callback);

  serial::Serial my_serial;
  serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
  my_serial.setPort("/dev/ttyUSB0");
  my_serial.setBaudrate(115200);
  my_serial.setTimeout(timeout);

  try{
    //serial::Serial my_serial("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(1000));
    my_serial.open();
  }
  catch(serial::IOException& e){
    ROS_ERROR_STREAM("Unable to open Serial USB port.");
    return -1;
  }
  

  if(my_serial.isOpen()){
    cout<<"Serial port open successfully!"<<endl;
  }else{
    cerr<<"Error openning serial port!"<<endl;
  }

  
  //uint16_t channelRoll = 200;
  
  uint8_t data_to_send[35];

  struct timeval tvstart, tvend;
  gettimeofday(&tvstart,NULL);

  int count = 0;
  srand((int)time(0));
  bool Start_timer = true;
  uint16_t channel[16] = {1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500};
  channel[CHANNEL_ROLL]     = 1500;  // Roll
  channel[CHANNEL_PITCH]    = 1500;  // Pitch
  channel[CHANNEL_YAW]      = 1500;  // Yaw
  channel[CHANNEL_THROTTLE] = 1000;  // Default throttle is all the way down

  while (ros::ok())
  {
    gettimeofday(&tvend,NULL);
    double totaltime = tvend.tv_sec - tvstart.tv_sec + 1e-6 * (tvend.tv_usec - tvstart.tv_usec);
    
    if(joy_control_ready == true){
      channel[CHANNEL_ROLL]     = 1500 + 500*joy_control.axes[0];
      channel[CHANNEL_PITCH]    = 1500 + 500*joy_control.axes[1];
      channel[CHANNEL_YAW]      = 1500 + 500*joy_control.axes[2];
      channel[CHANNEL_THROTTLE] = 1000 + 1000*joy_control.axes[3];
      channel[CHANNEL_FLIGHT_MODE] = 1000 + 1000*joy_control.buttons[1];
    }
    
    form_channel_data(data_to_send, channel);

    my_serial.write(data_to_send, 35);
        
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  
  return 0;
}
