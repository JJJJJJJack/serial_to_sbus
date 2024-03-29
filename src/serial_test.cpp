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
#define CHANNEL_BOARD_PITCH_ALIGNMENT 4
#define CHANNEL_FLIGHT_MODE 6
#define CHANNEL_ARM 7

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

float saturate_float(float input, float min, float max){
  if(input > max)
    input = max;
  if(input < min)
    input = min;
  return input;
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
  string USB_Dev;
  ros::NodeHandle nh("~");
  nh.param<std::string>("USB", USB_Dev, "/dev/ttyUSB0");
  
  ros::Subscriber sub_joy_control  = n.subscribe("joy", 1, joystick_command_callback);

  serial::Serial my_serial;
  serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);

  my_serial.setPort(USB_Dev);
  my_serial.setBaudrate(115200);
  my_serial.setTimeout(timeout);

  try{
    my_serial.open();
  }
  catch(serial::IOException& e){
    ROS_ERROR_STREAM("Unable to open Serial USB port: " << USB_Dev);
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

  int channel_loop = 0;
  while (ros::ok())
  {
    gettimeofday(&tvend,NULL);
    double totaltime = tvend.tv_sec - tvstart.tv_sec + 1e-6 * (tvend.tv_usec - tvstart.tv_usec);
    
    if(joy_control_ready == true){
      /*channel[CHANNEL_ROLL]     = saturate_float(1500 + 500*joy_control.axes[0],1000,2000);
      channel[CHANNEL_PITCH]    = saturate_float(1500 + 500*joy_control.axes[1],1000,2000);
      channel[CHANNEL_YAW]      = saturate_float(1500 + 500*joy_control.axes[2],1000,2000);
      channel[CHANNEL_THROTTLE] = saturate_float(1000 + 1000*joy_control.axes[3],1000,2000);
      channel[CHANNEL_ARM]      = saturate_float(1000 + 1000*joy_control.buttons[0],1000,2000);
      channel[CHANNEL_FLIGHT_MODE] = saturate_float(1000 + 1000*joy_control.buttons[1],1000,2000);
      channel[CHANNEL_BOARD_PITCH_ALIGNMENT] = saturate_float(1500 + 500*joy_control.axes[4],1000,2000);
      */
      channel_loop = (int)((int)totaltime % 16);
      channel[channel_loop] = saturate_float(1000 + 1000*joy_control.axes[3],1000,2000);
      cout<<channel_loop<<endl;
    }
    
    form_channel_data(data_to_send, channel);

    my_serial.write(data_to_send, 35);
        
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  
  return 0;
}
