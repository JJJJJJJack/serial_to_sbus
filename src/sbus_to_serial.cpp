#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>
#include "serial/serial.h"

#include <time.h>

#include "sensor_msgs/Joy.h"

#define CHANNEL_MAP_A_INV 0.625390694  // 1 / CHANNEL_MAP_A
#define CHANNEL_MAP_B_INV 879.111706   // -CHANNEL_MAP_B / CHANNEL_MAP_A

#define CHANNEL_ROLL 0
#define CHANNEL_PITCH 1
#define CHANNEL_YAW 3
#define CHANNEL_THROTTLE 2
#define CHANNEL_BOARD_PITCH_ALIGNMENT 4
#define CHANNEL_FLIP_SWITCH 9
#define CHANNEL_FLIGHT_MODE 6
#define CHANNEL_ARM 7

using namespace std;

ros::Publisher joy_pub;

float saturate_float(float input, float min, float max){
  if(input > max)
    input = max;
  if(input < min)
    input = min;
  return input;
}

bool parse_channel_data(uint8_t* data, uint16_t* channel){
  // Check start byte
  if(data[0] != 0x0F){
    return false;
  }
  
  // Calculate checksum
  uint8_t checkXOR = data[1];
  for(int i = 2; i < 33; i++){
    checkXOR ^= data[i];
  }
  
  // Verify checksum
  if(data[34] != (checkXOR ^ data[33])){
    return false;
  }
  
  // Parse channel data
  int channel_parity = 0;
  int channel_number = 0;
  uint16_t channel_value = 0;
  uint8_t lower_byte = 0;
  
  for(int i = 1; i < 33; i++){
    channel_parity = i % 2;
    channel_number = (i - 1) / 2;
    if(channel_parity == 0){
      // lower byte
      lower_byte = data[i];
    }else{
      // higher byte
      channel_value = (((uint16_t)data[i]) << 8) | lower_byte;
      // Convert from SBUS format back to channel value
      float channel_float = CHANNEL_MAP_A_INV * channel_value + CHANNEL_MAP_B_INV;
      channel[channel_number] = (uint16_t)round(channel_float);
    }
  }
  
  return true;
}

void publish_joy_message(uint16_t* channel){
  sensor_msgs::Joy joy_msg;
  
  // Initialize arrays
  joy_msg.axes.resize(5);
  joy_msg.buttons.resize(2);
  
  // Convert channel values back to joystick format (-1 to 1 for axes, 0 to 1 for buttons)
  joy_msg.axes[0] = saturate_float((channel[CHANNEL_ROLL] - 1500) / 500.0, -1.0, 1.0);        // Roll
  joy_msg.axes[1] = saturate_float((channel[CHANNEL_PITCH] - 1500) / 500.0, -1.0, 1.0);       // Pitch  
  joy_msg.axes[2] = saturate_float((channel[CHANNEL_YAW] - 1500) / 500.0, -1.0, 1.0);         // Yaw
  joy_msg.axes[3] = saturate_float((channel[CHANNEL_THROTTLE] - 1000) / 1000.0, 0.0, 1.0);    // Throttle
  joy_msg.axes[4] = saturate_float((channel[CHANNEL_BOARD_PITCH_ALIGNMENT] - 1500) / 500.0, -1.0, 1.0); // Board pitch alignment
  
  // Convert button channels (1000 = off, 2000 = on)
  joy_msg.buttons[0] = (channel[CHANNEL_ARM] > 1500) ? 1 : 0;           // Arm button
  joy_msg.buttons[1] = (channel[CHANNEL_FLIP_SWITCH] > 1500) ? 1 : 0;   // Flip switch
  
  // Set timestamp
  joy_msg.header.stamp = ros::Time::now();
  joy_msg.header.frame_id = "sbus_receiver";
  
  // Publish the message
  joy_pub.publish(joy_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbus_to_serial");
  ros::NodeHandle n;
	
  ros::Rate loop_rate(100);
  string USB_Dev;
  ros::NodeHandle nh("~");
  nh.param<std::string>("USB", USB_Dev, "/dev/ttyUSB0");
  
  joy_pub = n.advertise<sensor_msgs::Joy>("joy_control", 1);

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
    return -1;
  }

  uint8_t data_received[35];
  uint16_t channel[16] = {1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500};

  struct timeval tvstart, tvend;
  gettimeofday(&tvstart,NULL);

  int count = 0;
  
  while (ros::ok())
  {
    gettimeofday(&tvend,NULL);
    double totaltime = tvend.tv_sec - tvstart.tv_sec + 1e-6 * (tvend.tv_usec - tvstart.tv_usec);
    
    // Try to read SBUS data
    if(my_serial.available() >= 35){
      size_t bytes_read = my_serial.read(data_received, 35);
      
      if(bytes_read == 35){
        // Parse the received SBUS data
        if(parse_channel_data(data_received, channel)){
          // Successfully parsed, publish joy message
          publish_joy_message(channel);
          
          // Optional: Print channel values for debugging
          if(count % 100 == 0){  // Print every 100 iterations to avoid spam
            cout << "Channels: ";
            for(int i = 0; i < 8; i++){
              cout << channel[i] << " ";
            }
            cout << endl;
          }
        } else {
          ROS_WARN_THROTTLE(1.0, "Failed to parse SBUS data - invalid checksum or format");
        }
      }
    }
        
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  
  return 0;
}