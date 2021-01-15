#include "Arduino.h" 
#include <ros.h>
#include <std_msgs/Int32.h>

#define enc_A 7 
#define enc_B 8 

ros::NodeHandle nh;

std_msgs::Int32 msg;
ros::Publisher enc_data("enc_data", &msg);

int counter = 0; 
int aState;
int aLastState;  

void setup() { 
	pinMode (enc_A,INPUT);
	pinMode (enc_B,INPUT);
   
	//Serial.begin (9600);
	aLastState = digitalRead(enc_A);   

	// ROS setting
	nh.initNode();
	nh.advertise(enc_data);
} 
void loop() { 
	aState = digitalRead(enc_A); // Reads the "current" state of the outputA
	if (aState != aLastState){     
		if (digitalRead(enc_B) != aState)
			counter ++;
		else
			counter --;
		
		msg.data = counter;
		enc_data.publish(&msg);
		nh.spinOnce();
	} 
	aLastState = aState; // Updates the previous state of the enc_A with the current state
}
