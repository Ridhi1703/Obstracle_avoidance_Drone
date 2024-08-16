#include "mavlink/mavlink.h"


#include <NewPing.h> 
#include <SoftwareSerial.h> 
#define MAX_DISTANCE  70


SoftwareSerial mySerial(10, 11); 
NewPing sonar_B(5, 5, MAX_DISTANCE);
NewPing sonar_F(6, 6, MAX_DISTANCE);

unsigned long HeartbeatTime = 0;
int OUTPUT_PITCH = 0;
int TRIG_PITCH = 0;
int FRONT_SENSOR = 0;
int BACK_SENSOR = 0;
int PITCH_BACK = 0;
int PITCH_FRONT = 0;

void setup() {
  Serial.begin(9600);
  mySerial.begin(57600);
}


void loop() {
  if ( (millis() - HeartbeatTime) > 1000 ) {
    HeartbeatTime = millis();
    PIX_HEART_BEAT();
  } 
  TRIG_PITCH = sonar_F.ping_cm() + sonar_B.ping_cm(); 
  FRONT_SENSOR = sonar_F.ping_cm();
  BACK_SENSOR = sonar_B.ping_cm(); 
  OUTPUT_DATA();
}

uint8_t N = 0;
void OUTPUT_DATA() {
  CALCULATE_PITCH();
  SEND_DATA(OUTPUT_PITCH); 
}

void CALCULATE_PITCH(){
    if (BACK_SENSOR != 0 && BACK_SENSOR < MAX_DISTANCE) {
       PITCH_FRONT = 1500+30+((MAX_DISTANCE-BACK_SENSOR)*8); 
       OUTPUT_PITCH = PITCH_FRONT;
    }
    else if (FRONT_SENSOR != 0 && FRONT_SENSOR < MAX_DISTANCE){
       PITCH_BACK = 1500-30-((MAX_DISTANCE-FRONT_SENSOR)*8);
       OUTPUT_PITCH = PITCH_BACK;
    }
    else {
      OUTPUT_PITCH = 1500;
    }
}
mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
uint16_t len;
 
void PIX_HEART_BEAT() {
  mavlink_msg_heartbeat_pack(255, 0, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, 0, 1, 0);   
  len = mavlink_msg_to_send_buffer(buf, &msg);

  mySerial.write(buf, len);
}
void SEND_DATA(int P) {
    if (TRIG_PITCH != 0){
    mavlink_msg_rc_channels_override_pack(255, 0 , &msg, 1, 0, 0, P, 0, 0, 0, 0, 0, 0);    
    len = mavlink_msg_to_send_buffer(buf, &msg);
    }
    else{
    mavlink_msg_rc_channels_override_pack(255, 0 , &msg, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0);    
    len = mavlink_msg_to_send_buffer(buf, &msg);
    }     
  mySerial.write(buf, len);
}
