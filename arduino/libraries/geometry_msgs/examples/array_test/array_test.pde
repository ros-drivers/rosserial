/* 
 * rosserial::geometry_msgs::PoseArray Test 
 */

#include <ros.h>
#include <geometry_msgs.h>

float x; 

ros::NodeHandle nh;

ROS_CALLBACK(messageCb, geometry_msgs::PoseArray, msg)
  msg.

  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

std_msgs::Float64 test;
ros::Subscriber s("your_topic", &msg, &messageCb);
ros::Publisher p("my_topic", &test);

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(p);
  nh.subscribe(s);
}

void loop()
{
  test.data = x;
  p.publish( &test );
  nh.spinOnce();
  delay(10);
}



ROS_CALLBACK(led1Cb, std_msgs::Int8, msg1)
  if(msg1.data == 0){
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
  }else if(msg1.data == 1){
    digitalWrite(2, LOW);
    digitalWrite(3, HIGH);
    digitalWrite(4, HIGH);
  }else if(msg1.data == 2){
    digitalWrite(2, HIGH);
    digitalWrite(3, LOW);
    digitalWrite(4, HIGH);
  }else if(msg1.data == 3){
    digitalWrite(2, HIGH);
    digitalWrite(3, HIGH);
    digitalWrite(4, LOW); 
  }else{
    digitalWrite(2, HIGH);
    digitalWrite(3, HIGH);
    digitalWrite(4, HIGH); 
  }
}

ROS_CALLBACK(led2Cb, std_msgs::Int8, msg2)
  if(msg2.data == 0){
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(7, LOW);
  }else if(msg2.data == 1){
    digitalWrite(5, LOW);
    digitalWrite(6, HIGH);
    digitalWrite(7, HIGH);
  }else if(msg2.data == 2){
    digitalWrite(5, HIGH);
    digitalWrite(6, LOW);
    digitalWrite(7, HIGH);
  }else if(msg2.data == 3){
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);
    digitalWrite(7, LOW); 
  }else{
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);
    digitalWrite(7, HIGH); 
  }
}

ROS_CALLBACK(led3Cb, std_msgs::Int8, msg3)
  if(msg3.data == 0){
    digitalWrite(8, LOW);
    digitalWrite(9, LOW);
    digitalWrite(10, LOW);
  }else if(msg3.data == 1){
    digitalWrite(8, LOW);
    digitalWrite(9, HIGH);
    digitalWrite(10, HIGH);
  }else if(msg3.data == 2){
    digitalWrite(8, HIGH);
    digitalWrite(9, LOW);
    digitalWrite(10, HIGH);
  }else if(msg3.data == 3){
    digitalWrite(8, HIGH);
    digitalWrite(9, HIGH);
    digitalWrite(10, LOW); 
  }else{
    digitalWrite(8, HIGH);
    digitalWrite(9, HIGH);
    digitalWrite(10,HIGH); 
  }
}

ros::Subscriber l1("led1", &msg1, &led1Cb);
ros::Subscriber l2("led2", &msg2, &led2Cb);
ros::Subscriber l3("led3", &msg3, &led3Cb);

std_msgs::Bool b1;
std_msgs::Bool b2;
std_msgs::Bool b3;

ros::Publisher bp1("b1_pressed", &b1);
ros::Publisher bp2("b2_pressed", &b2);
ros::Publisher bp3("b3_pressed", &b3);

void setup()
{
  for(int i=2; i<11; i++)
  {
    digitalWrite(i,HIGH);
    pinMode(i,OUTPUT);
  }
  pinMode(A6, INPUT);
  digitalWrite(A6, HIGH);
  nh.initNode();
  nh.advertise(bp1);
  nh.advertise(bp2);
  nh.advertise(bp3);
  nh.subscribe(l1);
  nh.subscribe(l2);
  nh.subscribe(l3);
}

void loop()
{
  b1.data = (digitalRead(A6) == HIGH) ? false : true;
  bp1.publish(&msg1);
  b2.data = (digitalRead(A7) == HIGH) ? false : true;
  bp2.publish(&msg2);
  b3.data = (digitalRead(A8) == HIGH) ? false : true;
  bp3.publish(&msg3);
  nh.spinOnce();
  delay(10);
}

