#include <LiquidCrystal.h>
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

String out = String("                      ");

void messageCb(const std_msgs::String& toggle_msg) {
  if (toggle_msg.data){
    out = toggle_msg.data;}
}

ros::Subscriber<std_msgs::String> sub("chatter", messageCb );


const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {

  lcd.begin(16, 2);
  lcd.print(String("v = ") + out.substring(0,5));
  
  lcd.setCursor(0, 1);
  lcd.print(String("w = ") + out.substring(7,12));

  nh.spinOnce();
  delay(500);
}
