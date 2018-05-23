#include "hFramework.h"
#include "hCloudClient.h"
#include <ros.h>
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/BatteryState.h"
#include <std_msgs/Bool.h>

using namespace hFramework;

bool batteryLow = false;

ros::NodeHandle nh;

geometry_msgs::PoseStamped pose;
ros::Publisher pose_pub("/pose", &pose);

sensor_msgs::BatteryState battery;
ros::Publisher battery_pub("/battery", &battery);

uint16_t delay = 10; // milliseconds
uint8_t pose_pub_cnt;
float delay_s = (float)delay / (float)1000;
uint16_t enc_res = 1400; // encoder tics per wheel revolution

int32_t enc_FL = 0; // encoder tics
int32_t enc_RL = 0; // encoder tics
int32_t enc_FR = 0; // encoder tics
int32_t enc_RR = 0; // encoder tics

int32_t enc_L = 0;         // encoder tics
float wheel_L_ang_pos = 0; // radians
float wheel_L_ang_vel = 0; // radians per second

int32_t enc_R = 0;         // encoder tics
float wheel_R_ang_pos = 0; // radians
float wheel_R_ang_vel = 0; // radians per second

float robot_angular_pos = 0; // radians
float robot_angular_vel = 0; // radians per second

float robot_x_pos = 0; // meters
float robot_y_pos = 0; // meters
float robot_x_vel = 0; // meters per second
float robot_y_vel = 0; // meters per second

float robot_width = 0.3;    // meters
float robot_length = 0.105; //meters
float wheel_radius = 0.04;  //meters

void reset_encoders()
{
    hMot1.resetEncoderCnt();
    hMot2.resetEncoderCnt();
    hMot3.resetEncoderCnt();
    hMot4.resetEncoderCnt();
}

void reset_odom_vars()
{
    enc_FL = 0;            // encoder tics
    enc_RL = 0;            // encoder tics
    enc_FR = 0;            // encoder tics
    enc_RR = 0;            // encoder tics
    enc_L = 0;             // encoder tics
    wheel_L_ang_pos = 0;   // radians
    wheel_L_ang_vel = 0;   // radians per second
    enc_R = 0;             // encoder tics
    wheel_R_ang_pos = 0;   // radians
    wheel_R_ang_vel = 0;   // radians per second
    robot_angular_pos = 0; // radians
    robot_angular_vel = 0; // radians per second
    robot_x_pos = 0;       // meters
    robot_y_pos = 0;       // meters
    robot_x_vel = 0;       // meters per second
    robot_y_vel = 0;       // meters per second
}

void reset_odom(const std_msgs::Bool &msg)
{
    if (msg.data == true)
    {
        reset_encoders();
        reset_odom_vars();
    }
}

void twistCallback(const geometry_msgs::Twist &twist)
{
    float lin = twist.linear.x;
    float ang = twist.angular.z;
    float motorL = lin - ang * 0.5;
    float motorR = lin + ang * 0.5;
    hMot1.setPower(motorR * (-700) * !batteryLow);
    hMot2.setPower(motorR * (-700) * !batteryLow);
    hMot3.setPower(motorL * (-700) * !batteryLow);
    hMot4.setPower(motorL * (-700) * !batteryLow);
}

void batteryCheck()
{
    int i = 0;
    int publish_cnt = 0;
    for (;;)
    {
        if (sys.getSupplyVoltage() > 10.8)
        {
            i--;
        }
        else
        {
            i++;
        }
        if (i > 50)
        {
            batteryLow = true;
            i = 50;
        }
        if (i < -50)
        {
            batteryLow = false;
            i = -50;
        }
        if (batteryLow == true)
        {
            LED1.toggle();
        }
        else
        {
            LED1.on();
        }
        sys.delay(250);
        if (publish_cnt == 8)
        {
            battery.voltage = sys.getSupplyVoltage();
            battery_pub.publish(&battery);
            publish_cnt = 0;
        }
        publish_cnt++;
    }
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &twistCallback);
ros::Subscriber<std_msgs::Bool> odom_reset_sub("/reset_odom", &reset_odom);

void hMain()
{
    platform.begin(&RPi);
    nh.getHardware()->initWithDevice(&platform.LocalSerial);
    //nh.getHardware()->initWithDevice(&RPi);
    nh.initNode();
    nh.subscribe(sub);
    nh.subscribe(odom_reset_sub);
    hMot3.setMotorPolarity(Polarity::Reversed);
    hMot3.setEncoderPolarity(Polarity::Reversed);
    hMot4.setMotorPolarity(Polarity::Reversed);
    hMot4.setEncoderPolarity(Polarity::Reversed);
    hMot1.setSlewRate(0.05);
    hMot2.setSlewRate(0.05);
    hMot3.setSlewRate(0.05);
    hMot4.setSlewRate(0.05);
    LED1.on();
    sys.taskCreate(batteryCheck);

    pose.header.frame_id = "robot";
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    pose.pose.orientation = tf::createQuaternionFromYaw(0);

    nh.advertise(pose_pub);
    nh.advertise(battery_pub);
    pose_pub_cnt = 0;

    while (true)
    {
        enc_FR = hMot1.getEncoderCnt();
        enc_RR = hMot2.getEncoderCnt();
        enc_RL = hMot3.getEncoderCnt();
        enc_FL = hMot4.getEncoderCnt();

        enc_L = (enc_FL + enc_RL) / 2;
        enc_R = (enc_FR + enc_RR) / 2;

        wheel_L_ang_vel = ((2 * 3.14 * enc_L / enc_res) - wheel_L_ang_pos) / delay_s;
        wheel_R_ang_vel = ((2 * 3.14 * enc_R / enc_res) - wheel_R_ang_pos) / delay_s;

        wheel_L_ang_pos = 2 * 3.14 * enc_L / enc_res;
        wheel_R_ang_pos = 2 * 3.14 * enc_R / enc_res;

        robot_angular_vel = (((wheel_R_ang_pos - wheel_L_ang_pos) * wheel_radius / robot_width) -
                             robot_angular_pos) /
                            delay_s;
        robot_angular_pos = (wheel_R_ang_pos - wheel_L_ang_pos) * wheel_radius / robot_width;

        robot_x_vel = (wheel_L_ang_vel * wheel_radius + robot_angular_vel * robot_width / 2) *
                      cos(robot_angular_pos);
        robot_y_vel = (wheel_L_ang_vel * wheel_radius + robot_angular_vel * robot_width / 2) *
                      sin(robot_angular_pos);

        robot_x_pos = robot_x_pos + robot_x_vel * delay_s;
        robot_y_pos = robot_y_pos + robot_y_vel * delay_s;

        pose_pub_cnt++;
        if (pose_pub_cnt == 10)
        {
            pose.pose.position.x = robot_x_pos;
            pose.pose.position.y = robot_y_pos;
            pose.pose.orientation = tf::createQuaternionFromYaw(robot_angular_pos);
            pose_pub.publish(&pose);
            pose_pub_cnt = 0;
        }

        nh.spinOnce();
        sys.delay(delay);
    }
}