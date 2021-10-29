/*******************************************************************/
#include "ros/ros.h"  //ros需要的头文件
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
//以下为串口通讯需要的头文件
#include <string>
#include "serial/serial.h"        
#include <stdio.h>
#include <cmath>
#include <vector>
/****************************************************************************/
serial::Serial ser; //声明串口对象

float ratio = 1000.0f ;   //转速转换比例，执行速度调整比例
unsigned char speed_data[10]={0};   //要发给串口的数据
float linear_temp=0,angular_temp=0;//暂存的线速度和角速度
float a = 128/0.873;
float D = 0.33;

float left_speed_data,right_speed_data;
int left_direction_data,right_direction_data;

union intData
{
    unsigned int idata;
    unsigned char cdata[2];
} idl,idr;

void callback(const geometry_msgs::Twist & cmd_input)//订阅/cmd_vel主题回调函数
{
    angular_temp = cmd_input.angular.z ;//获取/cmd_vel的角速度,rad/s
    linear_temp = cmd_input.linear.x ;//获取/cmd_vel的线速度.m/s
 
    //将转换好的小车速度分量为左右轮速度
    left_speed_data = linear_temp - 0.5f*angular_temp*D ;
    right_speed_data = linear_temp + 0.5f*angular_temp*D ;

    if(left_speed_data>0)
        left_direction_data=0;
    else 
        left_direction_data=1;

    if(right_speed_data>0)
        right_direction_data=0;
    else 
        right_direction_data=1;

    //printf("speed_data.d : %f,%f,%d,%d\n",left_speed_data,right_speed_data,left_direction_data,right_direction_data);

    left_speed_data = fabs(a*left_speed_data);
    right_speed_data = fabs(a*right_speed_data);
 
    speed_data[0]=0xFF;
    speed_data[1]=0xFE;
    speed_data[2]=int(left_speed_data);
    speed_data[3]=int(right_speed_data);
    speed_data[4]=int(left_direction_data);
    speed_data[5]=int(right_direction_data);
    speed_data[6]=0x00;
    speed_data[7]=0x00;
    speed_data[8]=0x00;
    speed_data[9]=0x00;

    // printf("speed_data.data : %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n",
    // speed_data[0],speed_data[1],speed_data[2],speed_data[3],speed_data[4],
    // speed_data[5],speed_data[6],speed_data[7],speed_data[8],speed_data[9]);
 
    //写入数据到串口
    ser.write(speed_data,10);
}

void calculateOdom(std::vector<float>& input_data,std::vector<float>& output_data)
{
    //输出输出变量
    float x = input_data.at(0);
    float y = input_data.at(1);
    float theta = input_data.at(2);
    float dl = input_data.at(3);
    float dr = input_data.at(4);
    float t = input_data.at(5);
    float db = input_data.at(6);
    float x_,y_,theta_,wc,vc;

    //中间变量
    float vl = dl/t;
    float vr = dr/t;
    float dc = (dl+dr)/2;
    float delta_theta = (dr-dl)/db;
    wc=(vr-vl)/db;
    vc=(vl+vr)/2;
    theta_ = theta+delta_theta;
    x_ = x+dc*cos(theta);
    y_ = y+dc*sin(theta);

    output_data.push_back(x_);
    output_data.push_back(y_);
    output_data.push_back(theta_);
    output_data.push_back(wc);
    output_data.push_back(vc);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "base_controller");//初始化串口节点
    ros::NodeHandle n;  //定义节点进程句柄
 
    ros::Subscriber sub = n.subscribe("cmd_vel", 20, callback); //订阅/cmd_vel主题
    ros::Publisher odom_pub= n.advertise<nav_msgs::Odometry>("odom", 20); //定义要发布/odom主题
 
    tf2_ros::TransformBroadcaster odom_broadcaster;//定义tf对象

    try 
    { 
    //设置串口属性，并打开串口 
        ser.setPort("/dev/ttyUSB0"); 
        ser.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM( ser.write(speed_data,10);"Unable to open port "); 
        return -1; 
    } 

    //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    } 

    std::clock_t clock_last = std::clock();
    while(ros::ok())
    {
        std::clock_t clock_now = std::clock();    
        double delay_time = (clock_now-clock_last)/(double)CLOCKS_PER_SEC;
        //if(delay_time>0.01)   
        { 
            clock_last = clock_now;

            if(ser.available())
            {  ser.write(speed_data,10);
                std::string str = "";
                str = ser.read(ser.available());
                unsigned char* ptr = (unsigned char*)str.c_str();
                float left_sign = 0;
                float right_sign = 0;
                bool check = true;
                bool state_machine = true;
                if(str.length()>=6)
                {
                    if(!(*(ptr)==0xFF && *(ptr+1)==0xFE && state_machine)) state_machine=false;
                    if(*(ptr+2)==0x00 && state_machine) left_sign = 1;
                    else if(*(ptr+2)==0x01 && state_machine) left_sign = -1;
                    else state_machine=false;

                    if(state_machine)
                    {
                        idl.cdata[1] = *(ptr+3);
                        idl.cdata[0] = *(ptr+4);
                    }

                    if(*(ptr+5)==0x00 && state_machine) right_sign = 1;
                    else if(*(ptr+5)==0x01 && state_machine) right_sign = -1;
                    else state_machine=false;

                    if(state_machine)
                    {
                        idr.cdata[1] = *(ptr+6);
                        idr.cdata[0] = *(ptr+7);
                        //state_machine = (positionX.idata % 2==*(ptr+5))? true : false; 
                    }

                    // if(state_machine)
                    // {
                    //     printf("ptr : ");
                    //     for(int i=0;i<8;i++)
                    //     {
                    //         printf(" %02x",*(ptr+i));
                    //     }
                    //     printf("\n");
                    //     printf("idl : %f,idl :%f\n",idl.idata*left_sign,idr.idata*right_sign);
                    // }
                    if(state_machine)
                    {
                        static bool fist_flag=false;
                        static float x;
                        static float y;
                        static float theta;
                        static float db = 0.33;
                        static float t =0.5;
                        float wc = 0;
                        float vc = 0;

                        if(!fist_flag)
                        {
                            x=0;
                            y=0;
                            theta=0;
                            fist_flag = true;
                        }
                        else
                        {
                            float dl = idl.idata*left_sign/1000.0;
                            float dr = idr.idata*right_sign/1000.0;
                            std::vector<float> input_data;
                            std::vector<float> output_data;
                            input_data.push_back(x);
                            input_data.push_back(y);
                            input_data.push_back(theta);
                            input_data.push_back(dl);
                            input_data.push_back(dr);
                            input_data.push_back(t);
                            input_data.push_back(db);

                            calculateOdom(input_data,output_data);
                            x=output_data.at(0);
                            y=output_data.at(1);
                            theta=output_data.at(2);
                            wc=output_data.at(3);
                            vc=output_data.at(4);

                            printf("x:%f,y:%f,theta:%f,wc:%f,vc:%f\n",x,y,theta/3.1415926*180,wc,vc);
                        }

                        geometry_msgs::TransformStamped odom_trans;//创建一个tf发布需要使用的TransformStamped类型消息
                        nav_msgs::Odometry odom;//定义里程计对象
                        geometry_msgs::Quaternion odom_quat; //四元数变量

                        ros::Time now = ros::Time::now();
                        odom_trans.header.stamp = now;
                        odom_trans.header.frame_id = "odom";     
                        odom_trans.child_frame_id = "base_footprint";       
                        odom_trans.transform.translation.x = x;
                        odom_trans.transform.translation.y = y;
                        odom_trans.transform.translation.z = 0.0;
                        odom_quat = tf::createQuaternionMsgFromYaw(theta);
                        odom_trans.transform.rotation = odom_quat;
                        // odom_trans.transform.rotation.x = 0;
                        // odom_trans.transform.rotation.y = 0; 
                        // odom_trans.transform.rotation.x = 0; 
                        // odom_trans.transform.rotation.w = 1;  

                        //发布tf坐标变化
                        odom_broadcaster.sendTransform(odom_trans);
                        //载入里程计时间戳
                        odom.header.frame_id = "odom";
                        odom.child_frame_id = "base_footprint";    
                        odom.header.stamp = now;   
                        odom.pose.pose.position.x = x;     
                        odom.pose.pose.position.y = y;
                        odom.pose.pose.position.z = 0.0;
                        odom.pose.pose.orientation = odom_quat;
                        // odom.pose.pose.orientation.x = 0;   
                        // odom.pose.pose.orientation.y = 0;
                        // odom.pose.pose.orientation.z = 0;
                        // odom.pose.pose.orientation.w = 1;    
                        odom.twist.twist.linear.x = vc;
                        odom.twist.twist.linear.y = 0;
                        odom.twist.twist.angular.z = wc;       
                        //发布里程计
                        odom_pub.publish(odom);
                    }
                }
            }
            ros::spinOnce();//周期执行
        }
    }
    return 0;
}