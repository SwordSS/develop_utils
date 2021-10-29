#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include <string>
#include <vector>
#include <ctime>
#include <stdio.h>

  
serial::Serial ser; //声明串口对象
 

void SplitString(const std::string& s, std::vector<std::string>& v, const std::string& c)
{
    using namespace std;
    string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while(string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2-pos1));
        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if(pos1 != s.length())
        v.push_back(s.substr(pos1));
}

  
int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "serial_example_node"); 
    //声明节点句柄 
    ros::NodeHandle nh; 
  
    try 
    { 
    //设置串口属性，并打开串口 
        ser.setPort("/dev/ttyACM0"); 
        ser.setBaudrate(9600); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
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
  
    //指定循环的频率 
    //ros::Rate loop_rate(1); 

    //初始化
    if(ros::ok())
    {
        int count = 0;
        while(count!=10)
        {
            if(ser.available()){ 
                std::string str; 
                str = ser.read(ser.available());
                ser.flush();
                std::vector<std::string> v;
                v.clear();
                SplitString(str,v,"\t");
                if(v.size()==1)count++;
                else count = 0;
                ros::Duration(1).sleep(); //计时器需要更改的
                std::cout<< count <<std::endl; //需要改成ros的样式   
            } 
        }
    }

    std::clock_t time = std::clock();
    //开始接收
    while(ros::ok()) 
    { 
        if(ser.available()){ 
            //ROS_INFO_STREAM("Reading from serial port\n"); 
            std::string str; 
            str = ser.read(ser.available());
            ser.flush();
            std::vector<std::string> v;
            v.clear();
            SplitString(str,v,"\t");
            if(!v.empty())
            {
                    std::cout << "v.size() :"<<v.size()<<std::endl;
                if(v[0].length()==6)
                {
                    std::clock_t now = std::clock();
                    std::clock_t cost_time = now-time;
                    time = now;
                    std::cout << v[0] <<std::endl;
                    std::cout << "time cost : "<< 1000*(cost_time)/(double)CLOCKS_PER_SEC<<"s" <<std::endl;
                }
            }
            
            //loop_rate.sleep(); 
            //ros::Duration(1).sleep();  
            sleep(1);
        } 
        
  
    } 
} 