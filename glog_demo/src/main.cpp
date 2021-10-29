#include <iostream>
#include <glog/logging.h>

//glog的示例可以参考https://github.com/google/glog

int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);
    google::EnableLogCleaner ( 3 ); 
    //true只输出到终端，但不输出到日志文件中，false则都输出
    FLAGS_logtostderr = false;
    //处于对应等级以上的输出到终端上，0=INFO，1=WARNING，2=ERROR，3=FATAL
    FLAGS_stderrthreshold=0;
    //这里把日志文件指定输出到对应的路径下，否在就输出到/tmp下
    FLAGS_log_dir = "/home/yyj/glog_log";

    //打印INFO级别的信息
	LOG(INFO) << "I am INFO!";
    //打印WARNING级别的信息
	LOG(WARNING) << "I am WARNING!";
    //打印ERROR级别的信息
	 LOG(ERROR) << "I am ERROR!";
    //打印FATAL级别的信息，该级别若打印，则直接报错
	//LOG(FATAL) << "I am FATAL!";

    int num_cookies = 11;
    //当条件满足时输出日志
    LOG_IF(INFO, num_cookies > 10) << "Got lots of cookies"; 
    //google::COUNTER 记录该语句被执行次数，从1开始，在第一次运行输出日志之后，每隔 10 次再输出一次日志信息
    LOG_EVERY_N(INFO, 10) << "Got the " << google::COUNTER << "th cookie";  
    //上述两者的结合，不过要注意，是先每隔 10 次去判断条件是否满足，如果滞则输出日志；而不是当满足某条件的情况下，每隔 10 次输出一次日志信息
    int size = 1024;
    LOG_IF_EVERY_N(INFO, (size > 1024), 10) << "Got the " << google::COUNTER << "th big cookie"; 
    //当此语句执行的前 5 次都输出日志，然后不再输出
    LOG_FIRST_N(INFO, 5) << "Got the " << google::COUNTER << "th cookie";  

    int a = 1;
    int b = 1;
    //判断信息，若不符合则直接报错
    CHECK(a!=b)<<"a!=b";
    LOG(INFO) << "I am INFO!";

    google::ShutdownGoogleLogging();
    return 0;
}