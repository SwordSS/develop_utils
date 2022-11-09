#include <iostream>
#include <thread>
#include <chrono>

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

int main()
{
    //方法1(返回打印数据)
    std::string first_cmd_str = "ifconfig";
    FILE *fstream=NULL;    
    char buff[1024];  
    memset(buff,0,sizeof(buff));  
    if(NULL==(fstream=popen(first_cmd_str.c_str(),"r")))    
    {   
        fprintf(stderr,"execute command failed: %s",strerror(errno));    
        return -1;    
    }   
    while(NULL!=fgets(buff, sizeof(buff), fstream)) {
            printf("%s",buff);  
    }
    pclose(fstream);  

    //方法2(只返回运行结果)
    std::string second_cmd_str = "./process_test";
    int result = execl(second_cmd_str.c_str(),NULL);

    if(-1 == result)
    {
        std::cout << "Failed to execl" << std::endl;
    }

    return 0;

}
