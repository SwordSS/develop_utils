#include <iostream>
#include <thread>
#include <chrono>

#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>

void PrintAndDelay(int pid,int delay_ms)
{
    static int number = 0;

    for(int i=0;i<10;i++)
    {
        std::cout << "pid : " << pid <<" number = " << number << ",deylay_ms = " << delay_ms << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        number ++ ;
    }
}

/*
*参考：
* https://blog.csdn.net/geiii9/article/details/123814112
* http://blog.chinaunix.net/uid-25458223-id-206727.html
*/

int main()
{
    std::cout << "Parent Process " << getpid() << " is running " << std::endl;

    pid_t pid = fork();

    if(pid<0)
    {
        std::cout << "Create Child Process failure : " << strerror(errno) << std::endl;
    }
    else if(pid==0)
    {
        //setsid令子进程脱离终端控制，从而当终端结束，如ctrl+c，仍继续运行
        if(setsid()==-1);
        {
            std::cout << "Failed to disassociate from parent PID.error code: " << strerror(errno) << std::endl;
        }
        std::cout << "pid == 0,This Process PID is " << getpid() << ",This Parent PID is " << getppid() << std::endl;
        PrintAndDelay(getpid(),500);
    }
    else if(pid>0)
    {   
        std::cout << "pid > 0,This Process PID is " << getpid() << ",Child PID is " << pid << std::endl;
        PrintAndDelay(getpid(),250);
    }
    
    return 0;
}
