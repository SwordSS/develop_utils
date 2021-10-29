#include <iostream>
#include <fstream>

int main()
{
    std::ofstream f;
    f.open("/home/yyj/test.txt",std::ios::out);
    if(f.is_open())
    {
        f<<"file";
        f<< 123;
        f<<"\t";
        f<< 456;
        f<<"test";
        f.close();
    }
    std::cout<< "ok!"<<std::endl;
    return 0;
} 
