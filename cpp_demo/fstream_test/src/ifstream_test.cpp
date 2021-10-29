#include <fstream>
#include <iostream>
#include <string.h>
#include <sstream>

int main()
{
    std::ifstream f;
    f.open("test.txt",std::ios::in);
    std::string a,b,c;
    int number=0;
    while(!f.eof())
    {
        getline(f,a,'\n');
        std::stringstream ss(a);
        while(!ss.eof())
        {
            std::string str;
            ss >> str;
            std::cout<<str<<std::endl;
        }
    }
    f.close();
    return 0;
}
