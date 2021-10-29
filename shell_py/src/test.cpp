#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
  
int main()
{
    int fd;
  
    // FIFO file path
    char * myfifo = "/tmp/myfifo";
  
    // Creating the named file(FIFO)
    // mkfifo(<pathname>, <permission>)
    mkfifo(myfifo, 0666);
  
    char arr1[80];
    while (1)
    {
        // Open FIFO for write only
        fd = open(myfifo, O_WRONLY);
  
        // Take an input from user.
        fgets(arr1, 80, stdin);
  
        // Write the input on FIFO
        // and close it
        write(fd, arr1, strlen(arr1)+1);
        close(fd);
    }
    return 0;
}