#include "axp209.h"
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
void usage() __attribute__ ((noreturn));
void usage()
{
    printf("-----------usage----------->\n");
    printf("axp209 set reg value\n");
    printf("axp209 get reg\n");
    exit(-1);
}

int hex2int(char *szhex)
{
    int n = 0, len;
    int value = 0;
    char hs2i[128] ={
    ['0'] = 0,
    ['1'] = 1,
    ['2'] = 2,
    ['3'] = 3,
    ['4'] = 4,
    ['5'] = 5,
    ['6'] = 6,
    ['7'] = 7,
    ['8'] = 8,
    ['9'] = 9,
    ['A'] = 10,
    ['B'] = 11,
    ['C'] = 12,
    ['D'] = 13,
    ['E'] = 14,
    ['F'] = 15,
    ['a'] = 10,
    ['b'] = 11,
    ['c'] = 12,
    ['d'] = 13,
    ['e'] = 14,
    ['f'] = 15,
    };
    len = strlen(szhex);
    while(len)
    {
        value += hs2i[szhex[len - 1]] * pow(16, n);
        n ++;
        len --;
    }
    return value;
}
int str2int(char *hex)
{
   if(hex[1] == 'x' || hex[1] == 'X')
   {
      return hex2int(&hex[2]);
   }
   return atoi(hex);
}
int main(int argc, char *argv[])
{
    int fd = -1;
    unsigned char status = 0xff;
    unsigned short context = 0xff;
    fd = open("/dev/axp209",O_RDWR, S_IRUSR | S_IWUSR);
    ioctl(fd, AXP209_GET_ELEC, &status);
    printf("%s\t%d\t%d\n", __func__, __LINE__, status);
    if(argc > 2 && (!strcmp("get",argv[1])) )
    {
        context = (str2int(argv[2]) & 0xff) << 8;
        ioctl(fd, AXP209_GET_REG, &context);
        printf("%x\t%x\n",context >> 8, context & 0xff);
    }
    else if(argc > 3 && (!strcmp("set",argv[1])))
    {
        context = (str2int(argv[2]) & 0xff) << 8;
        context |= (str2int(argv[3]) & 0xff);
        ioctl(fd, AXP209_SET_REG, &context);
    }
    else
    {
    //    usage();
        ioctl(fd, AXP209_GET_ADDR, 0x00);
    }
    close(fd);
    return 0;
}
