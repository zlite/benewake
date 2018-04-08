#include "../include/ce30_sdk.h"
#include <fstream>
#include <iostream>
#include <termios.h>
#include <sys/select.h>

#define DEBUG
#define TTY_PATH    "/dev/tty"
#define STTY_US     "stty raw -echo -F "
#define STTY_DEF    "stty -raw echo -F "

bool gRun = true;
struct termios orig_termios;

void sig_stop(int value)
{
    gRun = false;
    return;
}

static int get_char()
{
    fd_set fds;
    struct timeval tv;
    int ch = 0;

    FD_ZERO(&fds);
    FD_SET(0, &fds);
    tv.tv_sec = 0;
    tv.tv_usec = 10;

    if(select(1, &fds, NULL, NULL, &tv) > 0)
    {
        ch = getchar();
    }

    return ch;
}

int main()
{
    gRun = true;
    signal(SIGINT, sig_stop);

    // enter ip address
    char ip[20];
    printf("Enter LiDAR's Ip address: ");
    setbuf(stdin, NULL);
    std::cin >> ip;

    printf("Init device\r\n");
    int sd = 0;
    if(!benewake::initDevice(sd, ip))
    {
        printf("ERROR: Connection failed!\r\n");
        return -1;
    }

    int h = 24, w = 660;
    unsigned short distData[h * w], ampData[h * w], tmp = 0;
    float coord_x[h * w], coord_y[h * w], coord_z[h * w];
    int i = 0;

    std::ofstream outFile("data.txt");
    printf("Start processing ...\r\n");
    printf("Press [C] to change LiDAR's IP address.\r\n");
    printf("Press [Q] to exit.\r\n");
    system(STTY_US TTY_PATH);
    while(gRun)
    {
        if(!benewake::getDistanceData(sd, distData, ampData))
        {
            printf("ERROR: Failed to receive data!\r\n");
            outFile.close();
            benewake::closeDevice(sd);
            return -1;
        }
        else
        {
            i++;
            //printf("Received %d frames.\r\n", i);

            // TODO
            // do something with data, for example:
            // output data to file
            outFile << "Distance:\n";
            for(int i = 0; i < h; i++)
            {
                for(int j = 0; j < w; j++)
                {
                    outFile << distData[i * w + j] << " ";
                }
                outFile << std::endl;
            }
        }

        //get point cloud
        //benewake::getPointCloud(distData, coord_x, coord_y, coord_z);

        // TODO
        // do something with point cloud data

        char key = get_char();
        switch(key)
        {
        case 'q':
        case 'Q':
            gRun = false;
            break;
        case 'c':
        case 'C':
            {
                system(STTY_DEF TTY_PATH);
                printf("Enter new IP address: ");
                setbuf(stdin, NULL);
                std::cin >> ip;

                if(benewake::changeIPAddress(sd, ip))
                {
                    printf("Press ENTER to exit.\r\n");
                    setbuf(stdin, NULL);
                    getchar();
                    return 0;
                }
            }
            break;
        default:
            break;
        }
    }
    outFile.close();
    benewake::closeDevice(sd);
    close(sd);

    system(STTY_DEF TTY_PATH);
    printf("Press ENTER to exit.\r\n");
    setbuf(stdin, NULL);
    getchar();
    return 0;
}
