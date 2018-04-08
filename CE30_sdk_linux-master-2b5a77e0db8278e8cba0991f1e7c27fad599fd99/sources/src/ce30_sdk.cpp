#include "../include/ce30_sdk.h"

namespace benewake
{
    unsigned char gDistData[TOTALDATA] = {0};
    unsigned char gAmpData[TOTALDATA] = {0};
    unsigned char gRecv[4] = {0};
    const int gHeight = 24, gWidth = 660;
    float gMapX[gHeight * gWidth] = {0}, gMapY[gHeight * gWidth] = {0};

    bool initDevice(int &_sd, char *_ip)
    {
        gCameraMatrix[0] = 149.905;
        gCameraMatrix[1] = 0;
        gCameraMatrix[2] = 159.5;
        gCameraMatrix[4] = 150.24;
        gCameraMatrix[5] = 11.5;
        gCameraMatrix[8] = 1.0;

        gDistCoeffs[0] = -0.059868055;
        gDistCoeffs[1] = -0.001303471;
        gDistCoeffs[2] = 0.010260736;
        gDistCoeffs[3] = -0.006102915;

        init_fisheye_map(gCameraMatrix, gDistCoeffs, gMapX, gMapY, gHeight, gWidth);

        int ret = 0;
        ret = TCP_connect(_sd, _ip);
        if(ret == -1)
        {
            printf("ERROR: Failed to connect device!\r\n");
            return false;
        }

        ret = send_command(_sd, kSetROI, strlen(kSetROI));
        if(ret < 0)
        {
            printf("ERROR: Failed to send set roi command!\r\n");
            return false;
        }
        if(recv_data(_sd, gRecv, 4) < 0)
        {
            printf("ERROR: Failed to receive set roi response!\r\n");
            return false;
        }
        else if(gRecv[0] != 0x00 && gRecv[1] != 0x00 && gRecv[2] != 0x00 && gRecv[3] != 0x00)
        {
            printf("ERROR: Set ROI command is rejected!\r\n");
        }

        ret = send_command(_sd, kStart, strlen(kStart));
        if(ret < 0)
        {
            printf("ERROR: Failed to send start command!\r\n");
            return false;
        }

        return true;
    }

    bool getDistanceData(int _sd, unsigned short *_dist, unsigned short *_amp)
    {
        if(recv_data(_sd, gDistData, TOTALDATA) < 0)
        {
            printf("ERROR: Failed to receive distance data!\r\n");
            return false;
        }
        if(recv_data(_sd, gAmpData, TOTALDATA) < 0)
        {
            printf("ERROR: Failed to receive amp data!\r\n");
            return false;
        }
        recv_data(_sd, gRecv, 3);

        const unsigned short* raw_dist = reinterpret_cast<const unsigned short*>(&gDistData[0]);
        const unsigned short* raw_amp = reinterpret_cast<const unsigned short*>(&gAmpData[0]);
        for(int i = 0; i < HEIGHT; i++)
        {
            for(int j = 0; j < WIDTH; j++)
            {
                gRawDistMatrix[i * WIDTH + j] = raw_dist[i * WIDTH + (WIDTH - 1 - j)];
                gRawAmpMatrix[i * WIDTH + j] = raw_amp[i * WIDTH + (WIDTH - 1 - j)];
            }
        }
        remap(gRawDistMatrix, _dist, gMapX, gMapY, gHeight, gWidth);
        remap(gRawAmpMatrix, _amp, gMapX, gMapY, gHeight, gWidth);

        return true;
    }

    bool closeDevice(int &_sd)
    {
        if(send_command(_sd, kStop, strlen(kStop)) < 0)
        {
            printf("ERROR: Failed to send stop command! Please cut off device's power directly.\r\n");
            return false;
        }
        usleep(100000);
        if(send_command(_sd, kDisconnect, strlen(kDisconnect)) < 0)
        {
            printf("ERROR: Failed to send disconnect command! Please cut off device's power directly.\r\n");
            return false;
        }

        printf("Device is successfully stopped and disconnected!\r\n");
        return true;
    }

    bool getPointCloud(unsigned short *_dist, float *_coordX, float *_coordY, float *_coordZ)
    {
        float center_x = (gWidth - 1) / 2, center_y = (gHeight - 1) / 2;
        float tmp_x = 0, tmp_y = 0;
        for(int i = 0; i < gHeight; i++)
        {
            for(int j = 0; j < gWidth; j++)
            {
                if(_dist[i * gWidth + j] != 0)
                {
                    tmp_x = (j - center_x) / gCameraMatrix[0];
                    tmp_y = (center_y - i) / gCameraMatrix[4];

                    _coordZ[i * gWidth + j] = (float)_dist[i * gWidth + j];
                    _coordX[i * gWidth + j] = tmp_x * _coordZ[i * gWidth + j];
                    _coordY[i * gWidth + j] = tmp_y * _coordZ[i * gWidth + j];
                }
                else
                {
                    _coordZ[i * gWidth + j] = 0.0;
                    _coordX[i * gWidth + j] = 0.0;
                    _coordY[i * gWidth + j] = 0.0;
                }
            }
        }
        return true;
    }

    bool changeIPAddress(int _sd, char *_newIP)
    {
        send_command(_sd, kStop, 50);

        const char *delim = " .";
        char *field1 = strtok(_newIP, delim);
        char *field2 = strtok(NULL, delim);
        char *field3 = strtok(NULL, delim);
        char *field4 = strtok(NULL, delim);

        if(field1 == NULL || field2 == NULL || field3 == NULL || field4 == NULL)
        {
            printf("ERROR: Failed to change IP address! The new IP address has incorrect format!\r\n");
            send_command(_sd, kStart, 50);
            return false;
        }
        else
        {
            int f1 = char_to_int(field1);
            int f2 = char_to_int(field2);
            int f3 = char_to_int(field3);
            int f4 = char_to_int(field4);

            if(f1 < 0 || f1 > 255 ||
               f2 < 0 || f2 > 255 ||
               f3 < 0 || f3 > 255 ||
               f4 < 0 || f4 > 255)
            {
                printf("ERROR: Failed to change IP address! The fields must be value from 0 to 255!\r\n");
                send_command(_sd, kStart, 50);
                return false;
            }
            else
            {
                char setIP[51] = "ipconfig ";
                strcat(setIP, field1);
                strcat(setIP, " ");
                strcat(setIP, field2);
                strcat(setIP, " ");
                strcat(setIP, field3);
                strcat(setIP, " ");
                strcat(setIP, field4);
                int fillSpaceNum = 50 - 12 - strlen(field1) - strlen(field2) - strlen(field3) - strlen(field4);
                for(int i = 0; i < fillSpaceNum; i++)
                {
                    strcat(setIP, " ");
                }

                if(send_command(_sd, setIP, 50) < 0)
                {
                    printf("ERROR: Failed to send change IP command!\r\n");
                    return false;
                }
                recv_data(_sd, gRecv, 4);
                if(gRecv[0] == 0x00 && gRecv[1] == 0x00 && gRecv[2] == 0x00 && gRecv[3] == 0x00)
                {
                    printf("New IP address is set! The LiDAR is rebooting!\r\n");
                    return true;
                }
                else
                {
                    printf("ERROR: Failed to change IP address! The command is rejected.\r\n");
                    send_command(_sd, kStart, 50);
                    return false;
                }
            }
        }
    }
}
