#include "undistort.h"
extern "C"
{
    namespace benewake
    {
        double gCameraMatrix[9], gDistCoeffs[4];
        unsigned short gRawDistMatrix[HEIGHT * WIDTH] = {0}, gRawAmpMatrix[HEIGHT * WIDTH] = {0};

        bool initDevice(int &_sd, float *_mapX, float *_mapY, int _height, int _width);
        bool getDistanceData(int _sd, unsigned short *_dist, unsigned short *_amp, float *_mapX, float *_mapY, int _height, int _width);
        bool closeDevice(int &_sd);
        bool getPointCloud(unsigned short *_dist, float *_coordX, float *_coordY, float *_coordZ, int _height, int _width);
    }
}
