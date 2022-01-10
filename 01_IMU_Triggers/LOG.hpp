#ifndef LOG_HPP
#define LOG_HPP

#include <fstream>

// Update the file paths now
#include "IMU.hpp" //../include/

// Maybe I don't need these?
//#include "../include/LIDAR.hpp"
//#include "../include/LDS.hpp"

using namespace std;
class LOG
{
public:
    ofstream mLog; // Log File

    //LOG(IMU*, LIDAR*, LDS*);
    LOG(IMU*);
    ~LOG();
    void receive();
    void halt();
    void write(string);

private:
    IMU* mImu;     // IMU
    //LIDAR* mLidar; // LIDAR
    //LDS* mLds;     // LDS

    static void callback(void*);
};

#endif