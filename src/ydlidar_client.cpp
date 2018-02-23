/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS Node Client 
 *
 *  Copyright 2015 - 2017 EAI TEAM
 *  http://www.eaibot.com
 * 
 */

#include <stdio.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/lidar_laser_t.hpp"
#include <cmath>
using namespace lcmtypes;

#define RAD2DEG(x) ((x)*180./M_PI)

class Handler 
{
    public:
        ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const lidar_laser_t* msg)
        {
            int i;
            printf("Received message on channel \"%s\":\n", chan.c_str());
            printf("  timestamp   = %lld\n", (long long)msg->utime);
            printf("  ranges:\n");
 		for(int i = 0; i < msg->nranges; i++) {
        	float degree = RAD2DEG(msg->angle_min + msg->angle_increment * i);
        	printf("[YDLIDAR INFO]: angle-distance : [%f, %f]\n", degree, msg->ranges[i]);
    		}
 
        }
};

int main(int argc, char** argv)
{
    lcm::LCM lcm;

    if(!lcm.good())
        return 1;

    Handler handlerObject;
    lcm.subscribe("scan", &Handler::handleMessage, &handlerObject);

    while(0 == lcm.handle());

    return 0;
}
