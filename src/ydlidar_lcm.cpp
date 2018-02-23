/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS Node Client 
 *
 *  Copyright 2015 - 2017 EAI TEAM
 *  http://www.eaibot.com
 * 
 */


#include "ydlidar_driver.h"
#include "common.h"
#include "ini_parser.hpp"
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/lidar_laser_t.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>
#include <memory>
#include <cmath>
#include <unistd.h>

#if !defined(_MSC_VER)
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#endif

#if !defined(_MSC_VER)
#	define _access access
#endif

#define DELAY_SECONDS 4
#define DEG2RAD(x) ((x)*M_PI/180.)

using namespace ydlidar;
using namespace impl;
using namespace lcmtypes;

static int nodes_count = 720;
static float each_angle = 0.5;

bool running = false;

static std::string INI_NAME="~/.settings.ini";

std::shared_ptr<lcm::LCM> slcm;

void publish_scan(/*const std::shared_ptr<lcm::LCM>& slcm, */ node_info *nodes,  size_t node_count, uint64_t start, double scan_time, float angle_min, float angle_max, std::string frame_id, std::vector<int> ignore_array, double min_range , double max_range)
{
    lidar_laser_t scan_msg;

    int counts = node_count*((angle_max-angle_min)/360.0f);
    int angle_start = 180+angle_min;
    int node_start = node_count*(angle_start/360.0f);

    scan_msg.nranges = counts;
    scan_msg.nintensities = counts;
    scan_msg.ranges.resize(counts);
    scan_msg.intensities.resize(counts);
    float range = 0.0;
    float intensity = 0.0;
    int index = 0;
    for (size_t i = 0; i < node_count; i++) {
	range = (float)nodes[i].distance_q2/4.0f/1000;
	intensity = (float)(nodes[i].sync_quality >> 2);
        //reverse data
	index = node_count-1-i;

        if(ignore_array.size() != 0){
	    float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
            if(angle>180){
                angle=360-angle;
            }else{
                angle=-angle;
            }
	    for(uint16_t j = 0; j < ignore_array.size();j = j+2){
                if((ignore_array[j] < angle) && (angle <= ignore_array[j+1])){
		   range = 0.0;
		   break;
		}
	    }
	}

	if(range > max_range || range < min_range){
	    range = 0.0;
        }

	int pos = index - node_start ;
        if(0<= pos && pos < counts){
	    scan_msg.ranges[pos] =  range;
	    scan_msg.intensities[pos] = intensity;
	}

    }

    scan_msg.utime = start;
    float radian_min = DEG2RAD(angle_min);
    float radian_max = DEG2RAD(angle_max);
    scan_msg.angle_min = radian_min;
    scan_msg.angle_max = radian_max;
    scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)counts;
    scan_msg.scan_time = scan_time;
    scan_msg.time_increment = scan_time / (double)counts;
    scan_msg.range_min = min_range;
    scan_msg.range_max = max_range;

    slcm->publish("scan",&scan_msg);
}


std::vector<int> split(const std::string &s, char delim) {
    std::vector<int> elems;
    std::stringstream ss(s);
    std::string number;
    while(std::getline(ss, number, delim)) {
        elems.push_back(atoi(number.c_str()));
    }
    return elems;
}

bool getDeviceInfo(std::string port)
{
    if (!YDlidarDriver::singleton()){
        return false;
    }

    device_info devinfo;
    if (YDlidarDriver::singleton()->getDeviceInfo(devinfo) !=RESULT_OK){
        fprintf(stderr,"YDLIDAR get DeviceInfo Error\n" );
        return false;
    }
    int _samp_rate=4;
    std::string model;
    float freq = 7.0f;
    switch(devinfo.model){
            case 1:
                model="F4";
                _samp_rate=4;
                freq = 7.0;
                break;
            case 4:
                model="S4";
                _samp_rate=4;
                freq = 7.0;
                break;
            case 5:
                model="G4";
                freq = 7.0;
                    sampling_rate _rate;
                    YDlidarDriver::singleton()->getSamplingRate(_rate);
                    switch(_rate.rate){
                        case 0:
                            break;
                        case 1:
                            nodes_count = 1440;
                            each_angle = 0.25;
			    _samp_rate=8;
                            break;
                        case 2:
                            nodes_count = 1440;
                            each_angle = 0.25;
			    _samp_rate=9;
                            break;
                    }
                break;
            case 6:
                model="X4";
                _samp_rate=5;
                freq = 7.0;
                break;
            default:
                model = "Unknown";
    }

    uint16_t maxv = (uint16_t)(devinfo.firmware_version>>8);
    uint16_t midv = (uint16_t)(devinfo.firmware_version&0xff)/10;
    uint16_t minv = (uint16_t)(devinfo.firmware_version&0xff)%10;
    if(midv==0){
        midv = minv;
        minv = 0;
    }

    printf("[YDLIDAR INFO] Connection established in %s:\n"
            "Firmware version: %u.%u.%u\n"
            "Hardware version: %u\n"
            "Model: %s\n"
            "Serial: ",
            port.c_str(),
            maxv,
            midv,
            minv,
            (uint16_t)devinfo.hardware_version,
            model.c_str());

    for (int i=0;i<16;i++){
        printf("%01X",devinfo.serialnum[i]&0xff);
    }
    printf("\n");

    printf("[YDLIDAR INFO] Current Sampling Rate : %dK\n" , _samp_rate);
    printf("[YDLIDAR INFO] Current Scan Frequency : %fHz\n" , freq);

    return true;

}


bool getDeviceHealth()
{
    if (!YDlidarDriver::singleton()){
        return false;
    }

    result_t op_result;
    device_health healthinfo;

    op_result = YDlidarDriver::singleton()->getHealth(healthinfo);
    if (op_result == RESULT_OK) { 
        printf("[YDLIDAR INFO] YDLIDAR running correctly! The health status: %s\n", healthinfo.status==0?"well":"bad");
        
        if (healthinfo.status == 2) {
            fprintf(stderr, "YDLIDAR internal error detected. Please reboot the device to retry.\n");
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "cannot retrieve YDLIDAR health code: %x\n", op_result);
        return false;
    }

}


bool fileExists(const std::string filename){
    return 0 == _access(filename.c_str(), 0x00 ); // 0x00 = Check for existence only!
}

static void Stop(int signo)   
{   
    running = false;
    signal(signo, SIG_DFL);
    printf("[YDLIDAR INFO] Now YDLIDAR is stopping .......\n");
  
     
} 


int main(int argc, char * argv[]) {

    std::string port = "/dev/ttyUSB0";
    int baudrate = 115200;
    std::string frame_id = "laser_frame";
    bool angle_fixed = true;
    bool intensities_ = false;
    bool low_power = false;
    double angle_max= 180;
    double angle_min = -180;
    result_t op_result;
    std::string list;
    std::vector<int> ignore_array;  
    double max_range = 16.0;
    double min_range = 0.08;

     slcm.reset(new lcm::LCM);

    if(!slcm->good()){
    	fprintf(stderr, "lcm is not good()\n");
	return 0;
    }

#if !defined(_MSC_VER)
 char *homedir=NULL;
  homedir = getenv("HOME");
  if(homedir == NULL){
    homedir = getpwuid(getuid())->pw_dir;
  }
  INI_NAME = (std::string)homedir + "/.settings.ini";
#endif


  for(int i=0; i< argc; i++){
	std::string str = argv[i];
	if(str.find(".ini") != string::npos){
		INI_NAME = str;
		break;
	}
  }

  if(fileExists(INI_NAME)){
  	ini_parser parser_instance(INI_NAME);
        const std::string section = "LASER";
	port = parser_instance.get_string("port","/dev/ttyUSB0", section);
	frame_id = parser_instance.get_string("frame_id","laser_frame", section);
	list = parser_instance.get_string("ignore_array","", section);
	baudrate = parser_instance.get_int("baudrate",115200, section);
	angle_fixed = parser_instance.get_bool("angle_fixed",true, section);
	low_power = parser_instance.get_bool("low_power",false, section);
	intensities_ = parser_instance.get_bool("intensities", false,section);
	angle_max = parser_instance.get_double("angle_max",180, section);
	angle_min = parser_instance.get_double("angle_min",-180, section);
	max_range = parser_instance.get_double("max_range", 16.0, section);
	min_range = parser_instance.get_double("min_range",0.08, section);
  }

    if(ignore_array.size()%2){
        fprintf(stderr,"ignore array is odd need be even\n");
    }

    for(uint16_t i =0 ; i < ignore_array.size();i++){
        if(ignore_array[i] < -180 && ignore_array[i] > 180){
            fprintf(stderr,"ignore array should be between -180 and 180\n");
        }
    }

    YDlidarDriver::initDriver(); 
    if (!YDlidarDriver::singleton()) {
        fprintf(stderr,"YDLIDAR Create Driver fail, exit\n");
        return -2;
    }

    signal(SIGINT, Stop); 
    signal(SIGTERM, Stop);

    printf("[YDLIDAR INFO] Current SDK Version: %s\n",YDlidarDriver::singleton()->getSDKVersion().c_str());

    op_result = YDlidarDriver::singleton()->connect(port.c_str(), (uint32_t)baudrate);
    if (op_result != RESULT_OK) {
        int seconds=0;
        while(seconds <= DELAY_SECONDS){
            sleep(2);
            seconds = seconds + 2;
            YDlidarDriver::singleton()->disconnect();
            op_result = YDlidarDriver::singleton()->connect(port.c_str(), (uint32_t)baudrate);
            printf("[YDLIDAR INFO] Try to connect the port %s again  after %d s .\n", port.c_str() , seconds);
            if(op_result==RESULT_OK){
                break;
            }
        }
        
        if(seconds > DELAY_SECONDS){
            fprintf(stderr,"YDLIDAR Cannot bind to the specified serial port %s\n" , port.c_str());
	    YDlidarDriver::singleton()->disconnect();
	    YDlidarDriver::done();
	    slcm.reset();
            return -1;
        }
    }

    printf("[YDLIDAR INFO] Connected to YDLIDAR on port %s at %d \n" , port.c_str(), baudrate);
    if(!getDeviceHealth()||!getDeviceInfo(port)){
        YDlidarDriver::singleton()->disconnect();
        YDlidarDriver::done();
    	slcm.reset();
        return -1;
    }
    YDlidarDriver::singleton()->setIntensities(intensities_);
    if(intensities_){
	scan_power power;
	int cnt = 0;
	    while(YDlidarDriver::singleton()->setLowPower(power) == RESULT_OK&&cnt<3){
		if(power.power != low_power){
			fprintf(stderr,"set POWER MODEL SUCCESS!!!\n");
			break;
		}
		cnt++;
	    }
	if(cnt>=3){
		fprintf(stderr,"set LOW POWER MODEL FALIED!!!\n");
	}
	
    }

    result_t ans=YDlidarDriver::singleton()->startScan();
    if(ans != RESULT_OK){
        ans = YDlidarDriver::singleton()->startScan();
        if(ans != RESULT_OK){
            fprintf(stderr,"start YDLIDAR is failed! Exit!! ......\n");
            YDlidarDriver::singleton()->disconnect();
            YDlidarDriver::done();
	    slcm.reset();
            return 0;
        }
    }
    uint64_t start_scan_time, end_scan_time;
    printf("[YDLIDAR INFO] Now YDLIDAR is scanning ......\n");
    double scan_duration;
    running = true;

    while (running) {
        try{
            node_info nodes[nodes_count];
            size_t   count = _countof(nodes);

            start_scan_time = getus();
            op_result = YDlidarDriver::singleton()->grabScanData(nodes, count);
            end_scan_time = getus();

            if (op_result == RESULT_OK) {

		if(nodes[0].stamp > 0){
	   	 	start_scan_time = nodes[0].stamp;

	    	}
	    	scan_duration = (end_scan_time - start_scan_time);

                op_result = YDlidarDriver::singleton()->ascendScanData(nodes, count);
            
                if (op_result == RESULT_OK) {
			each_angle = 360.0/count;
		    if(angle_fixed){
			node_info all_nodes[count];
                         memset(all_nodes, 0, count*sizeof(node_info));
                         for(size_t i = 0; i < count; i++) {
                             if (nodes[i].distance_q2 != 0) {
                                 float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                                 int inter =(int)( angle / each_angle );
                                 float angle_pre = angle - inter * each_angle;
                                 float angle_next = (inter+1) * each_angle - angle;
                                 if(angle_pre < angle_next){
                                     if(inter < count){
                                         all_nodes[inter]=nodes[i];
                                     }
                                 }else{
                                     if(inter < count-1){
                                         all_nodes[inter+1]=nodes[i];
                                     }

                                 }

                             }
                         }
	
			 publish_scan(all_nodes, count, start_scan_time, scan_duration, angle_min, angle_max, frame_id, ignore_array, min_range , max_range);

                    } else {
                        int start_node = 0, end_node = 0;
                        int i = 0;
                        while (nodes[i++].distance_q2 == 0&&i<count);
                        start_node = i-1;
                        i = count -1;
                        while (nodes[i--].distance_q2 == 0&&i>=0);
                        end_node = i+1;

                        angle_min = (float)(nodes[start_node].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
                        angle_max = (float)(nodes[end_node].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;

                        publish_scan(&nodes[start_node], end_node-start_node +1,  start_scan_time, scan_duration, angle_min, angle_max, frame_id, ignore_array, min_range , max_range);
                   }
                }
            }
	}catch(std::exception &e){//
            std::cout<<"Unhandled Exception: " << e.what()<<std::endl;
            break;
	}catch(...){//anthor exception
            fprintf(stderr,"Unhandled Exception:Unknown ");
            break;
	}
    }

    YDlidarDriver::singleton()->disconnect();
    printf("[YDLIDAR INFO] Now YDLIDAR is stopping .......\n");
    YDlidarDriver::done();
    slcm.reset();
    return 0;
}
