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

#define DELAY_SECONDS 2
#define DEG2RAD(x) ((x)*M_PI/180.)

using namespace ydlidar;
using namespace impl;
using namespace lcmtypes;

static int nodes_count = 720;
static float each_angle = 0.5;

bool running = false;
int type = 0;
int print = 0;

static std::string INI_NAME="~/.settings.ini";

std::shared_ptr<lcm::LCM> slcm;

void publish_scan(/*const std::shared_ptr<lcm::LCM>& slcm, */ node_info *nodes,  size_t node_count, uint64_t start, double scan_time, float angle_min, float angle_max, std::string frame_id, std::vector<double> ignore_array, double min_range , double max_range)
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
        //index = node_count-1-i;
        if (i<node_count/2) {
            index = node_count/2-1-i;
        } else {
            index =node_count-1-(i-node_count/2);
        }



	if(range > max_range || range < min_range){
	    range = 0.0;
        }

	int pos = index - node_start ;
        if(0<= pos && pos < counts){
            if (ignore_array.size() != 0) {
                float angle = angle_min + pos*(angle_max - angle_min)/(double)counts;
                for (uint16_t j = 0; j < ignore_array.size();j = j+2) {
                    if((ignore_array[j] < angle) && (angle <= ignore_array[j+1])){
                        range = 0.0;
                        break;
                    }
                }
            }


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


std::vector<double> split(const std::string &s, char delim) {
    std::vector<double> elems;
    std::stringstream ss(s);
    std::string number;
    while(std::getline(ss, number, delim)) {
        elems.push_back(atoi(number.c_str()));
    }
    return elems;
}

bool getDeviceInfo(std::string port, int& samp_rate, double _frequency, int baudrate) {

    if (!YDlidarDriver::singleton()){
        return false;
    }

    device_info devinfo;
    if (YDlidarDriver::singleton()->getDeviceInfo(devinfo,300) !=RESULT_OK) {
        if(print==3)
             fprintf(stderr,"YDLIDAR get DeviceInfo Error\n" );
        return false;
    }
    sampling_rate _rate;
    scan_frequency _scan_frequency;
    int _samp_rate=4;
    std::string model;
    float freq = 7.0f;
    int hz = 0;
    type = devinfo.model;
    switch (devinfo.model) {
            case 1:
                model="F4";
                _samp_rate=4;
                freq = 7.0;
                break;
            case 4:
                model="S4";
                if (baudrate==153600) {
                    model="S4Pro";
                }
                _samp_rate=4;
                freq = 7.0;
                break;
            case 5:
                model="G4";
                YDlidarDriver::singleton()->getSamplingRate(_rate);
                switch (samp_rate) {
                    case 4:
                        _samp_rate=0;
                        break;
                    case 8:
                        _samp_rate=1;
                        break;
                    case 9:
                        _samp_rate=2;
                        break;
                    default:
                        _samp_rate = _rate.rate;
                        break;
                }
                while (_samp_rate != _rate.rate) {
                    YDlidarDriver::singleton()->setSamplingRate(_rate);
                }
                switch (_rate.rate) {
                    case 0:
                        _samp_rate=4;
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

                if (YDlidarDriver::singleton()->getScanFrequency(_scan_frequency) != RESULT_OK){
                     fprintf(stderr,"YDLIDAR get frequency Error\n" );
                    return false;
                }
                freq = _scan_frequency.frequency/100;
                hz = _frequency - freq;
                if (hz>0) {
                    while (hz != 0) {
                        YDlidarDriver::singleton()->setScanFrequencyAdd(_scan_frequency);
                        hz--;
                    }
                    freq = _scan_frequency.frequency/100.0f;
                } else {
                    while (hz != 0) {
                        YDlidarDriver::singleton()->setScanFrequencyDis(_scan_frequency);
                        hz++;
                    }
                    freq = _scan_frequency.frequency/100.0f;
                }
                break;
            case 6:
                model="X4";
                _samp_rate=5;
                freq = 7.0;
                break;
            case 8:
                model="F4Pro";
                YDlidarDriver::singleton()->getSamplingRate(_rate);
                switch (samp_rate) {
                case 4:
                    _samp_rate=0;
                    break;
                case 6:
                    _samp_rate=1;
                    break;
                default:
                    _samp_rate=_rate.rate;
                    break;
                }

                while (_samp_rate != _rate.rate) {
                    YDlidarDriver::singleton()->setSamplingRate(_rate);
                }
                switch (_rate.rate) {
                    case 0:
                        _samp_rate=4;
                        break;
                    case 1:
                        nodes_count = 1440;
                        each_angle = 0.25;
                        _samp_rate=6;
                        break;
                }

                if (YDlidarDriver::singleton()->getScanFrequency(_scan_frequency) != RESULT_OK) {
                    fprintf(stderr,"YDLIDAR get frequency Error\n" );
                    return false;
                }
                freq = _scan_frequency.frequency/100;
                hz = _frequency - freq;
                if (hz>0) {
                    while (hz != 0) {
                        YDlidarDriver::singleton()->setScanFrequencyAdd(_scan_frequency);
                        hz--;
                    }
                    freq = _scan_frequency.frequency/100.0f;
                } else {
                    while (hz != 0) {
                        YDlidarDriver::singleton()->setScanFrequencyDis(_scan_frequency);
                        hz++;
                    }
                    freq = _scan_frequency.frequency/100.0f;
                }
                break;
                case 9:
                model ="G4C";
                 _samp_rate=4;
                freq = 7.0;
                break;
            default:
                model = "Unknown";
    }

    samp_rate = _samp_rate;
    uint16_t maxv = (uint16_t)(devinfo.firmware_version>>8);
    uint16_t midv = (uint16_t)(devinfo.firmware_version&0xff)/10;
    uint16_t minv = (uint16_t)(devinfo.firmware_version&0xff)%10;

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

    for (int i=0;i<16;i++) {
        printf("%01X",devinfo.serialnum[i]&0xff);
    }
    printf("\n");

    printf("[YDLIDAR INFO] Current Sampling Rate : %dK\n" , _samp_rate);
    printf("[YDLIDAR INFO] Current Scan Frequency : %fHz\n" , freq);


    return true;

}


bool getDeviceHealth() {
    if (!YDlidarDriver::singleton()) {
        return false;
    }

    result_t op_result;
    device_health healthinfo;

    op_result = YDlidarDriver::singleton()->getHealth(healthinfo,300);
    if (op_result == RESULT_OK) {
        printf("[YDLIDAR INFO] YDLIDAR running correctly! The health status: %s\n", healthinfo.status==0?"well":"bad");

        if (healthinfo.status == 2) {
            if (print ==3)
                fprintf(stderr,"YDLIDAR internal error detected. Please reboot the device to retry.");
            return false;
        } else {
            return true;
        }

    } else {
        if (print ==3)
            fprintf(stderr,"cannot retrieve YDLIDAR health code: %x", op_result);
        return false;
    }

}


bool fileExists(const std::string filename) {
    return 0 == _access(filename.c_str(), 0x00 ); // 0x00 = Check for existence only!
}

static void Stop(int signo) {
    running = false;
    signal(signo, SIG_DFL);
    printf("[YDLIDAR INFO] Now YDLIDAR is stopping .......\n");
  
     
} 


int main(int argc, char * argv[]) {

    std::string port = "/dev/ydlidar";
    int baudrate = 115200;
    int samp_rate = 4;
    std::string frame_id = "laser_frame";
    bool angle_fixed = true;
    bool intensities_ = false;
    bool resolution_fixed = true;
    bool heartbeat = false;
    bool low_exposure = false;
    bool reversion = false;
    double angle_max= 180;
    double angle_min = -180;
    result_t op_result;
    std::string list;
    std::vector<double> ignore_array;
    double max_range = 16.0;
    double min_range = 0.08;
    double frequency = 7.0;


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
        samp_rate = parser_instance.get_int("samp_rate",4, section);
	angle_fixed = parser_instance.get_bool("angle_fixed",true, section);
        resolution_fixed = parser_instance.get_bool("resolution_fixed", true,section);
        heartbeat = parser_instance.get_bool("heartbeat", false,section);
        low_exposure = parser_instance.get_bool("low_exposure", false,section);
	angle_max = parser_instance.get_double("angle_max",180, section);
	angle_min = parser_instance.get_double("angle_min",-180, section);
	max_range = parser_instance.get_double("max_range", 16.0, section);
	min_range = parser_instance.get_double("min_range",0.08, section);
        frequency = parser_instance.get_double("frequency",7.0, section);

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

    if (frequency<5) {
        frequency = 7.0;
    }
    if (frequency>12) {
        frequency = 12;
    }
    if (angle_max < angle_min) {
        double temp = angle_max;
        angle_max = angle_min;
        angle_min = temp;
    }

    signal(SIGINT, Stop); 
    signal(SIGTERM, Stop);

    //check lidar type
    std::map<int, bool> checkmodel;
    checkmodel.insert(std::map<int, bool>::value_type(115200, false));
    checkmodel.insert(std::map<int, bool>::value_type(128000, false));
    checkmodel.insert(std::map<int, bool>::value_type(153600, false));
    checkmodel.insert(std::map<int, bool>::value_type(230400, false));
    printf("[YDLIDAR INFO] Current SDK Version: %s\n",YDlidarDriver::singleton()->getSDKVersion().c_str());

    again:
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

    bool ret = getDeviceHealth();
    if(!getDeviceInfo(port,samp_rate,frequency,baudrate)&&!ret){
        checkmodel[baudrate] = true;
        map<int,bool>::iterator it;
        for(it=checkmodel.begin();it!=checkmodel.end();++it){
            if(it->second)
                continue;
            print++;
            YDlidarDriver::singleton()->disconnect();
            YDlidarDriver::done();
            YDlidarDriver::initDriver();
            if (!YDlidarDriver::singleton()) {
                fprintf(stderr,"YDLIDAR Create Driver fail, exit");
                return -1;
            }
            baudrate = it->first;
            goto again;

        }
        fprintf(stderr,"[YDLIDAR ERROR] Unsupported lidar\n");
        YDlidarDriver::singleton()->disconnect();
        YDlidarDriver::done();
        slcm.reset();
        return -1;
    }

    printf("[YDLIDAR INFO] Connected to YDLIDAR on port %s at %d \n" , port.c_str(), baudrate);
    print = 0;
    if (type !=4) {
        intensities_ = false;
    } else {
        intensities_ = true;
        if (baudrate != 153600) {
            intensities_ = false;
        }
    }
    YDlidarDriver::singleton()->setIntensities(intensities_);

    if (intensities_) {
        scan_exposure exposure;
        int cnt = 0;
        while ((YDlidarDriver::singleton()->setLowExposure(exposure) == RESULT_OK) && (cnt<3)) {
            if (exposure.exposure != low_exposure) {
                fprintf(stdout,"set EXPOSURE MODEL SUCCESS!!!");
                break;
            }
            cnt++;
        }

        if (cnt>=3) {
            fprintf(stderr,"set LOW EXPOSURE MODEL FALIED!!!");
        }

    }

    if (type == 5 || type == 8 || type == 9) {
        scan_heart_beat beat;
        if (type != 8)
           reversion=true;
       result_t ans = YDlidarDriver::singleton()->setScanHeartbeat(beat);
       if (heartbeat) {
           if (beat.enable&& ans == RESULT_OK) {
               ans = YDlidarDriver::singleton()->setScanHeartbeat(beat);
           }
           if (!beat.enable&& ans == RESULT_OK ) {
               YDlidarDriver::singleton()->setHeartBeat(true);
           }
       } else {
           if (!beat.enable&& ans == RESULT_OK) {
               ans = YDlidarDriver::singleton()->setScanHeartbeat(beat);
           }
           if (beat.enable && ans==RESULT_OK) {
               YDlidarDriver::singleton()->setHeartBeat(false);
           }

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
    int max_nodes_count = nodes_count;


    while (running) {
        try {
            node_info nodes[nodes_count];
            size_t   count = _countof(nodes);

            start_scan_time = getCurrentTime();
            op_result = YDlidarDriver::singleton()->grabScanData(nodes, count);
            end_scan_time = getCurrentTime();

            if (op_result == RESULT_OK) {
                op_result = YDlidarDriver::singleton()->ascendScanData(nodes, count);
            
                if (op_result == RESULT_OK) {
                    if (angle_fixed) {
                        if (!resolution_fixed) {
                            max_nodes_count = count;
                        } else {
                            max_nodes_count = nodes_count;
                        }


                        each_angle = 360.0/max_nodes_count;
                        node_info all_nodes[max_nodes_count];
                        memset(all_nodes, 0, max_nodes_count*sizeof(node_info));

                        uint64_t end_time = nodes[0].stamp;
                        uint64_t start_time = nodes[0].stamp;

                         for(size_t i = 0; i < count; i++) {
                             if (nodes[i].distance_q2 != 0) {
                                 float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                                 if (reversion) {
                                     angle=angle+180;
                                     if(angle>=360){ angle=angle-360;}
                                     nodes[i].angle_q6_checkbit = ((uint16_t)(angle * 64.0f)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT;
                                 }
                                 int inter =(int)( angle / each_angle );
                                 float angle_pre = angle - inter * each_angle;
                                 float angle_next = (inter+1) * each_angle - angle;
                                 if (angle_pre < angle_next) {
                                     if (inter < count) {
                                         all_nodes[inter]=nodes[i];
                                     }
                                 } else {
                                     if (inter < count-1) {
                                         all_nodes[inter+1]=nodes[i];
                                     }

                                 }

                             }

                             if (nodes[i].stamp < start_time) {
                                 start_time = nodes[i].stamp;
                             }
                             if (nodes[i].stamp > end_time) {
                                 end_time = nodes[i].stamp;

                             }
                         }

                         start_scan_time = start_time;
                         end_scan_time = end_time;
                         scan_duration = (end_scan_time - start_scan_time);

	
                         publish_scan(all_nodes, max_nodes_count, start_scan_time, scan_duration, angle_min, angle_max, frame_id, ignore_array, min_range , max_range);

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
        }catch (std::exception &e) {//
            std::cout<<"Unhandled Exception: " << e.what()<<std::endl;
            break;
        } catch(...) {//anthor exception
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
