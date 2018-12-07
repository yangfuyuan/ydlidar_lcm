/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS Node Client
 *
 *  Copyright 2015 - 2017 EAI TEAM
 *  http://www.eaibot.com
 *
 */


#include "CYdLidar.h"
#include "simpleini/SimpleIni.h"
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/lidar_laser_t.hpp"
#include <memory>


using namespace ydlidar;
using namespace lcmtypes;
using namespace std;

std::vector<float> split(const std::string &s, char delim) {
  std::vector<float> elems;
  std::stringstream ss(s);
  std::string number;

  while (std::getline(ss, number, delim)) {
    elems.push_back(atof(number.c_str()));
  }

  return elems;
}



int main(int argc, char *argv[]) {
  printf("__   ______  _     ___ ____    _    ____  \n");
  printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
  printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
  printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
  printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
  printf("\n");
  fflush(stdout);

  ydlidar::init(argc, argv);
  std::shared_ptr<lcm::LCM> ydlidar_lcm(new lcm::LCM);


  if (!ydlidar_lcm->good()) {
    ydlidar::console.error("lcm is not good");
    return 0;
  }

  std::string port = "/dev/ydlidar";
  int baudrate = 115200;
  int samp_rate = 4;
  std::string frame_id = "laser_frame";
  std::string list = "";
  bool intensity = false;
  bool resolution_fixed = true;
  bool low_exposure = false;
  bool reversion = false;
  double angle_max = 180;
  double angle_min = -180;
  bool autoReconnect = true;
  std::vector<float> ignore_array;
  double max_range = 16.0;
  double  min_range = 0.08;
  double frequency = 7.0;
  CSimpleIniA ini;
  ini.SetUnicode();
  std::string ini_file = "lidar.ini";
  bool ini_exist = fileExists(ini_file.c_str());

  if (ini_exist ||  argc > 1) {
    if (argc > 1) {
      ini_file = (std::string)argv[1];
    }

    if (fileExists(ini_file.c_str())) {
      SI_Error rc = ini.LoadFile(ini_file.c_str());

      if (rc >= 0) {
        const char *pszValue = ini.GetValue("LIDAR", "serialPort", "");
        port = pszValue;
        pszValue = ini.GetValue("LIDAR", "ignoreArray", "");
        list  = pszValue;
        ignore_array = split(pszValue, ',');
        pszValue = ini.GetValue("LIDAR", "frame_id", "laser_frame");
        frame_id = pszValue;
        baudrate = ini.GetLongValue("LIDAR", "serialBaudrate", baudrate);
        samp_rate = ini.GetLongValue("LIDAR", "sampleRate", samp_rate);
        frequency = ini.GetLongValue("LIDAR", "scanFrequency", frequency);

        intensity = ini.GetBoolValue("LIDAR", "intensity", intensity);
        autoReconnect = ini.GetBoolValue("LIDAR", "autoReconnect", autoReconnect);
        low_exposure = ini.GetBoolValue("LIDAR", "exposure", low_exposure);
        resolution_fixed = ini.GetBoolValue("LIDAR", "fixedResolution", resolution_fixed);
        reversion = ini.GetBoolValue("LIDAR", "reversion", reversion);

        angle_max = ini.GetDoubleValue("LIDAR", "maxAngle", angle_max);
        angle_min = ini.GetDoubleValue("LIDAR", "minAngle", angle_min);
        max_range = ini.GetDoubleValue("LIDAR", "maxRange", max_range);
        min_range = ini.GetDoubleValue("LIDAR", "minRange", min_range);
      }
    }
  }

  CYdLidar laser;
  laser.setSerialPort(port);
  laser.setSerialBaudrate(baudrate);
  laser.setIntensities(intensity);//intensity
  laser.setAutoReconnect(autoReconnect);//hot plug
  laser.setMaxRange(max_range);
  laser.setMinRange(min_range);
  laser.setMaxAngle(angle_max);
  laser.setMinAngle(angle_min);
  laser.setReversion(reversion);
  laser.setFixedResolution(resolution_fixed);
  laser.setExposure(low_exposure);
  laser.setSampleRate(samp_rate);
  laser.setScanFrequency(frequency);
  laser.initialize();

  while (ydlidar::ok()) {
    bool hardError;
    LaserScan scan;

    if (laser.doProcessSimple(scan, hardError)) {
      lidar_laser_t scan_msg;
      scan_msg.nranges =  scan.ranges.size();
      scan_msg.ranges = scan.ranges;
      scan_msg.nintensities = scan.intensities.size();
      scan_msg.intensities = scan.intensities;
      scan_msg.utime = scan.self_time_stamp;
      scan_msg.angle_min = scan.config.min_angle;
      scan_msg.angle_max = scan.config.max_angle;
      scan_msg.angle_increment = scan.config.ang_increment;
      scan_msg.scan_time = scan.config.scan_time;
      scan_msg.time_increment = scan.config.time_increment;
      scan_msg.range_min = scan.config.min_range;
      scan_msg.range_max = scan.config.max_range;
      ydlidar_lcm->publish("scan", &scan_msg);
    } else {
      ydlidar::console.warning("Failed to get Lidar Data");
    }

  }

  laser.turnOff();
  laser.disconnecting();
  return 0;


}

