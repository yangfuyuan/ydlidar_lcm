YDLIDAR LCM PACKAGE V1.2.3
=====================================================================

LCM node and test application for YDLIDAR

Visit EAI Website for more details about YDLIDAR.

How to build YDLIDAR LCM package
=====================================================================
    1) Clone this project to your computer folder
    2) Running cmake to build ydlidar_lcm and ydlidar_lcm_client
    3) Create the name "/dev/ydlidar" for YDLIDAR
    --$ roscd ydlidar_lcm/startup
    --$ sudo chmod 777 ./*
    --$ sudo sh initenv.sh

How to run YDLIDAR LCM package
=====================================================================

1. Run YDLIDAR node and view using test application
------------------------------------------------------------
./ydlidar_lcm

./ydlidar_client

You should see YDLIDAR's scan result in the console
	

