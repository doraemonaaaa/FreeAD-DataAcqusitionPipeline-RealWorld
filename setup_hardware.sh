
sudo rm -rf /dev/motor /dev/imu
sudo ln -s /dev/ttyUSB0 /dev/motor
sudo ln -s /dev/ttyUSB1 /dev/imu

sudo rm -rf /dev/CAM_FRONT /dev/CAM_FRONT_LEFT /dev/CAM_BACK_LEFT /dev/CAM_BACK /dev/CAM_BACK_RIGHT /dev/CAM_FRONT_RIGHT
sudo ln -s /dev/video4 /dev/CAM_FRONT
sudo ln -s /dev/video10 /dev/CAM_FRONT_LEFT
sudo ln -s /dev/video8 /dev/CAM_BACK_LEFT
sudo ln -s /dev/video2 /dev/CAM_BACK
sudo ln -s /dev/video6 /dev/CAM_BACK_RIGHT
sudo ln -s /dev/video0 /dev/CAM_FRONT_RIGHT


