/etc/systemd/system/ros2_robot.service

sudo systemctl daemon-reload
sudo systemctl enable ros2_robot
sudo systemctl start ros2_robot


docker-compose.ymlのPathは都合がつくように書き換えること
