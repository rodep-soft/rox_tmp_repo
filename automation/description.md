/etc/systemd/system/ros2_robot.service

sudo systemctl daemon-reload
sudo systemctl start ros2_robot


docker-compose.ymlのPathは都合がつくように書き換えること
aliasを書いておくと更に良いかも
systemd_serviceに書くときに絶対に名前を間違えないこと

sudo systemctl enable ros2_robot
enableしてちゃんと動くかはわからない。テストが必要。
時間に多少余力があるのであれば、sshしてからstartしたほうが確実であり安全

ex.
.bashrc
alias start = 'sudo systemctl start ros2_robot'