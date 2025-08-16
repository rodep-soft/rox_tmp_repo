/etc/systemd/system/robot.service

sudo systemctl daemon-reload
sudo systemctl start robot


docker-compose.ymlのPathは都合がつくように書き換えること
aliasを書いておくと更に良いかも
systemd_serviceに書くときに絶対に名前を間違えないこと

sudo systemctl enable robot
enableしてちゃんと動くかはわからない。テストが必要。
時間に多少余力があるのであれば、sshしてからstartしたほうが確実であり安全

ex.
.bashrc
alias start = 'sudo systemctl start robot'