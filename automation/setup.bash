#!/usr/bin/bash

# rootに切り替える必要がある。権限の問題
su

# systemd service用のファイルを作成
touch /etc/systemd/system/robot.service
# robot.serviceの内容を追加
cat ./robot.service >> /etc/systemd/system/robot.service

# .bashrcにエイリアスを追加する
echo 'alias start="sudo systemctl start robot.service"' >> ~/.bashrc

# ====== systemd serviceの設定周り =====

# 書き換えた後、絶対に必要
systemctl daemon-reload

# Raspi起動時に自動でサービスを立ち上げるか(非推奨)
# systemctl enable robot.service




