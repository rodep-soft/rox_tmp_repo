#!/usr/bin/bash

# Usage
# sudo ./setup.bash (automation dirにいる前提)


# rootで実行されてるかチェック
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root: sudo $0"
    exit 1
fi


# systemd service用のファイルを作成&追加
cat ./robot.service > /etc/systemd/system/robot.service

# もし.bashrcにaliasがなければsystemd service起動用のaliasを追加する
grep -qxF 'alias start="sudo systemctl start robot.service"' ~/.bashrc || \
    echo 'alias start="sudo systemctl start robot.service"' >> ~/.bashrc

# ====== systemd service configuration =====

# 書き換えた後、絶対に必要
systemctl daemon-reload

# Raspi起動時に自動でサービスを立ち上げるか(非推奨)
# systemctl enable robot.service




