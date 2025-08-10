#!/bin/bash
# 例: lsusb や /dev/input/js0 が存在するかをチェック
if ls /dev/input/js0 &> /dev/null; then
  exit 0
else
  exit 1
fi
