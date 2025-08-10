#!/bin/bash
# bluetoothctlで接続済みデバイスを確認
if bluetoothctl info | grep -q "Connected: yes"; then
  exit 0
else
  exit 1
fi
