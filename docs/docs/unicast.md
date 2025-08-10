# Discovery ServerでUnicast通信する方法

## Why Unicast?
学校のWifi、またTailscale(vpn)でros2ではデフォルトになっているmulticast通信が通らないため、Unicast通信に変更する必要がある.

## Precaution
Unicast用に環境変数を設定していると、普通のmulticastのros通信が無効化されるため、それに気づかないと詰む可能性がある。扱いは慎重に、知識がないならやらない。おとなしく独自wifiをたてるか、テザリングする。

## How

### Server側
```
$ fastdds discovery --server-id 0 
$ export ROS_DISCOVERY_SERVER=ip address(server):11811
```

### Client側
```
$ export ROS_DISCOVERY_SERVER=ip address(server):11811    
```

### よくあるQ
- ServerのIPアドレスがわからない
    - Serverで`hostname -I`
    - Tailscale使ってるなら'tailscale status'またはブラウザから確認


