# SSH周りの話  

## raspberrypi5へのssh
```
syntax:
ssh username@ipaddr

ex:
ssh rodep@raspberrypi
```

## 部室のwifi弱い問題
`ssh`の代替として`mosh`を使えば自動で再接続してくれて良いとか ...

### SSHできないとき
- 学校のwifiで通らない時が多い(windowsなら特に)
- Firewallの設定(tcp/22は空いているか)  
    ```
    $ sudo ufw status
    # ここでStatus Inactive又はallow 22(ssh)なら問題なし
    # blockしてるなら、
    $ sudo ufw disable
    # Firewallを立ち上げたままにしたいなら
    $ sudo ufw allow tcp/22
    ```
- IP addressがあっているか
 
