# デバイス周りの話

## PS5 Controller (DualSense)接続確認
まず、コントローラーのボタンを押して、Bluetooth接続モードになっていることを確認。  
  
次に、ターミナル上で
```
$ bluetoothctl
blue> devices
Controller... MACADDR(コロンでつながってるやつ)
blue> connect MACADDR
DualSense> exit
```
この時、DualSense>となっていれば接続ok。  

流石にないと思うが、Bluetoothが死んでるっぽいときは
`sudo systemctl status bluetooth`でactiveになっていることを確認する.



最後に、`ls /dev/input`でjs0, js1が存在することを確認。  
/*注意*/ Dockerに入っている時もコンテナの中から必ず確認すること。

## 足回りモーター接続確認
モーター駆動する前に黒い基盤に電源とRS485ラインがささっていることを確認。  
ターミナル上で
```
$ ls /dev/ttyACM*
# /dev/ttyACM0, /dev/ttyACM1等表示されれば問題なく認識している
```
実際に動かすときはdockerを介してインターフェース名が変わっていないか、また、コードの中に記述されているポート名と本当にあっているか必ず確認すること。
