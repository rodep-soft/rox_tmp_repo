# デバイス周りの話

## PS5 Controller (DualSense)接続確認
まず、コントローラーのボタンを押して、Bluetooth接続モードになっていることを確認。  
  
次に、ターミナル上で
```
$ bluetoothctl
> devices
Controller... MACADDR(コロンでつながってるやつ)
> connect MACADDR
DualSense> exit
```
この時、DualSense>となっていれば接続ok。  

最後に、`ls /dev/input`でjs0, js1が存在することを確認。  
/*注意*/ Dockerに入っている時もコンテナの中から必ず確認すること。

