# Docker/Docker-compose周りについて

## Setup
プロジェクトのroot(Dockerfile,docker-compose.yml)がある場所で`docker compose build`、justが使える環境なら`just build`でイメージ作成。

## 立ち上げ
`docker compose up`,または`just`でコンテナを立ち上げ、中にbashで入ることができる。`just`は`just up`と同じ。

## 片付け
作業はdocker外で続けたいけど、リソース消費したくないなら、`docker compose stop`, 再開したいときは`docker compose start`  
環境を完全に消すなら`docker compose down`

