
# 環境変数（オプション。使いやすくする）
set dotenv-load := false
set shell := ["bash", "-cu"]  # Fish対策にもなる

# デフォルト（何も指定しなかったら `just up` を実行）
default:
    just up

# Docker 操作
up container_name="ros2_rox_container":
    docker compose up -d
    docker exec -it {{container_name}} bash

down:
    docker compose down

build:
    docker compose build

ps:
    docker compose ps

# Git 操作
add:
    git add .

commit msg="Automated by script":
    git commit -m {{msg}}

push branch_name="develop":
    git push origin {{branch_name}}

git:
    just add
    just commit
    just push


# システムアップデート
update:
    sudo apt update && sudo apt upgrade -y

# Pythonフォーマット
pyfmt:
    black .
    isort .

