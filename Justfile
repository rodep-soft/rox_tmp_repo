
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
    git commit -m "{{msg}}"

push branch_name="develop":
    git push origin {{branch_name}}

git msg="Automated by script" branch_name="develop":
    just add
    just commit "{{msg}}"
    just push {{branch_name}}

ss service="docker":
    systemctl status {{service}}

blue:
    bluetoothctl



# システムアップデート
update:
    sudo apt update && sudo apt upgrade -y

# Pythonフォーマット
pyfmt:
    black .
    isort .

# 補完をきかせるためのsetup
setup:
    just --completions fish > ~/.config/fish/completions/just.fish
    just --completions bash > ~/.just-completion.bash
    echo 'source ~/.just-completion.bash' >> ~/.bashrc
    source ~/.bashrc


