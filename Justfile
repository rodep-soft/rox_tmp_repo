
# 環境変数（オプション。使いやすくする）
set dotenv-load := false
set shell := ["bash", "-cu"]  # Fish対策にもなる

# just up
default:
    just up

# docker-compose立ち上げ&入る
up container_name="ros2_rox_container":
    docker compose up -d
    docker exec -it {{container_name}} bash

# docker-compose stop
stop:
    docker compose stop

# docker-compose down
down:
    docker compose down

# docker-compose build
build:
    docker compose build

# docker-compose ps
ps:
    docker compose ps

# git add .
add:
    git add .

# just commit "messages"
commit msg="Automated by script":
    git commit -m "{{msg}}"

# just push "branch_name"
push branch_name="develop":
    git push origin {{branch_name}}

# just git "messages" "branch_name_to_push"
git msg="Automated by script" branch_name="develop":
    just add
    just commit "{{msg}}"
    just push {{branch_name}}

# just ss "service_name"
ss service="docker":
    systemctl status {{service}}

# check bluetooth
blue:
    bluetoothctl



# system update
update:
    sudo apt update && sudo apt upgrade -y

# Python format
pyfmt:
    black .
    isort .

# 補完をきかせるためのsetup
setup:
    just --completions fish > ~/.config/fish/completions/just.fish
    just --completions bash > ~/.just-completion.bash
    echo 'source ~/.just-completion.bash' >> ~/.bashrc
    source ~/.bashrc


