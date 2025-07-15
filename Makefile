CONTAINER_NAME = ros2_rox_container
TARGET_BRANCH  = develop
COMMIT_MESSAGE = wip
GITHUB_LINK    = https://github.com/rodep-soft/rox_tmp_repo/tree/main

.PHONY: up down build ps add commit push git github

up:
	docker compose up -d
	docker exec -it $(CONTAINER_NAME) bash

down:
	docker compose down

build:
	docker compose build

ps:
	docker compose ps

add:
	git add .

commit:
	git commit -m "$(COMMIT_MESSAGE)"

push:
	git push origin $(TARGET_BRANCH)

git:
	$(MAKE) add
	$(MAKE) commit
	$(MAKE) push

github:
	xdg-open $(GITHUB_LINK) 2>/dev/null || open $(GITHUB_LINK) || start $(GITHUB_LINK)

