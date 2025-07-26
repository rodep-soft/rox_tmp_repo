#!/usr/bin/env fish

# 削除したいリモートブランチ名リスト
set branches \
    bugfix/mecanum \
    bugfix/mecanum_working \
    chore/something \
    docs \
    format \
    hotfix/docker-compose.yml \
    lazytatzv-patch-1

for branch in $branches
    echo "Deleting remote branch: $branch"
    git push origin --delete $branch
end

