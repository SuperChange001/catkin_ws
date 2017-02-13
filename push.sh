eval "$(ssh-agent -s)"

ssh-add ~/git_key/key

git push
