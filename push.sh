eval "$(ssh-agent -s)"

ssh-add ~/ssh_key/gitkey

git push
