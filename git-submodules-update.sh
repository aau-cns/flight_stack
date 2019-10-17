#! /bin/bash
# https://stackoverflow.com/questions/10168449/git-update-submodules-recursively

git submodule update --recursive --remote --merge
git add . && git commit -m 'Update submodules to latest revisions'
git push origin master
