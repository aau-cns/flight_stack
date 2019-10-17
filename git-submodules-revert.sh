#! /bin/bash


git submodule foreach 'git clean -fx; git checkout . *'
