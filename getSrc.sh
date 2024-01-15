#!/bin/zsh
PARENT_DIR=BSP

srcs=("`find . -name "*.c"`")
srcs=(`echo ${srcs[@]}`)

for file in ${srcs[@]}
do
    echo ${PARENT_DIR}${file#.}
done
