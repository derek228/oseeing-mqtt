#! /bin/sh
PROJECT_PATH=$PWD
TARGET_PATH="/home/user/NUC970_Buildroot-master/board/oseeing/oseeing1s/"
echo "Project path = ${PROJECT_PATH}"
cp $PROJECT_PATH/ir8062/oseeing1s $TARGET_PATH
cp $PROJECT_PATH/leds/leds $TARGET_PATH


