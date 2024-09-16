#! /bin/sh
rm output/target/ir8062
rm output/target/uart
rm output/target/pwm
rm output/target/thermal.sh
rm -rf board/nuvoton/rootfs-chili
cp -rf board/oseeing/oseeing1s board/nuvoton/rootfs-chili

