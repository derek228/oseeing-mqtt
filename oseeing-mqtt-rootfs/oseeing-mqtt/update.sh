#!/bin/sh

# google drive file ID
file_id=$1

# download
echo "Start download update image files"
echo "https://docs.google.com/uc?export=download&id=$file_id"
curl -L -o /image "https://docs.google.com/uc?export=download&id=$file_id" -k
echo "Finished, update file in /image"
echo "Start update image"
/mnt/mtdblock1/fwupdate -p /image -w kernel
echo "Update image finished, please restart your system"
