export PATH=/opt/vtcs_toolchain/arm-eabi-uclibc/usr/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/local/g

# original source code taken from
# https://github.com/futuretekinc/ftm-50s/tree/master/linux-2.6.37/drivers/media/video/spear/hx280enc
make ARCH=arm CROSS_COMPILE=arm-linux-
sudo cp *.ko /mnt/noc
