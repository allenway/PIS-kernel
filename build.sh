CPU_JOB_NUM=$(($(grep processor /proc/cpuinfo | awk '{field=$NF};END{print field+1}')*3))
echo "CPU number is "$CPU_JOB_NUM

export ARCH=arm
export CROSS_COMPILE=/opt/Embedsky/gcc-4.6.2-glibc-2.13-linaro-multilib-2011.12/tq-linaro-toolchain/bin/arm-none-linux-gnueabi-
cp config_e9_linux .config
make uImage -j$CPU_JOB_NUM

