Le S3 Kernel Build mini-Howto
===============================

1. Build
--------

- get toolchain
    From android git server, codesourcery etc.
    - aarch64-linux-android-4.9

- Unpack kernel source
    Suppose kernel source has been unpacked to <kernel> dir.

- make output folder
    $ mkdir <kernel>/out

- export env variables
    export correct "CROSS_COMPILE" to use the toolchain path you have downloaded.
    $ export CROSS_COMPILE=<Your cross compilee dir>/aarch64-linux-android-4.9/bin/aarch64-linux-android-
    $ export JOBS=16    # Can be CPU core # x 2

- build kernel
    $ cd <kernel>
    $ make -C $PWD O=$PWD/out ARCH=arm64 s2-perf_defconfig
    $ make -j$JOBS -C $PWD O=$PWD/out ARCH=arm64 KCFLAGS=-mno-android
    $ make -j$JOBS -C $PWD O=$PWD/out ARCH=arm64 KCFLAGS=-mno-android modules

2. Output files
---------------

- Kernel: out/arch/arm64/boot/Image
- Kernel modules: out/drivers/*/*.ko

3. Clean up
-----------

$ rm -rf out
