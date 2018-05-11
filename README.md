# rtl8192eu linux drivers

**NOTE:** This branch is based on Realtek's driver versioned 4.4.1. `master` is based on 4.3.1.1 originally.

The official drivers for D-Link DWA-131 Rev E, with patches to keep it working on newer kernels.
Also works on Rosewill RNX-N180UBE v2 N300 Wireless Adapter.

**NOTE:** This is just a "mirror". I have no knowledge about this code or how it works. Expect no support from me or any contributors here. I just think GitHub is a nicer way of keeping track of this than random forum posts and precompiled binaries being sent by email. I don't want someone else to have to spend 5 days of googling and compiling with random patches until it works.

## Source for the official drivers

Official drivers were downloaded from D-Link Australia. D-Link USA and the european countries I checked only lists revision A and B. Australia lists all three.

* [Download page for DWA-131][driver-downloads]
* [Direct download link for Linux drivers][direct-download]
  * GitHub will not link to the `ftp://` schema. Raw link contents:

      `ftp://files.dlink.com.au/products/DWA-131/REV_E/Drivers/DWA-131_Linux_driver_v4.3.1.1.zip`

In addition, you can find the contents of this version in the initial commit of this repo: [1387cf623d54bc2caec533e72ee18ef3b6a1db29][initial-commit]

## Patches

You can see the applied patches, their sources and/or motivation by looking at the commits. The `master` branch will mostly be kept clean with a single commit per patch, except for Pull Requests. You can review commit by commit and then record the SHA in order to get a safe reference to use. As long as the SHA stays the same you know that what you get has been reviewed by you.

Note that updates to this README will show up as separate commits. I will not mix changes to this file with changes to the code in case you want to mirror this without the README.

## Building and installing using DKMS

This tree supports Dynamic Kernel Module Support (DKMS), a system for
generating kernel modules from out-of-tree kernel sources. It can be used to
install/uninstall kernel modules, and the module will be automatically rebuilt
from source when the kernel is upgraded (for example using your package manager).

#### Install DKMS and other required tools

* for normal Linux systems

    ```shell
    $ sudo apt install git linux-headers-generic build-essential dkms
    ```

* for Raspberry Pi

    ```shell
    $ sudo apt install git raspberrypi-kernel-headers build-essential dkms
    ```

#### Automatic install (x86/PC versions)

* Download install script:

    ```shell
    $ wget -P /tmp https://github.com/Mange/rtl8192eu-linux-driver/raw/realtek-4.4.x/install.sh
    ```

* Review install script at `/tmp/install.sh`

* Run it:

    ```shell
    $ bash /tmp/install.sh
    ```

#### Manual install

1. Add the driver to DKMS. This will copy the source to a system directory so
that it can used to rebuild the module on kernel upgrades.

    ```shell
    $ sudo dkms add .
    ```

2. Build and install the driver.

    ```shell
    $ sudo dkms install rtl8192eu/1.0
    ```

The Makefile is preconfigured to handle most x86/PC versions.  If you are compiling for something other than an intel x86 architecture, you need to first select the platform, e.g. for the Raspberry Pi, you need to set the I386 to n and the ARM_RPI to y:

```sh
...
CONFIG_PLATFORM_I386_PC = n
...
CONFIG_PLATFORM_ARM_RPI = y
```

```sh
# cd /usr/src/rtl8192eu
# sudo make clean
# sudo make
# sudo make install
# sudo modprobe -a 8192eu
```

3. Check that your kernel has loaded the right module:
 
    ```shell
    $ lshw -c network
    ```
   
You should see the line ```driver=8192eu```

#### Uninstall
If you wish to uninstall the driver at a later point, use
_sudo dkms uninstall rtl8192eu/1.0_. To completely remove the driver from DKMS use
_sudo dkms remove rtl8192eu/1.0 --all_.

## Submitting patches

1. Fork repo
2. Do your patch in a topic branch
3. Open a pull request on GH, or send it by email to `Magnus Bergmark <magnus.bergmark@gmail.com>`.
4. I'll squash your commits when everything checks out and add it to `master`.

## Copyright and licenses

The original code is copyrighted, but I don't know by whom. The driver download does not contain license information; please open an issue if you are the copyright holder.

Most C files are licensed under GNU General Public License (GPL), version 2.

[driver-downloads]: http://support.dlink.com.au/Download/download.aspx?product=DWA-131
[direct-download]: ftp://files.dlink.com.au/products/DWA-131/REV_E/Drivers/DWA-131_Linux_driver_v4.3.1.1.zip
[initial-commit]: https://github.com/Mange/rtl8192eu-linux-driver/commit/1387cf623d54bc2caec533e72ee18ef3b6a1db29
