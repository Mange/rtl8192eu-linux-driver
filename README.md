# rtl8192eu linux drivers

These drivers are patched to work with Linux Kernels up till 4.15 but they work fine with Linux Kernel 4.17.x too.

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

## Preparing for Building and Installing drivers

1. Install DKMS and other required tools

** For Ubuntu Based Systems

Start terminal and run :

sudo apt-get install git linux-headers-generic build-essential dkms lshw gedit

** For Fedora OS

Start terminal and run :

sudo dnf groupinstall "Development Tools" "Development Libraries"
sudo dnf install git lshw gedit dkms

2. Download the source code of the network drivers :

Click on GREEN button on top of the file listing on this page (https://github.com/Mange/rtl8192eu-linux-driver ) and click on the button which says "Clone or Download" and then finally on "Download Zip.

Once you have downloaded the ZIP file, extract it to any path of your choice.

OR 

Use git to replicate the repository in any directory of your choice by using commands :

** For Ubuntu and Fedora OS

mkdir net_drivers
cd net_drivers
git config --global user.name "[name]"
git config --global user.email "[email address]"
git clone https://github.com/Mange/rtl8192eu-linux-driver.git

This will download all the files to the current directory. Obviously, "name" and "email address" are your UserName and Email ID that you have registered with on GitHub (github.com) .

If you are not comfortable with using GitHub system, just download the ZIP file as stated earlier on.

3. Blacklist existing network drivers :

Now, again in terminal execute the command :

** For Ubuntu and Fedora OS

sudo lshw -C network

Now, you will get the list of Network Adapters attached to your PC. Check the driver that is being used by your network adapter that you want to install these drivers for.

Sample output :

sudo lshw -class network
  *-network               
       description: *
       product: *
       vendor: *
       physical id: *
       bus info: pci@0000:00:19.0
       logical name: eth0
       version: 02
       serial: 00:1c:c0:f8:79:ee
       size: *
       capacity: *
       width: *
       clock: *
       capabilities: pm msi bus_master cap_list ethernet physical tp 10bt 10bt-fd 100bt 100bt-fd 1000bt-fd autonegotiation
       configuration: autonegotiation=on broadcast=yes driver=e1000e driverversion=2.3.2-k duplex=full firmware=1.1-0 ip=192.168.1.2 latency=0 link=yes multicast=yes port=twisted pair speed=100Mbit/s
       resources: irq:43 memory:e0300000-e031ffff memory:e0324000-e0324fff ioport:20c0(size=32)

Look at "driver=e1000e". This means that driver loaded for this particular device is e1000e.

** In the case of our device, the driver name would be "rtl8xxxu" or something similar.

Now, we need to disable this inbuilt driver of the wireless adapter first otherwise the new driver that we will build and install will NOT load at all for our hardware.

So, again in terminal type -

For Ubuntu and Fedora :

sudo gedit /etc/modprobe.d/blacklist.conf

Now, this conf file will open in "gedit", so just add the line here :



blacklist rtl8xxxu



Obviously replace "rtl8xxxu" with whatever driver that your Linux OS loads up by default for your hardware. 
Now, save this file and quit gedit application.

4. Final Step in this section - Update initramfs

** For Ubuntu OS

Type in terminal :

sudo update-initramfs -u -k all

** For Fedora OS

Type in terminal :

sudo dracut /boot/initramfs-$(uname -r).img $(uname -r) --force

Now, we are ready to build and install drivers as described in the next section.

## Building and installing using DKMS

This tree supports Dynamic Kernel Module Support (DKMS), a system for
generating kernel modules from out-of-tree kernel sources. It can be used to
install/uninstall kernel modules, and the module will be automatically rebuilt
from source when the kernel is upgraded (for example using your package manager).


** Only for Raspberry Pi

    ```shell
    $ sudo apt-get install git raspberrypi-kernel-headers build-essential dkms
    ```

1. Add the driver to DKMS. This will copy the source to a system directory so
that it can used to rebuild the module on kernel upgrades.

Extract the ZIP archive and run the following command from within the extracted directory :

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

3. Now, restart your Linux OS.

After rebooting, check that your kernel has loaded the right module:
 
    ```shell
        $ sudo lshw -c network
    ```
   
You should see the line ```driver=8192eu``` for your wireless network adapter.
    
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
