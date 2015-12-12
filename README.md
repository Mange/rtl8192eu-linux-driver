# rtl8192eu linux drivers

The official drivers for D-Link DWA-131 Rev E, with patches to keep it working on newer kernels.
Also works on Rosewill RNX-N180UBE v2 N300 Wireless Adapter.

## Source for the official drivers

Official drivers were downloaded from D-Link Australia. D-Link USA and the european countries I checked only lists revision A and B. Australia lists all three.

* [Download page for DWA-131][driver-downloads]
* [Direct download link for Linux drivers][direct-download]
  * GitHub will not link to the `ftp://` schema. Raw link contents:

      `ftp://files.dlink.com.au/products/DWA-131/REV_E/Drivers/DWA-131_Linux_driver_v4.3.1.1.zip`

In addition, you can find the contents of this version in the initial commit of this repo: [1387cf623d54bc2caec533e72ee18ef3b6a1db29][initial-commit]

## Patches

You can see the applied patches, their sources and/or motivation by looking at the commits. The `master` branch will be kept clean with a single commit per patch. You can review patch by patch and then record the SHA for that commit in order to get a safe reference to use. As long as the SHA stays the same you know that what you get has been reviewed by you.

Note that updates to this README will show up as separate commits. I will not mix changes to this file with changes to the code in case you want to mirror this without the README.

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
