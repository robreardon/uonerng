# uonerng
DragonFly BSD driver for OneRNG device

## Description
This is a very early attempt at a driver for the OneRNG USB random number
generator:
http://onerng.info/

It's based on the previous 'onerng' FreeBSD driver and the 'ucom' driver.This is
my first attempt at a driver for any OS, so expect dragons!!!

### Device
The OneRNG device uses the OpenMoki VendorId (0x1d50), and has a ProductId of
0x6086

### Patch
To apply the patch to a freshly checked-out kernel tree:
```
cd /usr/src
patch -p1 < patch
```

#### Notes
Currently, the OneRNG is just told to start so should default to
AVALANCHE_WHITENER mode.  

#### TODO / WISHLIST
- [ ] Tidy / standardize code
- [ ] Make mode of device configurable
- [ ] Support firmware verification
