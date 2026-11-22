# Raspberry Pi from scratch (rpitx transmitter, RTL-SDR receiver)

Tried on a Raspberry Pi 3 Model B with a fresh Raspberry Pi OS 13 "trixie"
32 bit (armhf, kernel 6.18.60) on 2026-11-22. The Pi 4 on Raspberry Pi OS 11
64 bit was set up the same way, with the older packages. See `RPITX.md` for
what the transmitter does and how to test it.

## 1. Image

Raspberry Pi Imager, Raspberry Pi OS (Lite is enough). In its settings: host
name (e.g. `pi3`), a user (here `pi`), SSH on with your public key, Wi-Fi if
needed. Newer images have no default `pi` user.

If the user was made later by hand:

```sh
sudo adduser pi
sudo usermod -aG sudo,adm,dialout,plugdev,gpio,video,audio pi
mkdir -p ~/.ssh && chmod 700 ~/.ssh          # as pi
echo '<your public key>' >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys
```

Optional, for scripts over SSH (`sudo` without a password, as on the old
images):

```sh
sudo sh -c 'echo "pi ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/010_pi-nopasswd && chmod 440 /etc/sudoers.d/010_pi-nopasswd'
```

## 2. Update

```sh
sudo apt-get update
sudo apt-get -y full-upgrade
```

## 3. Packages

```sh
sudo apt-get install -y git build-essential autoconf automake libtool pkg-config \
    libosmocore-dev libasound2-dev libsoapysdr-dev soapysdr-tools soapysdr-module-rtlsdr \
    rtl-sdr librtlsdr-dev libfftw3-dev libsndfile1-dev \
    libsofia-sip-ua-dev libsofia-sip-ua-glib-dev \
    multimon-ng python3-numpy python3-scipy tmux rsync
```

`libraspberrypi-dev` (`bcm_host`) no longer exists on trixie. It is not
needed: librpitx has its own copy of the two functions (`rpi.c`), only its
Makefile still links `-lbcm_host` (see step 6).

## 4. Boot settings for rpitx

rpitx moves PLLC, which also clocks the GPU core: the core clock must be
fixed. Append to `/boot/firmware/config.txt` (`/boot/config.txt` on OS 11 and
older), at the end, under `[all]`:

```
gpu_freq=250
force_turbo=1
```

(the same lines rpitx's `install.sh` adds).

## 5. RTL-SDR

Keep the DVB-T kernel driver away from the stick:

```sh
printf 'blacklist dvb_usb_rtl28xxu\nblacklist rtl2832\nblacklist rtl2830\nblacklist rtl2832_sdr\n' | \
    sudo tee /etc/modprobe.d/blacklist-rtlsdr.conf
sudo reboot
```

After the reboot:

```sh
vcgencmd measure_clock core      # frequency(1)=250000000
rtl_test -t                      # Found 1 device(s)
SoapySDRUtil --find              # driver = rtlsdr
timedatectl show -p NTPSynchronized   # yes: librpitx corrects the crystal from NTP
```

## 6. librpitx

```sh
git clone https://github.com/F5OEO/librpitx      # tested: f01bdb6
cd librpitx/src
make LDFLAGS="-lm -lrt -lpthread"                # without -lbcm_host
sudo make install                                # /usr/local/lib/librpitx.a + headers
```

Optional, the rpitx tools (`tune` switches a carrier off that a killed
program left on):

```sh
git clone https://github.com/F5OEO/rpitx
```

(do not run its `install.sh` on trixie: it wants `libraspberrypi-dev`;
step 4 already did its `config.txt` part.)

## 7. libosmo-cc (3.2.0 or newer)

```sh
git clone https://gitea.osmocom.org/cc/libosmo-cc
cd libosmo-cc
autoreconf -fi && ./configure && make
sudo make install && sudo ldconfig
pkg-config --modversion libosmocc        # 3.2.0...
```

## 8. osmocom-analog

```sh
git clone https://github.com/VasylSamoilov/osmocom-analog
cd osmocom-analog
autoreconf -fi && ./configure
```

`configure` must print `Compiling with rpitx support for Raspberry Pi GPIO TX`
(and `SoapySDR support` for the RTL-SDR).

```sh
make -j3          # the Pi 3 has 1 GB: more jobs can run out of memory
sudo make install
```

## 9. SIP (optional, not built on the Pi 3 yet)

```sh
git clone https://github.com/VasylSamoilov/osmo-cc-sip-endpoint
cd osmo-cc-sip-endpoint
autoreconf -fi && ./configure && make && sudo make install
```

## 10. First test

Follow the test ladder in `RPITX.md`, starting with a single POCSAG page and
an RTL-SDR capture next to the Pi. Run as root, stop the transmitter after
each test.
