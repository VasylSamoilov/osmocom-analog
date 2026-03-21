AUTOMAKE_OPTIONS = foreign

SUBDIRS = \
	liblogging \
	liboptions \
	libmobile \
	libdisplay \
	libsample \
	libimage \
	libsendevolumenregler \
	libcompandor \
	libgoertzel \
	libjitter \
	libsquelch \
	libhagelbarger \
	libdtmf \
	libsamplerate \
	libpolyphase \
	libscrambler \
	libemphasis \
	libfsk \
	libam \
	libfm \
	libfilter \
	libpll \
	libchannelizer \
	libfastmath \
	libiqringbuf \
	libcalibrate \
	libwave \
	libfft \
	libclipper \
	libserial \
	libv27 \
	libmtp \
	libaaimage \
	libtones \
	libecho

if HAVE_ALSA
SUBDIRS += \
	libsound
endif

if HAVE_SDR
SUBDIRS += \
	libsdr
endif

SUBDIRS += \
	anetz \
	bnetz \
	cnetz \
	nmt \
	amps \
	r2000 \
	imts \
	mpt1327 \
	jolly \
	eurosignal \
	pocsag \
	flex \
	golay \
	mobitex \
	fuenf \
	tv \
	radio \
	zeitansage \
	sim \
	magnetic \
	fuvst \
	dcf77 \
	mate \
	osmocalibrate

if HAVE_ALSA
if HAVE_FUSE
SUBDIRS += \
	datenklo
endif
endif

SUBDIRS += \
	test

