AM_CPPFLAGS = -Wall -Wextra -Wmissing-prototypes -g $(all_includes) \
	$(SOAPY_CFLAGS) $(UHD_CFLAGS)

noinst_LIBRARIES = libcalibrate.a

libcalibrate_a_SOURCES = \
	calibrate.c \
	calibrate_sdr.c \
	fcch_detect.c \
	gsm_bands.c

noinst_HEADERS = \
	calibrate.h \
	calibrate_sdr.h \
	fcch_detect.h

if HAVE_UHD
AM_CPPFLAGS += -DHAVE_UHD
endif

if HAVE_SOAPY
AM_CPPFLAGS += -DHAVE_SOAPY
endif
