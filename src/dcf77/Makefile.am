AM_CPPFLAGS = -Wall -Wextra -Wmissing-prototypes -g $(all_includes)

DCF77_SOURCES = \
	dcf77.c \
	weather.c \
	weather_crypt.c \
	weather_pic.c \
	cities.c \
	image.c \
	timesignal.c \
	timesignal_tx.c \
	dcf77_proto.c \
	wwvb.c \
	msf.c \
	jjy.c \
	main.c

if HAVE_SDR

bin_PROGRAMS = dcf77

dcf77_SOURCES = $(DCF77_SOURCES)
dcf77_LDADD = \
	$(COMMON_LA) \
	$(top_builddir)/src/liboptions/liboptions.a \
	$(top_builddir)/src/libsdr/libsdr.a \
	$(top_builddir)/src/libchannelizer/libchannelizer.a \
	$(top_builddir)/src/libfm/libfm.a \
	$(top_builddir)/src/libam/libam.a \
	$(top_builddir)/src/libemphasis/libemphasis.a \
	$(top_builddir)/src/libsamplerate/libsamplerate.a \
	$(top_builddir)/src/libpolyphase/libpolyphase.a \
	$(top_builddir)/src/libjitter/libjitter.a \
	$(top_builddir)/src/libpll/libpll.a \
	$(top_builddir)/src/libclipper/libclipper.a \
	$(top_builddir)/src/libdisplay/libdisplay.a \
	$(top_builddir)/src/libfilter/libfilter.a \
	$(top_builddir)/src/libfft/libfft.a \
	$(top_builddir)/src/libwave/libwave.a \
	$(top_builddir)/src/libsample/libsample.a \
	$(top_builddir)/src/libaaimage/libaaimage.a \
	$(top_builddir)/src/liblogging/liblogging.a \
	$(LIBOSMOCORE_LIBS) \
	$(UHD_LIBS) \
	$(SOAPY_LIBS) \
	$(RPITX_LIBS) \
	-lm

if HAVE_ALSA
dcf77_LDADD += \
	$(top_builddir)/src/libsound/libsound.a \
	$(ALSA_LIBS)
AM_CPPFLAGS += -DHAVE_ALSA
endif

AM_CPPFLAGS += -DHAVE_SDR

else
# No SDR: sound card only (original build)

if HAVE_ALSA
bin_PROGRAMS = dcf77

dcf77_SOURCES = $(DCF77_SOURCES)
dcf77_LDADD = \
	$(COMMON_LA) \
	$(top_builddir)/src/liboptions/liboptions.a \
	$(top_builddir)/src/libdisplay/libdisplay.a \
	$(top_builddir)/src/libfilter/libfilter.a \
	$(top_builddir)/src/libwave/libwave.a \
	$(top_builddir)/src/libsample/libsample.a \
	$(top_builddir)/src/libsound/libsound.a \
	$(top_builddir)/src/libaaimage/libaaimage.a \
	$(top_builddir)/src/liblogging/liblogging.a \
	$(LIBOSMOCORE_LIBS) \
	$(ALSA_LIBS) \
	-lm
endif

endif
