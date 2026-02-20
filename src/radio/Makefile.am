AM_CPPFLAGS = -Wall -Wextra -Wmissing-prototypes -g $(all_includes)

noinst_LIBRARIES = librds.a

librds_a_SOURCES = \
	rds.c \
	rdsframe.c \
	rds_tables.c

bin_PROGRAMS = \
	osmordstest

osmordstest_SOURCES = \
	main_rds_test.c
osmordstest_LDADD = \
	$(COMMON_LA) \
	librds.a \
	$(top_builddir)/src/libfm/libfm.a \
	$(top_builddir)/src/libfilter/libfilter.a \
	$(top_builddir)/src/liblogging/liblogging.a \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOCC_LIBS) \
	-lm

if HAVE_SDR

bin_PROGRAMS += \
	osmoradio

osmoradio_SOURCES = \
	radio.c \
	audio_debug.c \
	sca.c \
	rds_protocol.c \
	rigctl_server.c \
	signal_meter.c \
	main.c
osmoradio_LDADD = \
	$(COMMON_LA) \
	librds.a \
	$(top_builddir)/src/liboptions/liboptions.a \
	$(top_builddir)/src/libwave/libwave.a \
	$(top_builddir)/src/libsample/libsample.a \
	$(top_builddir)/src/libsdr/libsdr.a \
	$(top_builddir)/src/libchannelizer/libchannelizer.a \
	$(top_builddir)/src/libclipper/libclipper.a \
	$(top_builddir)/src/libcompandor/libcompandor.a \
	$(top_builddir)/src/libfm/libfm.a \
	$(top_builddir)/src/libam/libam.a \
	$(top_builddir)/src/libemphasis/libemphasis.a \
	$(top_builddir)/src/libsamplerate/libsamplerate.a \
	$(top_builddir)/src/libpolyphase/libpolyphase.a \
	$(top_builddir)/src/libjitter/libjitter.a \
	$(top_builddir)/src/libfilter/libfilter.a \
	$(top_builddir)/src/libdisplay/libdisplay.a \
	$(top_builddir)/src/libfft/libfft.a \
	$(top_builddir)/src/liblogging/liblogging.a \
	$(LIBOSMOCORE_LIBS) \
	$(UHD_LIBS) \
	$(SOAPY_LIBS) \
	$(RPITX_LIBS) \
	-lm

if HAVE_ALSA
osmoradio_LDADD += \
	$(top_builddir)/src/libsound/libsound.a \
	$(ALSA_LIBS)
endif

if HAVE_ALSA
AM_CPPFLAGS += -DHAVE_ALSA
endif

AM_CPPFLAGS += -DHAVE_SDR

endif

