AM_CPPFLAGS = -Wall -Wextra -g $(all_includes)

if HAVE_SDR

bin_PROGRAMS = osmocalibrate

osmocalibrate_SOURCES = main.c
osmocalibrate_LDADD = \
	$(top_builddir)/src/libcalibrate/libcalibrate.a \
	$(top_builddir)/src/libsdr/libsdr.a \
	$(top_builddir)/src/liboptions/liboptions.a \
	$(top_builddir)/src/libwave/libwave.a \
	$(top_builddir)/src/libsample/libsample.a \
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
	-lm

if HAVE_ALSA
osmocalibrate_LDADD += \
	$(top_builddir)/src/libsound/libsound.a \
	$(ALSA_LIBS)
endif

AM_CPPFLAGS += -DHAVE_SDR

endif
