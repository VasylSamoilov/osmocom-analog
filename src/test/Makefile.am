AM_CPPFLAGS = -Wall -Wextra -Wmissing-prototypes -g $(all_includes)

noinst_PROGRAMS = \
	test_filter \
	test_sendevolumenregler \
	test_compandor \
	test_emphasis \
	test_goertzel \
	test_dtmf \
	test_dms \
	test_sms \
	test_performance \
	test_hagelbarger \
	test_v27scrambler \
	test_channelizer \
	test_echo_analysis \
	test_echo_analysis_edge_cases \
	test_echo_analysis_final_checkpoint

test_filter_SOURCES = test_filter.c dummy.c

test_filter_LDADD = \
	$(COMMON_LA) \
	$(top_builddir)/src/libfilter/libfilter.a \
	$(top_builddir)/src/liboptions/liboptions.a \
	$(top_builddir)/src/liblogging/liblogging.a \
	$(LIBOSMOCC_LIBS) \
	$(LIBOSMOCORE_LIBS) \
	-lm

test_sendevolumenregler_SOURCES = test_sendevolumenregler.c

test_sendevolumenregler_LDADD = \
	$(COMMON_LA) \
	$(top_builddir)/src/libsendevolumenregler/libsendevolumenregler.a \
	-lm

test_compandor_SOURCES = dummy.x test_compandor.c

test_compandor_LDADD = \
	$(COMMON_LA) \
	$(top_builddir)/src/libcompandor/libcompandor.a \
	$(top_builddir)/src/libfilter/libfilter.a \
	$(top_builddir)/src/liboptions/liboptions.a \
	$(top_builddir)/src/liblogging/liblogging.a \
	$(LIBOSMOCC_LIBS) \
	$(LIBOSMOCORE_LIBS) \
	-lm

test_emphasis_SOURCES = test_emphasis.c dummy.c

test_emphasis_LDADD = \
	$(COMMON_LA) \
	$(top_builddir)/src/libemphasis/libemphasis.a \
	$(top_builddir)/src/libfilter/libfilter.a \
	$(top_builddir)/src/liboptions/liboptions.a \
	$(top_builddir)/src/liblogging/liblogging.a \
	$(LIBOSMOCC_LIBS) \
	$(LIBOSMOCORE_LIBS) \
	-lm

test_dtmf_SOURCES = dummy.x test_dtmf.c

test_dtmf_LDADD = \
	$(COMMON_LA) \
	$(top_builddir)/src/libdtmf/libdtmf.a \
	$(top_builddir)/src/libfm/libfm.a \
	$(top_builddir)/src/libfilter/libfilter.a \
	-lm

test_dms_SOURCES = test_dms.c dummy.c

test_dms_LDADD = \
	$(COMMON_LA) \
	$(top_builddir)/src/libmobile/libmobile.a \
	$(top_builddir)/src/libdtmf/libdtmf.a \
	$(top_builddir)/src/libtones/libtones.a \
	$(top_builddir)/src/liboptions/liboptions.a \
	$(top_builddir)/src/libdisplay/libdisplay.a \
	$(top_builddir)/src/nmt/libdmssms.a \
	$(top_builddir)/src/libjitter/libjitter.a \
	$(top_builddir)/src/libsamplerate/libsamplerate.a \
	$(top_builddir)/src/libemphasis/libemphasis.a \
	$(top_builddir)/src/libfilter/libfilter.a \
	$(top_builddir)/src/libwave/libwave.a \
	$(top_builddir)/src/libsample/libsample.a \
	$(top_builddir)/src/libaaimage/libaaimage.a \
	$(top_builddir)/src/liblogging/liblogging.a \
	$(top_builddir)/src/libecho/libecho.a \
	$(top_builddir)/src/libgoertzel/libgoertzel.a \
	$(top_builddir)/src/libfm/libfm.a \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOCC_LIBS) \
	-lm

if HAVE_ALSA
test_dms_LDADD += \
	$(top_builddir)/src/libsound/libsound.a \
	$(ALSA_LIBS)
endif

if HAVE_SDR
test_dms_LDADD += \
	$(top_builddir)/src/libsdr/libsdr.a \
	$(top_builddir)/src/libchannelizer/libchannelizer.a \
	$(top_builddir)/src/libpolyphase/libpolyphase.a \
	$(top_builddir)/src/libfft/libfft.a \
	$(top_builddir)/src/libfm/libfm.a \
	$(top_builddir)/src/libam/libam.a \
	$(UHD_LIBS) \
	$(SOAPY_LIBS) \
	$(RPITX_LIBS)
endif

test_sms_SOURCES = dummy.c test_sms.c

test_sms_LDADD = \
	$(COMMON_LA) \
	$(top_builddir)/src/libmobile/libmobile.a \
	$(top_builddir)/src/libdtmf/libdtmf.a \
	$(top_builddir)/src/libtones/libtones.a \
	$(top_builddir)/src/liboptions/liboptions.a \
	$(top_builddir)/src/libdisplay/libdisplay.a \
	$(top_builddir)/src/nmt/libdmssms.a \
	$(top_builddir)/src/libjitter/libjitter.a \
	$(top_builddir)/src/libsamplerate/libsamplerate.a \
	$(top_builddir)/src/libemphasis/libemphasis.a \
	$(top_builddir)/src/libfilter/libfilter.a \
	$(top_builddir)/src/libwave/libwave.a \
	$(top_builddir)/src/libsample/libsample.a \
	$(top_builddir)/src/libaaimage/libaaimage.a \
	$(top_builddir)/src/liblogging/liblogging.a \
	$(top_builddir)/src/libecho/libecho.a \
	$(top_builddir)/src/libgoertzel/libgoertzel.a \
	$(top_builddir)/src/libfm/libfm.a \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOCC_LIBS) \
	-lm

if HAVE_ALSA
test_sms_LDADD += \
	$(top_builddir)/src/libsound/libsound.a \
	$(ALSA_LIBS)
endif

if HAVE_SDR
test_sms_LDADD += \
	$(top_builddir)/src/libsdr/libsdr.a \
	$(top_builddir)/src/libchannelizer/libchannelizer.a \
	$(top_builddir)/src/libpolyphase/libpolyphase.a \
	$(top_builddir)/src/libfft/libfft.a \
	$(top_builddir)/src/libfm/libfm.a \
	$(top_builddir)/src/libam/libam.a \
	$(UHD_LIBS) \
	$(SOAPY_LIBS) \
	$(RPITX_LIBS)
endif

test_performance_SOURCES = dummy.c test_performance.c

test_performance_LDADD = \
	$(COMMON_LA) \
	$(top_builddir)/src/libfm/libfm.a \
	$(top_builddir)/src/libfilter/libfilter.a \
	-lm

test_hagelbarger_SOURCES = dummy.c test_hagelbarger.c

test_hagelbarger_LDADD = \
	$(COMMON_LA) \
	$(top_builddir)/src/libhagelbarger/libhagelbarger.a \
	-lm

test_v27scrambler_SOURCES = dummy.c test_v27scrambler.c

test_v27scrambler_LDADD = \
	$(COMMON_LA) \
	$(top_builddir)/src/libv27/libv27.a \
	-lm

test_goertzel_SOURCES = test_goertzel.c dummy.c

test_goertzel_LDADD = \
	$(COMMON_LA) \
	$(top_builddir)/src/libgoertzel/libgoertzel.a \
	$(top_builddir)/src/liboptions/liboptions.a \
	$(top_builddir)/src/liblogging/liblogging.a \
	$(LIBOSMOCC_LIBS) \
	$(LIBOSMOCORE_LIBS) \
	-lm

test_channelizer_SOURCES = test_channelizer.c

test_channelizer_LDADD = \
	$(COMMON_LA) \
	$(top_builddir)/src/libchannelizer/libchannelizer.a \
	$(top_builddir)/src/libpolyphase/libpolyphase.a \
	$(top_builddir)/src/libsample/libsample.a \
	-lm


test_echo_analysis_SOURCES = test_echo_analysis.c dummy.c

test_echo_analysis_LDADD = \
	$(COMMON_LA) \
	$(top_builddir)/src/libecho/libecho.a \
	$(top_builddir)/src/libgoertzel/libgoertzel.a \
	$(top_builddir)/src/libsample/libsample.a \
	$(top_builddir)/src/liboptions/liboptions.a \
	$(top_builddir)/src/liblogging/liblogging.a \
	$(LIBOSMOCC_LIBS) \
	$(LIBOSMOCORE_LIBS) \
	-lm

test_echo_analysis_edge_cases_SOURCES = test_echo_analysis_edge_cases.c dummy.c

test_echo_analysis_edge_cases_LDADD = \
	$(COMMON_LA) \
	$(top_builddir)/src/libecho/libecho.a \
	$(top_builddir)/src/libgoertzel/libgoertzel.a \
	$(top_builddir)/src/libsample/libsample.a \
	$(top_builddir)/src/liboptions/liboptions.a \
	$(top_builddir)/src/liblogging/liblogging.a \
	$(LIBOSMOCC_LIBS) \
	$(LIBOSMOCORE_LIBS) \
	-lm

test_echo_analysis_final_checkpoint_SOURCES = test_echo_analysis_final_checkpoint.c dummy.c

test_echo_analysis_final_checkpoint_LDADD = \
	$(COMMON_LA) \
	$(top_builddir)/src/libecho/libecho.a \
	$(top_builddir)/src/libgoertzel/libgoertzel.a \
	$(top_builddir)/src/libsample/libsample.a \
	$(top_builddir)/src/liboptions/liboptions.a \
	$(top_builddir)/src/liblogging/liblogging.a \
	$(LIBOSMOCC_LIBS) \
	$(LIBOSMOCORE_LIBS) \
	-lm
