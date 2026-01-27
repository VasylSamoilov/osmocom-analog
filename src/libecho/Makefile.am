AM_CPPFLAGS = -Wall -Wextra -Wmissing-prototypes -g $(all_includes)

noinst_LIBRARIES = libecho.a

libecho_a_SOURCES = \
	kiss_fft.c \
	kiss_fftr.c \
	fftwrap.c \
	mdf.c \
	filterbank.c \
	preprocess.c

noinst_HEADERS = \
	speex_echo.h \
	speex_preprocess.h \
	speexdsp_types.h \
	speexdsp_config_types.h \
	arch.h \
	os_support.h \
	math_approx.h \
	pseudofloat.h \
	fftwrap.h \
	filterbank.h \
	kiss_fft.h \
	kiss_fftr.h \
	_kiss_fft_guts.h
