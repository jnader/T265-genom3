lib_LTLIBRARIES = libT265_codels.la

libT265_codels_la_SOURCES  =	T265_c_types.h
libT265_codels_la_SOURCES +=	T265_codels.cc
libT265_codels_la_SOURCES +=	T265_grabber_codels.cc
libT265_codels_la_SOURCES +=	T265_pose_port_refresh_codels.cc
libT265_codels_la_SOURCES +=	T265_undistort_images_codels.cc
libT265_codels_la_SOURCES +=	T265_apriltag_detection_codels.cc
libT265_codels_la_SOURCES +=	T265_image_display_codels.cc

libT265_codels_la_CPPFLAGS =	$(requires_CFLAGS)
libT265_codels_la_LIBADD   =	$(requires_LIBS)
libT265_codels_la_CPPFLAGS+=	$(codels_requires_CFLAGS)
libT265_codels_la_LIBADD  +=	$(codels_requires_LIBS)
libT265_codels_la_LDFLAGS  =	-release $(PACKAGE_VERSION)


# idl  mappings
BUILT_SOURCES=	T265_c_types.h
CLEANFILES=	${BUILT_SOURCES}

T265_c_types.h: ${top_srcdir}/T265.gen
	${GENOM3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/T265_c_types.h.d
