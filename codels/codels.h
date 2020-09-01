#ifndef _CODELS_H
#define _CODELS_H

#include <visp3/core/vpConfig.h>
#ifndef VISP_HAVE_REALSENSE2
#error "ViSP is not built with libRealSense support...";
#endif

#ifndef VISP_HAVE_X11
#error "ViSP is not built with X11 support...";
#endif

#include <visp3/core/vpCameraParameters.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/core/vpImageTools.h>

struct T265_realsense_grabber
{
    vpRealSense2 g;
};

struct T265_vp_image
{
    double timestamp;
    vpImage<unsigned char> I;
};

struct T265_vp_homogeneous_matrix
{
    vpHomogeneousMatrix mat;
};

struct T265_vp_odometry
{
    double timestamp;
    vpHomogeneousMatrix pose;
    vpColVector vel, acc;
    unsigned int tracker_confidence;
};

#endif