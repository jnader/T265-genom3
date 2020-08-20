#ifndef _CODELS_H
#define _CODELS_H

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
    vpHomogeneousMatrix data;
};

#endif