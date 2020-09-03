#include "acT265.h"
#include "codels.h"
#include "T265_c_types.h"


/* --- Task undistort_images -------------------------------------------- */

vpArray2D<int> mapU_left, mapV_left, mapU_right, mapV_right;
vpArray2D<float> mapDu_left, mapDv_left, mapDu_right, mapDv_right;
vpCameraParameters cam_left, cam_right;

/** Codel init_undist_images of task undistort_images.
 *
 * Triggered by T265_start.
 * Yields to T265_pause_start, T265_loop.
 */
genom_event
init_undist_images(bool detection_enabled, const T265_vp_image *I_left,
                   const T265_vp_image *I_right,
                   const T265_realsense_grabber *rs_grabber,
                   T265_vp_image **I_left_undistorted,
                   T265_vp_image **I_right_undistorted,
                   const genom_context self)
{
  if(detection_enabled)
  {
    (*I_left_undistorted)  = new T265_vp_image;
    (*I_right_undistorted) = new T265_vp_image;

    (*I_left_undistorted)->I.resize(I_left->I.getHeight(), I_left->I.getWidth());
    (*I_right_undistorted)->I.resize(I_right->I.getHeight(), I_right->I.getWidth());

    cam_left  = rs_grabber->g.getCameraParameters(RS2_STREAM_FISHEYE, vpCameraParameters::ProjWithKannalaBrandtDistortion, 1);
    cam_right = rs_grabber->g.getCameraParameters(RS2_STREAM_FISHEYE, vpCameraParameters::ProjWithKannalaBrandtDistortion, 2);

    vpImageTools::initUndistortMap(cam_left, I_left->I.getWidth(), I_left->I.getHeight(), mapU_left, mapV_left, mapDu_left, mapDv_left);
    vpImageTools::initUndistortMap(cam_right, I_right->I.getWidth(), I_right->I.getHeight(), mapU_right, mapV_right, mapDu_right, mapDv_right);

    return T265_loop;
  }

  else
    return T265_pause_start;
}


/** Codel undistort_images of task undistort_images.
 *
 * Triggered by T265_loop.
 * Yields to T265_pause_loop, T265_stop.
 */
genom_event
undistort_images(bool is_publishing, const T265_vp_image *I_left,
                 const T265_vp_image *I_right,
                 T265_vp_image **I_left_undistorted,
                 T265_vp_image **I_right_undistorted,
                 const genom_context self)
{
  if(is_publishing)
  {
    vpImageTools::undistort(I_left->I, mapU_left, mapV_left, mapDu_left, mapDv_left, (*I_left_undistorted)->I);
    vpImageTools::undistort(I_right->I, mapU_right, mapV_right, mapDu_right, mapDv_right, (*I_right_undistorted)->I);

    (*I_left_undistorted)->timestamp = I_left->timestamp;
    (*I_right_undistorted)->timestamp = I_right->timestamp;
  }

  return T265_pause_loop;
}


/** Codel stop_image_undistort of task undistort_images.
 *
 * Triggered by T265_stop.
 * Yields to T265_ether.
 */
genom_event
stop_image_undistort(T265_vp_image **I_left_undistorted,
                     T265_vp_image **I_right_undistorted,
                     const genom_context self)
{
  delete (*I_left_undistorted);
  delete (*I_right_undistorted);
  std::cout << "stop undistort\n";
  return T265_ether;
}
