#include "acT265.h"
#include "codels.h"
#include "T265_c_types.h"


/* --- Task apriltag_detection ------------------------------------------ */
vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
vpDetectorAprilTag *detector;
vpCameraParameters cam_undistort;
std::vector<vpHomogeneousMatrix> cMo_vec;

/** Codel init_detector of task apriltag_detection.
 *
 * Triggered by T265_start.
 * Yields to T265_loop.
 */
genom_event
init_detector(const T265_realsense_grabber *rs_grabber,
              double tag_size, double quad_decimate,
              optional_T265_corners *tag_corners,
              const genom_context self)
{
  vpCameraParameters cam_left = rs_grabber->g.getCameraParameters(RS2_STREAM_FISHEYE, vpCameraParameters::ProjWithKannalaBrandtDistortion, 1);
  cam_undistort.initPersProjWithoutDistortion(cam_left.get_px(), cam_left.get_py(), cam_left.get_u0(), cam_left.get_v0());

  detector = new vpDetectorAprilTag(tagFamily);

  detector->setAprilTagQuadDecimate(quad_decimate);
  detector->setAprilTagPoseEstimationMethod(poseEstimationMethod);
  detector->setAprilTagNbThreads(2);
  detector->setDisplayTag(false, -1 < 0 ? vpColor::none : vpColor::getColor(-1), 2);
  detector->setZAlignedWithCameraAxis(false);

  tag_corners->_present = false;
  for(int i = 0 ; i < 4; i++)
  {
    tag_corners->_value._buffer[i].i = 0.;
    tag_corners->_value._buffer[i].j = 0.;
  }

  return T265_loop;
}


/** Codel loop_detector of task apriltag_detection.
 *
 * Triggered by T265_loop.
 * Yields to T265_pause_loop, T265_stop.
 */
genom_event
loop_detector(const T265_vp_image *I_left_undistorted,
              const genom_context self)
{
  cMo_vec.clear();

  detector->detect(I_left_undistorted->I, 0.08, cam_undistort, cMo_vec);
  if(cMo_vec.size() != 0)
    std::cout << cMo_vec.size() << std::endl;

  return T265_pause_loop;
}


/** Codel kill_detector of task apriltag_detection.
 *
 * Triggered by T265_stop.
 * Yields to T265_ether.
 */
genom_event
kill_detector(const genom_context self)
{
  detector = NULL;
  return T265_ether;
}
