#include "acT265.h"

#include "T265_c_types.h"
#include "codels.h"

/* --- Task image_display ----------------------------------------------- */
vpDisplayX display_left;  // Left image.
vpDisplayX display_right; // Right image.
vpDisplayX display_left_undist; // Left undistorted image.
vpDisplayX display_right_undist; // Right undistorted image.

vpImagePoint tag_center;
vpImagePoint tag_corner;
vpHomogeneousMatrix cMo;

vpCameraParameters left_cam_undistort;

/** Codel init_display of task image_display.
 *
 * Triggered by T265_start.
 * Yields to T265_pause_start, T265_loop.
 */
genom_event
init_display(bool display_enabled, const T265_vp_image *I_left,
             const T265_vp_image *I_right,
             const T265_vp_image *I_left_undistorted,
             const T265_vp_image *I_right_undistorted,
             const T265_realsense_grabber *rs_grabber,
             const genom_context self)
{
  if(display_enabled)
  {
    display_left.init(const_cast<vpImage<unsigned char>&>(I_left->I), 10, 10, "Left image");
    display_right.init(const_cast<vpImage<unsigned char>&>(I_right->I), static_cast<int>(I_left->I.getWidth()) + 80, 10, "Right image"); // Right

    if(I_left_undistorted != NULL)
      display_left_undist.init(const_cast<vpImage<unsigned char>&>(I_left_undistorted->I), 2*static_cast<int>(I_left->I.getWidth()/2), 10, "Left undistorted image");
    if(I_right_undistorted != NULL)
      display_right_undist.init(const_cast<vpImage<unsigned char>&>(I_right_undistorted->I), 3*static_cast<int>(I_right->I.getWidth()/2), 10, "Right undistorted image");

    // cam_undistort used for frame display.
    vpCameraParameters cam_left = rs_grabber->g.getCameraParameters(RS2_STREAM_FISHEYE, vpCameraParameters::ProjWithKannalaBrandtDistortion, 1);
    left_cam_undistort.initPersProjWithoutDistortion(cam_left.get_px(), cam_left.get_py(), cam_left.get_u0(), cam_left.get_v0());

    return T265_loop;
  }

  else
  {
    return T265_pause_start;
  }
}


/** Codel refresh_display of task image_display.
 *
 * Triggered by T265_loop.
 * Yields to T265_pause_loop, T265_stop.
 */
genom_event
refresh_display(bool is_publishing, bool display_enabled,
                int32_t image_count, int16_t nb_display_coefficient,
                const T265_vp_image *I_left,
                const T265_vp_image *I_right,
                const T265_vp_image *I_left_undistorted,
                const T265_vp_image *I_right_undistorted,
                bool detection_enabled, const T265_tags *detected_tags,
                const genom_context self)
{
  if(is_publishing && display_enabled && (image_count % nb_display_coefficient == 0))
  {
    // Initialize display of undistorted image if not yet initialized and undistorted images exist.
    if(!display_left_undist.isInitialised() && I_left_undistorted != NULL)
      display_left_undist.init(const_cast<vpImage<unsigned char>&>(I_left_undistorted->I), 2*static_cast<int>(I_left->I.getWidth()/2), 10, "Left undistorted image");

    if(!display_right_undist.isInitialised() && I_right_undistorted != NULL)
      display_right_undist.init(const_cast<vpImage<unsigned char>&>(I_right_undistorted->I), 4*static_cast<int>(I_right->I.getWidth()/2), 10, "Right undistorted image");

    if(I_left != NULL)
    {
      vpDisplay::display(I_left->I);
      vpDisplay::displayText(I_left->I, 30, 30, "Click to quit", vpColor::red);
    }

    if(I_right != NULL)
    {
      vpDisplay::display(I_right->I);
      vpDisplay::displayText(I_right->I, 30, 30, "Click to quit", vpColor::red);
    }

    if(I_left_undistorted != NULL)
      vpDisplay::display(I_left_undistorted->I);
    if(I_right_undistorted != NULL)
      vpDisplay::display(I_right_undistorted->I);


    if(detected_tags->_buffer != NULL && detection_enabled) // Displaying AprilTag centers, corners, pose and message.
    {
      for(int i = 0; i < detected_tags->_maximum; i++)
      {
        // Display center.
        tag_center = vpImagePoint(detected_tags->_buffer[i].center._value.u, detected_tags->_buffer[i].center._value.v);
        vpDisplay::displayCross(I_left_undistorted->I, tag_center, 10, vpColor::red, 5);

        // Display corners.
        for(int j = 0; j < 4; j++)
        {
          vpDisplay::displayCross(I_left_undistorted->I, vpImagePoint(detected_tags->_buffer[i].corners_pos._value[j].u,detected_tags->_buffer[i].corners_pos._value[j].v), 20, vpColor::green, 2);
        }

        // Display pose.
        cMo.buildFrom(vpTranslationVector(detected_tags->_buffer[i].pos._value.x,detected_tags->_buffer[i].pos._value.y,detected_tags->_buffer[i].pos._value.z),
                      vpQuaternionVector(detected_tags->_buffer[i].att._value.qx,detected_tags->_buffer[i].att._value.qy,detected_tags->_buffer[i].att._value.qz,detected_tags->_buffer[i].att._value.qw));
        vpDisplay::displayFrame(I_left_undistorted->I, cMo, left_cam_undistort, 0.08);

        // Display message.
        vpDisplay::displayText(I_left_undistorted->I, tag_center.get_i(), tag_center.get_j(), detected_tags->_buffer[i].message._value, vpColor::green);
      }
    }

    if (vpDisplay::getClick(I_left->I, false) || vpDisplay::getClick(I_right->I, false)) {
        return T265_stop;
    }

    if(I_left != NULL)
      vpDisplay::flush(I_left->I);
    if(I_right != NULL)
      vpDisplay::flush(I_right->I);
    if(I_left_undistorted != NULL)
      vpDisplay::flush(I_left_undistorted->I);
    if(I_right_undistorted != NULL)
      vpDisplay::flush(I_right_undistorted->I);
  }

  return T265_pause_loop;
}


/** Codel stop_display of task image_display.
 *
 * Triggered by T265_stop.
 * Yields to T265_ether.
 */
genom_event
stop_display(T265_realsense_grabber **rs_grabber,
             T265_vp_image **I_left, T265_vp_image **I_right,
             T265_vp_image **I_left_undistorted,
             T265_vp_image **I_right_undistorted, bool *is_publishing,
             const genom_context self)
{
  *is_publishing = false; // Stop publishing
  (*rs_grabber)->g.close();

  if((*I_left) != NULL)
    vpDisplay::close((*I_left)->I);
  if((*I_right) != NULL)
    vpDisplay::close((*I_right)->I);
  if((*I_left_undistorted) != NULL)
    vpDisplay::close((*I_left_undistorted)->I);
  if((*I_right_undistorted) != NULL)
    vpDisplay::close((*I_right_undistorted)->I);

  std::cout << "stop image display\n";

  return T265_ether;
}
