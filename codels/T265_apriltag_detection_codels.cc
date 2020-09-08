#include "acT265.h"
#include "codels.h"
#include "T265_c_types.h"


/* --- Task apriltag_detection ------------------------------------------ */
vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
vpDetectorAprilTag *detector;
vpCameraParameters cam_undistort;
std::vector<vpHomogeneousMatrix> cMo_vec;
std::vector<std::vector<vpImagePoint> > tag_corners;
std::vector<int> tag_ids;
vpImagePoint center;
vpTranslationVector cto;
vpQuaternionVector cqo;
vpArray2D<int> mapU_left, mapV_left, mapU_right, mapV_right;
vpArray2D<float> mapDu_left, mapDv_left, mapDu_right, mapDv_right;
vpCameraParameters cam_left, cam_right;

T265_tags *tags;

long tmp_sec;
double tmp_nsec;

/** Codel init_detector of task apriltag_detection.
 *
 * Triggered by T265_start.
 * Yields to T265_pause_start, T265_loop.
 */
genom_event
init_detector(bool detection_enabled,
              const T265_realsense_grabber *rs_grabber,
              const T265_vp_image *I_left,
              const T265_vp_image *I_right, double *tag_size,
              T265_tags *detected_tags,
              const T265_port_tags *port_tags,
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

    vpCameraParameters cam_left = rs_grabber->g.getCameraParameters(RS2_STREAM_FISHEYE, vpCameraParameters::ProjWithKannalaBrandtDistortion, 1);
    cam_undistort.initPersProjWithoutDistortion(cam_left.get_px(), cam_left.get_py(), cam_left.get_u0(), cam_left.get_v0());

    detector = new vpDetectorAprilTag(tagFamily);

    detector->setAprilTagQuadDecimate(1);
    detector->setAprilTagPoseEstimationMethod(poseEstimationMethod);
    detector->setAprilTagNbThreads(2);
    detector->setDisplayTag(false, -1 < 0 ? vpColor::none : vpColor::getColor(-1), 2);
    detector->setZAlignedWithCameraAxis(false);

    *tag_size = 0.08; // Default value for tag size.

    // IDS
    //
    detected_tags->_buffer = NULL;
    detected_tags->_length = 0;
    //

    // OUTPORT
    //
    tags = port_tags->data(self);
    tags->_buffer = NULL;
    tags->_length = 0;

    if(port_tags->write(self))
      std::cout << "Error" << std::endl;
    //

    return T265_loop;
  }

  else
    return T265_pause_start;
}


/** Codel loop_detector of task apriltag_detection.
 *
 * Triggered by T265_loop.
 * Yields to T265_pause_loop, T265_stop.
 */
genom_event
loop_detector(bool detection_enabled, bool is_publishing,
              const T265_vp_image *I_left,
              const T265_vp_image *I_right,
              T265_vp_image **I_left_undistorted,
              T265_vp_image **I_right_undistorted, double tag_size,
              T265_tags *detected_tags,
              const T265_port_tags *port_tags,
              const genom_context self)
{
  if(is_publishing && detection_enabled)
  {
    vpImageTools::undistort(I_left->I, mapU_left, mapV_left, mapDu_left, mapDv_left, (*I_left_undistorted)->I);
    vpImageTools::undistort(I_right->I, mapU_right, mapV_right, mapDu_right, mapDv_right, (*I_right_undistorted)->I);

    (*I_left_undistorted)->timestamp = I_left->timestamp;
    (*I_right_undistorted)->timestamp = I_right->timestamp;

    cMo_vec.clear();

    detector->detect((*I_left_undistorted)->I, tag_size, cam_undistort, cMo_vec);

    if(cMo_vec.size() == 0) // No tags detected in this iteration, release the buffer of detected_tags.
    {
      if(detected_tags->_length != 0)
      {
        // IDS
        //
        detected_tags->_maximum = 0;
        detected_tags->_length = 0;
        delete [] detected_tags->_buffer;
        detected_tags->_buffer = NULL;
        //

        *tags = *detected_tags; // Empty output port.

        if(port_tags->write(self))
          std::cout << "Error" << std::endl;
      }
    }

    else // Tag(s) detected.
    {
      if(detected_tags->_length != 0) // Releasing already existing buffer.
      {
        delete [] detected_tags->_buffer;
        detected_tags->_buffer = NULL;
      }

      detected_tags->_maximum = cMo_vec.size();
      detected_tags->_length = 0;

      tag_corners = detector->getTagsCorners(); // Get all tags corners.
      tag_ids = detector->getTagsId(); // Get all tags IDs.

      if(detected_tags->_buffer == NULL)
      {
        detected_tags->_buffer = new apriltag_tag[detected_tags->_maximum];
      }

      // Filling detected_tags data structure.
      for(int i = 0; i < cMo_vec.size(); i++)
      {
        tmp_sec    = (*I_left_undistorted)->timestamp / 1000;
        tmp_nsec = ((long)(*I_left_undistorted)->timestamp % 1000) * 1000000;

        // Save timestamp of image as timestamp of apriltag.
        detected_tags->_buffer[i].ts.sec  = static_cast<int32_t>(tmp_sec);
        detected_tags->_buffer[i].ts.nsec = static_cast<int32_t>(tmp_nsec);

        // Save ID of tag.
        detected_tags->_buffer[i].id = tag_ids[i];

        // Save center of tag.
        center = detector->getCog(i);
        detected_tags->_buffer[i].center._present = true;
        detected_tags->_buffer[i].center._value.u = center.get_i();
        detected_tags->_buffer[i].center._value.v = center.get_j();

        // Save corners.
        detected_tags->_buffer[i].corners_pos._present = true;
        for(int j = 0; j < 4; j++)
        {
          detected_tags->_buffer[i].corners_pos._value[j].u = tag_corners[i][j].get_i();
          detected_tags->_buffer[i].corners_pos._value[j].v = tag_corners[i][j].get_j();
        }

        // Save apriltag's pose.
        cMo_vec[i].extract(cto);
        cMo_vec[i].extract(cqo);

        detected_tags->_buffer[i].pos._present = true;
        detected_tags->_buffer[i].pos._value.x = cto[0];
        detected_tags->_buffer[i].pos._value.y = cto[1];
        detected_tags->_buffer[i].pos._value.z = cto[2];

        detected_tags->_buffer[i].att._present = true;
        detected_tags->_buffer[i].att._value.qx = cqo[0];
        detected_tags->_buffer[i].att._value.qy = cqo[1];
        detected_tags->_buffer[i].att._value.qz = cqo[2];
        detected_tags->_buffer[i].att._value.qw = cqo[3];

        // Save apriltag's message.
        detected_tags->_buffer[i].message._present = true;
        strcpy(detected_tags->_buffer[i].message._value, detector->getMessage(i).c_str());

        detected_tags->_length++;
      }
    }

    // OUTPORT
    //
    tags = port_tags->data(self); // Is it necessary to read ?
    *tags = *detected_tags;

    if(port_tags->write(self))
      std::cout << "Error" << std::endl;
    //
  }

  else // In case disabling detection, should update port.
  {
    if(detected_tags->_length != 0)
      {
        // IDS
        //
        detected_tags->_maximum = 0;
        detected_tags->_length = 0;
        delete [] detected_tags->_buffer;
        detected_tags->_buffer = NULL;
        //

        *tags = *detected_tags; // Empty output port.

        if(port_tags->write(self))
          std::cout << "Error" << std::endl;
      }
  }

  return T265_pause_loop;
}


/** Codel kill_detector of task apriltag_detection.
 *
 * Triggered by T265_stop.
 * Yields to T265_ether.
 */
genom_event
kill_detector(T265_tags *detected_tags,
              T265_vp_image **I_left_undistorted,
              T265_vp_image **I_right_undistorted,
              const genom_context self)
{
  delete (*I_left_undistorted);
  delete (*I_right_undistorted);

  delete detector;
  detector = NULL;

  if(detected_tags->_buffer != NULL && detected_tags->_length != 0) // Releasing already existing buffer.
  {
    delete [] detected_tags->_buffer;
    detected_tags->_buffer = NULL;
  }

  std::cout << "stop detector\n";

  return T265_ether;
}
