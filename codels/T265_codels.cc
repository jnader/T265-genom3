#include "acT265.h"
#include "codels.h"
#include "T265_c_types.h"

long sec;
double timestamp, nsec;
vpTranslationVector ctw;
vpQuaternionVector cqw;

/* --- Function start_publishing ---------------------------------------- */

/** Codel start_publish of function start_publishing.
 *
 * Returns genom_ok.
 */
genom_event
start_publish(T265_ids *ids, const genom_context self)
{
  rs2::config cfg;
  // Configuring pipeline streams.
  cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
  cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
  cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
  // cfg.enable_stream(RS2_STREAM_ACCEL);
  // cfg.enable_stream(RS2_STREAM_GYRO);

  // Callback to be executed when a frame arrives.
  std::function<void(const rs2::frame)> callback = [=](const rs2::frame& frame)
  {
    timestamp = frame.get_timestamp();

    if (rs2::frameset fs = frame.as<rs2::frameset>())
    {
      // With callbacks, all synchronized stream will arrive in a single frameset
      rs2::video_frame left_frame = fs.get_fisheye_frame(1);
      size_t size = left_frame.get_width() * left_frame.get_height();
      memcpy(ids->I_left->I.bitmap, left_frame.get_data(), size);
      ids->I_left->timestamp = timestamp;

      rs2::video_frame right_frame = fs.get_fisheye_frame(2);
      size = right_frame.get_width() * right_frame.get_height();
      memcpy(ids->I_right->I.bitmap, right_frame.get_data(), size);
      ids->I_right->timestamp = timestamp;

      ids->image_count++; // Images received succesfully.

      // In case there's a pose in the frameset.
      rs2_pose pose_data = fs.get_pose_frame().get_pose_data();
      ctw = vpTranslationVector(static_cast<double>(pose_data.translation.x),
                              static_cast<double>(pose_data.translation.y),
                              static_cast<double>(pose_data.translation.z));

      cqw = vpQuaternionVector(static_cast<double>(pose_data.rotation.x),
                            static_cast<double>(pose_data.rotation.y),
                            static_cast<double>(pose_data.rotation.z),
                            static_cast<double>(pose_data.rotation.w));

      // Timestamp.
      ids->poseref_odo_sensor->timestamp = timestamp;

      // Pose
      ids->poseref_odo_sensor->pose.buildFrom(ctw, cqw);

      // Velocity
      ids->poseref_odo_sensor->vel[0] = static_cast<double>(pose_data.velocity.x);
      ids->poseref_odo_sensor->vel[1] = static_cast<double>(pose_data.velocity.y);
      ids->poseref_odo_sensor->vel[2] = static_cast<double>(pose_data.velocity.z);

      ids->poseref_odo_sensor->vel[3] = static_cast<double>(pose_data.angular_velocity.x);
      ids->poseref_odo_sensor->vel[4] = static_cast<double>(pose_data.angular_velocity.y);
      ids->poseref_odo_sensor->vel[5] = static_cast<double>(pose_data.angular_velocity.z);

      // Acceleration
      ids->poseref_odo_sensor->acc[0] = static_cast<double>(pose_data.acceleration.x);
      ids->poseref_odo_sensor->acc[1] = static_cast<double>(pose_data.acceleration.y);
      ids->poseref_odo_sensor->acc[2] = static_cast<double>(pose_data.acceleration.z);

      ids->poseref_odo_sensor->acc[3] = static_cast<double>(pose_data.angular_acceleration.x);
      ids->poseref_odo_sensor->acc[4] = static_cast<double>(pose_data.angular_acceleration.y);
      ids->poseref_odo_sensor->acc[5] = static_cast<double>(pose_data.angular_acceleration.z);

      ids->pose_count++;
      ids->poseref_odo_sensor->tracker_confidence = pose_data.tracker_confidence;
    }
    else
    {
      rs2_pose pose_data = frame.as<rs2::pose_frame>().get_pose_data();

      // Stream that bypass synchronization (such as IMU, Pose, ...) will produce single frames
      ctw = vpTranslationVector(static_cast<double>(pose_data.translation.x),
                              static_cast<double>(pose_data.translation.y),
                              static_cast<double>(pose_data.translation.z));

      cqw = vpQuaternionVector(static_cast<double>(pose_data.rotation.x),
                            static_cast<double>(pose_data.rotation.y),
                            static_cast<double>(pose_data.rotation.z),
                            static_cast<double>(pose_data.rotation.w));

      // Timestamp.
      ids->poseref_odo_sensor->timestamp = timestamp;

      // Pose
      ids->poseref_odo_sensor->pose.buildFrom(ctw, cqw);

      // Velocity
      ids->poseref_odo_sensor->vel[0] = static_cast<double>(pose_data.velocity.x);
      ids->poseref_odo_sensor->vel[1] = static_cast<double>(pose_data.velocity.y);
      ids->poseref_odo_sensor->vel[2] = static_cast<double>(pose_data.velocity.z);

      ids->poseref_odo_sensor->vel[3] = static_cast<double>(pose_data.angular_velocity.x);
      ids->poseref_odo_sensor->vel[4] = static_cast<double>(pose_data.angular_velocity.y);
      ids->poseref_odo_sensor->vel[5] = static_cast<double>(pose_data.angular_velocity.z);

      // Acceleration
      ids->poseref_odo_sensor->acc[0] = static_cast<double>(pose_data.acceleration.x);
      ids->poseref_odo_sensor->acc[1] = static_cast<double>(pose_data.acceleration.y);
      ids->poseref_odo_sensor->acc[2] = static_cast<double>(pose_data.acceleration.z);

      ids->poseref_odo_sensor->acc[3] = static_cast<double>(pose_data.angular_acceleration.x);
      ids->poseref_odo_sensor->acc[4] = static_cast<double>(pose_data.angular_acceleration.y);
      ids->poseref_odo_sensor->acc[5] = static_cast<double>(pose_data.angular_acceleration.z);

      ids->pose_count++;
      ids->poseref_odo_sensor->tracker_confidence = pose_data.tracker_confidence;
    }
  };

  // Start publishing => open() vpRealSense2 object and start.
  ids->rs_grabber->g.open(cfg, callback);

  ids->I_left->I.resize(ids->rs_grabber->g.getIntrinsics(RS2_STREAM_FISHEYE, 1).height,
                        ids->rs_grabber->g.getIntrinsics(RS2_STREAM_FISHEYE, 1).width);

  ids->I_right->I.resize(ids->rs_grabber->g.getIntrinsics(RS2_STREAM_FISHEYE, 2).height,
                        ids->rs_grabber->g.getIntrinsics(RS2_STREAM_FISHEYE, 2).width);

  ids->nb_display_coefficient = 1;
  ids->display_enabled = false;
  ids->detection_enabled = false;

  // This will enable pose_port_refresh to be executed. (not directly skipped)
  ids->is_publishing = true;

  return genom_ok;
}


/* --- Function pause_publishing ---------------------------------------- */

/** Codel pause_publish of function pause_publishing.
 *
 * Returns genom_ok.
 */
genom_event
pause_publish(bool *is_publishing, const genom_context self)
{
  *is_publishing = false;
  return genom_ok;
}


/* --- Function set_pre_tf ---------------------------------------------- */

/** Codel set_pre_tf_codel of function set_pre_tf.
 *
 * Returns genom_ok.
 */
genom_event
set_pre_tf_codel(const sequence4_sequence4_double *new_pre_tf,
                 T265_vp_homogeneous_matrix **pre_tf,
                 const genom_context self)
{
  for(int i = 0; i < 4; i++)
    for(int j = 0; j < 4; j++)
      (*pre_tf)->mat[i][j] = new_pre_tf->_buffer[i]._buffer[j];

  std::cout << (*pre_tf)->mat << std::endl;

  return genom_ok;
}


/* --- Function set_post_tf --------------------------------------------- */

/** Codel set_post_tf_codel of function set_post_tf.
 *
 * Returns genom_ok.
 */
genom_event
set_post_tf_codel(const sequence4_sequence4_double *new_post_tf,
                  T265_vp_homogeneous_matrix **post_tf,
                  const genom_context self)
{
  for(int i = 0; i < 4; i++)
    for(int j = 0; j < 4; j++)
      (*post_tf)->mat[i][j] = new_post_tf->_buffer[i]._buffer[j];

  std::cout << (*post_tf)->mat << std::endl;

  return genom_ok;
}


/* --- Function set_display_frequency ----------------------------------- */

/** Codel set_display_frequency_codel of function set_display_frequency.
 *
 * Returns genom_ok.
 */
genom_event
set_display_frequency_codel(double frequency,
                            int16_t *nb_display_coefficient,
                            bool *display_enabled,
                            const genom_context self)
{
  if(frequency > 0 && frequency <= 30)
  {
    *nb_display_coefficient = 30. / frequency;
    *display_enabled = true;
  }
  else
  {
    if(frequency == 0)
      *display_enabled = false;
    else
      std::cout << "Frame rate : [0.1,30Hz]" << std::endl;
  }
  return genom_ok;
}


/* --- Function enable_pose --------------------------------------------- */

/** Codel enable_pose_codel of function enable_pose.
 *
 * Returns genom_ok.
 */
genom_event
enable_pose_codel(bool *pose_enabled, const genom_context self)
{
  *pose_enabled = true;
  return genom_ok;
}


/* --- Function disable_pose -------------------------------------------- */

/** Codel disable_pose_codel of function disable_pose.
 *
 * Returns genom_ok.
 */
genom_event
disable_pose_codel(bool *pose_enabled, const genom_context self)
{
  *pose_enabled = false;
  return genom_ok;
}


/* --- Function enable_detection ---------------------------------------- */

/** Codel enable_detection_codel of function enable_detection.
 *
 * Returns genom_ok.
 */
genom_event
enable_detection_codel(bool *detection_enabled,
                       const genom_context self)
{
  *detection_enabled = true;
  return genom_ok;
}


/* --- Function disable_detection --------------------------------------- */

/** Codel disable_detection_codel of function disable_detection.
 *
 * Returns genom_ok.
 */
genom_event
disable_detection_codel(bool *detection_enabled,
                        const genom_context self)
{
  *detection_enabled = false;
  return genom_ok;
}


/* --- Function log ----------------------------------------------------- */

/** Codel T265_log of function log.
 *
 * Returns genom_ok.
 * Throws T265_e_sys.
 */
genom_event
T265_log(const char path[64], uint32_t decimation, T265_log_s **log,
         const genom_context self)
{
  int fd;

  fd = open(path, O_WRONLY|O_APPEND|O_CREAT|O_TRUNC, 0666);
  if (fd < 0) return T265_e_sys_error(path, self);

  if (write(fd, T265_log_header_fmt "\n", sizeof(T265_log_header_fmt)) < 0)
      return T265_e_sys_error(path, self);

  if ((*log)->req.aio_fildes >= 0) {
      close((*log)->req.aio_fildes);

      if ((*log)->pending)
      while (aio_error(&(*log)->req) == EINPROGRESS)
          /* empty body */;
  }

  (*log)->req.aio_fildes = fd;
  (*log)->pending = false;
  (*log)->skipped = false;
  (*log)->decimation = decimation < 1 ? 1 : decimation;
  (*log)->missed = 0;
  (*log)->total = 0;

  return genom_ok;
}


/* --- Function log_stop ------------------------------------------------ */

/** Codel T265_log_stop of function log_stop.
 *
 * Returns genom_ok.
 */
genom_event
T265_log_stop(T265_log_s **log, const genom_context self)
{
  if (*log && (*log)->req.aio_fildes >= 0)
  close((*log)->req.aio_fildes);
  (*log)->req.aio_fildes = -1;

  return genom_ok;
}
