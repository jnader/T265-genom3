#include "acT265.h"

#include "T265_c_types.h"
#include "codels.h"
#include <thread>

/* --- Task grabber ----------------------------------------------------- */

long sec;
double timestamp, nsec;
vpTranslationVector ctw;
vpQuaternionVector cqw;

/** Codel init_grabber of task grabber.
 *
 * Triggered by T265_start.
 * Yields to T265_ether.
 */
genom_event
init_grabber(T265_ids *ids, const genom_context self)
{
  ids->is_publishing = true; // Default start publishing.
  ids->rs_grabber = new T265_realsense_grabber;
  ids->I_left = new T265_vp_image;
  ids->I_right = new T265_vp_image;
  ids->poseref_odo_sensor = new T265_vp_odometry;
  ids->poseref_odo_sensor->vel.resize(6);
  ids->poseref_odo_sensor->acc.resize(6);
  ids->pre_tf = new T265_vp_homogeneous_matrix;
  ids->post_tf = new T265_vp_homogeneous_matrix;
  ids->nb_display_coefficient = 1;
  ids->display_enabled = false;
  ids->detection_enabled = false;

  /* start logging variables*/
  ids->log = new T265_log_s;
  if (!ids->log) abort();

  ids->log->req.aio_fildes = -1;
  ids->log->req.aio_offset = 0;
  ids->log->req.aio_buf = ids->log->buffer;
  ids->log->req.aio_nbytes = 0;
  ids->log->req.aio_reqprio = 0;
  ids->log->req.aio_sigevent.sigev_notify = SIGEV_NONE;
  ids->log->req.aio_lio_opcode = LIO_NOP;
  ids->log->pending = false;
  ids->log->skipped = false;
  ids->log->decimation = 1;
  ids->log->missed = 0;
  ids->log->total = 0;

  // Configuring pipeline streams.
  rs2::config cfg;
  // cfg.enable_all_streams();
  cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
  cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
  cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
  // cfg.enable_stream(RS2_STREAM_ACCEL);
  // cfg.enable_stream(RS2_STREAM_GYRO);

  std::function<void(rs2::frame)> callback = [&](const rs2::frame& frame)
  {
    timestamp = frame.get_timestamp();
    ids->frame_nb = frame.get_frame_number();

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

  ids->rs_grabber->g.open(cfg, callback);

  ids->I_left->I.resize(ids->rs_grabber->g.getIntrinsics(RS2_STREAM_FISHEYE, 1).height,
                        ids->rs_grabber->g.getIntrinsics(RS2_STREAM_FISHEYE, 1).width);

  ids->I_right->I.resize(ids->rs_grabber->g.getIntrinsics(RS2_STREAM_FISHEYE, 2).height,
                         ids->rs_grabber->g.getIntrinsics(RS2_STREAM_FISHEYE, 2).width);

  return T265_ether;
}
