#include "acT265.h"

#include "T265_c_types.h"
#include "codels.h"
#include <thread>

/* --- Task grabber ----------------------------------------------------- */

long sec;
double timestamp, nsec;

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
  ids->poseref_M_sensor = new T265_vp_homogeneous_matrix;
  ids->pose_data.pos._value.x = 0.;
  ids->pose_data.pos._value.y = 0.;
  ids->pose_data.pos._value.z = 0.;

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
      vpTranslationVector ctw(static_cast<double>(pose_data.translation.x),
                              static_cast<double>(pose_data.translation.y),
                              static_cast<double>(pose_data.translation.z));
      
      vpQuaternionVector cqw(static_cast<double>(pose_data.rotation.x),
                             static_cast<double>(pose_data.rotation.y),
                             static_cast<double>(pose_data.rotation.z),
                             static_cast<double>(pose_data.rotation.w));

      // Timestamp.
      sec    = timestamp / 1000;
      nsec = ((long)timestamp % 1000) * 1000000;
      ids->pose_data.ts.sec  = static_cast<int32_t>(sec);
      ids->pose_data.ts.nsec = static_cast<int32_t>(nsec);

      ids->pose_data.pos._present = true;
      ids->pose_data.pos._value.x = ctw[0];
      ids->pose_data.pos._value.y = ctw[1];
      ids->pose_data.pos._value.z = ctw[2];

      ids->pose_data.att._present = true;
      ids->pose_data.att._value.qw = cqw[3];
      ids->pose_data.att._value.qx = cqw[0];
      ids->pose_data.att._value.qy = cqw[1];
      ids->pose_data.att._value.qz = cqw[2];

      ids->poseref_M_sensor->data.buildFrom(ctw, cqw);

      ids->pose_data.vel._present  = true;
      ids->pose_data.avel._present = true;

      ids->pose_data.vel._value.vx = static_cast<double>(pose_data.velocity.x);
      ids->pose_data.vel._value.vy = static_cast<double>(pose_data.velocity.y);
      ids->pose_data.vel._value.vz = static_cast<double>(pose_data.velocity.z);

      ids->pose_data.avel._value.wx = static_cast<double>(pose_data.angular_velocity.x);
      ids->pose_data.avel._value.wy = static_cast<double>(pose_data.angular_velocity.y);
      ids->pose_data.avel._value.wz = static_cast<double>(pose_data.angular_velocity.z);

      ids->pose_data.acc._present  = true;
      ids->pose_data.aacc._present = true;

      ids->pose_data.acc._value.ax = static_cast<double>(pose_data.acceleration.x);
      ids->pose_data.acc._value.ay = static_cast<double>(pose_data.acceleration.y);
      ids->pose_data.acc._value.az = static_cast<double>(pose_data.acceleration.z);

      ids->pose_data.aacc._value.awx = static_cast<double>(pose_data.angular_acceleration.x);
      ids->pose_data.aacc._value.awy = static_cast<double>(pose_data.angular_acceleration.y);
      ids->pose_data.aacc._value.awz = static_cast<double>(pose_data.angular_acceleration.z);

      ids->pose_count++;
      // confidence = pose_data.tracker_confidence;
    }
    else
    {
      rs2_pose pose_data = frame.as<rs2::pose_frame>().get_pose_data();

      // Stream that bypass synchronization (such as IMU, Pose, ...) will produce single frames
      vpTranslationVector ctw(static_cast<double>(pose_data.translation.x),
                              static_cast<double>(pose_data.translation.y),
                              static_cast<double>(pose_data.translation.z));
      
      vpQuaternionVector cqw(static_cast<double>(pose_data.rotation.x),
                             static_cast<double>(pose_data.rotation.y),
                             static_cast<double>(pose_data.rotation.z),
                             static_cast<double>(pose_data.rotation.w));

      // Timestamp.
      sec    = timestamp / 1000;
      nsec = ((long)timestamp % 1000) * 1000000;
      ids->pose_data.ts.sec  = static_cast<int32_t>(sec);
      ids->pose_data.ts.nsec = static_cast<int32_t>(nsec);

      ids->pose_data.pos._present = true;
      ids->pose_data.pos._value.x = ctw[0];
      ids->pose_data.pos._value.y = ctw[1];
      ids->pose_data.pos._value.z = ctw[2];

      ids->pose_data.att._present = true;
      ids->pose_data.att._value.qw = cqw[3];
      ids->pose_data.att._value.qx = cqw[0];
      ids->pose_data.att._value.qy = cqw[1];
      ids->pose_data.att._value.qz = cqw[2];

      ids->poseref_M_sensor->data.buildFrom(ctw, cqw);

      ids->pose_data.vel._present  = true;
      ids->pose_data.avel._present = true;

      ids->pose_data.vel._value.vx = static_cast<double>(pose_data.velocity.x);
      ids->pose_data.vel._value.vy = static_cast<double>(pose_data.velocity.y);
      ids->pose_data.vel._value.vz = static_cast<double>(pose_data.velocity.z);

      ids->pose_data.avel._value.wx = static_cast<double>(pose_data.angular_velocity.x);
      ids->pose_data.avel._value.wy = static_cast<double>(pose_data.angular_velocity.y);
      ids->pose_data.avel._value.wz = static_cast<double>(pose_data.angular_velocity.z);

      ids->pose_data.acc._present  = true;
      ids->pose_data.aacc._present = true;

      ids->pose_data.acc._value.ax = static_cast<double>(pose_data.acceleration.x);
      ids->pose_data.acc._value.ay = static_cast<double>(pose_data.acceleration.y);
      ids->pose_data.acc._value.az = static_cast<double>(pose_data.acceleration.z);

      ids->pose_data.aacc._value.awx = static_cast<double>(pose_data.angular_acceleration.x);
      ids->pose_data.aacc._value.awy = static_cast<double>(pose_data.angular_acceleration.y);
      ids->pose_data.aacc._value.awz = static_cast<double>(pose_data.angular_acceleration.z);

      ids->pose_count++;

      // confidence = pose_data.tracker_confidence;
    }
  };

  ids->rs_grabber->g.open(cfg, callback);

  ids->I_left->I.resize(ids->rs_grabber->g.getPipelineProfile().get_stream(RS2_STREAM_FISHEYE).as<rs2::video_stream_profile>().height(),
                        ids->rs_grabber->g.getPipelineProfile().get_stream(RS2_STREAM_FISHEYE).as<rs2::video_stream_profile>().width());

  ids->I_right->I.resize(ids->rs_grabber->g.getPipelineProfile().get_stream(RS2_STREAM_FISHEYE).as<rs2::video_stream_profile>().height(),
                         ids->rs_grabber->g.getPipelineProfile().get_stream(RS2_STREAM_FISHEYE).as<rs2::video_stream_profile>().width());

  return T265_ether;
}
