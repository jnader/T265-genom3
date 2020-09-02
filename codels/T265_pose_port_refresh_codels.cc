#include "acT265.h"
#include "codels.h"
#include "T265_c_types.h"


/* --- Task pose_port_refresh ------------------------------------------- */
double odo_sec, odo_nsec;
vpHomogeneousMatrix world_M_robot;
vpTranslationVector world_t_robot;
vpQuaternionVector world_q_robot;
vpVelocityTwistMatrix world_V_poseref;
vpColVector world_v, world_a;
double cov_pos, cov_twist;

/** Codel init_port of task pose_port_refresh.
 *
 * Triggered by T265_start.
 * Yields to T265_loop.
 */
genom_event
init_port(const T265_odom_state *odom_state, const genom_context self)
{
  or_pose_estimator_state *s = odom_state->data(self);

  s->pos._present = true;
  s->att._present = true;
  s->vel._present = true;
  s->avel._present = true;
  s->acc._present  = true;
  s->aacc._present = true;

  if(odom_state->write(self))
    std::cout << "Error" << std::endl;

  return T265_loop;
}


/** Codel refresh_pose of task pose_port_refresh.
 *
 * Triggered by T265_loop.
 * Yields to T265_pause_loop, T265_stop.
 */
genom_event
refresh_pose(T265_log_s **log, bool is_publishing,
             const T265_realsense_grabber *rs_grabber,
             const T265_vp_odometry *poseref_odo_sensor,
             const T265_vp_homogeneous_matrix *pre_tf,
             const T265_vp_homogeneous_matrix *post_tf,
             const T265_odom_state *odom_state,
             const genom_context self)
{
  if(is_publishing)
  {
    // Apply transformations on pose/velocity/acceleration.
    // Pose
    world_M_robot = pre_tf->mat * poseref_odo_sensor->pose * post_tf->mat;
    // Velocity/Acceleration
    world_V_poseref.buildFrom(pre_tf->mat);
    world_v = world_V_poseref * poseref_odo_sensor->vel;
    world_a = world_V_poseref * poseref_odo_sensor->acc;

    // Extracting translation vector from final pose.
    world_M_robot.extract(world_t_robot);
    // Extracting quaternion vector from final pose.
    world_M_robot.extract(world_q_robot);

    or_pose_estimator_state *s = odom_state->data(self);

    odo_sec    = poseref_odo_sensor->timestamp / 1000;
    odo_nsec   = ((long)poseref_odo_sensor->timestamp % 1000) * 1000000;

    // Timestamp.
    s->ts.sec  = static_cast<int32_t>(odo_sec);
    s->ts.nsec = static_cast<int32_t>(odo_nsec);

    // Intrinsic.
    // Intrinsic = false, mainly because we are transforming velocity to world (base) frame.
    // Isn't T265 pose measures considered intrinsic? In this case, intrinsic = true will cause
    // problem because it is only dealing with magnetometers.
    s->intrinsic = false;

    // Pose.
    s->pos._present = true;
    s->pos._value.x = world_t_robot[0];
    s->pos._value.y = world_t_robot[1];
    s->pos._value.z = world_t_robot[2];

    s->att._present = true;
    s->att._value.qx = world_q_robot[0];
    s->att._value.qy = world_q_robot[1];
    s->att._value.qz = world_q_robot[2];
    s->att._value.qw = world_q_robot[3];

    // Velocity.
    s->vel._present = false;
    s->vel._value.vx = poseref_odo_sensor->vel[0];
    s->vel._value.vy = poseref_odo_sensor->vel[1];
    s->vel._value.vz = poseref_odo_sensor->vel[2];

    s->avel._present = false;
    s->avel._value.wx = poseref_odo_sensor->vel[3];
    s->avel._value.wy = poseref_odo_sensor->vel[4];
    s->avel._value.wz = poseref_odo_sensor->vel[5];

    // Acceleration.
    s->acc._present = false;
    s->acc._value.ax = poseref_odo_sensor->acc[0];
    s->acc._value.ay = poseref_odo_sensor->acc[1];
    s->acc._value.az = poseref_odo_sensor->acc[2];

    s->aacc._present = false;
    s->aacc._value.awx = poseref_odo_sensor->acc[3];
    s->aacc._value.awy = poseref_odo_sensor->acc[4];
    s->aacc._value.awz = poseref_odo_sensor->acc[5];

    // Covariances.
    /* This calculation is based on librealsense-ros package implementation. */
    // double cov_pos   = ids->uncertainties.linear_acc_cov  * static_cast<double>(pow(10, 3 - confidence));
    // double cov_twist = ids->uncertainties.angular_vel_cov * static_cast<double>(pow(10.0, static_cast<double>(1 - (int)confidence)));

    // Should be tested multiple times.
    cov_pos   = 0.0009 * 0.0009;
    cov_twist = 0.0019 * 0.0019;

    // Uncertainty on the estimated position.
    s->pos_cov._present  = true;
    s->pos_cov._value.cov[0] = cov_pos;
    s->pos_cov._value.cov[1] = 0;
    s->pos_cov._value.cov[2] = cov_pos;
    s->pos_cov._value.cov[3] = 0;
    s->pos_cov._value.cov[4] = 0;
    s->pos_cov._value.cov[5] = cov_pos;

    // Uncertainty on the estimated attitude.
    // Based on vicon-genom3.
    s->att_cov._present      = true;
    s->att_cov._value.cov[0] = cov_twist * (1 - world_q_robot.w() * world_q_robot.w());
    s->att_cov._value.cov[1] = cov_twist * -world_q_robot.w() * world_q_robot.x();
    s->att_cov._value.cov[2] = cov_twist * (1 - world_q_robot.x() * world_q_robot.x());
    s->att_cov._value.cov[3] = cov_twist * world_q_robot.w() * world_q_robot.y();
    s->att_cov._value.cov[4] = cov_twist * -world_q_robot.x() * world_q_robot.y();
    s->att_cov._value.cov[5] = cov_twist * (1 - world_q_robot.y() * world_q_robot.y());
    s->att_cov._value.cov[6] = cov_twist * -world_q_robot.w() * world_q_robot.z();
    s->att_cov._value.cov[7] = cov_twist * -world_q_robot.x() * world_q_robot.z();
    s->att_cov._value.cov[8] = cov_twist * -world_q_robot.y() * world_q_robot.z();
    s->att_cov._value.cov[9] = cov_twist * (1 - world_q_robot.z() * world_q_robot.z());

    // Uncertainty on the estimated linear velocity.
    s->vel_cov._present      = false;
    // s->vel_cov._value.cov[0] = ids->uncertainties.linear_acc_cov * pow(10, 3 - confidence);
    // s->vel_cov._value.cov[1] = 0;
    // s->vel_cov._value.cov[2] = ids->uncertainties.linear_acc_cov * pow(10, 3 - confidence);
    // s->vel_cov._value.cov[3] = 0;
    // s->vel_cov._value.cov[4] = 0;
    // s->vel_cov._value.cov[5] = ids->uncertainties.linear_acc_cov * pow(10, 3 - confidence);

    // Uncertainty on the estimated angular velocity.
    s->avel_cov._present      = false;
    // s->avel_cov._value.cov[0] = ids->uncertainties.angular_vel_cov * pow(10, 2 - (int)confidence);
    // s->avel_cov._value.cov[1] = 0;
    // s->avel_cov._value.cov[2] = ids->uncertainties.angular_vel_cov * pow(10, 2 - (int)confidence);
    // s->avel_cov._value.cov[3] = 0;
    // s->avel_cov._value.cov[4] = 0;
    // s->avel_cov._value.cov[5] = ids->uncertainties.angular_vel_cov * pow(10, 2 - (int)confidence);

    // Uncertainty on the estimated linear acceleration.
    s->acc_cov._present       = false;
    // s->acc_cov._value.cov[0]  = ids->uncertainties.linear_acc_cov * pow(10, 3 - confidence);
    // s->acc_cov._value.cov[1]  = 0;
    // s->acc_cov._value.cov[2]  = ids->uncertainties.linear_acc_cov * pow(10, 3 - confidence);
    // s->acc_cov._value.cov[3]  = 0;
    // s->acc_cov._value.cov[4]  = 0;
    // s->acc_cov._value.cov[5]  = ids->uncertainties.linear_acc_cov * pow(10, 3 - confidence);

    // Uncertainty on the estimated angular acceleration.
    s->aacc_cov._present      = false;
    // s->aacc_cov._value.cov[0] = ids->uncertainties.angular_vel_cov * pow(10, 2 - (int)confidence);
    // s->aacc_cov._value.cov[1] = 0;
    // s->aacc_cov._value.cov[2] = ids->uncertainties.angular_vel_cov * pow(10, 2 - (int)confidence);
    // s->aacc_cov._value.cov[3] = 0;
    // s->aacc_cov._value.cov[4] = 0;
    // s->aacc_cov._value.cov[5] = ids->uncertainties.angular_vel_cov * pow(10, 2 - (int)confidence);

    if(odom_state->write(self))
      std::cout << "Error" << std::endl;

    // Log data
    T265_main_log(*s, 3, *log);

  }

  return T265_pause_loop;
}


/** Codel stop_pose_display of task pose_port_refresh.
 *
 * Triggered by T265_stop.
 * Yields to T265_ether.
 */
genom_event
stop_pose_display(T265_vp_odometry **poseref_odo_sensor,
                  T265_vp_homogeneous_matrix **pre_tf,
                  T265_vp_homogeneous_matrix **post_tf,
                  const genom_context self)
{
  delete (*poseref_odo_sensor);
  delete (*pre_tf);
  delete (*post_tf);
  std::cout << "stop pose_refresh\n";
  return T265_ether;
}
