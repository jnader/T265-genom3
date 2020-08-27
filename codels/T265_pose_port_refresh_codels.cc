#include "acT265.h"
#include "codels.h"
#include "T265_c_types.h"


/* --- Task pose_port_refresh ------------------------------------------- */
double odo_sec, odo_nsec;
vpHomogeneousMatrix world_M_robot;
vpTranslationVector world_t_robot;
vpQuaternionVector world_q_robot;

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
refresh_pose(bool is_publishing,
             const T265_realsense_grabber *rs_grabber,
             const T265_vp_odometry *poseref_odo_sensor,
             const T265_odom_state *odom_state,
             const genom_context self)
{
  if(is_publishing)
  {
    // Apply transformations on pose/velocity/acceleration.

    world_M_robot = poseref_odo_sensor->pose;

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

    // Pose.
    s->pos._present = true;
    s->pos._value.x = world_t_robot[0];
    s->pos._value.y = world_t_robot[1];
    s->pos._value.z = world_t_robot[2];

    s->att._present = true;
    s->att._value.qx = world_q_robot[0];
    s->att._value.qy = world_q_robot[1];
    s->att._value.qz = world_q_robot[2];
    s->att._value.qz = world_q_robot[3];

    // Velocity.
    s->vel._present = true;
    s->vel._value.vx = poseref_odo_sensor->vel[0];
    s->vel._value.vy = poseref_odo_sensor->vel[1];
    s->vel._value.vz = poseref_odo_sensor->vel[2];

    s->avel._present = true;
    s->avel._value.wx = poseref_odo_sensor->vel[3];
    s->avel._value.wy = poseref_odo_sensor->vel[4];
    s->avel._value.wz = poseref_odo_sensor->vel[5];

    // Acceleration.
    s->acc._present = true;
    s->acc._value.ax = poseref_odo_sensor->acc[0];
    s->acc._value.ay = poseref_odo_sensor->acc[1];
    s->acc._value.az = poseref_odo_sensor->acc[2];

    s->aacc._present = true;
    s->aacc._value.awx = poseref_odo_sensor->acc[3];
    s->aacc._value.awy = poseref_odo_sensor->acc[4];
    s->aacc._value.awz = poseref_odo_sensor->acc[5];

    // Covariances.

    if(odom_state->write(self))
      std::cout << "Error" << std::endl;
  }

  return T265_pause_loop;
}


/** Codel stop_pose_display of task pose_port_refresh.
 *
 * Triggered by T265_stop.
 * Yields to T265_ether.
 */
genom_event
stop_pose_display(const genom_context self)
{
  std::cout << "stop pose_refresh\n";
  return T265_ether;
}
