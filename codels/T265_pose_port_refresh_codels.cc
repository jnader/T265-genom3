#include "acT265.h"
#include "codels.h"
#include "T265_c_types.h"


/* --- Task pose_port_refresh ------------------------------------------- */


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
    std::cout << "entered if" << std::endl;

  return T265_loop;
}


/** Codel refresh_pose of task pose_port_refresh.
 *
 * Triggered by T265_loop.
 * Yields to T265_pause_loop, T265_stop.
 */
genom_event
refresh_pose(const T265_realsense_grabber *rs_grabber,
             const or_pose_estimator_state *pose_data,
             const T265_vp_homogeneous_matrix *poseref_M_sensor,
             const T265_odom_state *odom_state,
             const genom_context self)
{
  double start = vpTime::measureTimeMs();

  or_pose_estimator_state *s = odom_state->data(self);

  s->pos._present = true;
  s->pos._value.x = pose_data->pos._value.x;
  s->pos._value.y = pose_data->pos._value.y;
  s->pos._value.z = pose_data->pos._value.z;

  s->att._present = true;
  s->att._value.qx = pose_data->att._value.qx;
  s->att._value.qy = pose_data->att._value.qy;
  s->att._value.qz = pose_data->att._value.qz;

  s->vel._present = true;
  s->vel._value.vx = pose_data->vel._value.vx;
  s->vel._value.vy = pose_data->vel._value.vy; 
  s->vel._value.vz = pose_data->vel._value.vz;

  s->avel._present = true;
  s->avel._value.wx = pose_data->avel._value.wx;
  s->avel._value.wy = pose_data->avel._value.wy; 
  s->avel._value.wz = pose_data->avel._value.wz;

  s->acc._present = true;
  s->acc._value.ax = pose_data->acc._value.ax;
  s->acc._value.ay = pose_data->acc._value.ay; 
  s->acc._value.az = pose_data->acc._value.az;

  s->aacc._present = true;
  s->aacc._value.awx = pose_data->aacc._value.awx;
  s->aacc._value.awy = pose_data->aacc._value.awy; 
  s->aacc._value.awz = pose_data->aacc._value.awz;

  if(odom_state->write(self))
    std::cout << "Error" << std::endl;

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
  // std::cout << "End" << std::endl;
  return T265_ether;
}
