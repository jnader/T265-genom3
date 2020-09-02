#ifndef _CODELS_H
#define _CODELS_H

#include "T265_c_types.h"

#include <visp3/core/vpConfig.h>
#ifndef VISP_HAVE_REALSENSE2
#error "ViSP is not built with libRealSense support...";
#endif

#ifndef VISP_HAVE_X11
#error "ViSP is not built with X11 support...";
#endif

#include <visp3/core/vpCameraParameters.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/core/vpImageTools.h>

#include <iomanip>
#include <iostream>
#include <sys/time.h>
#include <aio.h>
#include <err.h>
#include <unistd.h>
#include <fcntl.h>

struct T265_realsense_grabber
{
    vpRealSense2 g;
};

struct T265_vp_image
{
    double timestamp;
    vpImage<unsigned char> I;
};

struct T265_vp_homogeneous_matrix
{
    vpHomogeneousMatrix mat;
};

struct T265_vp_odometry
{
    double timestamp;
    vpHomogeneousMatrix pose;
    vpColVector vel, acc;
    unsigned int tracker_confidence;
};


/*
 Log
*/
static inline genom_event
T265_e_sys_error(const char *s, genom_context self)
{
  T265_e_sys_detail d;
  char buf[64], *p;

  d.code = errno;
#ifdef STRERROR_R_CHAR_P
  /* glibc managed to mess up with this function */
  p = strerror_r(d.code, buf, sizeof(buf));
#else
  char* c = strerror_r(d.code, buf, sizeof(buf));
  p = buf;
#endif
  snprintf(d.what, sizeof(d.what), "%s%s%s", s ? s : "", s ? ": " : "", p);

  return T265_e_sys(&d, self);
}

// Data structure
struct T265_log_s {
  struct aiocb req;
  char buffer[4096];
  bool pending, skipped;
  uint32_t decimation;
  size_t missed, total;

# define T265_logfmt	" %g"
# define T265_log_header_fmt                                                                \
  "ts confidence "                                                                           \
  "x y z roll pitch yaw "                                                                    \
  "vx vy vz wx wy wz ax ay az aaz aay aaz"
  /*"x y z qx qy qz qw "                                                                     \*/

# define T265_log_fmt                                                                       \
  "%d.%09d"                                                                                  \
  " %d"                                                                                      \
  T265_logfmt T265_logfmt T265_logfmt T265_logfmt T265_logfmt T265_logfmt              \
  T265_logfmt T265_logfmt T265_logfmt T265_logfmt T265_logfmt T265_logfmt              \
  T265_logfmt T265_logfmt T265_logfmt T265_logfmt T265_logfmt T265_logfmt
};

static void
T265_main_log(const or_pose_estimator_state &s, unsigned int confidence, T265_log_s *log)
{
  if (log->req.aio_fildes >= 0) {
    log->total++;
    if (log->total % log->decimation == 0) {
      if (log->pending) {
        if (aio_error(&log->req) != EINPROGRESS) {
          log->pending = false;
          if (aio_return(&log->req) <= 0) {
            warn("log");
            close(log->req.aio_fildes);
            log->req.aio_fildes = -1;
          }
        } else {
          log->skipped = true;
          log->missed++;
        }
      }
    }
  }

  if (log->req.aio_fildes >= 0 && !log->pending) {
    const double
      x   = s.pos._value.x,
      y   = s.pos._value.y,
      z   = s.pos._value.z,
      qx  = s.att._value.qx,
      qy  = s.att._value.qy,
      qz  = s.att._value.qz,
      qw  = s.att._value.qw,
      vx  = s.vel._value.vx,
      vy  = s.vel._value.vy,
      vz  = s.vel._value.vz,
      wx  = s.avel._value.wx,
      wy  = s.avel._value.wy,
      wz  = s.avel._value.wz,
      ax  = s.acc._value.ax,
      ay  = s.acc._value.ay,
      az  = s.acc._value.az,
      aax = s.aacc._value.awx,
      aay = s.aacc._value.awy,
      aaz = s.aacc._value.awz,
      roll  = atan2(2 * (qw*qx + qy*qz), 1 - 2 * (qx*qx + qy*qy)),
      pitch = asin(2 * (qw*qy - qz*qx)),
      yaw   = atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz));

      // pitch =  -asin(2.0 * (-qz*(-qy) - qw*qx)) * 180.0 / M_PI,
      // roll  =  -atan2(2.0 * (qw*(-qz) + qx*(-qy)), qw*qw - (-qz)*(-qz) - qx*qx + (-qy)*(-qy)) * 180.0 / M_PI,
      // yaw   =  -atan2(2.0 * (qw*(-qy) + (-qz)*qx), qw*qw + (-qz)*(-qz) - qx*qx - (-qy)*(-qy)) * 180.0 / M_PI;

    log->req.aio_nbytes = snprintf(
      log->buffer, sizeof(log->buffer),
      "%s" T265_log_fmt "\n",
      log->skipped ? "\n" : "",
      s.ts.sec, s.ts.nsec, confidence, x, y, z, roll, pitch, yaw, //qx, qy, qz, qw,
      vx, vy, vz, wx, wy, wz,
      ax, ay, az, aax, aay, aaz);

    if (aio_write(&log->req)) {
      warn("log");
      close(log->req.aio_fildes);
      log->req.aio_fildes = -1;
    } else
      log->pending = true;

    log->skipped = false;
  }
}

#endif