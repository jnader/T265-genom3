#include "acT265.h"
#include "codels.h"
#include "T265_c_types.h"


/* --- Function start_publishing ---------------------------------------- */

/** Codel start_publish of function start_publishing.
 *
 * Returns genom_ok.
 */
genom_event
start_publish(bool *is_publishing, const genom_context self)
{
  *is_publishing = true;
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
