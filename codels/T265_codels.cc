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
