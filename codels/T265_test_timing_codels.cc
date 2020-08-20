#include "acT265.h"
#include "codels.h"
#include "T265_c_types.h"


/* --- Task test_timing ------------------------------------------------- */


/** Codel start_codel of task test_timing.
 *
 * Triggered by T265_start.
 * Yields to T265_loop.
 */
genom_event
start_codel(const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return T265_loop;
}


/** Codel loop_codel of task test_timing.
 *
 * Triggered by T265_loop.
 * Yields to T265_pause_loop.
 */
genom_event
loop_codel(int32_t image_count, int32_t pose_count,
           const genom_context self)
{
  std::cout << "Image_nb = " << image_count << " Pose_nb = " << pose_count << std::endl;
  return T265_pause_loop;
}


/** Codel stop_codel of task test_timing.
 *
 * Triggered by T265_stop.
 * Yields to T265_ether.
 */
genom_event
stop_codel(const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return T265_ether;
}
