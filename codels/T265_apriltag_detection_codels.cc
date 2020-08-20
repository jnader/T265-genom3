#include "acT265.h"

#include "T265_c_types.h"


/* --- Task apriltag_detection ------------------------------------------ */


/** Codel init_detector of task apriltag_detection.
 *
 * Triggered by T265_start.
 * Yields to T265_loop.
 */
genom_event
init_detector(const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return T265_loop;
}


/** Codel loop_detector of task apriltag_detection.
 *
 * Triggered by T265_loop.
 * Yields to T265_pause_loop, T265_stop.
 */
genom_event
loop_detector(const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return T265_pause_loop;
}


/** Codel kill_detector of task apriltag_detection.
 *
 * Triggered by T265_stop.
 * Yields to T265_ether.
 */
genom_event
kill_detector(const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return T265_ether;
}
