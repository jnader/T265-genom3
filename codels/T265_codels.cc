#include "acT265.h"

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
