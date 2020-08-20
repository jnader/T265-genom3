#include "acT265.h"

#include "T265_c_types.h"
#include "codels.h"

/* --- Task image_display ----------------------------------------------- */
vpDisplayX display_left;  // Left image
vpDisplayX display_right; // Right image

/** Codel init_display of task image_display.
 *
 * Triggered by T265_start.
 * Yields to T265_loop.
 */
genom_event
init_display(const T265_vp_image *I_left, const T265_vp_image *I_right,
             const genom_context self)
{
  display_left.init(const_cast<vpImage<unsigned char>&>(I_left->I), 10, 10, "Left image");
  display_right.init(const_cast<vpImage<unsigned char>&>(I_right->I), static_cast<int>(I_left->I.getWidth()/2) + 80, 10, "Right image"); // Right

  return T265_loop;
}


/** Codel refresh_display of task image_display.
 *
 * Triggered by T265_loop.
 * Yields to T265_pause_loop, T265_stop.
 */
genom_event
refresh_display(const T265_vp_image *I_left,
                const T265_vp_image *I_right,
                const genom_context self)
{
  vpDisplay::display(I_left->I);
  vpDisplay::display(I_right->I);

  vpDisplay::displayText(I_left->I, 30, 30, "Click to quit", vpColor::red);
  vpDisplay::displayText(I_right->I, 30, 30, "Click to quit", vpColor::red);

  if (vpDisplay::getClick(I_left->I, false) || vpDisplay::getClick(I_right->I, false)) {
      return T265_stop;
  }
  vpDisplay::flush(I_left->I);
  vpDisplay::flush(I_right->I);

  return T265_pause_loop;
}


/** Codel stop_display of task image_display.
 *
 * Triggered by T265_stop.
 * Yields to T265_ether.
 */
genom_event
stop_display(T265_realsense_grabber **rs_grabber,
             T265_vp_image **I_left, T265_vp_image **I_right,
             const genom_context self)
{
  (*rs_grabber)->g.close();
  
  vpDisplay::close((*I_left)->I);
  vpDisplay::close((*I_right)->I);
  
  return T265_ether;
}
