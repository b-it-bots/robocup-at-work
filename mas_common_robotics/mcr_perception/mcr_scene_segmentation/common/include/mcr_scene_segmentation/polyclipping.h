#ifndef POLYCLIPPING_H
#define POLYCLIPPING_H

#include "aliases.h"

/** Clip the planar polygon by a certain number of meters.
  *
  * Performs inward polygon offseting with Clipper library.
  * see http://stackoverflow.com/questions/1109536)
  * and http://www.angusj.com/delphi/clipper.php
  *
  * @param planar_polygon polygon to clip, will be replaced by the clipped one.
  * @param clib_by how much to clip the polygon (in meters).
  */
void clipPlanarPolygon(PlanarPolygon& planar_polygon, double clip_by);

#endif /* POLYCLIPPING_H */

