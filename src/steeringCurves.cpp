#include <Arduino.h>
#include "steeringCurves.h"

//
// =======================================================================================================
// NONLINEAR ARRAYS FOR STEERING OVERLAY
// =======================================================================================================
//

// In order to optimize the steering behaviour for your vehicle, just change the steering curves in the arrays below



//
// =======================================================================================================
// ARRAY INTERPOLATION
// =======================================================================================================
//

// Credit: http://interface.khm.de/index.php/lab/interfaces-advanced/nonlinear-mapping/

int reMap(float pts[][2], int input) {
  int rr = 0;
  float mm;

  for (int nn = 0; nn < 5; nn++) {
    if (input >= pts[nn][0] && input <= pts[nn + 1][0]) {
      mm = ( pts[nn][1] - pts[nn + 1][1] ) / ( pts[nn][0] - pts[nn + 1][0] );
      mm = mm * (input - pts[nn][0]);
      mm = mm +  pts[nn][1];
      rr = mm;
    }
  }
  return (rr);
}