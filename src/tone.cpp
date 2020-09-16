#include <Arduino.h>
#include "tone.h"
#include "vehicleConfig.h"


//
// =======================================================================================================
// TONE FUNCTION FOR STAR WARS R2-D2
// =======================================================================================================
//

// Engine sound
bool engineOn = false;

int r2d2Tones[] = {
  3520, 3136, 2637, 2093, 2349, 3951, 2794, 4186
};

void R2D2_tell() 
{

  vehicleConfig* cfg = getConfigptr();

  if (cfg->gettoneOut()) {
  for (int notePlay = 0; notePlay < 8; notePlay++) {
      int noteRandom = random(7);
      tone(A2, r2d2Tones[noteRandom], 80); // Pin, frequency, duration
      delay(50);
      noTone(A2);
    }
  }
}