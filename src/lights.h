#ifndef LIGHTSH
#define LIGHTSH


void SetmillisLightOff( unsigned long millis );
void UpdatemillisLightOff( void );
unsigned long getMillisLightsOff( void );
void led();

bool getLeft( void );
bool getRight( void );
bool getHazard( void );

void setupLights( void );

#endif //LIGHTSH