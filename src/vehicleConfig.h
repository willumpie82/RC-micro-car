
#ifndef vehicleConfig_h
#define vehicleConfig_h

typedef enum vehicleconfig_e
{
  STANDARDCAR,
  DRIFTCAR,
  COKECANCAR
}vehicleconfig_t;

typedef enum vehicletype_e
{
  car = 0,
  vhctype_semi = 1,
  vhctype_unknown = 2,
  forklift = 3,
  balancingthing = 4,
  carwithMRSC = 5,
  simpledualmotorplane = 6
}vehicletype_t;

class vehicleConfig
{
  bool m_liPo;
  float m_cutoffVoltage; // trigger, as soon as VCC drops! (no battery sensing)

  // Board type
  float m_boardVersion;
  bool m_HP;

  // Vehicle address
  int m_vehicleNumber;

  // Vehicle type
  vehicletype_t m_vehicleType;

  // Lights
  bool m_escBrakeLights;
  bool m_tailLights;
  bool m_headLights;
  bool m_indicators;
  bool m_beacons;

  // Servo limits
  byte m_lim1L; 
  byte m_lim1R;
  byte m_lim2L;
  byte m_lim2R;
  byte m_lim3L;
  byte m_lim3R;
  byte m_lim3Llow;
  byte m_lim3Rlow;
  byte m_lim4L;
  byte m_lim4R;

  // Motor configuration
  int m_maxPWMfull;
  int m_maxPWMlimited;
  int m_minPWM;
  byte m_maxAccelerationFull;
  byte m_maxAccelerationLimited;

  // Variables for self balancing (vehicleType = 4) only!
  float m_tiltCalibration;
  // Steering configuration
  byte m_steeringTorque;

  // Motor 2 PWM frequency
  byte m_pwmPrescaler2;

  // Additional Channels
  bool m_TXO_momentary1;
  bool m_TXO_toggle1;
  bool m_potentiometer1;

  // Engine sound
  bool m_engineSound;

  // Tone sound
  bool m_toneOut;


public:
  vehicleConfig() {serialCommands= false;};

  bool serialCommands;


  void begin(vehicleconfig_t vehicle);

  void setBasicParams(
      bool liPo,
      float cutoffVoltage,
      float boardVersion,
      bool HP,
      int vehicleNumber,
      vehicletype_t vehicleType,
      bool engineSound,
      bool toneOut
  );

  void setServoLimits(   
      byte lim1L,
      byte lim1R,
      byte lim2L,
      byte lim2R,
      byte lim3L,
      byte lim3R,
      byte lim3Llow,
      byte lim3Rlow,
      byte lim4L,
      byte lim4R
      );

  void setIndicators(
      bool escBrakeLights,
      bool tailLights,
      bool headLights,
      bool indicators,
      bool beacons
    );

  void setMotorConfig(
      int maxPWMfull,
      int maxPWMlimited,
      int minPWM,
      byte maxAccelerationFull,
      byte maxAccelerationLimited,
      float tiltCalibration,
      byte steeringTorque,
      byte pwmPrescaler2
  );

  void setAdditionalChannels(
      bool TXO_momentary1,
      bool TXO_toggle1,
      bool potentiometer1
  );

  bool getliPo(){ return m_liPo; }
  float getcutoffVoltage(){ return m_cutoffVoltage;}
  float getboardVersion(){ return m_boardVersion;}
  bool getHP(){ return m_HP;}
  int getvehicleNumber(){ return m_vehicleNumber;}
  vehicletype_t getvehicleType(){ return m_vehicleType;}
  void setvehicleType(vehicletype_t vhc){m_vehicleType = vhc;}
  bool getescBrakeLights(){ return m_escBrakeLights;}
  bool gettailLights(){ return m_tailLights;}
  bool getheadLights(){ return m_headLights;}
  bool getindicators(){ return m_indicators;}
  bool getbeacons(){ return m_beacons;}
  byte getlim1L(){ return m_lim1L;}
  byte getlim1R(){ return m_lim2R;}
  byte getlim2L(){ return m_lim2L;}
  byte getlim2R(){ return m_lim2R;}
  byte getlim3L(){ return m_lim3L;}
  byte getlim3R(){ return m_lim3R;}
  byte getlim3Llow(){ return m_lim3Llow;}
  byte getlim3Rlow(){ return m_lim3Rlow;}
  byte getlim4L(){ return m_lim4L;}
  byte getlim4R(){ return m_lim4R;}
  int getmaxPWMfull(){ return m_maxPWMfull;}
  int getmaxPWMlimited(){ return m_maxPWMlimited;}
  int getminPWM(){ return m_minPWM;}
  byte getmaxAccelerationFull(){ return m_maxAccelerationFull;}
  byte getmaxAccelerationLimited(){ return m_maxAccelerationLimited;}
  float gettiltCalibration(){ return m_tiltCalibration;}
  byte getsteeringTorque(){ return m_steeringTorque;}
  byte getpwmPrescaler2(){ return m_pwmPrescaler2; }
  void setpwmPrescaler2(byte psc) { m_pwmPrescaler2 = psc;}
  bool getTXO_momentary1(){ return m_TXO_momentary1;}
  bool getTXO_toggle1(){ return m_TXO_toggle1;}
  bool getpotentiometer1(){ return m_potentiometer1; }
  bool getengineSound(){ return m_engineSound;}
  bool gettoneOut(){ return m_toneOut;}

}; //class vehicleConfig

vehicleConfig* getConfigptr( void );


#endif
