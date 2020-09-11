#ifndef MC34933MOTORH
#define MC34933MOTORH

enum e_direction
{
  LEFT,
  RIGHT
};

enum e_status
{
  IDLE,
  STARTING,
  RUN,
  STOPPING
};

class MC34933
{
  private:
    const int m_MA;
    const int m_MB;
    e_direction m_direction = LEFT;
    e_direction m_prevDirection;
    int m_speed;
    e_status m_status;
    bool m_startCmd = false;
    bool m_stopCmd = false;

  
  public:
    MC34933( int pin_ma,  int pin_mb) 
        : m_MA{pin_ma}, m_MB{pin_mb} {}

    void begin( void );
    void setDirection( e_direction dir);
    e_direction getDirection( void ) {return m_direction;}
    bool setSpeed( int speed );
    int getSpeed( void ) {return m_speed;}
    void start( void );
    bool start( int speed );
    bool start( int speed, int duration_ms);
    void stop ( void );
    void forward(int speed);
    void backward(int speed);
    e_status getStatus( void ) { return m_status;}

    void run(void);

};

#endif //MC34933MOTORH