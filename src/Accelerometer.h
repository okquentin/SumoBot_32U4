// Accelerometer Class -- extends the Zumo32U4IMU class to support reading and
//   averaging the x-y acceleration vectors from the accelerometer
#include <RunningAverage.h>
#include <Zumo32U4.h>
#include <Arduino.h>


// Accelerometer Settings
#define RA_SIZE 3  // number of readings to include in running average of accelerometer readings
// #define XY_ACCELERATION_THRESHOLD 2400  // for detection of contact (~16000 = magnitude of acceleration due to gravity)
#define XY_ACCELERATION_THRESHOLD 2000 // for detection of contact (~16000 = magnitude of acceleration due to gravity)
class Accelerometer : public Zumo32U4IMU
{
  typedef struct acc_data_xy
  {
    unsigned long timestamp;
    int x;
    int y;
    float dir;
  } acc_data_xy;

  public:
    Accelerometer() : ra_x(RA_SIZE), ra_y(RA_SIZE) {};
    ~Accelerometer() {};
    void getLogHeader(void);
    void readAcceleration(unsigned long timestamp);
    float len_xy() const;
    float dir_xy() const;
    int x_avg(void) const;
    int y_avg(void) const;
    long ss_xy_avg(void) const;
    float dir_xy_avg(void) const;
  private:
    acc_data_xy last;
    RunningAverage<int> ra_x;
    RunningAverage<int> ra_y;
};