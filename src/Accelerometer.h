// Accelerometer Class -- extends the Zumo32U4IMU class to support reading and
//   averaging the x-y acceleration vectors from the accelerometer
#include <RunningAverage.h>
#include <Zumo32U4.h>

// Accelerometer Settings
#define RA_SIZE 3  // number of readings to include in running average of accelerometer readings
#define XY_ACCELERATION_THRESHOLD 2400  // for detection of contact (~16000 = magnitude of acceleration due to gravity)

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

// class Accelerometer -- member function definitions

void Accelerometer::getLogHeader(void)
{
  Serial.print("millis    x      y     len     dir  | len_avg  dir_avg  |  avg_len");
  Serial.println();
}

void Accelerometer::readAcceleration(unsigned long timestamp)
{
  readAcc();
  if (a.x == last.x && a.y == last.y) return;

  last.timestamp = timestamp;
  last.x = a.x;
  last.y = a.y;

  ra_x.addValue(last.x);
  ra_y.addValue(last.y);

#ifdef LOG_SERIAL
 Serial.print(last.timestamp);
 Serial.print("  ");
 Serial.print(last.x);
 Serial.print("  ");
 Serial.print(last.y);
 Serial.print("  ");
 Serial.print(len_xy());
 Serial.print("  ");
 Serial.print(dir_xy());
 Serial.print("  |  ");
 Serial.print(sqrt(static_cast<float>(ss_xy_avg())));
 Serial.print("  ");
 Serial.print(dir_xy_avg());
 Serial.println();
#endif
}

float Accelerometer::len_xy() const
{
  return sqrt(last.x*a.x + last.y*a.y);
}

float Accelerometer::dir_xy() const
{
  return atan2(last.x, last.y) * 180.0 / M_PI;
}

int Accelerometer::x_avg(void) const
{
  return ra_x.getAverage();
}

int Accelerometer::y_avg(void) const
{
  return ra_y.getAverage();
}

long Accelerometer::ss_xy_avg(void) const
{
  long x_avg_long = static_cast<long>(x_avg());
  long y_avg_long = static_cast<long>(y_avg());
  return x_avg_long*x_avg_long + y_avg_long*y_avg_long;
}

float Accelerometer::dir_xy_avg(void) const
{
  return atan2(static_cast<float>(x_avg()), static_cast<float>(y_avg())) * 180.0 / M_PI;
}
