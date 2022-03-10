#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <functional>
#include <map>
#include <vector>
#include "stats.h"

#define kTestTwirlBPM 0 // 0 for off, 5 is slowish, 17 is enough to trigger rotation overlays

typedef enum : unsigned int {
  JumpActivity,
  ActivityTypeCount,
} ActivityType;

typedef std::function<void()> ActivityHandler;
typedef std::map<const char *, ActivityHandler> ActivityHandlerMap;

class MotionManager {
private:
  unsigned int retainCount;
  std::vector<ActivityHandlerMap> activityHandlers;
  RunningStats accelerationStats;
  
public:
  Adafruit_BNO055 bno;
  MotionManager() : accelerationStats(900) {
    bno = Adafruit_BNO055(55, 0x28);
    
    bool hasBNO = bno.begin(bno.OPERATION_MODE_NDOF);
    logf("Has BNO sensor: %s", (hasBNO ? "yes" : "no"));
    bno.setExtCrystalUse(true);

    for (unsigned i = 0; i < ActivityTypeCount; ++i) {
      activityHandlers.push_back(ActivityHandlerMap());
    }
  }

  void addActivityHandler(ActivityType activity, const char *identifier, ActivityHandler handler) {
    activityHandlers[activity][identifier] = handler;
  }

  void removeActivityHandler(ActivityType activity, const char *identifier) {
    activityHandlers[activity].erase(identifier);
  }

  /* FIXME: once belt is constructed, save and restore calibration data:
   *  https://github.com/adafruit/Adafruit_BNO055/blob/master/Adafruit_BNO055.h
  bool getSensorOffsets(uint8_t *calibData);
  bool getSensorOffsets(adafruit_bno055_offsets_t &offsets_type);
  void setSensorOffsets(const uint8_t *calibData);
  void setSensorOffsets(const adafruit_bno055_offsets_t &offsets_type);
  bool isFullyCalibrated();
   */

  void subscribe() {
    // logf("+MotionManager");
    retainCount++;
    if (retainCount == 1) {
      logf("Powering up BNO...");
      bno.enterNormalMode();
      logf("Powered up BNO");
    }
  }
  
  void unsubscribe() {
    // logf("-MotionManager");
    assert(retainCount != 0, "Not subscribed");
    retainCount--;
    if (retainCount == 0) {
      bno.enterSuspendMode();
      logf("Powered down BNO");
    }
  }

  void loop() {
    sensors_event_t accel;
    bno.getEvent(&accel, Adafruit_BNO055::VECTOR_ACCELEROMETER);

    accelerationStats.push(accel.acceleration.x);
  }
  
  float bouncyEnergy() {
    return accelerationStats.variance();
  }

  // the jankiest activity classifier evar
  unsigned long jumpClock = 0;

  void getEvent(sensors_event_t *event) {
    bno.getEvent(event);
    
    sensors_event_t linear_accel;
    sensors_event_t accel;
    bno.getEvent(&accel, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&linear_accel, Adafruit_BNO055::VECTOR_LINEARACCEL);
    // logf("accel, lin_accel: (%0.3f, %0.3f)", accel.acceleration.x, linear_accel.acceleration.x);

    // a "jump" is a short interval between linear_accel.acceleration.x moving high and then low, indicating the liftoff and then land
    const int kJumpClockMin = 80;
    const int kJumpClockMax = 250;

    // 
    static float runningMin = 10000;
    static float runningMax = -1;
    static long minCapture = 0;
    static long maxCapture = 0;

    if (millis() - minCapture > kJumpClockMax) {
      runningMin = 1000;
      minCapture = 0;
    }
    if (millis() - maxCapture > kJumpClockMax) {
      runningMax = -1;
      maxCapture = 0;
    }
    if (linear_accel.acceleration.x < runningMin) {
      runningMin = linear_accel.acceleration.x;
      minCapture = millis();
    }
    if (linear_accel.acceleration.x > runningMax) {
      runningMax = linear_accel.acceleration.x;
      maxCapture = millis();
    }
    //

    if (linear_accel.acceleration.x > 12) {
      jumpClock = millis();
    }
    if (linear_accel.acceleration.x < -26 && millis() - jumpClock < kJumpClockMax && millis() - jumpClock > kJumpClockMin) {
      logf("Firing jump activity with runningMin %f, runningMax %f, capture clock %li, jumpClock at %lu", runningMin, runningMax, minCapture-maxCapture, millis() - jumpClock);
      jumpClock = 0;
      for (const auto &item : activityHandlers[JumpActivity]) {
        logf("Firing jump handler for identifier %s", item.first);
        item.second();
      }
    }
  }

private:
  float twirlVelocityAccum;
  float prevXOrientation;
public:
  float twirlVelocity(int samples=10, float *outOrientation=NULL) {
    // FIXME: rework the twirl accumulator so this can be called multiple times in a single frame without smashing the accumulator, since MotionManager is a singleton.
    float orientation;

#if kTestTwirlBPM != 0
    orientation = (beatsin16(kTestTwirlBPM, 0, 1000) - 500) / 2;
#else
    sensors_event_t event;
    getEvent(&event);
    orientation = event.orientation.x;
#endif
    twirlVelocityAccum = (samples * twirlVelocityAccum + MOD_DISTANCE(prevXOrientation, orientation, 360)) / (samples + 1);
    prevXOrientation = orientation;

    if (outOrientation) {
      *outOrientation = orientation;
    }

    return twirlVelocityAccum;
  }

  void printStatus() {
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    logf("calibration level: sys: %i, gyro: %i, accel: %i, mag: %i", system, gyro, accel, mag);

    logf("Manager usage count: %u", retainCount);
    int8_t boardTemp = bno.getTemp();
    Serial.print(F("temperature: "));
    Serial.println(boardTemp);
  }
  
  void printEvent(sensors_event_t& event) {
    logf("Event of type %i (accel = %i, orientation = %i, magnetic = %i, gyro = %i, rotation_vec = %i)", event.type, SENSOR_TYPE_ACCELEROMETER, SENSOR_TYPE_ORIENTATION, SENSOR_TYPE_MAGNETIC_FIELD, SENSOR_TYPE_GYROSCOPE, SENSOR_TYPE_ROTATION_VECTOR);
    double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
    if (event.type == SENSOR_TYPE_ACCELEROMETER) {
      x = event.acceleration.x;
      y = event.acceleration.y;
      z = event.acceleration.z;
    }
    else if (event.type == SENSOR_TYPE_ORIENTATION) {
      x = event.orientation.x;
      y = event.orientation.y;
      z = event.orientation.z;
    }
    else if (event.type == SENSOR_TYPE_MAGNETIC_FIELD) {
      x = event.magnetic.x;
      y = event.magnetic.y;
      z = event.magnetic.z;
    }
    else if ((event.type == SENSOR_TYPE_GYROSCOPE) || (event.type == SENSOR_TYPE_ROTATION_VECTOR)) {
      x = event.gyro.x;
      y = event.gyro.y;
      z = event.gyro.z;
    }
  
    Serial.print(": x= ");
    Serial.print(x);
    Serial.print(" | y= ");
    Serial.print(y);
    Serial.print(" | z= ");
    Serial.println(z);
  }
};
