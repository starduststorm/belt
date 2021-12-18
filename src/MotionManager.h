#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <functional>
#include <map>
#include <vector>

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
  
public:
  Adafruit_BNO055 bno;
  MotionManager() {
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
    }
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

    // FIXME: Run MotionManager all the time so it can run activity classifiers
    // a "jump" is 15-20 frames btween linear_accel.acceleration.x > 10 and < -20
    if (linear_accel.acceleration.x > 10) {
      jumpClock = millis();
    }
    if (linear_accel.acceleration.x < -25 && millis() - jumpClock < 250) {
      logf("Firing jump activity with jumpClock at %lu", millis() - jumpClock);
      jumpClock = 0;
      // for (handler : activityHandlers[JumpActivity].) {
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
  float twirlVelocity(int samples, float *outOrientation=NULL) {
    sensors_event_t event;
    getEvent(&event);

    float orientation = event.orientation.x;
    if (outOrientation) {
      *outOrientation = orientation;
    }
    twirlVelocityAccum = (samples * twirlVelocityAccum + MOD_DISTANCE(prevXOrientation, orientation, 360)) / (samples + 1);
    prevXOrientation = orientation;

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
