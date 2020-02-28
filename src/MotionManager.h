#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


class MotionManager {
private:
  unsigned int retainCount;
public:
  Adafruit_BNO055 bno;
  MotionManager() {
    bno = Adafruit_BNO055(55, 0x28);
    
    bool hasBNO = bno.begin();
    logf("Has BNO senror: %s", (hasBNO ? "yes" : "no"));
    bno.setExtCrystalUse(true);
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
    logf("+MotionManager");
    retainCount++;
    if (retainCount == 1) {
      logf("Powering up BNO...");
      bno.enterNormalMode();
      logf("Powered up BNO");
    }
  }
  
  void unsubscribe() {
    logf("-MotionManager");
    assert(retainCount != 0, "Not subscribed");
    retainCount--;
    if (retainCount == 0) {
      bno.enterSuspendMode();
    }
  }

  void getEvent(sensors_event_t *event) {
    bno.getEvent(event);
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
    logf("calibration level: %i", system);

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
