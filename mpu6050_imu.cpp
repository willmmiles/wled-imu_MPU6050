#include "wled.h"
#include "imu_interface.h"

/* MPU-6050 driver: reads the on-chip DMP's fused quaternion/gravity and linear-
   acceleration output and provides it to the motion_reactive IMU accessor.
   The DMP does the sensor fusion in hardware, so no software filter is needed.
   Adapted from https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050/examples/MPU6050_DMP6_ESPWiFi
*/

#include "I2Cdev.h"

#undef DEBUG_PRINT
#undef DEBUG_PRINTLN
#undef DEBUG_PRINTF
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

// Restore debug macros -- MPU6050 unfortunately uses the same macro names as WLED
#undef DEBUG_PRINT
#undef DEBUG_PRINTLN
#undef DEBUG_PRINTF
#ifdef WLED_DEBUG
  #define DEBUG_PRINT(x) DEBUGOUT.print(x)
  #define DEBUG_PRINTLN(x) DEBUGOUT.println(x)
  #define DEBUG_PRINTF(x...) DEBUGOUT.printf(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(x...)
#endif

// DMP FIFO packet scaling (see i2cdevlib MPU6050_6Axis_MotionApps20.h):
// accel is +/-2g range but represented in the FIFO at 8192 LSB/g (not the
// register-read 16384 LSB/g); gyro DMP init fixes the range at +/-2000 dps,
// i.e. 16.4 LSB/(deg/s).
static constexpr float ACCEL_LSB_PER_G  = 8192.f;
static constexpr float GYRO_LSB_PER_DPS = 16.4f;
static constexpr float GRAVITY_MPS2     = 9.81f;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void IRAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}


class MPU6050Driver : public Usermod, public IMUBase {
  private:
    MPU6050 mpu;

    struct config_t {
      bool    enabled;
      int8_t  interruptPin;
      long    update_interval_ms; // FIFO poll interval when interruptPin is unset; ignored in interrupt mode
      int16_t gyro_offset[3];
      int16_t accel_offset[3];
    };
    config_t config = { false, -1, 20, {0, 0, 0}, {0, 0, 0} };
    bool configDirty = true;

    // MPU control/status vars
    bool     irqBound  = false;  // set true if we have bound the IRQ pin
    bool     dmpReady  = false;  // set true if DMP init was successful
    bool     data_valid = false; // set true once at least one DMP packet has been parsed
    uint16_t packetSize = 0;     // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount  = 0;     // count of all bytes currently in FIFO
    uint8_t  fifoBuffer[64];     // FIFO storage buffer
    decltype(millis()) next_read = 0; // polling mode only: next scheduled FIFO check

    Quaternion   qat;           // [w, x, y, z]  quaternion container
    VectorInt16  aa;            // [x, y, z]     raw accel sensor measurements
    VectorInt16  gy;            // [x, y, z]     raw gyro sensor measurements
    VectorInt16  aaReal;        // [x, y, z]     gravity-free accel sensor measurements
    VectorFloat  gravity;       // [x, y, z]     unit gravity vector

    // Converted (SI-unit) outputs
    float corrAccel[3] = {0.f, 0.f, 0.f};  // m/s^2
    float corrGyro[3]  = {0.f, 0.f, 0.f};  // deg/s
    float gravEst[3]   = {0.f, 0.f, -GRAVITY_MPS2}; // m/s^2
    float linAccel[3]  = {0.f, 0.f, 0.f};  // m/s^2

    static const char _name[];
    static const char _enabled[];
    static const char _interrupt_pin[];
    static const char _update_interval_ms[];
    static const char _x_acc_bias[];
    static const char _y_acc_bias[];
    static const char _z_acc_bias[];
    static const char _x_gyro_bias[];
    static const char _y_gyro_bias[];
    static const char _z_gyro_bias[];

    void convertUnits() {
      corrAccel[0] = aa.x / ACCEL_LSB_PER_G * GRAVITY_MPS2;
      corrAccel[1] = aa.y / ACCEL_LSB_PER_G * GRAVITY_MPS2;
      corrAccel[2] = aa.z / ACCEL_LSB_PER_G * GRAVITY_MPS2;

      corrGyro[0] = gy.x / GYRO_LSB_PER_DPS;
      corrGyro[1] = gy.y / GYRO_LSB_PER_DPS;
      corrGyro[2] = gy.z / GYRO_LSB_PER_DPS;

      gravEst[0] = gravity.x * GRAVITY_MPS2;
      gravEst[1] = gravity.y * GRAVITY_MPS2;
      gravEst[2] = gravity.z * GRAVITY_MPS2;

      linAccel[0] = aaReal.x / ACCEL_LSB_PER_G * GRAVITY_MPS2;
      linAccel[1] = aaReal.y / ACCEL_LSB_PER_G * GRAVITY_MPS2;
      linAccel[2] = aaReal.z / ACCEL_LSB_PER_G * GRAVITY_MPS2;
    }

  public:

    // ── IMUBase interface ────────────────────────────────────────────────────

    bool  isValid()              const override { return config.enabled && dmpReady && data_valid; }
    float accel(int axis)        const override { return corrAccel[axis]; }
    float gyro(int axis)         const override { return corrGyro[axis]; }
    float gravityAccel(int axis) const override { return gravEst[axis]; }
    float linearAccel(int axis)  const override { return linAccel[axis]; }

    // ── Usermod interface ────────────────────────────────────────────────────

    void setup() override {
      dmpReady   = false;
      data_valid = false;
      configDirty = false;

      if (!config.enabled) return;
      if (i2c_scl < 0 || i2c_sda < 0) { config.enabled = false; return; }

      if (config.interruptPin >= 0) {
        irqBound = PinManager::allocatePin(config.interruptPin, false, PinOwner::UM_IMU);
        if (!irqBound) {
          DEBUG_PRINTF_P(PSTR("%s: IRQ pin already in use\n"), FPSTR(_name));
          config.enabled = false;
          return;
        }
        pinMode(config.interruptPin, INPUT);
      }

      #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.setClock(400000U); // 400kHz I2C clock. Comment this line if having compilation difficulties
      #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
      #endif

      mpu.initialize();
      if (!mpu.testConnection()) {
        DEBUG_PRINTF_P(PSTR("%s: Device did not respond\n"), FPSTR(_name));
        config.enabled = false;
        return;
      }

      auto devStatus = mpu.dmpInitialize();

      mpu.setXGyroOffset(config.gyro_offset[0]);
      mpu.setYGyroOffset(config.gyro_offset[1]);
      mpu.setZGyroOffset(config.gyro_offset[2]);
      mpu.setXAccelOffset(config.accel_offset[0]);
      mpu.setYAccelOffset(config.accel_offset[1]);
      mpu.setZAccelOffset(config.accel_offset[2]);

      mpu.setRate(16);  // ~100Hz

      if (devStatus != 0) {
        // 1 = initial memory load failed, 2 = DMP configuration updates failed
        DEBUG_PRINTF_P(PSTR("%s: DMP initialization failed (code %d)\n"), FPSTR(_name), devStatus);
        config.enabled = false;
        return;
      }

      mpu.setDMPEnabled(true);

      mpuInterrupt = true;
      if (irqBound) {
        attachInterrupt(digitalPinToInterrupt(config.interruptPin), dmpDataReady, RISING);
      }

      packetSize = mpu.dmpGetFIFOPacketSize();
      fifoCount  = 0;
      next_read  = 0;
      dmpReady   = true;
      DEBUG_PRINTF_P(PSTR("%s: Device initialized\n"), FPSTR(_name));
    }

    void loop() override {
      if (configDirty) setup();
      if (!config.enabled || !dmpReady || strip.isUpdating()) return;

      if (irqBound) {
        // Interrupt mode: wait for MPU interrupt or extra packet(s) already buffered
        if (!mpuInterrupt && fifoCount < packetSize) return;
      } else {
        // Polling mode: throttle FIFO checks to update_interval_ms so we don't
        // hit the I2C bus on every WLED main-loop iteration
        auto now   = millis();
        auto tdiff = static_cast<long>(next_read - now);
        if (tdiff > 0) return;
        if (tdiff >= -config.update_interval_ms) {
          next_read += config.update_interval_ms;
        } else {
          auto reads_missed = (-tdiff) / config.update_interval_ms;
          next_read += (reads_missed + 1) * config.update_interval_ms;
        }
      }

      auto mpuIntStatus = mpu.getIntStatus();
      fifoCount = mpu.getFIFOCount();

      if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // FIFO overflow -- reset so we can continue cleanly
        mpu.resetFIFO();
        DEBUG_PRINTF_P(PSTR("%s: FIFO overflow!\n"), FPSTR(_name));
      } else if (fifoCount >= packetSize) {
        // clear local interrupt pending status, if not polling
        mpuInterrupt = !irqBound;

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&qat, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &qat);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGyro(&gy, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

        convertUnits();
        data_valid = true;
      }
    }

    void addToJsonInfo(JsonObject& root) override {
      if (!config.enabled) return;

      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");
      auto& imu_meas = user;

      JsonArray accel_json = imu_meas.createNestedArray("Accel").createNestedArray();
      JsonArray gyro_json  = imu_meas.createNestedArray("Gyro").createNestedArray();
      JsonArray grav_json  = imu_meas.createNestedArray("GravEst").createNestedArray();
      JsonArray lin_json   = imu_meas.createNestedArray("LinAccel").createNestedArray();

      if (data_valid) {
        for (int i = 0; i < 3; i++) accel_json.add(corrAccel[i]);
        for (int i = 0; i < 3; i++) gyro_json.add(corrGyro[i]);
        for (int i = 0; i < 3; i++) grav_json.add(gravEst[i]);
        for (int i = 0; i < 3; i++) lin_json.add(linAccel[i]);
      } else {
        for (int i = 0; i < 3; i++) {
          accel_json.add("N/A"); gyro_json.add("N/A");
          grav_json.add("N/A");  lin_json.add("N/A");
        }
      }
    }

    void addToConfig(JsonObject& root) override {
      JsonObject top = root.createNestedObject(FPSTR(_name));
      top[FPSTR(_enabled)]            = config.enabled;
      top[FPSTR(_interrupt_pin)]      = config.interruptPin;
      top[FPSTR(_update_interval_ms)] = config.update_interval_ms;
      top[FPSTR(_x_acc_bias)]         = config.accel_offset[0];
      top[FPSTR(_y_acc_bias)]         = config.accel_offset[1];
      top[FPSTR(_z_acc_bias)]         = config.accel_offset[2];
      top[FPSTR(_x_gyro_bias)]        = config.gyro_offset[0];
      top[FPSTR(_y_gyro_bias)]        = config.gyro_offset[1];
      top[FPSTR(_z_gyro_bias)]        = config.gyro_offset[2];
    }

    bool readFromConfig(JsonObject& root) override {
      auto old_cfg = config;
      JsonObject top = root[FPSTR(_name)];

      bool configComplete = !top.isNull();
      configComplete &= getJsonValue(top[FPSTR(_enabled)],            config.enabled,            false);
      configComplete &= getJsonValue(top[FPSTR(_interrupt_pin)],      config.interruptPin,       (int8_t)-1);
      configComplete &= getJsonValue(top[FPSTR(_update_interval_ms)], config.update_interval_ms, (long)20);
      configComplete &= getJsonValue(top[FPSTR(_x_acc_bias)],         config.accel_offset[0],    (int16_t)0);
      configComplete &= getJsonValue(top[FPSTR(_y_acc_bias)],         config.accel_offset[1],    (int16_t)0);
      configComplete &= getJsonValue(top[FPSTR(_z_acc_bias)],         config.accel_offset[2],    (int16_t)0);
      configComplete &= getJsonValue(top[FPSTR(_x_gyro_bias)],        config.gyro_offset[0],     (int16_t)0);
      configComplete &= getJsonValue(top[FPSTR(_y_gyro_bias)],        config.gyro_offset[1],     (int16_t)0);
      configComplete &= getJsonValue(top[FPSTR(_z_gyro_bias)],        config.gyro_offset[2],     (int16_t)0);

      DEBUG_PRINT(F("MPU6050: "));
      if (top.isNull()) {
        DEBUG_PRINTLN(F("No config found. (Using defaults.)"));
      } else if (memcmp(&config, &old_cfg, sizeof(config)) == 0) {
        DEBUG_PRINTLN(F("config unchanged."));
      } else {
        DEBUG_PRINTLN(F("config updated."));
        if (irqBound && ((old_cfg.interruptPin != config.interruptPin) || !config.enabled)) {
          detachInterrupt(old_cfg.interruptPin);
          PinManager::deallocatePin(old_cfg.interruptPin, PinOwner::UM_IMU);
          irqBound = false;
        }
        configDirty = true;
      }

      return configComplete;
    }

    uint16_t getId() override {
      return USERMOD_ID_IMU;
    }
};


const char MPU6050Driver::_name[]           PROGMEM = "MPU6050_IMU";
const char MPU6050Driver::_enabled[]        PROGMEM = "enabled";
const char MPU6050Driver::_interrupt_pin[]  PROGMEM = "interrupt_pin";
const char MPU6050Driver::_update_interval_ms[] PROGMEM = "update_interval_ms";
const char MPU6050Driver::_x_acc_bias[]     PROGMEM = "x_acc_bias";
const char MPU6050Driver::_y_acc_bias[]     PROGMEM = "y_acc_bias";
const char MPU6050Driver::_z_acc_bias[]     PROGMEM = "z_acc_bias";
const char MPU6050Driver::_x_gyro_bias[]    PROGMEM = "x_gyro_bias";
const char MPU6050Driver::_y_gyro_bias[]    PROGMEM = "y_gyro_bias";
const char MPU6050Driver::_z_gyro_bias[]    PROGMEM = "z_gyro_bias";


static MPU6050Driver mpu6050_imu;
REGISTER_USERMOD(mpu6050_imu);

// Compile-time provider registration: strong definition overrides the weak
// default in motion_reactive.cpp.  Linking two IMU drivers simultaneously
// produces a duplicate-symbol error -- the correct behavior.
IMUBase* IMU_getProvider() { return &mpu6050_imu; }
