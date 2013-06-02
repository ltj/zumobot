#include <ChibiOS_AVR.h>
#include <chevents.h>
#include <ZumoMotors.h>
#include <ZumoBuzzer.h>
#include <Pushbutton.h>
#include <Wire.h>
#include <LSM303.h>

#define SPEED_FWD       300
#define SPEED_BACK      200
#define SPEED_TURN      150

#define CALIBRATION_SAMPLES 70  // Number of compass readings to take when calibrating
#define CRA_REG_M_220HZ 0x1C    // CRA_REG_M value for magnetometer 220 Hz update rate
#define DEVIATION_THRESHOLD 5 // Allowed deviation (in degrees) relative to target angle

#define IRF_SCALE_FACTOR 27 // 27 V*cm / Vout V if 
#define IRF_VTOCM(raw) (IRF_SCALE_FACTOR / (5.0/1024*raw)) // raw analog val to cm

#define SWEEP_ANGLE 60 // sweep ~30 degrees to each side on collision/proximity

// state enum
enum {STOP, GO, DRIVE, TURN, SWEEP_RIGHT, SWEEP_LEFT};

ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);
LSM303 compass;
ZumoBuzzer buzzer;

//------------------------------------------------------------------------------
// Shared data, use volatile to insure correct access.

// Mutex for atomic access to data.
MUTEX_DECL(dataMutex);

// current state
volatile int state;

// Distance data
volatile uint32_t distance;

// Collision bit
volatile boolean collision;

// Heading data
volatile int orientation;
volatile int target;

//------------------------------------------------------------------------------
// Events
// EVENTSOURCE_DECL(esCollision);

//------------------------------------------------------------------------------
// Distance sensor thread

static WORKING_AREA(waThread1, 64);

static msg_t Thread1 (void *arg) {

  const uint8_t SENSOR_PIN = A5;
  // Read data every 40 ms.
  systime_t wakeTime = chTimeNow();

  while (1) {
    // Add ticks for 40 ms.
    wakeTime += MS2ST(50);
    chThdSleepUntil(wakeTime);

    int sensorVal = analogRead(SENSOR_PIN);

    // Write new distance 
    chMtxLock(&dataMutex);
    distance = sensorVal;
    chMtxUnlock();
  }
}

//------------------------------------------------------------------------------
// Accelerometer thread

static WORKING_AREA(waThread2, 64);

static msg_t Thread2 (void *arg) {

  // Read data every 15 ms.
  systime_t wakeTime = chTimeNow();

  while (1) {
    // Add ticks for 15 ms.
    wakeTime += MS2ST(15);
    chThdSleepUntil(wakeTime);

    // read INT1
    byte col = compass.readAccReg(LSM303_INT1_SRC_A);

    // write collision bit if necessary
    chMtxLock(&dataMutex);
    if(state == DRIVE) collision = col>>6 & 0x01;
    chMtxUnlock();

  }
}

//------------------------------------------------------------------------------
// Compass thread

static WORKING_AREA(waThread3, 128);

static msg_t Thread3 (void *arg) {

  while (1) {

    LSM303::vector v = {0, 0, 0};
    // average of 10 readings
    for(int i = 0; i < 10; i ++) {
      compass.readMag();
      v.x += compass.m.x;
      v.y += compass.m.y;
    }
    v.x /= 10.0;
    v.y /= 10.0;
    
    float x_scaled =  2.0*(float)(v.x - compass.m_min.x) / ( compass.m_max.x - compass.m_min.x) - 1.0;
    float y_scaled =  2.0*(float)(v.y - compass.m_min.y) / (compass.m_max.y - compass.m_min.y) - 1.0;
  
    int angle = round(atan2(y_scaled, x_scaled)*180 / M_PI);
    if (angle < 0)
      angle += 360;

    // write heading
    chMtxLock(&dataMutex);
    orientation = angle;
    chMtxUnlock();

    chThdSleepMilliseconds(20);
  }
}

//------------------------------------------------------------------------------
// Motor thread

static WORKING_AREA(waThread4, 128);

static msg_t Thread4 (void *arg) {

  int l_head = 0;
  int l_dist = 0;

  while (1) {

    // Read shared data.
    chMtxLock(&dataMutex);

    boolean col = collision;
    int dist = distance;
    int tmpstate = state;
    int head = orientation;
    int tmptarget = target;

    chMtxUnlock();

    // get relative heading
    int relative_heading = relativeHeading(head, tmptarget);
    // get longest distance readings
    if(dist > l_dist) {
      l_dist = dist;
      l_head = head;
    }

    // ready to go
    if(tmpstate == GO) {
      chMtxLock(&dataMutex);
      state = DRIVE;
      chMtxUnlock();
      motors.setSpeeds(SPEED_FWD, SPEED_FWD);
      chThdSleepMilliseconds(100);
    }
    // full stop
    else if(tmpstate == STOP) {
      motors.setSpeeds(0, 0);
    }
    // closing in: decelerate
    else if(tmpstate == DRIVE && (dist >= 300 && dist <= 500)) {
      int newSpeed = map(dist, 300, 500, SPEED_FWD, 100);
      motors.setSpeeds(newSpeed, newSpeed);
    }
    // COLLISION or proximity: turn
    else if(tmpstate == DRIVE && (dist > 500 || col)) {
      // reset ldist
      l_dist = 0;
      l_head = 0;

      // back up
      motors.setSpeeds(0, 0);
      chThdSleepMilliseconds(100);
      motors.setSpeeds(-SPEED_BACK, -SPEED_BACK);
      chThdSleepMilliseconds(200);

      // sweep right
      chMtxLock(&dataMutex);
      collision = false; // better clear this
      state = SWEEP_RIGHT;
      target = (head + SWEEP_ANGLE) % 360; // set right target angle
      chMtxUnlock();
    }
    else if(tmpstate == SWEEP_RIGHT) {
      // if reached destination within threshold stop and sweep left else continue right sweep
      if(abs(relative_heading) < DEVIATION_THRESHOLD) {
        motors.setSpeeds(0, 0);
        chThdSleepMilliseconds(100);
        chMtxLock(&dataMutex);
        target = (head - 2*SWEEP_ANGLE) % 360; // set right target angle
        state = SWEEP_LEFT;
        chMtxUnlock();
      }
      else {
        int speed = map(abs(relative_heading), 0, 180, SPEED_TURN, 70);
        motors.setSpeeds(speed, -speed);
      }
    }
    else if(tmpstate == SWEEP_LEFT) {
      // if reached destination within threshold stop and sweep left else continue right sweep
      if(abs(relative_heading) < DEVIATION_THRESHOLD) {
        motors.setSpeeds(0, 0);
        chThdSleepMilliseconds(100);
        chMtxLock(&dataMutex);
        //target = (head + 2*SWEEP_ANGLE) % 360; // set longest distance heading as target
        target = l_head;
        state = TURN;
        chMtxUnlock();
      }
      else {
        int speed = map(abs(relative_heading), 0, 180, SPEED_TURN, 90);
        motors.setSpeeds(-speed, speed);
      }
    }
    else if(tmpstate == TURN) {
      // if reached destination within threshold stop and sweep left else continue right sweep
      if(abs(relative_heading) < DEVIATION_THRESHOLD) {
        motors.setSpeeds(0, 0);
        chThdSleepMilliseconds(100);
        chMtxLock(&dataMutex);
        state = GO;
        chMtxUnlock();
      }
      else {
        int speed = map(abs(relative_heading), 0, 180, SPEED_TURN, 90);
        if(relative_heading < 0) {
          motors.setSpeeds(-speed, speed);
        }
        else {
          motors.setSpeeds(speed, -speed);
        }
      }
    }

    chThdSleepMilliseconds(20);

  }
}

//------------------------------------------------------------------------------

void setup() {
  Serial.begin(57600);
  Wire.begin();
  compass.init();
  compass.enableDefault();

  // setup accellerometer
  compass.writeAccReg(LSM303_CTRL_REG1_A, 0x57); // 100HZ

  // setup pick-up and collision interrupts
  compass.writeAccReg(LSM303_INT1_THS_A , 0x15); // I2 threshold
  compass.writeAccReg(LSM303_INT1_DURATION_A, 0x01); // I2 duration
  compass.writeAccReg(LSM303_INT1_CFG_A, 0x02); // I2 config : X high (collision)

  // setup and calibrate compass
  LSM303::vector running_min = {2047, 2047, 2047}, running_max = {-2048, -2048, -2048};
  unsigned char index;

  compass.setMagGain(compass.magGain_25);                  // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems
  compass.writeMagReg(LSM303_CRA_REG_M, CRA_REG_M_220HZ);  // 220 Hz compass update rate

  float min_x_avg[CALIBRATION_SAMPLES];
  float min_y_avg[CALIBRATION_SAMPLES];
  float max_x_avg[CALIBRATION_SAMPLES];
  float max_y_avg[CALIBRATION_SAMPLES];
  
  button.waitForButton();
  
  // To calibrate the magnetometer, the Zumo spins to find the max/min
  // magnetic vectors. This information is used to correct for offsets
  // in the magnetometer data.
  motors.setLeftSpeed(200);
  motors.setRightSpeed(-200);

  for(index = 0; index < CALIBRATION_SAMPLES; index ++) {
    // Take a reading of the magnetic vector and store it in compass.m
    compass.read();

    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);

    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);

    delay(50);
  }

  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  // Set calibrated values to compass.m_max and compass.m_min
  compass.m_max.x = running_max.x;
  compass.m_max.y = running_max.y;
  compass.m_min.x = running_min.x;
  compass.m_min.y = running_min.y;

  // set default state
  state = GO;

  button.waitForButton();

  // // play audible countdown
  // for (int i = 0; i < 3; i++) {
  //   delay(1000);
  //   buzzer.playNote(NOTE_G(3), 200, 15);
  // }
  // delay(1000);
  // buzzer.playNote(NOTE_G(4), 500, 15);  
  // delay(1000);

  chBegin(mainThread);
  // chBegin never returns, main thread continues with mainThread()
  while(1) {
  }
}

//------------------------------------------------------------------------------
// main thread runs at NORMALPRIO
void mainThread() {

  // start distance thread
  chThdCreateStatic(waThread1, sizeof(waThread1),
  NORMALPRIO + 1, Thread1, NULL);

  // start accelerometer thread
  chThdCreateStatic(waThread2, sizeof(waThread2),
  NORMALPRIO + 1, Thread2, NULL);
  
  // start compass thread
  chThdCreateStatic(waThread3, sizeof(waThread3),
  NORMALPRIO + 2, Thread3, NULL);

  // start motor thread
  chThdCreateStatic(waThread4, sizeof(waThread4),
  NORMALPRIO + 3, Thread4, NULL);

}

//------------------------------------------------------------------------------
void loop() {

}

//------------------------------------------------------------------------------
// Yields the angle difference in degrees between two headings
int relativeHeading(int heading_from, int heading_to) {
  int relative_heading = heading_to - heading_from;

  // constrain to -180 to 180 degree range
  if (relative_heading > 180)
    relative_heading -= 360;
  if (relative_heading < -180)
    relative_heading += 360;

  return relative_heading;
}


