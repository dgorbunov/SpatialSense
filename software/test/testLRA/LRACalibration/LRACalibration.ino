#include <Wire.h>
#include "Adafruit_DRV2605.h"

Adafruit_DRV2605 drv;

void setup() {
  Serial.begin(9600);
  // while (!Serial) {delay(1);}

  if (! drv.begin()) {
    Serial.println("Could not find DRV2605");
    while (1) delay(10);
  }

  // autocal();
  // autocal_fixed();
  nocal();

  drv.selectLibrary(1);
  drv.setMode(DRV2605_MODE_REALTIME); 
}

void autocal() {
    /**
    a. ERM_LRA—selectionwilldependondesiredactuator.
    b. FB_BRAKE_FACTOR[2:0] — A value of 2 is valid for most actuators.
    c. LOOP_GAIN[1:0] — A value of 2 is valid for most actuators.
    d. RATED_VOLTAGE[7:0] — See the Rated Voltage Programming section for calculating the correct register value.
    e. OD_CLAMP[7:0] — See the Overdrive Voltage-Clamp Programming section for calculating the correct register value.
    f. AUTO_CAL_TIME[1:0] — A value of 3 is valid for most actuators.
    g. DRIVE_TIME[3:0]—SeetheDrive-TimeProgrammingforcalculatingthecorrectregistervalue.
    h. SAMPLE_TIME[1:0] — A value of 3 is valid for most actuators.
    i. BLANKING_TIME[1:0] — A value of 1 is valid for most actuators.
    j. IDISS_TIME[1:0] — A value of 1 is valid for most actuators
  **/

  float v_rms = 1.8;
  float v_peak = 1.8;
  float f_lra = 235;

  // Set auto-calibration mode
  drv.setMode(DRV2605_MODE_AUTOCAL);

  // Set LRA, FB_BRAKE_FACTOR, LOOP_GAIN, BEMF_GAIN (DEFAULS)
  uint8_t feedback_val = 0;
  feedback_val |= (1 << 7);  // Set LRA mode
  feedback_val |= (2 << 4);  // Set Brake Factor to 2
  feedback_val |= (2 << 2);  // Set Loop Gain to 2
  drv.writeRegister8(0x1A, feedback_val); // Original 0xBA

  // Set DRIVE_TIME (NEEDS CONFIRMATION)
  uint8_t ctrl1_val = 0;
  // float period_lra = 1.0 / f_lra;
  // uint8_t drive_time = (uint8_t) (((period_lra * 1000.0 / 2.0) -0.5) / 0.1);
  uint8_t drive_time = (uint8_t) (0.5 * (1/f_lra * 1000) - 0.5) / 0.1;
  ctrl1_val |= (drive_time & 0x1f);
  drv.writeRegister8(0x1B, ctrl1_val); // Original 0x93

  // Set SAMPLE_TIME, BLANKING_TIME, IDISS_TIME (DEFAULTS)
  uint8_t ctrl2_val = 0;
  ctrl2_val |= (3 << 4);  // Set sample to 300 us
  ctrl2_val |= (1 << 2);  // Set blanking time to 25 us
  ctrl2_val |= (1 << 0);  // Set idiss time to 25 us
  drv.writeRegister8(0x1C, ctrl2_val); // Original 0x35

  // // Set NG_THRESH and MODE
  // drv.writeRegister8(0x1D, 0x8C);

  // Set AUTO_CAL_TIME (DEFAULTS)
  uint8_t ctrl4_val = 0;
  ctrl4_val |= (3 << 4);  // Set auto cal time to 1000ms (min) to 1200ms (max)
  drv.writeRegister8(0x1E, ctrl4_val); // Original 0x20

  // Set RATED_VOLTAGE
  float sample_time = 300 * pow(10, -6);  // Sample time, default value 300 us
  float v_avg_abs = v_rms * sqrt(1 - (4 * sample_time + 300 * pow(10, -6)) * f_lra);
  uint8_t ratedv_val = (uint8_t) ((v_avg_abs * 255) / 5.3);
  drv.writeRegister8(0x16, ratedv_val);

  // Set OD_CLAMP
  uint8_t overdrive_val = (uint8_t) ((v_peak * 255) / 5.6);
  drv.writeRegister8(0x17, overdrive_val); // Original 0x69

  // Start auto-calibration
  drv.go();

  int8_t go = 1;
  int8_t i = 0;
  const int8_t loop_delay = 100;
  const int8_t timeout = 4000;

  // Wait for autocalibration to finish
  do {
    go = drv.readRegister8(DRV2605_REG_GO);
    delay(loop_delay);
    ++i;
  } while (go == 1 && i < timeout / loop_delay);

  Serial.print("Autocal Result: ");
  uint8_t DIAG_RESULT = (drv.readRegister8(0x00) & ( 1 << 3 )) >> 3;
  if (!DIAG_RESULT) Serial.println("SUCCESS");
  else Serial.println("FAILED");
}

void autocal_fixed() {
  drv.useLRA();
  drv.setMode(DRV2605_MODE_AUTOCAL);

  // Set ERM, Brake Factor and loop gain
  drv.writeRegister8(DRV2605_REG_FEEDBACK, 0xBA);
  drv.writeRegister8(DRV2605_REG_CONTROL1, 0x93);
  drv.writeRegister8(DRV2605_REG_CONTROL2, 0x35);
  drv.writeRegister8(DRV2605_REG_CONTROL3, 0x8C);
  drv.writeRegister8(DRV2605_REG_CONTROL4, 0x20);

  // Set voltages
  drv.writeRegister8(DRV2605_REG_RATEDV, 0x66);
  drv.writeRegister8(DRV2605_REG_CLAMPV, 0x69);

  // Set auto-calibration mode
  drv.setMode(DRV2605_MODE_AUTOCAL);

  // Start auto-calibration
  drv.go();

  int8_t go = 1;
  int8_t i = 0;
  const int8_t loop_delay = 100;
  const int8_t timeout = 4000;

  // Wait for autocalibration to finish
  do {
    go = drv.readRegister8(DRV2605_REG_GO);
    delay(loop_delay);
    ++i;
  } while (go == 1 && i < timeout / loop_delay);

  Serial.print("Autocal Result: ");
  uint8_t DIAG_RESULT = (drv.readRegister8(0x00) & ( 1 << 3 )) >> 3;
  if (!DIAG_RESULT) Serial.println("SUCCESS");
  else Serial.println("FAILED");
}

void nocal() {
  drv.useLRA();
}

uint8_t rtp_index = 0;
uint8_t rtp[] = {
  0x30, 100, 0x32, 100, 
  0x34, 100, 0x36, 100, 
  0x38, 100, 0x3A, 100,
  0x00, 100,
  0x40, 200, 0x00, 100, 
  0x40, 200, 0x00, 100, 
  0x40, 200, 0x00, 100
};

void loop() {

  drv.setRealtimeValue(127);
  Serial.println("hello!");
  // delay(100);

  // if (rtp_index < sizeof(rtp)/sizeof(rtp[0])) {
  //   drv.setRealtimeValue(rtp[rtp_index]);
  //   rtp_index++;
  //   delay(rtp[rtp_index]);
  //   rtp_index++;
  // } else {
  //   drv.setRealtimeValue(0x00);
  //   delay(1000);
  //   rtp_index = 0;
  // }
  
}