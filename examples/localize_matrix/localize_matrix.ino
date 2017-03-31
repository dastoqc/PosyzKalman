// Please read the ready-to-localize tuturial together with this example.
// https://www.pozyx.io/Documentation/Tutorials/ready_to_localize
/**
  This file is greatly inspired by the Poxyz 'ready_to_lcalize' example.

  Please read the tutorial that accompanies this sketch: https://www.pozyx.io/Documentation/Tutorials/ready_to_localize/Arduino

  by David St-Onge, 2017
  
*/
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>
#include <Kalman.h>

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanRX(0); // Create the angles' Kalman instances
Kalman kalmanRY(0);
Kalman kalmanX(1); // Create the positions' Kalman instances
Kalman kalmanY(1);
Kalman kalmanZ(1);
bool kalInit = false;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
coordinates_t Kalposition;

uint32_t timer;
////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

uint16_t remote_id = 0x6000;                            // set this to the ID of the remote device
bool remote = false;                                    // set this to true to use the remote ID

boolean use_processing = true;                         // set this to true to output data for the processing sketch

uint8_t num_anchors = 4;                                    // the number of anchors
uint16_t anchors[4] = {0x6077, 0x604d, 0x607c, 0x6028};     // the network id of the anchors: change these to the network ids of your anchors.
int32_t anchors_y[4] = {0, 0, 2500, 2500};               // anchor x-coorindates in mm
int32_t anchors_x[4] = {0, 1360, 1900, 0};                  // anchor y-coordinates in mm
int32_t heights[4] = {610, 610, 680, 720};              // anchor z-coordinates in mm

uint8_t algorithm = POZYX_POS_ALG_UWB_ONLY;             // positioning algorithm to use
uint8_t dimension = POZYX_3D;                           // positioning dimension
int32_t height = 1000;                                  // height of device, required in 2.5D positioning

int8_t cells[2] = {10, 10};
////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  //Pozyx.begin(true, MODE_POLLING)
  if (Pozyx.begin(false, MODE_INTERRUPT, POZYX_INT_MASK_IMU) == POZYX_FAILURE) {
    Serial.println(F("ERROR: Unable to connect to POZYX shield"));
    Serial.println(F("Reset required"));
    delay(100);
    abort();
  }

  if (!remote) {
    remote_id = NULL;
  }

  cells[0] = round((getMaximumValue(anchors_x,4) - getMinimumValue(anchors_x,4)) / 300);
  cells[1] = round((getMaximumValue(anchors_y,4) - getMinimumValue(anchors_y,4)) / 300);
  Serial.println(F("----------POZYX POSITIONING V1.0----------"));
  Serial.println(F("NOTES:"));
  Serial.println(F("- No parameters required."));
  Serial.println();
  Serial.println(F("- System will auto start anchor configuration"));
  Serial.println();
  Serial.println(F("- System will auto start positioning"));
  Serial.println(F("----------POZYX POSITIONING V1.0----------"));
  Serial.println();
  Serial.println(F("Performing manual anchor configuration:"));
  Serial.println();
  Serial.print(F("Creating grid: "));
  Serial.print(cells[0]);Serial.print(" x ");Serial.println(cells[1]);

  // clear all previous devices in the device list
  Pozyx.clearDevices(remote_id);
  // sets the anchor manually
  setAnchorsManual();
  
  printCalibrationResult();
  Serial.print(",");
  uint8_t calibration_status =0; Pozyx.getCalibrationStatus(&calibration_status);
  printCalibrationStatus(calibration_status);
  
  delay(2000);

  Serial.println(F("Starting positioning: "));
}

void loop() {
  coordinates_t position;
  sensor_raw_t sensor_raw;
  uint8_t calibration_status = 0;
  int dt;
  int statusP, statusR;
  if (remote) {
    statusP = Pozyx.doRemotePositioning(remote_id, &position, dimension, height, algorithm);
    statusR = Pozyx.getRawSensorData(&sensor_raw, remote_id);
    //statusR &= Pozyx.getCalibrationStatus(&calibration_status, remote_id);
  } else {
    statusP = Pozyx.doPositioning(&position, dimension, height, algorithm);
    if (Pozyx.waitForFlag(POZYX_INT_STATUS_IMU, 10) == POZYX_SUCCESS){
      statusR = Pozyx.getRawSensorData(&sensor_raw);
      //statusR &= Pozyx.getCalibrationStatus(&calibration_status);
      if(!kalInit)
        initKalman(sensor_raw, position);
      else
        updateKalman(sensor_raw, position);
    }else{
      uint8_t interrupt_status = 0;
      Pozyx.getInterruptStatus(&interrupt_status);
      return;
    }
  }

  if (statusP == POZYX_SUCCESS) {
    // prints out the result
    printCoordinates(Kalposition);//position);
  } else {
    // prints out the error code
    printErrorCode("positioning");
  }

  if (statusR == POZYX_SUCCESS) {
    // prints out the result
    //printRawSensorData(sensor_raw);
    //Serial.print(",");
    printKalman();
    //Serial.print(",");
    // will be zeros for remote devices as unavailable remotely.
    //printCalibrationStatus(calibration_status);
    Serial.println();
  } else {
    // prints out the error code
    printErrorCode("sensor");
  }

  playfile(getGridIndex(Kalposition), 0);
}

void playfile(int id, int level) {
  Serial.print("Should play file ");Serial.println(id);
}

void initKalman(sensor_raw_t sensor_raw, coordinates_t coor) {
  /* Set kalman and gyro starting angle */
  accX = sensor_raw.acceleration[0];
  accY = sensor_raw.acceleration[1];
  accZ = sensor_raw.acceleration[2];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanRX.setvalue(roll); // Set starting angle
  kalmanRY.setvalue(pitch);
  kalmanX.setvalue(coor.x); // Set starting position
  kalmanY.setvalue(coor.y);
  kalmanZ.setvalue(coor.z);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
  kalInit = true;
}

void updateKalman(sensor_raw_t sensor_raw, coordinates_t coor) {
/* Update all the values */
  accX = sensor_raw.acceleration[0];
  accY = sensor_raw.acceleration[1];
  accZ = sensor_raw.acceleration[2];
//  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = sensor_raw.angular_vel[0];
  gyroY = sensor_raw.angular_vel[1];
  gyroZ = sensor_raw.angular_vel[2];

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanRX.setvalue(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanRX.getvalue(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanRY.getvalue(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanRY.setvalue(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanRY.getvalue(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanRX.getvalue(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanRX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanRY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  // Position filtering
  Kalposition.x = kalmanX.getvalue(coor.x, accX, dt);
  Kalposition.y = kalmanY.getvalue(coor.y, accY, dt);
  Kalposition.z = kalmanZ.getvalue(coor.z, accZ, dt);
}

void printKalman() {
  //Serial.print(roll); Serial.print("\t");
  Serial.print(gyroXangle); Serial.print("\t");
  Serial.print(compAngleX); Serial.print("\t");
  Serial.print(kalAngleX); Serial.print("\t");

  Serial.print("\t");

  //Serial.print(pitch); Serial.print("\t");
  Serial.print(gyroYangle); Serial.print("\t");
  Serial.print(compAngleY); Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");
}
int32_t getMaximumValue(int32_t* array, int size){
 int8_t maxIndex = 0;
 int32_t maxV = array[maxIndex];
 for (int i=1; i<size; i++){
   if (maxV<array[i]){
     maxV = array[i];
     maxIndex = i;
   }
 }
 return maxV;
}

int32_t getMinimumValue(int32_t* array, int size){
 int8_t minIndex = 0;
 int32_t minV = array[minIndex];
 for (int i=1; i<size; i++){
   if (minV>array[i]){
     minV = array[i];
     minIndex = i;
   }
 }
 return minV;
}

int32_t getGridIndex(coordinates_t coor) {
  int32_t c = round((coor.x - getMinimumValue(anchors_x,4))/300)+1;
  int32_t r = round((coor.y - getMinimumValue(anchors_y,4))/300)+1;
  int32_t cellnb = c + cells[0]*(r-1);
  //Serial.print("Cell grid: ");Serial.print(c);Serial.print(" ");Serial.print(r);Serial.print(" ");Serial.println(cellnb);
  return cellnb;
}

void printRawSensorData(sensor_raw_t sensor_raw){
  Serial.print(sensor_raw.pressure);
  Serial.print(",");
  Serial.print(sensor_raw.acceleration[0]);
  Serial.print(",");
  Serial.print(sensor_raw.acceleration[1]);
  Serial.print(",");
  Serial.print(sensor_raw.acceleration[2]);
  Serial.print(",");
  Serial.print(sensor_raw.magnetic[0]);
  Serial.print(",");
  Serial.print(sensor_raw.magnetic[1]);
  Serial.print(",");
  Serial.print(sensor_raw.magnetic[2]);
  Serial.print(",");
  Serial.print(sensor_raw.angular_vel[0]);
  Serial.print(",");
  Serial.print(sensor_raw.angular_vel[1]);
  Serial.print(",");
  Serial.print(sensor_raw.angular_vel[2]);
  Serial.print(",");
  Serial.print(sensor_raw.euler_angles[0]);
  Serial.print(",");
  Serial.print(sensor_raw.euler_angles[1]);
  Serial.print(",");
  Serial.print(sensor_raw.euler_angles[2]);
  Serial.print(",");
  Serial.print(sensor_raw.quaternion[0]);
  Serial.print(",");
  Serial.print(sensor_raw.quaternion[1]);
  Serial.print(",");
  Serial.print(sensor_raw.quaternion[2]);
  Serial.print(",");
  Serial.print(sensor_raw.quaternion[3]);
  Serial.print(",");
  Serial.print(sensor_raw.linear_acceleration[0]);
  Serial.print(",");
  Serial.print(sensor_raw.linear_acceleration[1]);
  Serial.print(",");
  Serial.print(sensor_raw.linear_acceleration[2]);
  Serial.print(",");
  Serial.print(sensor_raw.gravity_vector[0]);
  Serial.print(",");
  Serial.print(sensor_raw.gravity_vector[1]);
  Serial.print(",");
  Serial.print(sensor_raw.gravity_vector[2]);
  Serial.print(",");
  Serial.print(sensor_raw.temperature);
}

void printCalibrationStatus(uint8_t calibration_status){
  Serial.print(calibration_status & 0x03);
  Serial.print(",");
  Serial.print((calibration_status & 0x0C) >> 2);
  Serial.print(",");
  Serial.print((calibration_status & 0x30) >> 4);
  Serial.print(",");
  Serial.print((calibration_status & 0xC0) >> 6);  
}

// prints the coordinates for either humans or for processing
void printCoordinates(coordinates_t coor) {
  uint16_t network_id = remote_id;
  if (network_id == NULL) {
    network_id = 0;
  }
  if (!use_processing) {
    Serial.print("POS ID 0x");
    Serial.print(network_id, HEX);
    Serial.print(", x(mm): ");
    Serial.print(coor.x);
    Serial.print(", y(mm): ");
    Serial.print(coor.y);
    Serial.print(", z(mm): ");
    Serial.print(coor.z);
    Serial.print(", Grid cell: ");
    Serial.println(getGridIndex(coor));
  } else {
    Serial.print("POS,0x");
    Serial.print(network_id, HEX);
    Serial.print(",");
    Serial.print(coor.x);
    Serial.print(",");
    Serial.print(coor.y);
    Serial.print(",");
    Serial.print(coor.z);
    Serial.print(",");
    int32_t tmp = getGridIndex(coor);
    Serial.print(tmp);
    Serial.println(",");
  }
}

// error printing function for debugging
void printErrorCode(String operation) {
  uint8_t error_code;
  if (remote_id == NULL) {
    Pozyx.getErrorCode(&error_code);
    Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(", local error code: 0x");
    Serial.println(error_code, HEX);
    return;
  }
  int status = Pozyx.getErrorCode(&error_code, remote_id);
  if (status == POZYX_SUCCESS) {
    Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(" on ID 0x");
    Serial.print(remote_id, HEX);
    Serial.print(", error code: 0x");
    Serial.println(error_code, HEX);
  } else {
    Pozyx.getErrorCode(&error_code);
    Serial.print("ERROR ");
    Serial.print(operation);
    Serial.print(", couldn't retrieve remote error code, local error: 0x");
    Serial.println(error_code, HEX);
  }
}

// print out the anchor coordinates (also required for the processing sketch)
void printCalibrationResult() {
  uint8_t list_size;
  int status;

  status = Pozyx.getDeviceListSize(&list_size, remote_id);
  Serial.print("list size: ");
  Serial.println(status * list_size);

  if (list_size == 0) {
    printErrorCode("configuration");
    return;
  }

  uint16_t device_ids[list_size];
  status &= Pozyx.getDeviceIds(device_ids, list_size, remote_id);

  Serial.println(F("Calibration result:"));
  Serial.print(F("Anchors found: "));
  Serial.println(list_size);

  coordinates_t anchor_coor;
  for (int i = 0; i < list_size; i++)
  {
    Serial.print("ANCHOR,");
    Serial.print("0x");
    Serial.print(device_ids[i], HEX);
    Serial.print(",");
    Pozyx.getDeviceCoordinates(device_ids[i], &anchor_coor, remote_id);
    Serial.print(anchor_coor.x);
    Serial.print(",");
    Serial.print(anchor_coor.y);
    Serial.print(",");
    Serial.println(anchor_coor.z);
  }
}

// function to manually set the anchor coordinates
void setAnchorsManual() {
  for (int i = 0; i < num_anchors; i++) {
    device_coordinates_t anchor;
    anchor.network_id = anchors[i];
    anchor.flag = 0x1;
    anchor.pos.x = anchors_x[i];
    anchor.pos.y = anchors_y[i];
    anchor.pos.z = heights[i];
    Pozyx.addDevice(anchor, remote_id);
  }
}
