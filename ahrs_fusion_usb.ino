#include <BasicLinearAlgebra.h>
#include <Geometry.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Mahony.h>
#include <Madgwick.h>

#define NXP_FXOS8700_FXAS21002      (2)

// Define your target sensor(s) here based on the list above!
// #define AHRS_VARIANT    ST_LSM303DLHC_L3GD20
#define AHRS_VARIANT   NXP_FXOS8700_FXAS21002

#if AHRS_VARIANT == NXP_FXOS8700_FXAS21002
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#else
#error "AHRS_VARIANT undefined! Please select a target sensor combination!"
#endif

//Loop timing
unsigned int read_imu_time = 0;

// Create sensor instances.
#if AHRS_VARIANT == NXP_FXOS8700_FXAS21002
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
#endif

// Mag calibration values are calculated via ahrs_calibration.
// These values must be determined for each baord/environment.
// See the image in this sketch folder for the values used
// below.

//FYI on 6-13-2017 my declination is: 1° 26' W  ± 0° 21'  changing by  0° 5' W per year

// Offsets applied to raw x/y/z mag values
float mag_offsets[3]            = { 0.93F, -7.47F, -35.23F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { {  0.943,  0.011,  0.020 },
                                    {  0.022,  0.918, -0.008 },
                                    {  0.020, -0.008,  1.156 } };

float mag_field_strength        = 50.23F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
float gyro_zero_offsets[3]      = { 0.0F, 0.0F, 0.0F };

// Mahony is lighter weight as a filter and should be used
// on slower systems
//Mahony filter;
Madgwick filter;

int filter_delay = 10; //in ms

void setup()
{
  Serial.begin(115200);

  // Wait for the Serial Monitor to open (comment out to run without Serial Monitor)
  // while(!Serial);

  Serial.println(F("Adafruit AHRS Fusion Example")); Serial.println("");

  // Initialize the sensors.
  if(!gyro.begin())
  {
    /* There was a problem detecting the gyro ... check your connections */
    Serial.println("Ooops, no gyro detected ... Check your wiring!");
    while(1);
  }

#if AHRS_VARIANT == NXP_FXOS8700_FXAS21002
  if(!accelmag.begin(ACCEL_RANGE_4G))
  {
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while(1);
  }
#else
  if (!accel.begin())
  {
    /* There was a problem detecting the accel ... check your connections */
    Serial.println("Ooops, no accel detected ... Check your wiring!");
    while (1);
  }

  if (!mag.begin())
  {
    /* There was a problem detecting the mag ... check your connections */
    Serial.println("Ooops, no mag detected ... Check your wiring!");
    while (1);
  }
#endif

  filter.begin(filter_delay);
}

//LOOOOOOOOOOOOOOOOOOOOOOOOP
void loop(void)
{
  unsigned long start_time = millis(); //overflow is near 50 days so it doesn't matter for this application.
  if (read_imu_time <= start_time) {
    //time to read the IMU!
      
    sensors_event_t gyro_event;
    sensors_event_t accel_event;
    sensors_event_t mag_event;
  
    // Get new data samples
    gyro.getEvent(&gyro_event);
  #if AHRS_VARIANT == NXP_FXOS8700_FXAS21002
    accelmag.getEvent(&accel_event, &mag_event);
  #else
    accel.getEvent(&accel_event);
    mag.getEvent(&mag_event);
  #endif
  
    // Apply mag offset compensation (base values in uTesla)
    float x = mag_event.magnetic.x - mag_offsets[0];
    float y = mag_event.magnetic.y - mag_offsets[1];
    float z = mag_event.magnetic.z - mag_offsets[2];
  
    // Apply mag soft iron error compensation
    float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
    float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
    float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];
  
    // Apply gyro zero-rate error compensation
    // gyro data in radians/sec
    float gx = gyro_event.gyro.x + gyro_zero_offsets[0];
    float gy = gyro_event.gyro.y + gyro_zero_offsets[1];
    float gz = gyro_event.gyro.z + gyro_zero_offsets[2];
  
    
  
    // Update the filter
    // The filter library expects gyro data in degrees/s, but adafruit sensor uses rad/s.
    filter.update(to_degrees(gx), to_degrees(gy), to_degrees(gz),
                  accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                  mx, my, mz);
  
    // Print the orientation filter output
    // Note: To avoid gimbal lock you should read quaternions not Euler
    // angles, but Euler angles are used here since they are easier to
    // understand looking at the raw values. See the ble fusion sketch for
    // and example of working with quaternion data.
    float roll = filter.getRoll();
    float pitch = filter.getPitch();
    float heading = filter.getYaw();
  
    //acceleration in body frame coordinates in accel_event.accerlation.x/y/z
    //transform to NED coordinates, which requires the rotation matrix from body to earth:
    Rotation Rbe;
    Rotation Reb;
    //It appears that this gives me the rotation from body to earth/inertial.  Huh.
    Rbe.FromEulerAngles(roll, pitch, heading);
    //Rotation from earth to body
    Reb = Rbe.Transpose();
    
    //acceleration (body frame) in (x,y,z) which corresponds to (roll, pitch, yaw)
    Matrix<3> Ab;
    Ab(0) = accel_event.acceleration.x;
    Ab(1) = accel_event.acceleration.y;
    Ab(2) = accel_event.acceleration.z;
    //Earth frame measured acceleration
    Matrix<3> Ae;
    Ae = Rbe * Ab;
  
    
    Serial.println(millis());
  //  Serial.print(" - Orientation: ");
  //  Serial.print(heading);
  //  Serial.print(" ");
  //  Serial.print(pitch);
  //  Serial.print(" ");
  //  Serial.println(roll);

    //set up when we should read IMU next:
    //overflow is near 50 days so it doesn't matter for this application.
    unsigned long end_time = millis();
    unsigned long processing_time = end_time - start_time;
    read_imu_time = start_time + filter_delay;
  }

float to_degrees(float rad) {
  return rad * 57.2958F;
}

float to_radians(float deg) {
  return rad / 57.2958F;
}
}
