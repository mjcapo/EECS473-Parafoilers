/* freertos.c - */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "Servo.h"
#include "Xbee.h"
#include "NEO_M9_UART.h"
#include "Control.h"
#include "ekfAlgo.h"
/* Private typedef -----------------------------------------------------------*/
// EKF INTEGRATION: Define loop rate for EKF predict step
#define EKF_LOOP_RATE_HZ 100
#define EKF_LOOP_PERIOD_MS (1000 / EKF_LOOP_RATE_HZ)
#define EKF_DT_S (1.0f / EKF_LOOP_RATE_HZ)

typedef struct {
    // [0-2] Fused State (EKF outputs)
    float fused_lat;           // 0: Fused latitude (deg)
    float fused_lon;           // 1: Fused longitude (deg)
    float fused_heading;       // 2: Fused heading (deg, 0-360)

    // [3-7] EKF Internal State (NED frame)
    float ekf_pos_n;           // 3: North position (m)
    float ekf_pos_e;           // 4: East position (m)
    float ekf_vel_n;           // 5: North velocity (m/s)
    float ekf_vel_e;           // 6: East velocity (m/s)
    float ekf_bias;            // 7: Heading bias (rad)

    // [8-11] Raw GPS Data
    float raw_gps_lat;         // 8: Raw GPS latitude (deg)
    float raw_gps_lon;         // 9: Raw GPS longitude (deg)
    float raw_gps_speed;       // 10: Raw GPS speed (m/s)
    float raw_gps_heading;     // 11: Raw GPS COG (deg, 0-360)

    // [12-14] Euler Angles (computed from quaternion)
    float roll;                // 12: Roll angle (deg, -180 to +180)
    float pitch;               // 13: Pitch angle (deg, -90 to +90)
    float yaw;                 // 14: Yaw angle (deg, 0-360)

    // [15-17] Gyroscope (body frame angular rates)
    float gyro_x;              // 15: Roll rate (deg/s)
    float gyro_y;              // 16: Pitch rate (deg/s)
    float gyro_z;              // 17: Yaw rate (deg/s)

    // [18-20] Accelerometer (body frame linear accel)
    float accel_x;             // 18: X acceleration (m/s²)
    float accel_y;             // 19: Y acceleration (m/s²)
    float accel_z;             // 20: Z acceleration (m/s²)

    // [21-23] Quaternion (for advanced debugging)
    float quat_w;              // 21: Quaternion W (real part)
    float quat_x;              // 22: Quaternion X
    float quat_y;              // 23: Quaternion Y

    // [24] Altitude
    float alt;

    // [25] Servo Command (Visual feedback)
    float servo_cmd;           // 25: Commanded Turn Angle (- = Left, + = Right)

    // [26] Target turn angle (how much we need to turn)
    float servo_left;
    float servo_right;

} TelemetryData_t;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

osMutexId_t printfMutexHandle;
const osMutexAttr_t printfMutex_attributes = {
  .name = "printfMutex"
};

// Task Handles and Attributes
osThreadId_t readGPSHandle;
const osThreadAttr_t readGPS_attributes = {
  .name = "readGPS",
  .stack_size = 512 * 8,
  .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t UpdateServosHandle;
const osThreadAttr_t UpdateServos_attributes = {
  .name = "UpdateServos",
  .stack_size = 512 * 2,
  .priority = (osPriority_t) osPriorityNormal
};

osThreadId_t ReadCoordHandle;
const osThreadAttr_t ReadCoord_attributes = {
  .name = "ReadCoord",
  .stack_size = 512 * 2,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

osThreadId_t NewGeoFenceHandle;
const osThreadAttr_t NewGeoFence_attributes = {
  .name = "NewGeoFence",
  .stack_size = 512 * 2,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

osThreadId_t bno085TaskHandle;
const osThreadAttr_t bno085Task_attributes = {
  .name = "bno085Task",
  .stack_size = 1024 * 4, // EKF uses matrix math, give it more stack
  .priority = (osPriority_t) osPriorityAboveNormal, // This is now our main EKF task
};

osThreadId_t SendDataHandle;
const osThreadAttr_t SendData_attributes = {
  .name = "SendData",
  .stack_size = 512 * 2,
  .priority = (osPriority_t) osPriorityLow,
};

// Semaphore Handle
osSemaphoreId_t gpsDataReadySemHandle;
const osSemaphoreAttr_t gpsDataReadySem_attributes = {
  .name = "gpsDataReadySem"
};

//Sempahore Handle IMU
osSemaphoreId_t imuDataReadySemHandle;
const osSemaphoreAttr_t imuDataReadySem_attributes = {
  .name = "imuDataReadySem"
};

//Sempahore Handle New Coordinate
osSemaphoreId_t newCoordSemHandle;
const osSemaphoreAttr_t newCoordSem_attributes = {
  .name = "newCoordSem"
};

//Sempahore Handle New Coordinate
osSemaphoreId_t newGeoFenceSemHandle;
const osSemaphoreAttr_t newGeoFenceSem_attributes = {
  .name = "newGeoFenceSem"
};

osMutexId_t navDataMutexHandle;
const osMutexAttr_t navDataMutex_attributes = {
  .name = "navDataMutex"
};

// External hardware handles and variables needed by the tasks

volatile control_mode_t control_mode = MODE_GPS_AUTO;
volatile uint8_t manual_servo_command = 0;
volatile uint32_t last_manual_command_time = 0;
volatile uint8_t servo_deadzone = 0;

volatile uint8_t undeployFlag = 0;

extern UART_HandleTypeDef huart1;
extern GPS_DATA_t myGpsData;
extern UBX_NAV_GEO_t geoData;
extern GPS_Info targetLocation;
extern uint8_t NEW_COORD_BUF[8];
extern uint8_t NEW_GEO_BUF[49];
extern volatile uint8_t gpsRxBuffer[];
extern volatile uint16_t gps_data_length;
extern sensor_meta sensor1;

ParafoilEKF my_ekf;

// EKF INTEGRATION: Noise parameters - THESE MUST BE TUNED
// Q: Process noise (model uncertainty). How much we trust the IMU prediction.
//    {pos_n, pos_e, pos_d, vel_n, vel_e, vel_d, heading_bias}
//    High val = less trust (model drifts), Low val = more trust (model is smooth)
float Q_init[EKF_NUM_STATES] = {
    0.01f, 0.01f, 0.01f,   // Position noise (m^2)
    0.1f,  0.1f,  0.1f,    // Velocity noise (m/s)^2
    0.2f                 // Heading bias noise (rad^2)
};

// R: Measurement noise (sensor uncertainty). How much we trust the GPS.
//    {pos_n, pos_e, pos_d, vel_n, vel_e, vel_d, heading}
//    High val = less trust (GPS is noisy), Low val = more trust (GPS is accurate)
float R_init[EKF_NUM_MEASUREMENTS] = {
    1.0f,  1.0f,  3.0f,    // GPS Pos noise (m^2) - N/E are ~1m, D is ~3m
    0.5f,  0.5f,  0.5f,    // GPS Vel noise (m/s)^2
    0.01f                   // GPS Heading noise (rad^2) ~ 5.7 degrees
};


// EKF INTEGRATION: This struct holds the fused state from the EKF
typedef struct {
    float fused_heading;      // 0-360 EKF Fused Heading
    double fused_latitude;    // EKF Fused Latitude (double for precision)
    double fused_longitude;   // EKF Fused Longitude (double for precision)
    float target_angle;       // 0-360 desired heading
    uint32_t lastGpsUpdateTick;
} NavData_t;

volatile NavData_t navData = {0};

volatile TelemetryData_t telemetryData = {0};

volatile float global_accel_x = 0, global_accel_y  = 0, global_accel_z = 0;
volatile float global_gyro_x = 0, global_gyro_y  = 0, global_gyro_z = 0;
volatile float global_quat_i = 0, global_quat_j = 0, global_quat_k = 0, global_quat_real = 0;


/* Private function prototypes -----------------------------------------------*/
void read_GPS(void *argument);
void updateServos(void *argument);
void readCoord(void *argument);
void newGeoFence(void *argument);
void StartBno085Task(void *argument);
// EKF INTEGRATION: Remove updateHeadingOffset prototype
void sendData(void *argument);
const char* get_accuracy_string(uint8_t accuracy);
void MX_FREERTOS_Init(void);


/**
 * @brief Convert quaternion to Euler angles (roll, pitch, yaw)
 * @param qw, qx, qy, qz: Quaternion components (normalized)
 * @param roll: Output roll angle in degrees (-180 to +180)
 * @param pitch: Output pitch angle in degrees (-90 to +90)
 * @param yaw: Output yaw angle in degrees (0 to 360)
 */
void quat_to_euler(float qw, float qx, float qy, float qz,
                   float *roll, float *pitch, float *yaw) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    *roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / M_PI;

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(90.0f, sinp); // Use 90 degrees if out of range
    else
        *pitch = asinf(sinp) * 180.0f / M_PI;

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    *yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / M_PI;

    // Normalize yaw to 0-360
    if (*yaw < 0.0f) *yaw += 360.0f;
}


/**
  * @brief  FreeRTOS initialization
  */
void MX_FREERTOS_Init(void) {
  printfMutexHandle = osMutexNew(&printfMutex_attributes);
  if (printfMutexHandle == NULL) {
      printf("ERROR: printfMutex creation failed\r\n");
      Error_Handler();
  }

  gpsDataReadySemHandle = osSemaphoreNew(1, 0, &gpsDataReadySem_attributes);
  if (gpsDataReadySemHandle == NULL) {
      printf("ERROR: gpsDataReadySem creation failed\r\n");
      Error_Handler();
  }

  imuDataReadySemHandle = osSemaphoreNew(1, 0, &imuDataReadySem_attributes);
  if (imuDataReadySemHandle == NULL) {
      printf("ERROR: imuDataReadySem creation failed\r\n");
      Error_Handler();
  }
  navDataMutexHandle = osMutexNew(&navDataMutex_attributes);
  if (navDataMutexHandle == NULL) {
      printf("ERROR: navDataMutex creation failed\r\n");
      Error_Handler();
  }

  newCoordSemHandle = osSemaphoreNew(1, 0, &newCoordSem_attributes);
  if (newCoordSemHandle == NULL) {
      printf("ERROR: newCoordSem creation failed\r\n");
      Error_Handler();
  }

  newGeoFenceSemHandle = osSemaphoreNew(1, 0, &newGeoFenceSem_attributes);
  if (newGeoFenceSemHandle == NULL) {
      printf("ERROR: newGeoFenceSem creation failed\r\n");
      Error_Handler();
  }

  printf("Semaphore/mutex created successfully\r\n");
  // EKF INTEGRATION: Initialize your EKF here
  // Pinit (10.0) is the initial uncertainty. High value = "I don't know my state"
  ParafoilEKF_Init(&my_ekf, 10.0f, Q_init, R_init);
  printf("EKF Initialized.\r\n");


  printf("Creating readGPS task...\r\n");
  readGPSHandle = osThreadNew(read_GPS, NULL, &readGPS_attributes);
  if (readGPSHandle == NULL) { Error_Handler(); }
  printf("readGPS task created\r\n");

  printf("Creating UpdateServos task...\r\n");
  UpdateServosHandle = osThreadNew(updateServos, NULL, &UpdateServos_attributes);
  if (UpdateServosHandle == NULL) { Error_Handler(); }
  printf("UpdateServos task created\r\n");

  //Suspend controls until release
	osThreadSuspend(UpdateServosHandle);


  printf("Creating ReadCoord task...\r\n");
  ReadCoordHandle = osThreadNew(readCoord, NULL, &ReadCoord_attributes);
  if (ReadCoordHandle == NULL) { Error_Handler(); }
  printf("ReadCoord task created\r\n");

  printf("Creating NewGeoFence task...\r\n");
  NewGeoFenceHandle = osThreadNew(newGeoFence, NULL, &NewGeoFence_attributes);
  if (NewGeoFenceHandle == NULL) { Error_Handler(); }
  printf("NewGeoFence task created\r\n");

  printf("Creating bno085Task...\r\n");
  bno085TaskHandle = osThreadNew(StartBno085Task, NULL, &bno085Task_attributes);
  if (bno085TaskHandle == NULL) { Error_Handler(); }
  printf("bno085Task created\r\n");


  printf("Creating SendData task...\r\n");
  SendDataHandle = osThreadNew(sendData, NULL, &SendData_attributes);
  if (SendDataHandle == NULL) { Error_Handler(); }
  printf("SendData created\r\n");
}



void read_GPS(void *argument)
{
    uint8_t local_gps_buffer[RX_BUFFER_LEN];
    uint16_t local_data_length;
    // EKF INTEGRATION: Target angle is no longer calculated here

    osMutexAcquire(printfMutexHandle, osWaitForever);
    printf("GPS Task Started\r\n");
    osMutexRelease(printfMutexHandle);

    osMutexAcquire(printfMutexHandle, osWaitForever);
    printf("Configuring GPS module...\r\n");
    M9N_Init(&huart1);
    osMutexRelease(printfMutexHandle);


    osDelay(1000);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)gpsRxBuffer, RX_BUFFER_LEN);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

    osMutexAcquire(printfMutexHandle, osWaitForever);
    printf("GPS DMA reception started\r\n");
    osMutexRelease(printfMutexHandle);

    osDelay(1000);

    osSemaphoreAcquire(gpsDataReadySemHandle, osWaitForever);

    osMutexAcquire(printfMutexHandle, osWaitForever);
    printf("Got GPS Semaphore\r\n");
    osMutexRelease(printfMutexHandle);


    for(;;)
    {
        if (osSemaphoreAcquire(gpsDataReadySemHandle, osWaitForever) == osOK)
        {
        	//enter area where GPS data cannot be interrupted as we get its data
            taskENTER_CRITICAL();
            memcpy(local_gps_buffer, (void*)gpsRxBuffer, gps_data_length);
            local_data_length = gps_data_length;
            taskEXIT_CRITICAL();

            // processGPS will parse the data and set myGpsData.newData = 1
            processGPS(local_gps_buffer, local_data_length);

            // EKF INTEGRATION: The EKF task will now handle this logic.
            // This task's *only* job is to parse data and set the flag.

            // We can add a print here to show GPS is alive
            if (myGpsData.newData) {
            	 osMutexAcquire(printfMutexHandle, osWaitForever);
                 printf("GPS: %.6f, %.6f, %.1fm, Sats=%d, %.6f, %.6f\r\n",
                        myGpsData.Latitude, myGpsData.Longitude,
                        myGpsData.Altitude, myGpsData.Satellites, myGpsData.headMot, myGpsData.headAcc);
                 osMutexRelease(printfMutexHandle);
            }
        }

    }
}


void updateServos(void *argument)
{
  osMutexAcquire(printfMutexHandle, osWaitForever);
  printf("UpdateServos Task Started.\r\n");
  osMutexRelease(printfMutexHandle);


  // Local copies of shared variables
  float local_target_angle;
  float local_heading_real;
  uint32_t local_lastGpsUpdateTick;
  float commanded_turn_val = 0.0f;
  float servo_p = 1.25f;
  //float target_turn_val = 0.0f;
  float servo_left_cmd = 0.0f;
  float servo_right_cmd = 0.0f;

  // Wait for GPS fix or manual mode
  // EKF INTEGRATION: Wait for EKF to be ready
  while (!ParafoilEKF_IsReady(&my_ekf) && control_mode == MODE_GPS_AUTO) {
      servosReset();
      osDelay(1000);
  }

  osMutexAcquire(printfMutexHandle, osWaitForever);
  printf("Servo Task: Control active.\r\n");
  osMutexRelease(printfMutexHandle);


  for(;;)
  {
	  if(undeployFlag) {
		  commanded_turn_val = 0.0f;
          servo_left_cmd = 0.0f;
          servo_right_cmd = 0.0f;
		  osThreadSuspend(UpdateServosHandle);
	  }

        uint32_t current_tick = osKernelGetTickCount();
        commanded_turn_val = 0.0f;
        servo_left_cmd = 0.0f;
        servo_right_cmd = 0.0f;

        // Safely read the shared variable using mutex
        if (osMutexAcquire(navDataMutexHandle, 10) == osOK) {
            local_lastGpsUpdateTick = navData.lastGpsUpdateTick;
            osMutexRelease(navDataMutexHandle);
        } else {
            // Failed to get mutex, use stale data for this loop
        }

	    // Auto-exit FALLBACK manual mode when GPS+target is ready
	    if(control_mode == MODE_MANUAL_FALLBACK) {
            // Need to read targetLocation safely
            float local_target_lat = 0;
            if (osMutexAcquire(navDataMutexHandle, 10) == osOK) {
                local_target_lat = targetLocation.latitude;
                osMutexRelease(navDataMutexHandle);
            }

	        // EKF INTEGRATION: Check if EKF is ready
	        bool ekf_is_ready = ParafoilEKF_IsReady(&my_ekf);
	        bool has_target = (local_target_lat != 0);
	        bool gps_is_fresh = (local_lastGpsUpdateTick > 0) && ((current_tick - local_lastGpsUpdateTick) < 2000);

	        if(ekf_is_ready && has_target && gps_is_fresh) {
	            osMutexAcquire(printfMutexHandle, osWaitForever);
	            printf("Auto-switching from fallback to GPS control mode\r\n");
	            osMutexRelease(printfMutexHandle);
	            control_mode = MODE_GPS_AUTO;
	            manual_servo_command = 0;
	        }
	    }

	    // Execute control based on mode
	if(control_mode == MODE_MANUAL_FALLBACK || control_mode == MODE_MANUAL_OVERRIDE) {
		// Manual control (either fallback or override)
		if(manual_servo_command == 1) {
			turnLeft(144); //Left
			commanded_turn_val = -90.0f;
            servo_left_cmd = 144.0f;
            servo_right_cmd = 0.0f;
		} else if(manual_servo_command == 2) {
			turnRight(144);  // Right
			commanded_turn_val = 90.0f;
            servo_left_cmd = 0.0f;
            servo_right_cmd = 144.0f;
		} else {
			servosReset();
			commanded_turn_val = 0.0f;
            servo_left_cmd = 0.0f;
            servo_right_cmd = 0.0f;
		}
    }
//	else if (withinTarget(targetLocation.longitude, targetLocation.latitude, myGpsData.Longitude, myGpsData.Latitude)) {
//		deadSpin();
//	}
    else {
        // Attempt at automatic gps-based control
        if (local_lastGpsUpdateTick == 0) {
            local_lastGpsUpdateTick = current_tick; // Initialize if it's zero
        }

        bool gps_signal_lost = (current_tick - local_lastGpsUpdateTick) > 2000;
        // EKF INTEGRATION: Check if EKF is ready
        bool ekf_is_ready = ParafoilEKF_IsReady(&my_ekf);

        // Check if the payload is moving which means the parafoil deployed
        bool valid_heading = (myGpsData.Ground_Speed > 0.5f);


        // Need to read targetLocation safely
        float local_target_lat = 0;
        if (osMutexAcquire(navDataMutexHandle, 10) == osOK) {
            local_target_lat = targetLocation.latitude;
            osMutexRelease(navDataMutexHandle);
        }

        if(ekf_is_ready && local_target_lat != 0 && !gps_signal_lost) //maybe add valid_heading here
        {
            // Safely read shared variables using mutex
            if (osMutexAcquire(navDataMutexHandle, 10) == osOK) {
                local_target_angle = navData.target_angle;
                // EKF INTEGRATION: Use the fused heading
                local_heading_real = navData.fused_heading;
                osMutexRelease(navDataMutexHandle);
            } else {
                // Failed to get mutex, skip this control loop
                continue;
            }

            // Geofencing check
			if(geoData.combState != 2 && geoData.status == 1) {
				deadSpin();
				commanded_turn_val = 360.0f;
                servo_right_cmd = 90.0f;
                servo_left_cmd = 0.0f;
				continue;
			}

            // The turn angle must be normalized to the range [-180, 180]
        	float turn_angle = normalize_angle(local_target_angle - local_heading_real);
        	commanded_turn_val = turn_angle;
        	float servo_angle_val = fabs(turn_angle / servo_p);

        	if(servo_angle_val < 45.0f) {
        		servo_angle_val = 45.0f;
        	}


            if(fabs(turn_angle) >= 20) { // 20 degree deadband
            	if(fabs(turn_angle) < 160){
            		servo_deadzone = 0;
            	}

            	if(servo_deadzone == 1) {
            		continue;
            	}

                if(turn_angle >= 160){ //180 degrees deadzone
                	servo_deadzone = 1;

					turnRight(servo_angle_val);
                    servo_right_cmd = servo_angle_val;
                    servo_left_cmd = 0.0f;
                }
                else if(turn_angle <= -160){ //180 degrees deadzone
                	servo_deadzone = 1;

					turnLeft(servo_angle_val);
                    servo_left_cmd = servo_angle_val;
                    servo_right_cmd = 0.0f;
                }
            	else if(turn_angle < 0) {
                    // Need to turn left
                    turnLeft(servo_angle_val);
                    servo_left_cmd = servo_angle_val;
                    servo_right_cmd = 0.0f;
                } else if(turn_angle > 0){
                    // Need to turn right
                    turnRight(servo_angle_val);
                    servo_right_cmd = servo_angle_val;
                    servo_left_cmd = 0.0f;
                }
            } else {
                // We are on target, go straight (stop servos)
                servosReset();
                servo_left_cmd = 0.0f;
                servo_right_cmd = 0.0f;
            }
        } else {
            // No fix, no target, or lost signal = STOP
            servosReset();
            commanded_turn_val = 0.0f;
            servo_left_cmd = 0.0f;
            servo_right_cmd = 0.0f;
        }
    }

    if (osMutexAcquire(navDataMutexHandle, 10) == osOK) {
        telemetryData.servo_cmd = commanded_turn_val;
        telemetryData.servo_left = servo_left_cmd;
        telemetryData.servo_right = servo_right_cmd;
        osMutexRelease(navDataMutexHandle);
    }


    // Reduced delay for a faster control loop (10 Hz)
    osDelay(50);
  }
}


void readCoord(void *argument)
{
  osMutexAcquire(printfMutexHandle, osWaitForever);
  printf("ReadCoord Task Started.\r\n");
  osMutexRelease(printfMutexHandle);

  for(;;)
  {
    if(osSemaphoreAcquire(newCoordSemHandle, osWaitForever) == osOK) {
        int32_t lat = (NEW_COORD_BUF[0] << 24) | (NEW_COORD_BUF[1] << 16) |
                      (NEW_COORD_BUF[2] << 8) | NEW_COORD_BUF[3];
        int32_t lon = (NEW_COORD_BUF[4] << 24) | (NEW_COORD_BUF[5] << 16) |
                      (NEW_COORD_BUF[6] << 8) | NEW_COORD_BUF[7];

        float lat_float = (float)lat / 10000000.0f;
        float lon_float = (float)lon / 10000000.0f;

        // Protect shared variable write
        if (osMutexAcquire(navDataMutexHandle, 100) == osOK) {
            targetLocation.latitude = lat_float;
            targetLocation.longitude = lon_float;

            // EKF INTEGRATION: Update the EKF's reference if it's not set,
            // or just update our fused position to this new target.
            // This prevents a large jump when the EKF calculates target angle
            if (!ParafoilEKF_IsReady(&my_ekf)) {
                 // EKF hasn't been initialized by GPS yet, just store target
            } else {
                // EKF is running, update our fused state to match
                // This assumes receiving a new coordinate resets our position
                navData.fused_latitude = lat_float;
                navData.fused_longitude = lon_float;
            }
            osMutexRelease(navDataMutexHandle);
        }

        osMutexAcquire(printfMutexHandle, osWaitForever);
        printf("New coordinate received from Xbee!\r\n");
        // Removed %f, casting to int to prevent stack corruption
        printf("  Target Lat_int:  %ld\r\n", (int32_t)(lat_float * 10000));
        printf("  Target Lon_int: %ld\r\n", (int32_t)(lon_float * 10000));
        osMutexRelease(printfMutexHandle);
    }
    osDelay(100);
  }
}

void newGeoFence(void *argument)
{
  osMutexAcquire(printfMutexHandle, osWaitForever);
  printf("NewGeoFence Task Started.\r\n");
  osMutexRelease(printfMutexHandle);

  for(;;)
  {
    if(osSemaphoreAcquire(newGeoFenceSemHandle, osWaitForever) == osOK) {
        int numFences = NEW_GEO_BUF[0];

        int32_t lat_buf[numFences];
        int32_t lon_buf[numFences];
        uint32_t rad_buf[numFences];

        for(int i = 0; i < numFences; i++)
        {
        	lat_buf[i] = (NEW_GEO_BUF[1+i*12] << 24) | (NEW_GEO_BUF[2+i*12] << 16) |
						  (NEW_GEO_BUF[3+i*12] << 8) | NEW_GEO_BUF[4+i*12];
			lon_buf[i] = (NEW_GEO_BUF[5+i*12] << 24) | (NEW_GEO_BUF[6+i*12] << 16) |
						  (NEW_GEO_BUF[7+i*12] << 8) | NEW_GEO_BUF[8+i*12];
			rad_buf[i] =(NEW_GEO_BUF[9+i*12] << 24) | (NEW_GEO_BUF[10+i*12] << 16) |
						  (NEW_GEO_BUF[11+i*12] << 8) | NEW_GEO_BUF[12+i*12];

        }

        config_geofence(&huart1, numFences, 2, lat_buf, lon_buf, rad_buf);

        osMutexAcquire(printfMutexHandle, osWaitForever);
        printf("New GeoFence from Xbee!\r\n");

        for(int i = 0; i < numFences; ++i) {
            float lat_float = (float)lat_buf[i] / 10000000.0f;
            float lon_float = (float)lon_buf[i] / 10000000.0f;
            float rad_float = (float)rad_buf[i] / 100.0f;

        	printf(" Fence #%d Latitude: %.6f, Longitude: %.6f, Radius: %.6f\r\n", i, lat_float, lon_float, rad_float);
        }

        osMutexRelease(printfMutexHandle);
    }
    osDelay(100);
  }
}

void StartBno085Task(void *argument)
{
  osMutexAcquire(printfMutexHandle, osWaitForever);
  printf("BNO085 Task (EKF_Task): Initializing...\r\n");
  osMutexRelease(printfMutexHandle);

  // Enable sensors at 100Hz (10ms = 100Hz)
  enable_Gyroscope(&sensor1, 10);
  osDelay(250);
  enable_LinearAcceleration(&sensor1, 10);
  osDelay(250);
  enable_GameRotationVector(&sensor1, 10);
  osDelay(250);

  osMutexAcquire(printfMutexHandle, osWaitForever);
  printf("BNO085 Task: Waiting for sensor to stabilize...\r\n");
  osMutexRelease(printfMutexHandle);
  osDelay(500);

  osMutexAcquire(printfMutexHandle, osWaitForever);
  printf("BNO085 Task: Taring sensor...\r\n");
  osMutexRelease(printfMutexHandle);
  tare_now(&sensor1, TARE_AXES_ALL, TARE_BASIS_GAME_ROT_VEC);
  osDelay(100);

  osMutexAcquire(printfMutexHandle, osWaitForever);
  printf("BNO085 Task (EKF_Task): Ready, starting interrupt-driven loop\r\n");
  osMutexRelease(printfMutexHandle);

  uint32_t last_print_time = 0;
  uint32_t interrupt_count = 0;
  uint32_t data_read_count = 0;

  // EKF INTEGRATION: Local copies for EKF data
  float quat[4]; // [qw, qx, qy, qz]
  float accel_body[3]; // [ax, ay, az]
  float gyro[3]; // [gx, gy, gz]


  // For calculating dt dynamically
  uint32_t last_update_tick = osKernelGetTickCount();

  get_and_clear_Reset_Status(&sensor1);

  while(osSemaphoreAcquire(imuDataReadySemHandle, 0) == osOK) {
      // Flush semaphore
  }

  osMutexAcquire(printfMutexHandle, osWaitForever);
  printf("BNO085 Task: Cleared init artifacts, starting main loop\r\n");
  osMutexRelease(printfMutexHandle);

  //uint32_t loop_iteration = 0;
  uint32_t timeout_count = 0;       // ← ADD THIS

  for(;;)
  {

    // Check for reset (still polled, happens rarely)
    if(get_and_clear_Reset_Status(&sensor1)) {
        osMutexAcquire(printfMutexHandle, osWaitForever);
        printf("IMU reset detected, re-enabling reports...\r\n");
        osMutexRelease(printfMutexHandle);

        enable_Gyroscope(&sensor1, 10);
        osDelay(250);
        enable_LinearAcceleration(&sensor1, 10);
        osDelay(250);
        enable_GameRotationVector(&sensor1, 10);
        osDelay(250);
        //tare_now(&sensor1, TARE_AXES_ALL, TARE_BASIS_GAME_ROT_VEC);
        osDelay(1000);  // Increased wait time

        // Clear any spurious semaphore signals
        while(osSemaphoreAcquire(imuDataReadySemHandle, 0) == osOK) {
            // Flush semaphore
        }

        sensor1.INTN_ready = 0;
        last_update_tick = osKernelGetTickCount();
    }

    // Wait for interrupt signal (with 100ms timeout as safety)
    if(osSemaphoreAcquire(imuDataReadySemHandle, 100) == osOK) {
        interrupt_count++;

        // *** USE FORCE READ - bypasses pin check ***
        if(force_read_data(&sensor1)) {
            data_read_count++;

            if(data_read_count % 50 == 0) {
                if(osMutexAcquire(printfMutexHandle, 10) == osOK) {
                    printf("IMU: Read #%lu, Quat_real=%.3f, Accel_Z=%.3f, Gyro_Z=%.3f\r\n",
                           data_read_count, global_quat_real, global_accel_z, global_gyro_z);
                    osMutexRelease(printfMutexHandle);
                }
            }
            // Calculate dynamic dt
            uint32_t current_tick = osKernelGetTickCount();
            float dt = (float)(current_tick - last_update_tick) / 1000.0f;
            last_update_tick = current_tick;

            // Clamp dt to reasonable values
            if(dt < 0.001f) dt = 0.01f; // dt is too small, use target
            if(dt > 0.1f)   dt = 0.1f;  // dt is too large, clamp to a max (e.g., 100ms)

            // Read sensor data and protect globals
            taskENTER_CRITICAL();
            global_quat_i = get_Quat_I(&sensor1);
            global_quat_j = get_Quat_J(&sensor1);
            global_quat_k = get_Quat_K(&sensor1);
            global_quat_real = get_Quat_Real(&sensor1);
            global_accel_x = get_LinearAcceleration_X(&sensor1);
            global_accel_y = get_LinearAcceleration_Y(&sensor1);
            global_accel_z = get_LinearAcceleration_Z(&sensor1);
            global_gyro_x = get_Gyroscope_X(&sensor1);
            global_gyro_y = get_Gyroscope_Y(&sensor1);
            global_gyro_z = get_Gyroscope_Z(&sensor1);
            taskEXIT_CRITICAL();

            // Prepare data for EKF
            quat[0] = global_quat_real;
            quat[1] = global_quat_i;
            quat[2] = global_quat_j;
            quat[3] = global_quat_k;
            accel_body[0] = global_accel_x;
            accel_body[1] = global_accel_y;
            accel_body[2] = global_accel_z;
            gyro[0] = global_gyro_x;
            gyro[1] = global_gyro_y;
            gyro[2] = global_gyro_z;

            float roll_deg, pitch_deg, yaw_deg;
            quat_to_euler(quat[0], quat[1], quat[2], quat[3],
                          &roll_deg, &pitch_deg, &yaw_deg);


            if (osMutexAcquire(navDataMutexHandle, 10) == osOK) {
                // Raw IMU data (always update these)
                telemetryData.roll = roll_deg;
                telemetryData.pitch = pitch_deg;
                telemetryData.yaw = yaw_deg;
                telemetryData.gyro_x = gyro[0];
                telemetryData.gyro_y = gyro[1];
                telemetryData.gyro_z = gyro[2];
                telemetryData.accel_x = accel_body[0];
                telemetryData.accel_y = accel_body[1];
                telemetryData.accel_z = accel_body[2];
                telemetryData.quat_w = quat[0];
                telemetryData.quat_x = quat[1];
                telemetryData.quat_y = quat[2];


                osMutexRelease(navDataMutexHandle);
            }

            // --- EKF PREDICT STEP ---
            ParafoilEKF_Predict(&my_ekf, quat, accel_body, dt);

            // --- EKF UPDATE STEP ---
            bool has_new_gps_data = false;
            taskENTER_CRITICAL();
            if (myGpsData.newData) {
                has_new_gps_data = true;
                myGpsData.newData = 0;
            }
            taskEXIT_CRITICAL();

            if (has_new_gps_data) {
                uint8_t valid_heading = (myGpsData.Ground_Speed > 0.5f);
                if(osMutexAcquire(printfMutexHandle, 10) == osOK) {
					printf("GROUND SPEED = %f\r\n", myGpsData.Ground_Speed);
					osMutexRelease(printfMutexHandle);
				}
                ParafoilEKF_UpdateGPS(&my_ekf, myGpsData.Latitude, myGpsData.Longitude,
                                      myGpsData.Altitude, myGpsData.Ground_Speed,
                                      myGpsData.headMot, quat, valid_heading);
            }

            // --- UPDATE SHARED NAVDATA ---
            if (ParafoilEKF_IsReady(&my_ekf)) {
                float new_fused_heading = ParafoilEKF_GetHeading(&my_ekf, quat);
                double new_fused_lat, new_fused_lon;
                float new_fused_alt;
                ParafoilEKF_GetPositionGPS(&my_ekf, &new_fused_lat, &new_fused_lon, &new_fused_alt);

                // Update all telemetry data
                if (osMutexAcquire(navDataMutexHandle, 10) == osOK) {
                    navData.fused_heading = new_fused_heading;
                    navData.fused_latitude = new_fused_lat;
                    navData.fused_longitude = new_fused_lon;

                    telemetryData.fused_lat = (float)new_fused_lat;
                    telemetryData.fused_lon = (float)new_fused_lon;
                    telemetryData.fused_heading = new_fused_heading;
                    telemetryData.ekf_pos_n = my_ekf.pos_n;
                    telemetryData.ekf_pos_e = my_ekf.pos_e;
                    telemetryData.ekf_vel_n = my_ekf.vel_n;
                    telemetryData.ekf_vel_e = my_ekf.vel_e;
                    telemetryData.ekf_bias = my_ekf.heading_bias * RAD_TO_DEG;
                    telemetryData.roll = roll_deg;
                    telemetryData.pitch = pitch_deg;
                    telemetryData.yaw = yaw_deg;
                    telemetryData.gyro_x = gyro[0];
                    telemetryData.gyro_y = gyro[1];
                    telemetryData.gyro_z = gyro[2];
                    telemetryData.accel_x = accel_body[0];
                    telemetryData.accel_y = accel_body[1];
                    telemetryData.accel_z = accel_body[2];
                    telemetryData.quat_w = quat[0];
                    telemetryData.quat_x = quat[1];
                    telemetryData.quat_y = quat[2];
                    telemetryData.alt = new_fused_alt;

                    if(has_new_gps_data) {
                        navData.lastGpsUpdateTick = osKernelGetTickCount();
                        telemetryData.raw_gps_lat = myGpsData.Latitude;
                        telemetryData.raw_gps_lon = myGpsData.Longitude;
                        telemetryData.raw_gps_speed = myGpsData.Ground_Speed;
                        telemetryData.raw_gps_heading = myGpsData.headMot;
                    }

                    float local_target_lat = targetLocation.latitude;
                    float local_target_lon = targetLocation.longitude;
                    if (local_target_lat != 0) {
                        float new_target_angle = calc_angle(local_target_lon, local_target_lat,
                                                            new_fused_lon, new_fused_lat);
                        navData.target_angle = new_target_angle;
                    }
                    osMutexRelease(navDataMutexHandle);
                }

                // Periodic status print
                uint32_t current_time = osKernelGetTickCount();
                if (current_time - last_print_time >= 500) {
                    uint8_t accuracy = get_Quat_Accuracy(&sensor1);
                    if(osMutexAcquire(printfMutexHandle, 10) == osOK) {
                        printf("EKF: H=%.1f T=%.1f dt=%.3f Ints=%lu Reads=%lu %s\r\n",
                               new_fused_heading, navData.target_angle, dt,
                               interrupt_count, data_read_count,
                               get_accuracy_string(accuracy));
                        osMutexRelease(printfMutexHandle);
                    }
                    last_print_time = current_time;
                }
            }
        } else {
            // force_read_data failed

            timeout_count++;

            if(timeout_count % 10 == 0) {
                if(osMutexAcquire(printfMutexHandle, 10) == osOK) {
                    printf("BNO085: Semaphore timeout #%lu (no interrupt)\r\n", timeout_count);
                    osMutexRelease(printfMutexHandle);
                }
            }
        }
    }

    osDelay(10);
  }
}


void sendData(void *argument)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 500;
    xLastWakeTime = xTaskGetTickCount();

    // UPDATED: 25 floats = 100 bytes + 1 newline = 101 bytes total
    // UPDATE V2: 26 floats (Added Servo Cmd) = 104 bytes + 1 newline = 105 bytes
    uint16_t data_size = sizeof(float) * 28;
    uint8_t buf[data_size + 1];
    float payload[28];

    osMutexAcquire(printfMutexHandle, osWaitForever);
    printf("SendData Task Started.\r\n");
    osMutexRelease(printfMutexHandle);

    for(;;) {
        // Safely copy telemetry data
        if (osMutexAcquire(navDataMutexHandle, 10) == osOK) {
            // [0-2] Fused State
            payload[0] = telemetryData.fused_lat;
            payload[1] = telemetryData.fused_lon;
            payload[2] = telemetryData.fused_heading;

            // [3-7] EKF State
            payload[3] = telemetryData.ekf_pos_n;
            payload[4] = telemetryData.ekf_pos_e;
            payload[5] = telemetryData.ekf_vel_n;
            payload[6] = telemetryData.ekf_vel_e;
            payload[7] = telemetryData.ekf_bias;

            // [8-11] Raw GPS
            payload[8] = telemetryData.raw_gps_lat;
            payload[9] = telemetryData.raw_gps_lon;
            payload[10] = telemetryData.raw_gps_speed;
            payload[11] = telemetryData.raw_gps_heading;

            // [12-14] Euler Angles
            payload[12] = telemetryData.roll;
            payload[13] = telemetryData.pitch;
            payload[14] = telemetryData.yaw;

            // [15-17] Gyroscope
            payload[15] = telemetryData.gyro_x;
            payload[16] = telemetryData.gyro_y;
            payload[17] = telemetryData.gyro_z;

            // [18-20] Accelerometer
            payload[18] = telemetryData.accel_x;
            payload[19] = telemetryData.accel_y;
            payload[20] = telemetryData.accel_z;

            // [21-23] Quaternion
            payload[21] = telemetryData.quat_w;
            payload[22] = telemetryData.quat_x;
            payload[23] = telemetryData.quat_y;

            payload[24] = telemetryData.alt;

            // [25] Servo Command
            payload[25] = telemetryData.servo_cmd;

            payload[26] = telemetryData.servo_left;
            payload[27] = telemetryData.servo_right;


            osMutexRelease(navDataMutexHandle);
        }

        memcpy(buf, payload, data_size);
        buf[data_size] = '\n';

        xbee_send_data(buf, data_size + 1);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

const char* get_accuracy_string(uint8_t accuracy) {
    switch (accuracy) {
        case 0: return "Unreliable";
        case 1: return "Low";
        case 2: return "Medium";
        case 3: return "High";
        default: return "Unknown";
    }
}
