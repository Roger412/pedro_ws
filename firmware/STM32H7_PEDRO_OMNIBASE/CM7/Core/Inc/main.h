/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

#define PD7_Pin GPIO_PIN_7
#define PD6_Pin GPIO_PIN_6
#define PD5_Pin GPIO_PIN_5
#define PD4_Pin GPIO_PIN_4

#define PE2_Pin GPIO_PIN_2
#define PE3_Pin GPIO_PIN_3
#define PE4_Pin GPIO_PIN_4
#define PE6_Pin GPIO_PIN_6

typedef struct {
    float Kp;
    float Ki;
    float Kd;
} PIDGains;

typedef struct {
    PIDGains x_pid;
    PIDGains y_pid;
    PIDGains phi_pid;
    PIDGains u_pid[4];  // Per-wheel PID
} PIDConfig;

typedef struct
{
	double x_desired;
	double y_desired;
	double phi_end;
	double d;
	double x_off;
	double y_off;
	double r;
//	uint8_t state;
	float u1_desired;
	float u2_desired;
	float u3_desired;
	float u4_desired;

} InputData;

typedef struct {
    double yaw;     // radians, from BNO085 rotation vector
    double roll;    // radians
    double pitch;   // radians
    float qx;       // quaternion x (ROS convention)
    float qy;
    float qz;
    float qw;
    float wx;       // body-frame angular velocity, rad/s
    float wy;
    float wz;
    float ax;       // body-frame linear acceleration (gravity removed), m/s^2
    float ay;
    float az;
} IMUData;

/**
 * @brief Robot state machine states.
 *
 * IDLE     — booted, motors held at zero PWM, accepting commands. Default after reset.
 * RUNNING  — actively driving motors from command setpoints.
 * STOP     — explicit stop requested or watchdog tripped; motors at zero, brake direction.
 *            Recoverable by sending a fresh command with non-zero setpoint or by sending
 *            a "go" command (any non-zero wheel speed).
 * ESTOP    — emergency stop; motors at zero and braked. Requires reset to clear.
 */
typedef enum {
    ROBOT_STATE_IDLE  = 0,
    ROBOT_STATE_RUNNING = 1,
    ROBOT_STATE_STOP  = 2,
    ROBOT_STATE_ESTOP = 3
} RobotState;

typedef struct {
    double err_x;
    double err_y;
    double err_phi;
    double u_errs[4];
} Errors;

typedef struct {
    uint32_t current;
    uint32_t previous;
    uint32_t delta;
    uint32_t print_prev;
} TimeState;

typedef struct {
	uint16_t cnt_vals[4];
	uint16_t prevcnt_vals[4];
	float angleVals[4];
	double omegaVals[4];
} EncoderData;

typedef struct {
	double x_pos;
	double phi;
	double y_pos;
	double q_dot[3]; // <- Inertial x_dot, y_dot, z_dot
} OdomData;

typedef struct {
  double x_dot;   // desired x velocity
  double y_dot;   // desired y velocity
  double phi_dot;   // desired rotational velocity
  double PWM_vals[4]; // Motor PIDs outputs
  uint16_t dutyCycles[4]; // Motor control limited duty cycles
  uint8_t M_dirs[4]; // Motor directions,  0 = forward, 1 = back <- arbitrary
  double u[4]; // ik computed required wheel speeds (wheel velocity control input)
} CtrlOutData;


typedef struct {
    IMUData imu;
    EncoderData encoders;
    Errors error;
    TimeState time;
    OdomData odom;
    CtrlOutData ctrl;
    uint8_t robot_state;   // RobotState value (IDLE/RUNNING/STOP/ESTOP)
} CtrlTsk_Data;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD

/* MCP2515 CS — kept for the original PEDRO MCP2515 build path, but the merged
 * PEDRO_OMNIBASE target does not initialise SPI1/MCP2515, so this pin is reused
 * by the BNO085 INT line below. The define is preserved here for source-level
 * compatibility with the original PEDRO firmware files. */
#define MCP2515_CS_Pin GPIO_PIN_14
#define MCP2515_CS_GPIO_Port GPIOD

/* BNO085 IMU wiring — pins re-used from OMNIBASE_CAN_BNO085. PB6/PB7 is I2C1
 * already configured by CubeMX. PD14/PD15/PA4 are added in MX_GPIO_Init via
 * USER CODE blocks (these are NOT in the CubeMX-generated table because the
 * BNO085 was retro-fitted on the PEDRO_OMNIBASE merge — see status.md TODOs). */
#define BNO085_RST_Pin       GPIO_PIN_15
#define BNO085_RST_GPIO_Port GPIOD
#define BNO085_INT_Pin       GPIO_PIN_14
#define BNO085_INT_GPIO_Port GPIOD
#define BNO085_WAKE_Pin      GPIO_PIN_4
#define BNO085_WAKE_GPIO_Port GPIOA

/* Debug-only routing: when DEBUG_LEDS is non-zero, PWM duty cycles and
 * direction bits are mirrored onto Nucleo on-board LEDs (LD1=PB0, LD2=PE1,
 * LD3=PB14) instead of the H-bridge pins. Useful for desk-testing without
 * connected motors / power supply. Define =0 in production builds. */
#ifndef DEBUG_LEDS
#define DEBUG_LEDS 0
#endif

#define LD1_Pin       GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define LD2_Pin       GPIO_PIN_1
#define LD2_GPIO_Port GPIOE
#define LD3_Pin       GPIO_PIN_14
#define LD3_GPIO_Port GPIOB  /* NOTE: shared with M1 PWM (TIM12_CH1) — only active when DEBUG_LEDS=1 */

/* Robot-level watchdogs / kinematic constants */
#define CMD_WATCHDOG_TIMEOUT_MS 500u   /* if no command in this window → STOP */
#define ESTOP_CMD_KEY  (-9999.0)        /* sentinel value the host sends in x_desired to trigger ESTOP */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
