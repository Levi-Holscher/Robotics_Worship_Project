/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "quad_encoder.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NO_FAULTS   0x00
#define DURATION    0x01
#define OVER_VOLT   0x02
#define UNDER_VOLT  0x04
#define OVER_TEMP   0x08
#define START_UP    0x10
#define SPEED_FDBK  0x20
#define BREAK_IN    0x40
#define SW_ERROR    0x80

#define BUFFER_SIZE 300  // Storing 3 seconds of data at 100 Hz
#define SAMPLE_INTERVAL 1  // Take a sample every 1 index (every 10ms at 100 Hz)
#define NUM_SAMPLES 100  // 100 samples spread over 3 seconds
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
encoder_instance theta1_encoder;
encoder_instance theta2_encoder;
encoder_instance x_encoder;

float theta1; 		//Theta1 Position (rad), 0 = up. CW is Positive
float theta1_dot;				//Theta1 Velocity (rad/s), CW is Positive
float old_theta1 = 0;
uint16_t theta1_timer_counter;	//Clock for encoder

float theta2; 				//Theta2 Position (rad), 0 = in line with theta1. CW is Positive
float theta2_dot;				//Theta2 Velocity (rad/s), CW is Positive
float old_theta2 = 0;
uint16_t theta2_timer_counter;	//Clock for encoder

float x = 0;					//Cart Position (m), 0 = center track. Right is Positive
float x_dot;					//Cart Velocity (m/s), Right is Positive
float old_x = 0;
uint16_t x_timer_counter;		//Clock for encoder

float u;						//Control Input 	(Newtons)
int torque_limit = 40;			//Max Torque (Newtons)
float friction_offsets = 4;
float temp_u;
float natural_u;
float catch_window1 = .5;
float catch_window2 = .5;
float soft_limit_length = .8;	//Max distance from center
float speed_limit = 100;


float x_offset = 0;
float theta1_offset = 0; 		//Offsets to make encoders read zero
float theta2_offset = 0;		//Offsets to make encoders read zero
float x_buffer;
float theta1_buffer;
float theta2_buffer;
float theta1_drift = 0;
float theta2_drift = 0;

// Matrix K (1theta2_dot)
float K[6] = {-100.f, 1549.f, -2720.f, -173.6f, -26.36f, -465.f};


//Motor Variables
double current_constant = 0.265882353; //Amps/N
double current_requested = 0;
double radius = .02239;

double x_homing_offset;			//Used for homing sequence with limit switches
float elapsed_time = 0;			//Time in seconds
float swingup_time = 0;
float swingup_time_datum;
float tracking_time = 0;		//Time the system has been actively tracking in seconsd
int control_mode = 0;			//Set system control. 0 is initialization
uint16_t fault_code;
const char* fault_name;

bool actively_tracking = false;		//Keeps track of if the motor is actively tracking
bool actively_swinging_up = false;	//Keeps track of if the motor is actively swinging up
bool motor_enabled = false; 		//Motor safety switch. true means enabled
bool limit_switch_open;				//Disable Motor if limit switch is hit
bool motor_temp_high;				//Disable Motor if it gets too hot
bool soft_limits_breached;			//Disable control if outside of bounds
bool speed_limit_breached;			//Disable control if going too fast
bool catch_window_active;			//Enable control if in window

bool limit_switch_open_flag;		//Disable Motor if limit switch is hit
bool motor_temp_high_flag;			//Disable Motor if temp got too high
bool soft_limits_breached_flag;		//Disable control if outside of bounds
bool speed_limit_breached_flag;		//Disable control if going too fast
bool catch_window_active_flag;		//Enable control if in window

bool test_motor_init;
bool red_light_on;				//Indicates Red Light On
bool yellow_light_on;			//Indicates Yellow Light On
bool green_light_on;			//Indicates Green Light On
bool white_light_on;			//Indicates White Light On

int center_toggle;				//Stores Position of Center Toggle Switch
int edge_toggle;				//Stores Position of Outer Toggle Switch
int state_machine;

float motor_temp;

float velocity_swing_up_profile[][2] = {
	{ 0.001, 0 },
    { 0.141414, 3.143556 },
    { 0.161616, 3.184708 },
    { 0.363636, 0.521649 },
    { 0.666667, 0.043843 },
    { 0.747475, -0.201979 },
    { 1.010101, -4.477034 },
    { 1.393939, 2.359946 },
    { 1.838384, -0.432700 },
    { 1.979798, 0.000000 }
};

float force_swing_up_profile[][2] = {
    { 0.020202, 23.999787 },
    { 0.161616, 7.042774 },
    { 0.181818, -12.394399 },
    { 0.383838, -1.030226 },
    { 0.686869, -5.944503 },
    { 0.767677, -23.989512 },
    { 1.030303, 21.953005 },
    { 1.414141, -6.206167 },
    { 1.858586, 2.846756 }
};


float programed_speed;
float programed_ramp;
float programed_torque;
int counter_i;

MCI_State_t motor_state;

// Circular buffer structure
typedef struct {
    float data[BUFFER_SIZE];
    int index;
    int count;
} CircularBuffer;

// Global buffer instances for drift data
CircularBuffer drift_buffer1;
CircularBuffer drift_buffer2;
volatile uint32_t sample_counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

//void update_buffer(CircularBuffer *buffer, float new_value);
//float compute_average(const CircularBuffer *buffer);
float get_rpm(float requested_speed);
bool check_limit_switch_open();
void drive_motor();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

//High Speed Tasks - System Monitoring and Open Loop Swing Up Response
	if (htim == &htim14) {

//System Monitoring
		x_timer_counter = __HAL_TIM_GET_COUNTER(&htim2);
		update_encoder(&x_encoder, &htim2);
		x = x_encoder.position * radius * 3.141 / (4*2048) + x_offset; //Ticks to m;
		x_buffer = x_encoder.position * radius * 3.141 / (4*2048);

		theta1_timer_counter = __HAL_TIM_GET_COUNTER(&htim3);
		update_encoder(&theta1_encoder, &htim3);
		theta1 = -1* theta1_encoder.position * 6.28318/(4*2048) + 3.14159 + theta1_offset - theta1_drift; //Ticks to radians
		theta1_buffer = -1* theta1_encoder.position * 6.28318/(4*2048) + 3.14159;

		theta2_timer_counter = __HAL_TIM_GET_COUNTER(&htim4);
		update_encoder(&theta2_encoder, &htim4);
		theta2 = theta1 + theta2_encoder.position * 6.28318/(4*2048) + theta2_offset - theta2_drift; //Ticks to Radians
		theta2_buffer = theta1 + theta2_encoder.position * 6.28318/(4*2048);

	    x_dot = (x - old_x) / .001;
	    old_x = x;

		elapsed_time = elapsed_time + .001;

		if(state_machine == 5){
			drive_motor();
		}
	}


//Low speed interrupts - 100 Hz
	if (htim == &htim13) {

////Angular Drift Compensation
//		if(tracking_time > 3){
//			update_buffer(&drift_buffer1, theta1);
//			update_buffer(&drift_buffer2, theta2);
//			sample_counter++;
//
//
//// Re-Compute average every 1 second (100 samples at 100Hz)
//			if (sample_counter >= 100) {
//				theta1_drift = compute_average(&drift_buffer1);
//				theta2_drift = compute_average(&drift_buffer2);
//
//				sample_counter = 0;  // Reset counter
//			}
//		}
	}

//Very Low speed interrupts - 50 Hz
	if (htim == &htim11) {
			static float theta1_dot_old;
			static float theta1_dot_new;
			static float theta2_dot_old;
			static float theta2_dot_new;

		//System Monitoring for Angular Speeds
			theta1_dot_new = (theta1 - old_theta1) / .02;
			theta2_dot_new = (theta2 - old_theta2) / .02;
			theta1_dot = theta1_dot_old * .4 + theta1_dot_new *.6;
			theta2_dot = theta2_dot_old * .4 + theta2_dot_new *.6;
			old_theta1 = theta1;
			old_theta2 = theta2;
			theta1_dot_old = theta1_dot;
			theta2_dot_old = theta2_dot;
	}

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

//void init_buffer(CircularBuffer *buffer) {
//    buffer->index = 0;
//    buffer->count = 0;
//}

// Function to store new data in the circular buffer
//void update_buffer(CircularBuffer *buffer, float new_value) {
//    buffer->data[buffer->index] = new_value;
//    buffer->index = (buffer->index + 1) % BUFFER_SIZE;
//    if (buffer->count < BUFFER_SIZE) {
//        buffer->count++;
//    }
//}
//
//// Function to compute the average using 100 samples spaced 10ms apart over 3 seconds
//float compute_average(const CircularBuffer *buffer) {
//    float sum = 0.0;
//    int start = (buffer->index - NUM_SAMPLES + BUFFER_SIZE) % BUFFER_SIZE;
//
//    for (int i = 0; i < NUM_SAMPLES; i++) {
//        sum += buffer->data[(start + i) % BUFFER_SIZE];
//    }
//
//    return sum / NUM_SAMPLES;
//}

float get_u_value(float x1, float x2, float x3, float x4, float x5, float x6){
	temp_u = x1*K[0] + x2*K[1] + x3*K[2] + x4*K[3] + x5*K[4] + x6*K[5]; 	//Computes u in Newtons
	natural_u = temp_u;
	if(x_dot > 0){
		temp_u = temp_u + friction_offsets; 	//add a bit to offset friction forces
	}

	if(x_dot < 0){
		temp_u = temp_u - friction_offsets;		//add a bit to offset friction forces
	}

  //Torque Limit to keep thing safe
	if(temp_u < -1*torque_limit){
		  temp_u = -1*torque_limit;
		  u = temp_u;
	  }
	else if(temp_u > torque_limit){
		  temp_u = torque_limit;
		  u = temp_u;
	  }
	else{
		u = temp_u;
	}
return u;
}

void speed_swing_up(){
	// Swing-up parameters
	static int current_index = 0;
	static int current_ramp_called = 0;
	static float last_ramp_start_time = 0; // Track when the last ramp started

	// Reset everything if swing-up just started
	if (swingup_time == 0) {
		current_index = 0;
		current_ramp_called = 0;
		last_ramp_start_time = 0;
		MC_StartMotor1();
	}

	if(swingup_time == 0){
		swingup_time_datum = elapsed_time;
	}
	actively_swinging_up = true;
	swingup_time = .00001 + elapsed_time - swingup_time_datum;

	if (current_index < (sizeof(velocity_swing_up_profile) / sizeof(velocity_swing_up_profile[0])) - 1) {

		float current_time = velocity_swing_up_profile[current_index][0];
		float next_time = velocity_swing_up_profile[current_index + 1][0];
		float next_speed = velocity_swing_up_profile[current_index + 1][1];

		// If we're in the correct time range
		if (swingup_time >= current_time && swingup_time < next_time) {
			if (current_ramp_called == 0) {
				float ramp_time = next_time - current_time;
				programed_speed = next_speed;
				programed_ramp = ramp_time;

				// Call speed ramp function
				MC_ProgramSpeedRampMotor1_F(get_rpm(programed_speed), programed_ramp);

				current_ramp_called = 1;  // Mark ramp as called
				last_ramp_start_time = swingup_time; // Track ramp start
			}
		}

		// Check if the ramp duration has passed, then move to the next index
		if (current_ramp_called == 1 && (swingup_time - last_ramp_start_time) >= (next_time - current_time)) {
			current_index++;
			current_ramp_called = 0; // Allow the next ramp to be called
		}
	}
}

void force_swing_up() {
	// Swing-up parameters
	static int current_index = 0;
	static int current_ramp_called = 0;
	static float last_ramp_start_time = 0; // Track when the last ramp started

	// Reset everything if swing-up just started
	if (swingup_time == 0) {
		current_index = 0;
		current_ramp_called = 0;
		last_ramp_start_time = 0;
		MC_StartMotor1();
	}

	if (swingup_time == 0) {
		swingup_time_datum = elapsed_time;
	}
	actively_swinging_up = 1;
	swingup_time = 0.00001f + elapsed_time - swingup_time_datum;

	if (current_index < (sizeof(force_swing_up_profile) / sizeof(force_swing_up_profile[0]))) {
		float current_time = force_swing_up_profile[current_index][0];
		float current_torque = force_swing_up_profile[current_index][1];

		// Adjust timing to trigger 0.02 seconds earlier
		float trigger_time = current_time - 0.02f;

		// Prevent first torque from triggering at t=0, should trigger at t=1.98
		if (current_index == 0 && swingup_time < trigger_time) {
			return; // Don't execute until we reach 1.98s
		}

		// If we're in the adjusted time range
		if (swingup_time >= trigger_time) {
			if (current_ramp_called == 0) {
				// Apply friction compensation based on direction
				if (x_dot > 0) {
					current_torque += friction_offsets;
				} else if (x_dot < 0) {
					current_torque -= friction_offsets;
				}

				programed_torque = current_torque;
				programed_ramp = 0.02f;  // Fixed ramp time

				// Call torque ramp function
				MC_ProgramTorqueRampMotor1_F(programed_torque, programed_ramp);

				current_ramp_called = 1;  // Mark ramp as called
				last_ramp_start_time = swingup_time; // Track ramp start
			}
		}

		// Ensure the ramp finishes before incrementing index
		if (current_ramp_called == 1 && (swingup_time - last_ramp_start_time) >= 0.02f) {
			current_index++;
			current_ramp_called = 0; // Allow the next ramp to be called
		}
	}
}

void check_toggle_positions(){
//Read Center Toggle Position
	if(HAL_GPIO_ReadPin(center_toggle_in_GPIO_Port, center_toggle_in_Pin) == GPIO_PIN_SET){
		center_toggle = 1;
	}
	else if(HAL_GPIO_ReadPin(center_toggle_out_GPIO_Port, center_toggle_out_Pin) == GPIO_PIN_SET){
		center_toggle = 3;
	}
	else{
		center_toggle = 2;
	}
//Read Outer Toggle Position
	if(HAL_GPIO_ReadPin(edge_toggle_in_GPIO_Port, edge_toggle_in_Pin) == GPIO_PIN_SET){
		edge_toggle = 3;
	}
	else if(HAL_GPIO_ReadPin(edge_toggle_out_GPIO_Port, edge_toggle_out_Pin) == GPIO_PIN_SET){
		edge_toggle = 1;
	}
	else{
		edge_toggle = 2;
	}

	if(center_toggle == 2){
		white_light_on = true;
	}
	else {
	    white_light_on = false;
	}
}

bool check_motor_temp(){
	  motor_temp = NTC_GetAvTemp_C(&TempSensor_M1);
	  if(motor_temp > 70){
		  motor_temp_high = true;
		  motor_temp_high_flag = true;
	  }
	  else{
		  motor_temp_high = false;
	  }
}

bool check_limit_switch_open(){
//	  if(HAL_GPIO_ReadPin(limit_switch_GPIO_Port, limit_switch_Pin) == GPIO_PIN_RESET){
//		  limit_switch_open = true;
//		  motor_enabled = false;
//		  limit_switch_open_flag = true;
//		  return true;
//	  }
//	  else {
//		  limit_switch_open = false;
//		  return false;
//	  }
}

bool check_soft_limits_breached(){
	  if(x < -1 * soft_limit_length || x > soft_limit_length){
		  soft_limits_breached = true;
		  soft_limits_breached_flag = true;
		  motor_enabled = false;
		  return true;
	  }
	  else {
		  soft_limits_breached = false;
		  return false;
	  }
}

bool check_speed_limit_breached(){
	  if(x_dot < -1* speed_limit || x_dot > speed_limit){
		  speed_limit_breached = true;
		  speed_limit_breached_flag = true;
		  motor_enabled = false;
		  return true;
	  }
	  else{
		  speed_limit_breached = false;
		  return false;
	  }
}

bool check_catch_window_active(){
	  if(theta1 < -1 * catch_window1 || theta1 > catch_window1 || theta2 < -1 * catch_window2 || theta2 > catch_window2 ){
		  catch_window_active = false;
		  motor_enabled = false;
		  yellow_light_on = false;
		  return false;
	  }
	  else{
		  catch_window_active = true;
		  yellow_light_on = true;
		  return true;
	  }
}

const char* GetFaultName(uint8_t fault_code) {
    switch (fault_code) {
        case NO_FAULTS:   return "NO_FAULTS";
        case DURATION:    return "DURATION";
        case OVER_VOLT:   return "OVER_VOLT";
        case UNDER_VOLT:  return "UNDER_VOLT";
        case OVER_TEMP:   return "OVER_TEMP";
        case START_UP:    return "START_UP";
        case SPEED_FDBK:  return "SPEED_FDBK";
        case BREAK_IN:    return "BREAK_IN";
        case SW_ERROR:    return "SW_ERROR";
        default:          return "UNKNOWN_FAULT";
    }
}

bool check_motor_enabled(){
	motor_state = MC_GetSTMStateMotor1();
	if(motor_state == FAULT_OVER){
		fault_code = MC_GetOccurredFaultsMotor1();
		fault_name = GetFaultName(fault_code);
	}

	if(limit_switch_open == false && soft_limits_breached == false && speed_limit_breached == false && catch_window_active == true && center_toggle == 2){
		motor_enabled = true;
		red_light_on = false;
		return true;
	}
	else{
		motor_enabled = false;
		return false;
	}
}

bool check_actively_tracking(){
	if(motor_enabled == true && catch_window_active == true && actively_swinging_up == false){
		return true;
	}
	else{
		return false;
	}
}

void set_indicator_lights(){
//Set Red Light
	if(red_light_on == true){
		HAL_GPIO_WritePin(red_light_GPIO_Port, red_light_Pin, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(red_light_GPIO_Port, red_light_Pin, GPIO_PIN_RESET);
	}
//Set Yellow Light
	if(yellow_light_on == true){
		HAL_GPIO_WritePin(yellow_light_GPIO_Port, yellow_light_Pin, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(yellow_light_GPIO_Port, yellow_light_Pin, GPIO_PIN_RESET);
	}
//Set Green Light
	if(green_light_on == true){
		HAL_GPIO_WritePin(green_light_GPIO_Port, green_light_Pin, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(green_light_GPIO_Port, green_light_Pin, GPIO_PIN_RESET);
	}
//Set White Light
	if(white_light_on == true){
		HAL_GPIO_WritePin(white_light_GPIO_Port, white_light_Pin, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(white_light_GPIO_Port, white_light_Pin, GPIO_PIN_RESET);
	}
}

void drive_motor(){
	  motor_state = MC_GetSTMStateMotor1();
//motor_temp = NTC_GetAvTemp_C(REAL_SENSOR);
	  if(motor_enabled == true){
		  //turn off red light for motor enabled
		  red_light_on = false;

		  //turn on green light for motor enabled
		  green_light_on = true;

		  if(MC_GetSTMStateMotor1() == IDLE){
			  MC_StartMotor1();
		  }

		  current_requested = u * current_constant; //Requested Current in A
		  MC_ProgramTorqueRampMotor1_F(current_requested, 0);
	  }
	  else{
		  MC_StopMotor1();
		  current_requested = 0;
		  MC_ProgramTorqueRampMotor1_F(0, 0);
	  }
}

//moves requested speed into an rpm for speed control signals
float get_rpm(float requested_speed) {
    float set_rpm = (requested_speed * 60) / (2 * 3.1416 * radius);
    return set_rpm;
}

int read_state_machine(){
	//Zero Encoders
	if(state_machine == 2){
		//Stay in state if needed
		if(elapsed_time >3){
			state_machine = 3;
			return state_machine;
		}
		//Return to Hub if anything else
		else{
			state_machine = 2;
		}
		return state_machine;
	}


	//Moving from hub state (no action)
	if(state_machine == 3){
		//Move to Swing Up Control
		if(center_toggle == 1 && edge_toggle == 2){
			state_machine = 4;
		}
		//Move to Actively Tracking Control State
		else if(center_toggle == 2 && edge_toggle == 2 && check_catch_window_active()){
			state_machine = 5;
		}
		//Move to Reset Faults Control State
		else if (edge_toggle == 1){
			state_machine = 6;
		}
		//Move to Zero Encoders State
		else if (edge_toggle == 3) {
			state_machine = 7;
		}
		//Stay in Hub State
		else{
			state_machine = 3;
		}
		return state_machine;
	}

	// In Swing Up State
	if(state_machine == 4){
		//Return if flags are set
		if(soft_limits_breached_flag || speed_limit_breached_flag || limit_switch_open_flag || motor_temp_high_flag){
			state_machine = 3;
		}
		if(soft_limits_breached == true){
			state_machine = 3;
		}

		//Return to Hub for moving control knobs
		if(center_toggle == 2 || center_toggle == 3 || edge_toggle != 2){
			state_machine = 3;
		}
		//Swing Up Successful, transition to active tracking
//			else if(actively_swinging_up == false && catch_window_active){
//				state_machine = 5;
//			}
		//Swing Up Failed, return to hub
//			else if (actively_swinging_up == false && catch_window_active == false) {
//				state_machine = 3;
//			}

		//Stay in Swing Up State
		else{
			state_machine = 4;
		}
		return state_machine;
	}

	//Actively Tracking and praising the Lord
	if(state_machine == 5){
		//Toggles moved back to hub
		if(center_toggle == 1 || edge_toggle != 2){
			state_machine = 3;
		}
		//Arms fall down from active tracking
		if(catch_window_active == false){
			state_machine = 3;
		}
		//Tracking well - praise the Lord and keep going
		else{
			state_machine = 5;
		}
		return state_machine;
	}

	//Zero Encoders
	if(state_machine == 6){
		//Stay in state if needed
		if(edge_toggle == 1){
			state_machine = 6;
		}
		//Return to Hub if anything else
		else{
			state_machine = 3;
		}
		return state_machine;
	}

	//Reset faults and flags
	if(state_machine == 7){
		//Stay in state if needed
		if(edge_toggle == 3){
			state_machine = 7;
		}
		//Return to Hub if anything else
		else{
			state_machine = 3;
		}
		return state_machine;
	}

	else{
		state_machine = 3;
		return state_machine;
	}
	return state_machine;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_MotorControl_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM11_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

//Start Interrupt Timers
  HAL_TIM_Base_Start_IT(&htim11); 	//20 Hz
  HAL_TIM_Base_Start_IT(&htim13); 	//100 Hz
  HAL_TIM_Base_Start_IT(&htim14);	//1000 HZ

  NTC_Init(&TempSensor_M1);
//  CircularBuffer theta1_buffer;
//  init_buffer(&theta1_buffer);

  reset_encoder(&theta1_encoder);
  reset_encoder(&theta2_encoder);
  reset_encoder(&x_encoder);

  MC_ProgramTorqueRampMotor1_F(0.00, 1);
  MC_StartMotor1();
  state_machine = 2;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  check_toggle_positions();
	  check_motor_temp();
	  check_soft_limits_breached();
	  check_speed_limit_breached();
	  check_catch_window_active();
	  check_motor_enabled();
	  set_indicator_lights();
	  read_state_machine();
	  get_u_value(x, theta1, theta2, x_dot, theta1_dot, theta2_dot);

	  if(state_machine == 2){
		  motor_enabled = true;
	  	  }
	  if(state_machine == 3){
		  MC_StopMotor1();
		  motor_enabled = false;
		  swingup_time = 0;
		  tracking_time = 0;
		  actively_swinging_up = false;
		  actively_tracking = false;
	  }
	  if(state_machine == 4){//Swing Up
		  speed_swing_up();
	  }
	  if(state_machine == 5){//Tracking
		  actively_swinging_up = false;
		  swingup_time = 0;
		  actively_tracking = true;
		  get_u_value(x, theta1, theta2, x_dot, theta1_dot, theta2_dot);
//		  drive_motor();
	  }
	  if(state_machine == 6){//Reset Encoders
		  reset_encoder(&theta1_encoder);
		  reset_encoder(&theta2_encoder);
		  reset_encoder(&x_encoder);
		  x_offset = 0 - x_buffer;
		  theta1_offset = 3.14159 - (theta1_buffer);
		  theta2_offset = 3.14159 - (theta2_buffer);
	  }
	  if(state_machine == 7){//Clear Faults
		  MC_AcknowledgeFaultMotor1();
		  limit_switch_open_flag = false;			//Disable Motor if limit switch is hit
		  motor_temp_high_flag = false;				//Disable Motor if too hot
		  soft_limits_breached_flag = false;		//Disable control if outside of bounds
		  speed_limit_breached_flag = false;		//Disable control if going too fast
		  catch_window_active_flag = false;
	  }


	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 3, 1);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* TIM1_UP_TIM10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  /* TIM1_BRK_TIM9_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 4, 1);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_11;
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_10;
  sConfigInjected.InjectedRank = 3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 2;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = ((TIM_CLOCK_DIVIDER) - 1);
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = ((PWM_PERIOD_CYCLES) / 2);
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim1.Init.RepetitionCounter = (REP_COUNTER);
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = ((PWM_PERIOD_CYCLES) / 4);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = (((PWM_PERIOD_CYCLES) / 2) - (HTMIN));
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = ((DEAD_TIME_COUNTS) / 2);
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = M1_PULSE_NBR;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = M1_ENC_IC_FILTER;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = M1_ENC_IC_FILTER;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //Start x encoder
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); //Start theta1 encoder
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); //Start theta2 encoder
  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 180-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 20000-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 90-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 10000-1;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 90-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, red_light_Pin|white_light_Pin|green_light_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(yellow_light_GPIO_Port, yellow_light_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : red_light_Pin white_light_Pin green_light_Pin */
  GPIO_InitStruct.Pin = red_light_Pin|white_light_Pin|green_light_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : edge_toggle_in_Pin edge_toggle_out_Pin limit_switch_Pin */
  GPIO_InitStruct.Pin = edge_toggle_in_Pin|edge_toggle_out_Pin|limit_switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : center_toggle_out_Pin center_toggle_in_Pin */
  GPIO_InitStruct.Pin = center_toggle_out_Pin|center_toggle_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : yellow_light_Pin */
  GPIO_InitStruct.Pin = yellow_light_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(yellow_light_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
