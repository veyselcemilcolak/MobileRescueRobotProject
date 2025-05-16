#include <atmel_start.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define min(x,y) ((x) > (y) ? (y) : (x))
#define abs_(x) ((x) > (0) ? (x) : (-(x)))
//#define LED_PIN GPIO(GPIO_PORTA, 0)	// TODO: the pin should be renamed in Atmel Start to LED_PIN, and this line deleted

#define UART_CMD_SERVO "SERVO"
#define UART_CMD_LED "LED"
#define UART_CMD_LOCALIZE "LOCALIZE"
#define UART_CMD_LOCALIZE_CONTINUOUS "LOCALIZE2"
#define UART_CMD_LOCALIZE_FULL "LOCALIZE3"
#define UART_CMD_SENSORS "SENSORS"

#define SERVO_ID_GRIPPER 0
#define SERVO_ID_ARM 1
#define SERVO_ID_LED 2
#define SERVO_ID_LOCALIZATION 3

char uart_send_buffer[200];

// servo variables
int angle_360_ref = 100;
float current_360_angle = 0.0;

float vel_ref_direction = 1.0;

volatile bool position_controlled_360 = true;

bool localize_with_ir = false;
bool localize_with_ir_continuous = false;
bool localize_with_ir_full = false;

bool send_empty_angle_msg_time = false;

bool read_gripper_sensors_flag = false;

// servo definitions
typedef struct{
	short* counter_reg;
	const int counter_max;
	const float pwm_min;
	const float pwm_max;
	const float angle_min;
	const float angle_max;
	bool position_controlled;
} Servo;


// Declare global servo instances
Servo ledServo =		{.counter_max = 3750, .pwm_min = 3.0, .pwm_max = 13.0, .angle_min = 0.0, .angle_max = 180.0, .counter_reg = &(TC0->COUNT16.CC[1].reg), .position_controlled=true};
Servo baseServo =		{.counter_max = 3750, .pwm_min = 3.0, .pwm_max = 13.0, .angle_min = 0.0, .angle_max = 180.0, .counter_reg = &(TC1->COUNT16.CC[1].reg), .position_controlled=true};
Servo gripperServo =	{.counter_max = 3750, .pwm_min = 3.0, .pwm_max = 13.0, .angle_min = 0.0, .angle_max = 180.0, .counter_reg = &(TC5->COUNT16.CC[1].reg), .position_controlled=true};
Servo locationServo =	{.counter_max = 3750, .pwm_min = 3.0, .pwm_max = 13.0, .angle_min = 0.0, .angle_max = 180.0, .counter_reg = &(TC6->COUNT16.CC[1].reg), .position_controlled=false};
Servo locationServoSpeed =	{.counter_max = 3750, .pwm_min = 3.0, .pwm_max = 13.0, .angle_min = 0.0, .angle_max = 180.0, .counter_reg = &(TC6->COUNT16.CC[1].reg), .position_controlled=true};

// Declare global array of Servo instances
Servo* servo_list[] = {&gripperServo, &baseServo, &ledServo, &locationServo, &locationServoSpeed};



void init_uart() {
	// Initializing the clock
	MCLK->APBBMASK.bit.SERCOM2_ = 1;
	GCLK->PCHCTRL[23].bit.CHEN = 1;
	// Connecting the pins to the peripheral option
	PORT->Group[1].PMUX[12].bit.PMUXE = 3;
	PORT->Group[1].PMUX[12].bit.PMUXO = 3;
	PORT->Group[1].PINCFG[24].bit.PMUXEN = 1;
	PORT->Group[1].PINCFG[25].bit.PMUXEN = 1;
	// CTRLA Configuration
	SERCOM2->USART.CTRLA.bit.MODE = 1;
	SERCOM2->USART.CTRLA.bit.DORD = 1;
	SERCOM2->USART.CTRLA.bit.FORM = 0;
	SERCOM2->USART.CTRLA.bit.RXPO = 1;
	SERCOM2->USART.CTRLA.bit.TXPO = 0;
	// CTRLB Configuration
	SERCOM2->USART.CTRLB.bit.RXEN = 1;
	SERCOM2->USART.CTRLB.bit.TXEN = 1;
	SERCOM2->USART.CTRLB.bit.PMODE = 0;
	SERCOM2->USART.CTRLB.bit.SBMODE = 0;
	SERCOM2->USART.CTRLB.bit.CHSIZE = 0;
	// BaudRate configuration
	SERCOM2->USART.BAUD.reg = 64697; //Baudrate is 9600! (65536*(1-16*(f_baud/f_ref)))
	SERCOM2->USART.CTRLA.bit.ENABLE = 1;
}

void init_uart_ir() {
	// Initializing the clock
	MCLK->APBDMASK.bit.SERCOM7_ = 1;
	GCLK->PCHCTRL[23].bit.CHEN = 1;
	// Connecting the pins to the peripheral option
	PORT->Group[3].PMUX[12].bit.PMUXE = 3;
	PORT->Group[3].PMUX[12].bit.PMUXO = 3;
	PORT->Group[3].PINCFG[8].bit.PMUXEN = 1;
	PORT->Group[3].PINCFG[9].bit.PMUXEN = 1;
	// CTRLA Configuration
	SERCOM7->USART.CTRLA.bit.MODE = 1;
	SERCOM7->USART.CTRLA.bit.DORD = 1;
	SERCOM7->USART.CTRLA.bit.FORM = 0;
	SERCOM7->USART.CTRLA.bit.RXPO = 1;
	SERCOM7->USART.CTRLA.bit.TXPO = 0;
	// CTRLB Configuration
	SERCOM7->USART.CTRLB.bit.RXEN = 1;
	SERCOM7->USART.CTRLB.bit.TXEN = 1;
	SERCOM7->USART.CTRLB.bit.PMODE = 0;
	SERCOM7->USART.CTRLB.bit.SBMODE = 0;
	SERCOM7->USART.CTRLB.bit.CHSIZE = 0;
	// BaudRate configuration
	SERCOM7->USART.BAUD.reg = 65326;
	SERCOM7->USART.CTRLA.bit.ENABLE = 1;
}

void update_servo_angle(int servo_id, float servo_angle){
	volatile float copy_angle = servo_angle;
	Servo servo = *servo_list[servo_id];
	// todo: some sanity checks?
	
	if(servo.position_controlled){
		// transfer angle to pwm value:
		volatile float pwm_value = ((float)servo_angle - servo.angle_min)/(servo.angle_max - servo.angle_min) * (servo.pwm_max - servo.pwm_min) + servo.pwm_min;
		*(servo.counter_reg) = pwm_value/100.0 * servo.counter_max;
		
		if (servo_id == 4){
			position_controlled_360 = false;
		}
	}
	else{
		position_controlled_360 = true;
		angle_360_ref = servo_angle;
	}
}



void process_cmd_received(char* input) {
	// MESSAGE FORMAT
	// {command} {parameter1} {parameter2}\n
	// SERVO {id} {angle}
	// LED {on/off}
	// LOCALIZE
	
	// for example:
	// SERVO 2 180
	// LED 0
	
	// PARSING
	char cmd[10];
	memset(cmd, '\0', sizeof(cmd));
	int param1;
	volatile float param2;
	
	char *pt;
	pt = strtok(input, " ");
	strcpy(cmd, pt);
	
	pt = strtok(NULL, " ");
	param1 = atoi(pt);
	
	pt = strtok(NULL, " ");
	param2 = atof(pt);
	
	
	// PROCESSING
	if (strcmp(cmd, UART_CMD_SERVO) == 0) {
		int servo_id = param1;
		volatile float angle = param2;
		
		// TODO: update servo PWM
		update_servo_angle(servo_id, angle);
		//control_360_servo();
		//angle_360_ref = angle;
	}
	else if (strcmp(cmd, UART_CMD_LED) == 0) {
		bool state = param1;
		gpio_set_pin_level(LED_PIN, state);
	}
	else if (strcmp(cmd, UART_CMD_LOCALIZE) == 0) {
		// TODO: execute IR reading and reply with the received data
		// respond only if a corner has been detected
		// strcpy(uart_send_buffer, "SE\n");
		localize_with_ir = true;
	}
	else if (strcmp(cmd, UART_CMD_LOCALIZE_CONTINUOUS) == 0) {
		volatile int state = param1;
		if (state == 1){
			localize_with_ir_continuous = true;
		}
		else {
			localize_with_ir_continuous = false;
		}
			
	}
	else if (strcmp(cmd, UART_CMD_LOCALIZE_FULL) == 0) {
			volatile int state = param1;
			if (state == 1){
				localize_with_ir_full = true;
				vel_ref_direction = 1.0;
			}
			else if (state == 2){
				localize_with_ir_full = true;
				vel_ref_direction = -1.0;
			}
			else {
				// set desired angle to current angle.
				angle_360_ref = current_360_angle;
				// set all the location variables to defaults.
				localize_with_ir_full = false;
				position_controlled_360 = true;
				localize_with_ir_continuous = false;
			}
			
	}
	
	else if (strcmp(cmd, UART_CMD_SENSORS) == 0) {
			bool state = param1;
			read_gripper_sensors_flag = true;
		}
	
	
	//sprintf(uart_send_buffer, "Received: %s %d %d\n", cmd, param1, param2);
}

void uart_read() {
	static char buffer[200];
	static uint8_t i = 0;	// number of bytes read into the buffer
	
	if (!SERCOM2->USART.INTFLAG.bit.RXC) return;	// no new data to read
	
	buffer[i] = SERCOM2->USART.DATA.reg;
	i++;
	
	if (buffer[i - 1] == '\n') {	// \n read
		buffer[i - 1] = '\0';
		if (buffer[i-2] == '\r') buffer[i-2] = '\0'; // makes debugging in microchip studio possible
		process_cmd_received(buffer);
		memset(buffer, '\0', sizeof(buffer));
		i = 0;
	}
}

void uart_write(){
	static uint8_t i = 0;
	if (uart_send_buffer[0] == '\0') return;		// no message to send
	if (!SERCOM2->USART.INTFLAG.bit.DRE) return;	// transmit buffer isn't empty yet
	
	SERCOM2->USART.DATA.reg = uart_send_buffer[i];
	i++;
	
	if (i == strlen(uart_send_buffer)) {	// the whole message has been sent, clear the buffer
		i = 0;
		memset(uart_send_buffer, '\0', sizeof(uart_send_buffer));
	}
}

void send_empty_angle_msg(){
	int intpart = (int)current_360_angle;
	int decpart = (current_360_angle - intpart) * 1000;
	if (uart_send_buffer[0] == '\0') // only send it if the buffer is empty
		sprintf(uart_send_buffer, "%s %d.%d\n", "NC", intpart, decpart);	
}

void uart_ir_read() {
	static char buffer[20];
	static uint8_t i = 0;	// number of bytes read into the buffer
	
	if (!SERCOM7->USART.INTFLAG.bit.RXC){
		if (send_empty_angle_msg_time)
			send_empty_angle_msg();
		send_empty_angle_msg_time = false;
		return;	// no new data to read
	}
	// if we receive smth, we dont have to send an empty angle, right?
	send_empty_angle_msg_time = false;
	
	buffer[i] = SERCOM7->USART.DATA.reg;
	i++;
	
	// a hacky way to only process valid messages
	if (i == 5) {
		if ((strcmp(buffer, "IMNW\r") == 0)
			|| (strcmp(buffer, "IMNE\r") == 0)
			|| (strcmp(buffer, "IMSW\r") == 0)
			|| (strcmp(buffer, "IMSE\r") == 0)
		)
		{
			char dir[3] = "";
			memcpy(dir, buffer + 2, 2);	// send only the direction without IM
			
			// angle
			int intpart = (int)current_360_angle;
			int decpart = (current_360_angle - intpart) * 1000;
			
			// actually make sure uart send buffer is empty - this might delete other msg, but these are the more important ones.
			memset(uart_send_buffer, '\0', sizeof(uart_send_buffer));
			sprintf(uart_send_buffer, "%s %d.%d\n", dir, intpart, decpart);
			
			// reset buffer
			memset(buffer, '\0', sizeof(buffer));
			i = 0;
		}
		else {
			for (int j = 0; j < 4; j++) {
				buffer[j] = buffer[j + 1];
			}
			buffer[4] = '\0';
			i = 4;
		}
	}
}

void uart_ir_write(){
	char imrp[] = "IMRP\r";
	static uint8_t i = 0;	// index of the byte to send next
	
	if (!localize_with_ir) return;					// localize command not received yet
	if (!SERCOM7->USART.INTFLAG.bit.DRE) return;	// transmit buffer isn't empty yet
	
	SERCOM7->USART.DATA.reg = imrp[i];
	i++;
	
	if (i == sizeof(imrp) - 1) {	// the whole message has been sent
		i = 0;						// reset index
		localize_with_ir = false;	// stop sending
	}
}

	
/*
void uart_ir_write_continuous(){
	char imrp[] = "IMRP\r";
	static uint8_t i = 0;	// index of the byte to send next
	
	if (!localize_with_ir_continuous) return;					// localize command not received yet
	if (!SERCOM7->USART.INTFLAG.bit.DRE) return;	// transmit buffer isn't empty yet
	
	SERCOM7->USART.DATA.reg = imrp[i];
	i++;
	
	if (i == sizeof(imrp) - 1) {	// the whole message has been sent
		i = 0;						// reset index
		//localize_with_ir_continuous = false;	// stop sending
	}
}
*/

void timed_uart_ir_write_continuous(){
	if(TC2->COUNT16.INTFLAG.bit.MC0){
		TC2->COUNT16.INTFLAG.bit.MC0 = 1;
		localize_with_ir = true;
		send_empty_angle_msg_time = true;
		//uart_ir_write_continuous();
		
		
		if (localize_with_ir_full){
			velocity_control_360(locationServo, 1.0f);
		}
	}
	else if(TC2->COUNT16.INTFLAG.bit.MC1){
		TC2->COUNT16.INTFLAG.bit.MC1 = 1;
	}
}

void update_servo_pwm(int servo_id, int pwm){
	Servo servo = *servo_list[servo_id];
	
	// if we want to update by pwm value
	int counter_max = 3750;
	*(servo.counter_reg) = pwm/100.0 * servo.counter_max;
	//TC1->COUNT16.CC[1].reg = pwm/100.0 * counter_max;
	
}


float read_motor_position(){
	static long count = 0;
	volatile static uint32_t pulse_width = 0;
	volatile static uint32_t pulse_period = 0;
	volatile static float pulse_width_us = 0.0;
	volatile static float pulse_period_us = 0.0;
	volatile static float angle = 0.0;
	
		// High Time Measurement
		if(TCC0->INTFLAG.bit.MC0){
			TCC0->INTFLAG.bit.MC0 = 1;
			pulse_period = TCC0->CC[0].bit.CC;
			//sprintf(uart_send_buffer, "pulse period %d\n",pulse_period);
		}
		else if(TCC0->INTFLAG.bit.MC1){
			TCC0->INTFLAG.bit.MC1 =1;
			pulse_width = TCC0->CC[1].bit.CC;
			// calculate the pulse width in ms
			pulse_width_us = pulse_width / 12.0f;
			// calculate the distance in cm
			//sprintf(uart_send_buffer, "%d, %d\n", pulse_width, (int)pulse_width_us);
		}
		else if(TCC0->INTFLAG.bit.OVF){
			TCC0->INTFLAG.bit.OVF = 1;
			pulse_width_us = 0;
		}
		else if (TCC0->INTFLAG.bit.ERR)
		{
			TCC0->INTFLAG.bit.ERR =1;
			pulse_width = TCC0->CC[0].bit.CC;
			pulse_width = TCC0->CC[1].bit.CC;
		}
		
		
		// calculate angle
		// currently not reading in the pulse period, its hardcoded.
		pulse_period_us = 1090.0; // us
		angle = pulse_width_us/pulse_period_us * 360.0;
		

		count++;
		
		// scale between 10 and 350
		//angle = ((angle - 10) / 340) * 360;
		//10.77
		
		
		//3555.4
		//11.73
		// current version
		//angle = ((angle - 10.77) /(355.33-10.77)) * 360.0;
		angle =	  ((angle - 10.73) /(355.4-10.73)) * 360.0;
		
		// small sanity check
		//if (angle > 360.0){
		//	angle = 360.0;
		//}
		//else if (angle < 0.0){
		//	angle = 0.0;
		//}
		if (count%10000 == 0){
			//sprintf(uart_send_buffer, "%d\n", (int)pulse_width_us);
			//sprintf(uart_send_buffer, "angle %d\n",(int)angle);
		}
		
		return angle;
}


void control_360_servo(Servo servo, int angle_ref){
	// get current motor angle
	current_360_angle = read_motor_position();
	
	// calculate the error
	volatile float delta = (float)angle_ref - current_360_angle;
	volatile static float delta_old = 0.0;
	
	// all the necessary controller definitions
	volatile static float D_term = 0.0;
	volatile static float I_term = 0.0;
	volatile float control = 0.0;
	//angle_ref = 100;

	
	volatile float K = 0.0008f; // 0.0008f;
	volatile float Ki = 0.00000002f;//0.00000001f;
	volatile float Kd = 0.02f;

	
	if (delta > 180.0)
		delta -= 360.0;
	else if (delta < -180.0)
		delta += 360.0;
		
	delta = delta/2.0;
	
	// add to I part
	I_term += delta;
	
	// calculate D part
	D_term = delta - delta_old;
	
	volatile float control_input = K * delta + Kd * D_term;// + Ki * I_term; 
	
	// pwm freq. where the motor stands still
	volatile float baseline_pwm = 1.5/20.0;
	//baseline_pwm = 0.0;
	
	volatile int counter_max = 3750;
	// this results in a motor speed of 0
	//TC0->COUNT16.CC[1].reg = (baseline_pwm) * counter_max;
	
	control = baseline_pwm + control_input;
	if (control < 0.05){
		control = 0.05;
	}
	else if (control > 0.2){
		control = 0.2 ;
	}
	
	//TC6->COUNT16.CC[1].reg = control * counter_max;
	*(servo.counter_reg) = control * counter_max;
	
	// set old angle for d part
	delta_old = delta;
}

void velocity_control_360_old(Servo servo, float vel_ref){
	vel_ref = 0.1;
	volatile static count = 0;
	count++;
	//if (count%1 != 0){
	//	return;
	//}
	volatile float position = read_motor_position();
	volatile static float old_position = 0.0;//current_360_angle;
	
	volatile static float I_term = 0.0;
	
	if (position < (old_position -0.1)){
		position = position + 360.0;
	}
	volatile float current_vel = position-old_position;//min(abs_(position - old_position), abs_(position +360.0 - old_position));
	
	volatile float delta = vel_ref - current_vel;//min(vel_ref - current_vel, vel_ref+360.0 - current_vel);
	
	volatile float K = 0.008;
	volatile float Ki =0.00001;
	
	volatile float baseline_pwm = 0.0761;
	volatile int counter_max = 3750;
	
	volatile float control_input = K * delta;// + Ki * I_term;
	
	volatile float control = baseline_pwm + control_input; //0.0766
	
	// make it one way only
	if (control < baseline_pwm -0.05){
		control = baseline_pwm -0.05;
	}
	
	if (control > baseline_pwm +0.05){
		control = baseline_pwm+0.05;
	}
	

	// write into register
	*(servo.counter_reg) = control * counter_max;
	
	
	old_position = position - (int)(position/360)*360;
	
	if (delta < 0.0){
		volatile int someint;
		someint++;
	}
	
	I_term += delta;
	
	if (I_term > 2000.0){
		I_term = 2000.0;
	}
	if (I_term < -2000.0){
		I_term = -2000.0;
	}
	sprintf(uart_send_buffer, ": %d, %d\n", (int)(delta*100000), (int)(I_term));
	//sprintf(uart_send_buffer,"iterm: %d\n", );
		
}
void velocity_control_360(Servo servo, float vel_ref){
	vel_ref = vel_ref_direction * 0.5f;
	// get current motor angle
	float position = read_motor_position();
	//volatile static float vel_ref_old = 0.0;
	volatile static float position_old = -9999.0f;
	
	// cant initalize position old as non constnat value - hack it like this
	if (position_old < -9998.0f){
		position_old = position;
	}
	
	//if (position > (position_old + 0.1)){
	//		position = position - 360.0;
	//	}
	
	volatile float current_vel = position - position_old;
	
	//volatile float test_var = abs( position - (position_old+360.0f) );
	
	//volatile float test_var2 = abs_(test_var);
	
	if (position < position_old){
		if (abs_(position + 360.0f - position_old) < abs_(position - position_old)){
			current_vel = position + 360.0f - position_old;
		}
	}
	else if (position > position_old){
		if (abs_(position - (position_old+360.0f)) < abs_(position - position_old)){
			current_vel = position - (position_old + 360.0f);
		}
	}
	
	

	//volatile float current_vel = min(position - position_old, position + 360.0 - position_old);
	

	volatile float test_vel_ref = vel_ref;
	
	// calculate the error
	volatile float delta = vel_ref - current_vel;
	volatile static float delta_old = 0.0;
	
	// all the necessary controller definitions
	volatile static float D_term = 0.0;
	volatile static float I_term = 0.0;
	volatile float control = 0.0;
	//angle_ref = 100;

	
	volatile float K = 0.0008f;//0.0008;//0.00125f;//0.00145f;//.0019f;//1.6f;
	volatile float Ki = 0.00025f;//0.001f;
	volatile float Kd = 0.00005f;//0.0019f;//0.5f;

	

	//if (delta > 180.0)
	//	delta -= 360.0;
	//else if (delta < -180.0)
	//	delta += 360.0;
		
	delta = delta/2.0;
	
	// add to I part
	I_term += delta;
	
	if (I_term > 500.0){
		I_term = 500.0;
	}
	else if (I_term < -500.0){
		I_term = -500.0;
	}	
	
	
	// calculate D part
	D_term = delta - delta_old;
	
	volatile float control_input = K * delta + Ki * I_term + Kd * D_term;//
	
	// pwm freq. where the motor stands still
	volatile float baseline_pwm = 0.0761;
	//baseline_pwm = 0.0;
	
	volatile int counter_max = 3750;
	// this results in a motor speed of 0
	//TC0->COUNT16.CC[1].reg = (baseline_pwm) * counter_max;
	
	control = baseline_pwm + control_input;
	if (control < 0.07){
		control = 0.07;
	}
	else if (control > 0.85){
		control = 0.85 ;
	}
	
	// one more try
	//if (control < baseline_pwm){
	//	control = baseline_pwm;
	//}
	//TC6->COUNT16.CC[1].reg = control * counter_max;
	*(servo.counter_reg) = control * counter_max;
	
	// set old angle for d part
	delta_old = delta;
	
	
	
		// TODO: Delete as soon as possible
	//memset(uart_send_buffer, '\0', sizeof(uart_send_buffer));
	//sprintf(uart_send_buffer, "position: %d, position_old: %d, speed: %d control: %d, iterm: %d, delta: %d, current_vel: %d, ref_vel: %d\n", (int)position, (int)position_old, (int)(current_vel), (int)(control*10000), (int)I_term, (int)(delta * 1000),(int)current_vel, (int)vel_ref);


	position_old = current_360_angle;
}



void read_gripper_close_sensors(){
	bool sensor_1;
	bool sensor_2;
	sensor_1 = (PORT->Group[GRIPPER_SENSOR_1/32].IN.reg & (1<<(GRIPPER_SENSOR_1%32)));
	sensor_2 = (PORT->Group[GRIPPER_SENSOR_2/32].IN.reg & (1<<(GRIPPER_SENSOR_2%32)));
	
	//memset(uart_send_buffer, '\0', sizeof(uart_send_buffer));
	//sprintf(uart_send_buffer, "gripper sensor inputs: %d, %d\n", (int)sensor_1, (int)sensor_2);
	
	if (uart_send_buffer[0] == '\0'){ // only send it if the buffer is empty
		sprintf(uart_send_buffer, "%s %d %d\n", "SENSORS", (int)sensor_1, (int)sensor_2);
	}
}


int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	init_uart();
	init_uart_ir();
	

	// Declare global array of Servo instances
	//Servo servo_list[2] = {blueServo, testServo};

	long count = 0;

	/* Replace with your application code */
	while (1) {
		uart_read();
		uart_write();
		uart_ir_read();
		uart_ir_write();
		
		current_360_angle = read_motor_position();
		
		if (localize_with_ir_full){
			// make it velocity controlled
			position_controlled_360 = false;
			// enable the ir stuff continuously
			localize_with_ir_continuous = true;
			// make sure it actually keeps the desired velocity
			//velocity_control_360(locationServo,0.0001);
			// moved this into the timed uart function
		}
		
		if (localize_with_ir_continuous){
			timed_uart_ir_write_continuous();
		}
		
		if(read_gripper_sensors_flag){
			read_gripper_close_sensors();
			read_gripper_sensors_flag = false;
		}
		//read_motor_position();
		//update_servo(0,0);
		//TC0->COUNT16.CC[1].reg = 3000;
		
		/*count++;
		if (count > 19999) {
			count = 0;
			//uart_ir_send_buffer[0] = 0x92;
			//uart_ir_send_buffer[1] = 0xD2;
			//uart_ir_send_buffer[2] = 0x4A;
			//uart_ir_send_buffer[3] = 0x0A;
			//uart_ir_send_buffer[4] = 0xD0;
			//uart_ir_send_buffer[5] = 0x00;
			
			//strcpy(uart_ir_send_buffer, "IKRP\r");
			//uart_ir_send_buffer[4] = 11;
			
			//strcpy(uart_ir_send_buffer, "IMRP\r");
			localize_with_ir = true;
		}*/
		
		//gpio_set_pin_level(ECHO_PIN, gpio_get_pin_level(GPIO(GPIO_PORTB, 25)));
		
		// control of 360 servo - has to be done every iteration
		if (position_controlled_360){
			control_360_servo(locationServo, angle_360_ref);
		}
		
	}
}
