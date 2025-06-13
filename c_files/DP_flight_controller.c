#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <signal.h>
#include <time.h>
#include <math.h>
#include <pigpio.h>

#define LFMOT_PIN    27
#define RFMOT_PIN    13
#define LBMOT_PIN    12
#define RBMOT_PIN    22
#define PWM_FREQ     2000
#define PWM_RANGE    100

// MPU9250 Address //
#define MPU_ADD	 0x68
// AK8963 Address //
#define MAG_ADD	 0x0C
// MPU9250 Registers Addresses //
#define PWR_MGMT_1	0x6B
#define GYRO_XOUT_H	 0x43
#define ACCEL_XOUT_H  0x3B
#define ACCEL_CONFIG2  0x1D
#define INT_PIN_CFG	 0x37
#define XG_OFFSET_H	 0x13
#define YG_OFFSET_H	 0x15
#define ZG_OFFSET_H	 0x17
#define XA_OFFSET_H	 0x77
#define XA_OFFSET_L	 0x78
#define YA_OFFSET_H	 0x7A
#define YA_OFFSET_L	 0x7B
#define ZA_OFFSET_H	 0x7D
#define ZA_OFFSET_L	 0x7E
// Values
#define A_FCHOICE_B	 (0x00 << 2)
#define A_DLPF_1  0x01	  //218.1 Hz	  ACCELEROMETER
#define A_DLPF_2  0x02	  //99 Hz	  DIGITAL
#define A_DLPF_3  0x03	  //44.8 Hz	  LOW
#define A_DLPF_4  0x04	  //21.2 Hz	  PASS
#define A_DLPF_5  0x05	  //10.2 Hz	  FILTER
#define A_DLPF_6  0x06	  //5.05 Hz	  CONFIGURATION
#define BYPASS_EN  0x02
// AK8963 Registers Addresses //
#define ST1	 0x02
#define CNTL_1	0x0A
#define ASA	 0x10
#define MAG_OUT	 0x03
// Values
#define PWDOWN_MODE	 0x00
#define FUSE_ROM_MODE  0x0F
#define C8HZ_MODE  0x02
#define C100HZ_MODE	 0x06
#define RES_14_BIT	0x00
#define RES_16_BIT	0x01
#define DRDY  0x01

// General constants and parameters
const uint8_t init_thrust = 12;
const uint8_t landing_thrust = 24;
uint8_t thrust = 62;			//Base propulsion level of the motors
double ref[3] = {1.0, 1.0, 100};	//Reference angle values
const double abs_horizontal_correction[2] = {-0.25, 1.8};
const double CF_alpha = 0.986;
uint8_t step = 0;

// Roll and Pitch control systems constants and parameters
const double Kp = 0.0;//1.5; //1.20; // 1.00;		// Proportional PID constant
const double Ki = 0.0;//3.6; //2.40; // 1.50;		// Integral PID constant
const double Kd = 0.0;//0.28; //0.28; // 0.14;		// Derivative PID constant
//ROLL
double roll_error = 0;
double prev_roll_error = 0;
double roll_IncEr = 0;
double roll_SumaEr = 0;	   // Error's accumulation (integral)
double roll_pid = 0;
double cur_d_roll_pid = 0;
double prev_d_roll_pid = 0;
double d_roll_pid = 0;
//PITCH
double pitch_error = 0;
double prev_pitch_error = 0;
double pitch_IncEr = 0;
double pitch_SumaEr = 0;	   // Error's accumulation (integral)
double pitch_pid = 0;
double cur_d_pitch_pid = 0;
double prev_d_pitch_pid = 0;
double d_pitch_pid = 0;

// Yaw control system constants and parameters
const double Kpy = 8.0; //2.0; // 3.00; // 1.8;	 // Proportional PID constant
const double Kiy = 0.0; //2.0; // 0.80; // 0.6;	 // Integral PID constant
const double Kdy = 0.0; // 0.02;		   // Derivative PID constant
double yaw_error = 0;
double prev_yaw_error = 0;
double yaw_IncEr = 0;
double yaw_SumaEr = 0;	  // Error's accumulation (integral)
double yaw_pid = 0;
double prev_d_yaw_pid = 0;

// Scaling parameters
const double gyro_scale = 131.0;			// scaling parameter for gyroscope readings
const double accel_scale = 16384.0;		// scaling parameter for accelerometer readings
const double mag_scale = 4912.0/32760.0;	// scaling parameter for magnetometer readings

// Calibration parameters
double mag_x_offset = -20.0;
double mag_y_offset = 15.0;
double mag_z_offset = -15.0;

// Timing variables
double initial_time, prev_time, cur_time, Tm, exec_time;

// I2C variables
uint8_t mpu_handle, mag_handle;

// Printing arrays //TODO: Right now it is implemented with a limited in size buffer of 4096 samples, i.e. aprox 4 s of samples
uint16_t tlm_index = 0;
double tm_array[4096] = {0};
double tp_array[4096] = {0};
double cfx_array[4096] = {0};
double cfy_array[4096] = {0};
double heading_array[4096] = {0};
double roll_er_array[4096] = {0};
double pitch_er_array[4096] = {0};
double yaw_er_array[4096] = {0};
double roll_pid_array[4096] = {0};
double pitch_pid_array[4096] = {0};
double yaw_pid_array[4096] = {0};
double ref0_array[4096] = {0};
double ref1_array[4096] = {0};
double ref2_array[4096] = {0};


uint8_t read_byte(uint8_t handle, uint8_t adr);
uint8_t write_byte(uint8_t handle, uint8_t adr, uint8_t value);
uint8_t read_block(uint8_t handle, uint8_t adr, uint8_t *buf, uint8_t size);
uint8_t write_block(uint8_t handle, uint8_t adr, uint8_t *values);
uint8_t read_all(double *accel_scaled_x, double *accel_scaled_y, double *accel_scaled_z, double *gyro_scaled_x, double *gyro_scaled_y, double *gyro_scaled_z);
double dist(double a, double b);
double get_y_rotation(double x, double y, double z);
double get_x_rotation(double x, double y, double z);
void set_normal_mode();
void bypass_mag();
void set_accel_DLPF(uint8_t val);
double north_to_deg(double x, double y);
uint8_t tilt_compensation(double mag_x, double mag_y, double mag_z, double rad_pitch, double rad_roll, double *comp_x, double *comp_y);
uint8_t format_data(FILE *data, double *matrix);
uint8_t load_mag_offsets(double *mag_off_x, double *mag_off_y, double *mag_off_z);
uint8_t load_accel_offsets(uint8_t *a_offset_xh, uint8_t *a_offset_xl, uint8_t *a_offset_yh, uint8_t *a_offset_yl, uint8_t *a_offset_zh, uint8_t *a_offset_zl);
uint8_t write_accel_offsets_to_MPU(uint8_t a_offset_xh, uint8_t a_offset_xl, uint8_t a_offset_yh, uint8_t a_offset_yl, uint8_t a_offset_zh, uint8_t a_offset_zl);
uint8_t write_gyro_offsets_to_MPU(uint8_t g_offset_xh, uint8_t g_offset_xl, uint8_t g_offset_yh, uint8_t g_offset_yl, uint8_t g_offset_zh, uint8_t g_offset_zl);

void exit_cleanup(int signum);
void finally();

int main (int argc, char **argv){

    double accel_scaled_x, accel_scaled_y, accel_scaled_z, gyro_scaled_x, gyro_scaled_y, gyro_scaled_z;
    double CF_x, CF_y, CF_x_total, CF_y_total, CF_x_average, CF_y_average;
    double gyro_offset_x, gyro_offset_y;
    double gyro_total_x, gyro_total_y;
    double gyro_x_delta, gyro_y_delta;
    double rotation_x, rotation_y;
    uint8_t a_offset_xh, a_offset_xl, a_offset_yh, a_offset_yl, a_offset_zh, a_offset_zl;

    uint8_t cf_data[3];
    double mag_x_cf, mag_y_cf, mag_z_cf;
    uint8_t raw_mag[6];
    int16_t raw_mag_x, raw_mag_y, raw_mag_z;
    double mag_x, mag_y, mag_z, mag_x_comp, mag_y_comp;
    double north_deg_comp;

    int pwm_LF = 0, pwm_RF = 0, pwm_LB = 0, pwm_RB = 0;

    gpioCfgClock(4,1,0); // 4us sample rate, PCM and ignored value
    if (gpioInitialise() < 0) {
        return 1;
    }

    signal(SIGINT, exit_cleanup); //This must be placed after gpioInitialise()!

    gpioSetPWMfrequency(LFMOT_PIN,PWM_FREQ);
    gpioSetPWMfrequency(RFMOT_PIN,PWM_FREQ);
    gpioSetPWMfrequency(LBMOT_PIN,PWM_FREQ);
    gpioSetPWMfrequency(RBMOT_PIN,PWM_FREQ);

    gpioSetPWMrange(LFMOT_PIN,PWM_RANGE);
    gpioSetPWMrange(RFMOT_PIN,PWM_RANGE);
    gpioSetPWMrange(LBMOT_PIN,PWM_RANGE);
    gpioSetPWMrange(RBMOT_PIN,PWM_RANGE);

    gpioPWM(LFMOT_PIN,0);
    gpioPWM(RFMOT_PIN,0);
    gpioPWM(LBMOT_PIN,0);
    gpioPWM(RBMOT_PIN,0);

	// Open the i2c infrastructure and create handles
	mpu_handle = i2cOpen(1,MPU_ADD,0);
	mag_handle = i2cOpen(1,MAG_ADD,0);
	// Now wake the MPU9250 up as it starts in sleep mode and set magnetomer bypass enable
	set_normal_mode();
	bypass_mag();
	// Set the DLPF filtering frequency (BW)
	set_accel_DLPF(A_DLPF_6);
	time_sleep(0.01);

	// Set the correct accelerometer offsets
	load_accel_offsets(&a_offset_xh, &a_offset_xl, &a_offset_yh, &a_offset_yl, &a_offset_zh, &a_offset_zl);
	write_accel_offsets_to_MPU(a_offset_xh, a_offset_xl, a_offset_yh, a_offset_yl, a_offset_zh, a_offset_zl);
//	printf("a_offset_xh:%d a_offset_xl:%d a_offset_yh:%d a_offset_yl:%d a_offset_zh:%d a_offset_zl:%d\n",a_offset_xh, a_offset_xl, a_offset_yh, a_offset_yl, a_offset_zh, a_offset_zl);
	//  time_sleep(10);

	// Set the correct gyroscope offsets
	// write_gyro_offsets_to_MPU(255,126,0,18,0,0);
	// write_gyro_offsets_to_MPU(0,0,0,0,0,0);

    time_sleep(0.5);
	prev_time = time_time();

	// Read accelerometer and gyroscope raw values and get the scaled values
	read_all(&accel_scaled_x, &accel_scaled_y, &accel_scaled_z, &gyro_scaled_x, &gyro_scaled_y, &gyro_scaled_z);
//	printf("accel_scaled_x:%.4lf, accel_scaled_y:%.4lf, accel_scaled_z:%.4lf ", accel_scaled_x, accel_scaled_y, accel_scaled_z);
//	printf("gyro_scaled_x:%.4f, gyro_scaled_y:%.4f, gyro_scaled_z:%.4f\n", gyro_scaled_x, gyro_scaled_y, gyro_scaled_z);
	// First complimentary filter (CF) variables assignments only with accelerometer readings
	CF_x = get_x_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z);
	CF_y = get_y_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z);
//    printf("CF_x:%lf, CF_y:%lf, dist_yz:%lf, dist_xz:%lf",CF_x,CF_y,dist(accel_scaled_y,accel_scaled_z),dist(accel_scaled_x,accel_scaled_z));
	// Create an offset with the first read value
	gyro_offset_x = gyro_scaled_x;
	gyro_offset_y = gyro_scaled_y;
	// Sets first gyroscope values from the CF variables
	gyro_total_x = CF_x;
	gyro_total_y = CF_y;

	// Power down and Fuse ROM access mode established
	write_byte(mag_handle, CNTL_1, PWDOWN_MODE);
	time_sleep(0.01);
	write_byte(mag_handle, CNTL_1, FUSE_ROM_MODE);
	time_sleep(0.01);

	// Readings of magnetometer sensitivity data
	read_block(mag_handle, ASA, cf_data, 3);
	mag_x_cf = (cf_data[0] - 128) / 256.0 + 1;
	mag_y_cf = (cf_data[1] - 128) / 256.0 + 1;
	mag_z_cf = (cf_data[2] - 128) / 256.0 + 1;
//    printf("x_cf:%lf y_cf:%lf z_cf:%lf\n", mag_x_cf, mag_y_cf, mag_z_cf);

	// Power down for 10 ms
	write_byte(mag_handle, CNTL_1, PWDOWN_MODE);
	time_sleep(0.01);

	// Setting 16bit resolution and continous 100Hz mode
	write_byte(mag_handle, CNTL_1, C100HZ_MODE|(RES_16_BIT<<4));
	time_sleep(0.01);

	// printf ("%.4f %.2f %.2f %.2f %.2f %.2f %.2f", time_time() - initial, CF_x, gyro_total_x, CF_x, CF_y, gyro_total_y, CF_y));

	load_mag_offsets(&mag_x_offset, &mag_y_offset, &mag_z_offset);
//	printf("mag_x_offset:%lf mag_y_offset:%lf mag_z_offset:%lf\n", mag_x_offset, mag_y_offset, mag_z_offset);

	// Here there was a "try:" in python
	read_block(mag_handle, MAG_OUT, raw_mag, 7);
	time_sleep(0.01);
	if((raw_mag[6] & 0x08) != 0x08){	// if there is a read value make the conversions
		raw_mag_x = (int16_t)((raw_mag[1] << 8) + raw_mag[0]);
		raw_mag_y = (int16_t)((raw_mag[3] << 8) + raw_mag[2]);
		raw_mag_z = (int16_t)((raw_mag[5] << 8) + raw_mag[4]);
//        printf("raw_x:%d, raw_y:%d, raw_z:%d\n", raw_mag_x,raw_mag_y,raw_mag_z);
		mag_x = round(((raw_mag_x - mag_x_offset) * mag_x_cf * mag_scale) * 1000) / 1000.0;
		mag_y = round(((raw_mag_y - mag_y_offset) * mag_y_cf * mag_scale) * 1000) / 1000.0;
		mag_z = round(((raw_mag_z - mag_z_offset) * mag_z_cf * mag_scale) * 1000) / 1000.0;
//        printf("mag_x:%lf, mag_y:%lf, mag_z:%lf\n", mag_x,mag_y,mag_z);

		// north_deg = north_to_deg(mag_x, mag_y);
		// make a compensation of the values, necessary due to the tilt of the sensor
		tilt_compensation(mag_x, mag_y, mag_z, CF_y*M_PI/180.0, CF_x*M_PI/180.0, &mag_x_comp, &mag_y_comp);
//        printf("comp_x:%lf, comp_y:%lf, CF_y:%lf, CF_x:%lf\n", mag_x_comp, mag_y_comp, CF_y, CF_x);
		north_deg_comp = north_to_deg(mag_x_comp, mag_y_comp);
	}
	ref[2] = north_deg_comp;
	printf("ndegcomp:%lf\n",north_deg_comp);

	CF_x_total = 0;
	CF_y_total = 0;

	for (uint16_t i = 1; i <= 500; i++) {
		cur_time = time_time();
		Tm = (cur_time - prev_time); // Sample time variable measures time between loop code runs
		prev_time = cur_time;		// This variable keeps current time as previous one for the next Tm assignment
		// read accelerometer and gyroscope raw values and get the scaled values
        read_all(&accel_scaled_x, &accel_scaled_y, &accel_scaled_z, &gyro_scaled_x, &gyro_scaled_y, &gyro_scaled_z);
		// substract the offset from the gyro scaled value
		gyro_scaled_x -= gyro_offset_x;
		gyro_scaled_y -= gyro_offset_y;
		// calculate the increment of angle
		gyro_x_delta = (gyro_scaled_x * Tm);
		gyro_y_delta = (gyro_scaled_y * Tm);
		// calculate the current angle
		gyro_total_x += gyro_x_delta;
		gyro_total_y += gyro_y_delta;
		// get the rotations based only on accelerometer readings
		rotation_x = get_x_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z);
		rotation_y = get_y_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z);

		// CF assignment with gyroscope readings and accelerometer readings
		// CF calculation ponderates readings from both sensors,
		// that derives into more reliable readings, from accelerometer at lower speeds
		// and from gyroscope at greater ones
		CF_x = 0.96 * (CF_x + gyro_x_delta) + (0.04 * rotation_x);
		CF_x_total += CF_x;
		CF_y = 0.96 * (CF_y + gyro_y_delta) + (0.04 * rotation_y);
		CF_y_total += CF_y;
	}
	CF_x_average = CF_x_total / 500.0;
	CF_y_average = CF_y_total / 500.0;
//	   printf("CFX_av:{:+07.4f}%f, CFY_av:{:+07.4f}%f  ", CF_x_average, CF_y_average); //TODO

	ref[0] = CF_x_average + abs_horizontal_correction[0];
	ref[1] = CF_y_average + abs_horizontal_correction[1];
	step = 0;
	initial_time = time_time();

	prev_d_roll_pid = 0;
	prev_d_pitch_pid = 0;

	while (1){	   // keep running until a key press
		cur_time = time_time();
		Tm = (cur_time - prev_time);	   // Sample time variable measures time between loop code runs
		prev_time = cur_time;		// this variable keeps current time as previous one for the next Tm assignment

		// read accelerometer and gyroscope raw values and get the scaled values
		read_all(&accel_scaled_x, &accel_scaled_y, &accel_scaled_z, &gyro_scaled_x, &gyro_scaled_y, &gyro_scaled_z);
//		printf("accel_scaled_x:%.4lf, accel_scaled_y:%.4lf, accel_scaled_z:%.4lf ", accel_scaled_x, accel_scaled_y, accel_scaled_z);
//		printf("gyro_scaled_x:%.4f, gyro_scaled_y:%.4f, gyro_scaled_z:%.4f\n", gyro_scaled_x, gyro_scaled_y, gyro_scaled_z);

		// substract the offset from the gyro scaled value
		gyro_scaled_x -= gyro_offset_x;
		gyro_scaled_y -= gyro_offset_y;
		// calculate the increment of angle
		gyro_x_delta = (gyro_scaled_x * Tm);
		gyro_y_delta = (gyro_scaled_y * Tm);
		// calculate the current angle
		gyro_total_x += gyro_x_delta;
		gyro_total_y += gyro_y_delta;
		// get the rotations based only on accelerometer readings
		rotation_x = get_x_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z);
		rotation_y = get_y_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z);

		// CF assignment with gyroscope readings and accelerometer readings
		// CF calculation ponderates readings from both sensors,
		// that derives into more reliable readings, from accelerometer at lower speeds
		// and from gyroscope at greater ones
		CF_x = CF_alpha * (CF_x + gyro_x_delta) + ((1-CF_alpha) * rotation_x);
		roll_error = CF_x - ref[0];
		CF_y = CF_alpha * (CF_y + gyro_y_delta) + ((1-CF_alpha) * rotation_y);
		pitch_error = CF_y - ref[1];
		// print("Roll_Err:{:+08.4f}, Pitch_Err:{:+08.4f}  ".format(roll_error, pitch_error), end = '') //TODO

		read_block(mag_handle, MAG_OUT, raw_mag, 7);
		if((raw_mag[6] & 0x08) != 0x08){	// if there is a read value make the conversions
			raw_mag_x = (int16_t)((raw_mag[1] << 8) + raw_mag[0]);
		    raw_mag_y = (int16_t)((raw_mag[3] << 8) + raw_mag[2]);
		    raw_mag_z = (int16_t)((raw_mag[5] << 8) + raw_mag[4]);

			mag_x = roundf(((raw_mag_x - mag_x_offset) * mag_x_cf * mag_scale) * 1000) / 1000.0;
		    mag_y = roundf(((raw_mag_y - mag_y_offset) * mag_y_cf * mag_scale) * 1000) / 1000.0;
		    mag_z = roundf(((raw_mag_z - mag_z_offset) * mag_z_cf * mag_scale) * 1000) / 1000.0;

			// north_deg = north_to_deg(mag_x, mag_y);
			// make a compensation of the values, necessary due to the tilt of the sensor
			tilt_compensation(mag_x, mag_y, mag_z, CF_y*M_PI/180.0, CF_x*M_PI/180.0, &mag_x_comp, &mag_y_comp);
			north_deg_comp = north_to_deg(mag_x_comp, mag_y_comp);

			// print('MAG_XYZ:(', mag_x, ',', mag_y, ',', mag_z, ')','CMAG_XYZ:',mag_x_comp,mag_y_comp ,', ORIENTATION:', north_deg, 'COMP_Orientation:',north_deg_comp,end='') //TODO
			// print("%.4f %.4f}" % (raw_mag_x,raw_mag_y)) //TODO
			// time_sleep(0.005);
		}
		yaw_error = north_deg_comp - ref[2];

		if (yaw_error > 180) {
			yaw_error -= 360;
		} else if (yaw_error < -180) {
			yaw_error += 360;
		}
		// print("yaw_Er:", yaw_error, " ", end='') //TODO

		// set initial thrust limitation
		if (cur_time - initial_time < 0.5) {
			pwm_LF = init_thrust;
			pwm_RF = init_thrust;
			pwm_LB = init_thrust;
			pwm_RB = init_thrust;

			prev_roll_error = roll_error;
			prev_pitch_error = pitch_error;
			prev_yaw_error = yaw_error;

		} else {
			// // ROLL ERROR CALCS
			roll_IncEr = roll_error - prev_roll_error;		// Error increment
			//  roll_SumaEr += roll_error;					   // Error accumulation (integral)(Euler II)
			roll_SumaEr += (roll_error + prev_roll_error)/2; // Error accumulation (integral)(Tustin)

//			if (prev_roll_error * roll_error < 0) {		   // Anti-windup mechanism
//				roll_SumaEr = 0;
//			}
			// prev_roll_error = roll_error;					   // Previous roll error update

			// // PITCH ERROR CALCS
			pitch_IncEr = pitch_error - prev_pitch_error;	// Error increment
			// pitch_SumaEr += pitch_error;					   // Error accumulation (integral)(Euler II)
			pitch_SumaEr += (pitch_error + prev_pitch_error)/2; // Error accumulation (integral)(Tustin)

//			if (prev_pitch_error * pitch_error < 0) {	   // Anti-windup mechanism
//				pitch_SumaEr = 0;
//			}
			// prev_pitch_error = pitch_error;				   // Previous pitch error update

			// // YAW ERROR CALCS
			yaw_IncEr = yaw_error - prev_yaw_error;			// Error increment
			// yaw_SumaEr += yaw_error;						   // Error accumulation (integral)(Euler II)
			yaw_SumaEr += (yaw_error + prev_yaw_error)/2; // Error accumulation (integral)(Tustin)

//			if (prev_yaw_error * yaw_error < 0) {		   // Anti-windup mechanism
//				yaw_SumaEr = 0;
//			}
			// prev_yaw_error = yaw_error;					   // Previous yaw error update

			// PID controllers calculation (parallel)
			// Roll derivative PID term filtering
			cur_d_roll_pid = Kd * roll_IncEr / Tm;
			d_roll_pid = 0.1 * cur_d_roll_pid + 0.9 * prev_d_roll_pid;
			prev_d_roll_pid = cur_d_roll_pid;

			roll_pid = (Kp * roll_error) + d_roll_pid + (Ki * roll_SumaEr * Tm);
			// Pitch derivative PID term filtering
			cur_d_pitch_pid = Kd * pitch_IncEr / Tm;
			d_pitch_pid = 0.1 * cur_d_pitch_pid + 0.9 * prev_d_pitch_pid;
			prev_d_pitch_pid = cur_d_pitch_pid;

			pitch_pid = (Kp * pitch_error) + d_pitch_pid + (Ki * pitch_SumaEr * Tm);

			yaw_pid = (Kpy * yaw_error) + (Kdy * yaw_IncEr / Tm) + (Kiy * yaw_SumaEr * Tm);
			// print("Roll_Pid:{:+08.4f}, Pitch_Pid:{:+08.4f}, Yaw_Pid:{:+08.4f}  ".format(roll_pid, pitch_pid, yaw_pid)) //TODO

			// Anti-windup mechanism: Don't increment error integral while saturated pid values
			if(roll_pid > 100 || roll_pid < -100) {
				roll_SumaEr -= (roll_error + prev_roll_error)/2;
			}
			if(pitch_pid > 100 || pitch_pid < -100) {
				pitch_SumaEr -= (roll_error + prev_roll_error)/2;
			}
			if(yaw_pid > 100 || yaw_pid < -100) {
				yaw_SumaEr -= (yaw_error + prev_yaw_error)/2;
			}
			prev_roll_error = roll_error;
			prev_pitch_error = pitch_error;
			prev_yaw_error = yaw_error;

			pwm_LF = thrust + roll_pid - pitch_pid + yaw_pid -8;	 // PWM motorL value (left-front) - Extra 6 points compensation due to parcially burned red-blue wires motors
			pwm_RB = thrust - roll_pid + pitch_pid + yaw_pid -8;	 // PWM motorR value (right-back) - Extra 6 points compensation due to parcially burned red-blue wires motors
			pwm_RF = thrust - roll_pid - pitch_pid - yaw_pid +8;	 // PWM motorF value (right-front) - Extra 6 points compensation due to parcially burned red-blue wires motors
			pwm_LB = thrust + roll_pid + pitch_pid - yaw_pid +8;	 // PWM motorB value (left-back) - Extra 6 points compensation due to parcially burned red-blue wires motors

			//TODO
//			print(ref[0]," ",CF_x)
			if (cur_time - initial_time >= 1.0 && cur_time - initial_time < 1.6 && step == 0) {
				ref[2] += 5;
				step = 1;}
//				print(ref[0]," ",CF_x);
//			} else if (cur_time - initial_time >= 2.5 && cur_time - initial_time < 2.6 && step == 1) {
//				ref[0] += 5;
//				step = 2;
//				print("step 2");
//			}
		}

		if (pwm_LF < 2) {
			pwm_LF = 2;
		} else if (pwm_LF > 98) {   // PWM
			pwm_LF = 98;            //
		}                           //
		if (pwm_RB < 2) {           // SATURATION
			pwm_RB = 2;             //
		} else if (pwm_RB > 98) {   // FILTER
			pwm_RB = 98;
		}
		if (pwm_RF < 2) {
			pwm_RF = 2;
		} else if (pwm_RF > 98) {   // PWM
			pwm_RF = 98;            //
		}                           //
		if (pwm_LB < 2) {           // SATURATION
			pwm_LB = 2;             //
		} else if (pwm_LB > 98) {   // FILTER
			pwm_LB = 98;
		}
//		   print("PWM_LF:{:06.4f}, PWM_RB:{:06.4f} ".format(pwm_LF, pwm_RB),end='') //TODO
//		   print("PWM_RF:{:06.4f}, PWM_LB:{:06.4f}".format(pwm_RF, pwm_LB)) //TODO

		// Motor control pins PWM value assignments
		gpioPWM(LFMOT_PIN,pwm_LF);
		gpioPWM(RFMOT_PIN,pwm_RF);
		gpioPWM(LBMOT_PIN,pwm_LB);
		gpioPWM(RBMOT_PIN,pwm_RB);

//		   print("elapsed:{0:.4f} Acc_x:{1:.2f} Gyr_T_x:{2:.2f} CF_x:{3:.2f} Acc_y:{4:.2f} Gyr_T_y:{5:.2f} CF_y:{6:.2f}".format(time.time()-initial_time,(rotation_x),(gyro_total_x),(CF_x),(rotation_y),(gyro_total_y),(CF_y)))
//		   print("{0:.4f} {1:.2f} {2:.2f} {3:.2f} {4:.2f} {5:.2f} {6:.2f}".format(time.time()-initial_time,(rotation_x),(gyro_total_x),(CF_x),(rotation_y),(gyro_total_y),(CF_y)))
//	    printf("%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n", cur_time-initial_time,CF_x,CF_y,north_deg_comp,roll_error,pitch_error,yaw_error,roll_pid,pitch_pid,yaw_pid,ref[0],ref[1],ref[2]);

		 //TODO: Change for a circular buffer
        if (tlm_index < 4096) {
    		tm_array[tlm_index] = Tm;
    		tp_array[tlm_index] = cur_time - initial_time;
    		cfx_array[tlm_index] = CF_x;
    		cfy_array[tlm_index] = CF_y;
    		heading_array[tlm_index] = north_deg_comp;
    		roll_er_array[tlm_index] = roll_error;
    		pitch_er_array[tlm_index] = pitch_error;
    		yaw_er_array[tlm_index] = yaw_error;
    		roll_pid_array[tlm_index] = roll_pid;
    		pitch_pid_array[tlm_index] = pitch_pid;
    		yaw_pid_array[tlm_index] = yaw_pid;
    		ref0_array[tlm_index] = ref[0];
    		ref1_array[tlm_index] = ref[1];
    		ref2_array[tlm_index] = ref[2];
    		tlm_index++;
//    		printf("W:Tm_Array!\n");
		}
        exec_time = time_time() - cur_time;
		if (exec_time < 0.0015 - 0.0001) {
            time_sleep(0.0015 - 0.0001 - exec_time);
		}
	}
}

uint8_t read_byte(uint8_t handle, uint8_t adr) {
	return i2cReadByteData(handle, adr);
}

uint8_t write_byte(uint8_t handle, uint8_t adr, uint8_t value) {
	return i2cWriteByteData(handle, adr, value);
}

uint8_t read_block(uint8_t handle, uint8_t adr, uint8_t *buf, uint8_t size) {
	return i2cReadI2CBlockData(handle, adr, buf, size);
}

uint8_t write_block(uint8_t handle, uint8_t adr, uint8_t *values) {
	return i2cWriteI2CBlockData(handle, adr, values, sizeof(values));
}

uint8_t read_all(double *accel_scaled_x, double *accel_scaled_y, double *accel_scaled_z, double *gyro_scaled_x, double *gyro_scaled_y, double *gyro_scaled_z) {
	uint8_t raw_data[14];
	read_block(mpu_handle, ACCEL_XOUT_H, raw_data, 14);
//	printf("a_x:%d|%d, a_y:%d|%d, a_z:%d|%d ", raw_data[0],raw_data[1],raw_data[2],raw_data[3],raw_data[4],raw_data[5]);
//	printf("g_x:%d|%d, g_y:%d|%d, g_z:%d|%d\n", raw_data[8],raw_data[9],raw_data[10],raw_data[11],raw_data[12],raw_data[13]);

	*accel_scaled_x = ((int16_t)((raw_data[0] << 8) + raw_data[1])) / accel_scale;
	*accel_scaled_y = ((int16_t)((raw_data[2] << 8) + raw_data[3])) / accel_scale;
	*accel_scaled_z = ((int16_t)((raw_data[4] << 8) + raw_data[5])) / accel_scale;

//    double temp = ((int16_t)(raw_data[6] << 8) + raw_data[7])) / 321.0 + 21;

	*gyro_scaled_x = ((int16_t)((raw_data[8] << 8) + raw_data[9])) / gyro_scale;
	*gyro_scaled_y = ((int16_t)((raw_data[10] << 8) + raw_data[11])) / gyro_scale;
	*gyro_scaled_z = ((int16_t)((raw_data[12] << 8) + raw_data[13])) / gyro_scale;

	return 0;
}

double dist(double a, double b) {
	return sqrt((a * a) + (b * b));
}

double get_y_rotation(double x, double y, double z) {
	double radians;
	radians	= atan2(x, dist(y, z));
	return -(radians*180/M_PI);
}

double get_x_rotation(double x, double y, double z) {
	double radians;
	radians = atan2(y, dist(x, z));
	return (radians*180/M_PI);
}

void set_normal_mode() {
	write_byte(mpu_handle, PWR_MGMT_1, 0);
}

void bypass_mag() {
	write_byte(mpu_handle, INT_PIN_CFG, BYPASS_EN);
}

void set_accel_DLPF(uint8_t val) {
	write_byte(mpu_handle, ACCEL_CONFIG2, val);
}

double north_to_deg(double x, double y) {
	double north_rad;
	north_rad = atan2(y, x) + M_PI / 2.0;
	if (north_rad < 0) {
		north_rad += 2 * M_PI;
	}
	return (north_rad*180/M_PI);
}

uint8_t tilt_compensation(double mag_x, double mag_y, double mag_z, double rad_pitch, double rad_roll, double *comp_x, double *comp_y) {
	*comp_x = (mag_x * cos(rad_roll)) + (mag_z * sin(rad_roll));
	*comp_y = (mag_x * sin(rad_pitch) * sin(rad_roll)) + (mag_y * cos(rad_pitch)) - (mag_z * sin(rad_pitch) * cos(rad_roll));
	return 0;
}

uint8_t format_data(FILE *data, double *matrix) {
	char *token;
	char buf[43];
	uint8_t i = 0;
	char *res;

	res = fgets(buf, 43, data);
	if (res == NULL) {
	    return 1;
	}
//    printf("res: %s, buf:%s\n",res,buf);
    token = strtok(buf, " ");
	while(token != NULL) {
	    sscanf(token, "%lf", &matrix[i]);
	    i++;
	    token = strtok(NULL, " ");
	}
	return 0;
}

uint8_t load_mag_offsets(double *mag_off_x, double *mag_off_y, double *mag_off_z) {
	FILE *fichero = fopen("mag_calib.txt","r");
	double mag_offsets[3];

	format_data(fichero, mag_offsets);

	*mag_off_x = mag_offsets[0];
	*mag_off_y = mag_offsets[1];
	*mag_off_z = mag_offsets[2];

	return fclose(fichero);
}

uint8_t load_accel_offsets(uint8_t *a_offset_xh, uint8_t *a_offset_xl, uint8_t *a_offset_yh, uint8_t *a_offset_yl, uint8_t *a_offset_zh, uint8_t *a_offset_zl) {
	FILE *fichero = fopen("accel_calib.txt","r");
	double accel_offsets[6];

	format_data(fichero, accel_offsets);

	*a_offset_xh = (uint8_t)accel_offsets[0];
	*a_offset_xl = (uint8_t)accel_offsets[1];
	*a_offset_yh = (uint8_t)accel_offsets[2];
	*a_offset_yl = (uint8_t)accel_offsets[3];
	*a_offset_zh = (uint8_t)accel_offsets[4];
	*a_offset_zl = (uint8_t)accel_offsets[5];

	return fclose(fichero);
}

uint8_t write_accel_offsets_to_MPU(uint8_t a_offset_xh, uint8_t a_offset_xl, uint8_t a_offset_yh, uint8_t a_offset_yl, uint8_t a_offset_zh, uint8_t a_offset_zl) {
    uint8_t values[2];
    values[0] = a_offset_xh;
    values[1] = a_offset_xl;
	write_block(mpu_handle, XA_OFFSET_H, values);
	time_sleep(0.01);
    values[0] = a_offset_yh;
    values[1] = a_offset_yl;
	write_block(mpu_handle, YA_OFFSET_H, values);
	time_sleep(0.01);
	values[0] = a_offset_zh;
    values[1] = a_offset_zl;
	write_block(mpu_handle, ZA_OFFSET_H, values);
	time_sleep(0.01);
}

uint8_t write_gyro_offsets_to_MPU(uint8_t g_offset_xh, uint8_t g_offset_xl, uint8_t g_offset_yh, uint8_t g_offset_yl, uint8_t g_offset_zh, uint8_t g_offset_zl) {
    uint8_t values[2];
    values[0] = g_offset_xh;
    values[1] = g_offset_xl;
	write_block(mpu_handle, XG_OFFSET_H, values);
	time_sleep(0.01);
    values[0] = g_offset_yh;
    values[1] = g_offset_yl;
	write_block(mpu_handle, YG_OFFSET_H, values);
	time_sleep(0.01);
	values[0] = g_offset_zh;
    values[1] = g_offset_zl;
	write_block(mpu_handle, ZG_OFFSET_H, values);
	time_sleep(0.01);
}

void exit_cleanup(int signum) {
	// exception routine for a clean exit
	gpioPWM(LFMOT_PIN,landing_thrust);
	gpioPWM(RFMOT_PIN,landing_thrust);
	gpioPWM(LBMOT_PIN,landing_thrust);
	gpioPWM(RBMOT_PIN,landing_thrust);
	time_sleep(0.20);
	gpioPWM(LFMOT_PIN,0);
	gpioPWM(RFMOT_PIN,0);
	gpioPWM(LBMOT_PIN,0);
	gpioPWM(RBMOT_PIN,0);

	i2cClose(mpu_handle);
	i2cClose(mag_handle);

	time_sleep(0.05);
	gpioTerminate();
//    printf("Terminated!\n");
	finally();
	exit(signum);
}

void finally(){
    double tm_av = 0;
	for (uint16_t i=0; i<tlm_index; i++) {
		printf("%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n", tp_array[i],cfx_array[i],cfy_array[i],heading_array[i],roll_er_array[i],pitch_er_array[i],yaw_er_array[i],roll_pid_array[i],pitch_pid_array[i],yaw_pid_array[i],ref0_array[i],ref1_array[i],ref2_array[i]);
        tm_av += tm_array[i];
    }
	printf("Tm_mean: %.5f\n", tm_av/tlm_index);
//	   print("Comp time:", post_comp_time - pre_comp_time)
}
