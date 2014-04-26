/*
 * main.c
 * Lab 9 - Task 2
 * ECE 4550 - L01
 * Authors: Ryan Sanders, Chris Paucar
 */
#define pi 3.14159
#define size 10
#include "F2806x_Device.h"
#include "F2806x_PieVect.h"
#include "math.h"

void initClock();				// initialize clock
void initTimer(Int32);			// initialize timer to appropriate freq
void initInterrupt();			// initialize all interrupts
void initADC();					// initialize 	ADC module
void wait(float32);				// wait a number of seconds (or fraction)
void initPWM();					// initialize PWM module
void setVoltage(float32, float32, float32);		// set new Voltage
void initQEP();
interrupt void timerISR(void);
interrupt void AdcISR(void);

// system variables
int32 fsys  = 0;				// system frequency
int16 count = 0;				// counter for voltage changes


// constants
float32 alpha, beta, K;
float32 V_dc, I_ABCmax, V_max, I_max, T, lambda_r, lambda_e, Qs, Qa;
float32 N, L, R, J, F, lambda;

// ADC measurements
float32 currentA = 0.0, currentB = 0.0, currentC = 0.0;

// Data logger
int16   count1 = 0,count2 = 0;
float32 currentALog[1000],currentBLog[1000],currentCLog[1000], posLog[1000];

// position
int32 counter = 0;
float32 y     = 0;

// voltage vector generator
float32 v = 0.0, phi = 0.0;

// over-current protection
int16 low_thresh = 809;
int16 hi_thresh  = 3267;

// controller
float32 u=0, x1_hat=0, x2_hat=0, sigma=0, r = 0;
float32 u_new=0, x2hat_new = 0, x1hat_new = 0, sigma_new = 0;
int32 r_counter = 0;
float32 L1, L2, K11, K12, K2;
float32 voltageA = 0, voltageB = 0, voltageC = 0;
float32 Vdc   = 24.0;
float32 Vm    =  2.0;
float32 theta =  0.0;

// reference command shaping
float32 accel_max, t_accel, t_cruise, t_f, t_rest, theta_i, theta_f, T_max, vel_max;
int32 back = 0;
float32 k  = 0.0, mid;

void main(void) {
	EALLOW;
	SysCtrlRegs.WDCR = 0x68;				// disable watchdog
	initClock();
	initTimer(5000);						// initialize timer to 5kHz
	initInterrupt();
	initADC();
	initPWM();
	initQEP();
	EDIS;

	// motor parameters
	N      = 4;
	L      = 1.8E-3;
	R      = 1.2;
	J      = 4.8E-6;
	F      = 5E-5;
	lambda = 11E-3;

	// controller parameters
	V_dc     = 24;
	I_ABCmax = 5;
	V_max    = 1.061 * V_dc;
	I_max    = 0.707*I_ABCmax;
	T        = 2E-4;
	lambda_r = 100;
	lambda_e = 4*lambda_r;
	Qs       = 2*pi/4000;
	Qa       = 24/1500;

	// plant dynamics
	alpha = (K*K + F*R)/(J*R);
	beta  = K/(J*R);
	K     = lambda*R;
	L1    = (2.0*lambda_e) - alpha;
	L2    = lambda_e*lambda_e - 2*alpha*lambda_e + alpha*alpha;
	K11   = 3*lambda_r*lambda_r/beta;
	K12   = (3*lambda_r - alpha)/beta;
	K2    = pow(lambda_r,3)/beta;

	// reference command shaping
	T_max     = lambda*N*I_max;
	vel_max   = V_max / (N*sqrt(lambda*lambda + L*L*I_max));
	accel_max = T_max / (J*3.7);
	t_f       = (10.0*pi/vel_max) + (vel_max / accel_max);
	t_accel   = vel_max / accel_max;
	t_cruise  = 10.0*pi/vel_max - vel_max/accel_max;
	t_rest    = 0.25;
	theta_i   = 0.0;
	theta_f   = 10.0*pi;

	// pre-startup procedure
	setVoltage(12.0, 12.0, 12.0);			// set all legs to 50% duty cycle.
	GpioDataRegs.GPBSET.bit.GPIO32 = 1;		// drive it high to  enable PWM output

	// start-up procedure
	voltageA = 0.5*Vdc + Vm * cos(N*theta - (pi/6.0));
	voltageB = 0.5*Vdc + Vm * cos(N*theta - (pi/6.0) - (2.0*pi/3.0));
	voltageC = 0.5*Vdc + Vm * cos(N*theta - (pi/6.0) + (2.0*pi/3.0));

	setVoltage(voltageA, voltageB, voltageC);	// set voltage to new values

	wait(0.5);								// wait one second for the rotor to align

	CpuTimer0Regs.TCR.bit.TSS = 0;			// start timer
	EINT;									// allow timer interrupts

	EQep1Regs.QEPCTL.bit.SWI = 1;			// reset this position to zero

	while (1) {
		EALLOW;
		SysCtrlRegs.WDKEY = 0x55;
		SysCtrlRegs.WDKEY = 0xAA;
		EDIS;
	}

}

void initClock() {
	if (SysCtrlRegs.PLLSTS.bit.MCLKSTS == 1) {
		ESTOP0;
	}      // Checks if oscillator is working if not terminate
	if (SysCtrlRegs.PLLSTS.bit.DIVSEL == 2
			|| SysCtrlRegs.PLLSTS.bit.DIVSEL == 3) {
		SysCtrlRegs.PLLSTS.bit.DIVSEL = 0;
	}
	SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;
	SysCtrlRegs.PLLCR.bit.DIV = 9;  			//Set new DIV value (1-18)
	while (SysCtrlRegs.PLLSTS.bit.PLLLOCKS != 1) {
	}
	SysCtrlRegs.PLLSTS.bit.MCLKOFF = 0;
	SysCtrlRegs.PLLSTS.bit.DIVSEL = 3;  		//Set new DIVSEL value (0-3)

	fsys = 90000000;							// system at 10 MHz
}

void initTimer(Int32 freq) {
	CpuTimer0Regs.TCR.bit.TSS = 1; 				// Stop Timer
	CpuTimer0Regs.PRD.all     = (fsys / (freq) - 1);// Setting the PDR to appropriate freq
	CpuTimer0Regs.TCR.bit.TRB = 1;				// Reload Timer
	CpuTimer0Regs.TCR.bit.TIE = 1;				// Enabling Timer interrupt
}

void setVoltage(float32 vA, float32 vB, float32 vC) {
	float32 dutyA            = vA / Vdc;			// duty = V / Vdc
	float32 dutyB 			 = vB / Vdc;
	float32 dutyC 			 = vC / Vdc;

	EPwm1Regs.CMPA.half.CMPA = dutyA * 1500;		// set each duty value
	EPwm1Regs.CMPB           = dutyB * 1500;
	EPwm2Regs.CMPA.half.CMPA = dutyC * 1500;
}

void initPWM() {

	GpioCtrlRegs.GPAMUX1.bit.GPIO0     = 1;		// Set GPIO0 (PWM - HB1) to output (1A)
	GpioCtrlRegs.GPAMUX1.bit.GPIO1     = 1;		// set GPIO1 (PWM - HB2) to output (1B)
	GpioCtrlRegs.GPAMUX1.bit.GPIO2     = 1;		// set GPIO2 (PWM - HB3) to output (2A)
	SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 1;	    // PWM1 clock enabled (w/system clock)
	asm(" NOP"); asm(" NOP");
	SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 1;	    // PWM2 clock enabled (w/system clock)

	asm(" NOP"); asm(" NOP");				    // wait two system clock cycles

	EPwm1Regs.TBCTL.bit.CTRMODE   = 2;		// up-down count mode
	EPwm1Regs.TBPRD               = 2 * 24 / .032;// resolution of 32 mV; = 1500 (sets fpwm = fclk/(2*1500))
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;		// 1
	EPwm1Regs.TBCTL.bit.CLKDIV    = 0;		// 1

	EPwm1Regs.AQCTLA.bit.CAU = 1;			// clear
	EPwm1Regs.AQCTLA.bit.CAD = 2;			// set
	EPwm1Regs.AQCTLB.bit.CBU = 1;			// clear
	EPwm1Regs.AQCTLB.bit.CBD = 2;			// set

	EPwm1Regs.ETSEL.bit.SOCASEL = 2;		// enable event when counter = max
	EPwm1Regs.ETSEL.bit.SOCAEN  = 1;		// enable SOCA
	EPwm1Regs.ETPS.bit.SOCAPRD  = 1;		// enable at every event
	EPwm1Regs.ETSEL.bit.SOCBSEL = 2;		// enable event when counter = max
	EPwm1Regs.ETSEL.bit.SOCBEN  = 1;		// enable SOCA
	EPwm1Regs.ETPS.bit.SOCBPRD  = 1;		// enable at every event
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	EPwm2Regs.TBCTL.bit.CTRMODE   = 2;		// up-down count mode
	EPwm2Regs.TBPRD               = 2*24/.032;// resolution of 32 mV; = 1500 (sets fpwm = fclk/(2*1500))
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;		// 1
	EPwm2Regs.TBCTL.bit.CLKDIV    = 0;		// 1

	EPwm2Regs.AQCTLA.bit.CAU = 1;			// clear
	EPwm2Regs.AQCTLA.bit.CAD = 2;			// set

	EPwm2Regs.ETSEL.bit.SOCASEL = 2;		// enable event when counter = max
	EPwm2Regs.ETSEL.bit.SOCAEN  = 1;		// enable SOCA
	EPwm2Regs.ETPS.bit.SOCAPRD  = 1;		// enable at every event

	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;	// enable ePWM module clock
	asm(" NOP"); asm(" NOP");				// wait two system clock cycles

	GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1;		// pin set as output

}

void initInterrupt() {

	PieCtrlRegs.PIECTRL.bit.ENPIE  = 1;		// enable the table
	PieCtrlRegs.PIEIER1.bit.INTx7  = 1;		// enable at 1x7 (Timer 0)
	PieVectTable.TINT0             = &timerISR;	// point interrupt to timerISR
	PieCtrlRegs.PIEIER10.bit.INTx4 = 1;		// enable at 10x4 (ADCINT4)
	PieVectTable.ADCINT4           = &AdcISR;	// point to AdcISR (measure voltage/current)

	IER = 0x201;							// Enable INT1 and INT10
	EINT;
}

void initADC() {

	SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;	// Enable ADC clock
	asm(" NOP"); asm(" NOP");				// wait two system clock cycles
	AdcRegs.ADCCTL2.bit.CLKDIV2EN = 1;		// ADC clock = CPU clock/2 (45 MHz)
	AdcRegs.ADCCTL1.bit.ADCPWDN   = 1;		// Power up ADC
	AdcRegs.ADCCTL1.bit.ADCBGPWD  = 1;		// Power up active low
	AdcRegs.ADCCTL1.bit.ADCREFPWD = 1;		// Power up active low reference buffers
	AdcRegs.ADCCTL1.bit.ADCENABLE = 1;		// ADC is enabled

	wait(0.001);							// wait 0.001 seconds

	AdcRegs.ADCSOC0CTL.bit.CHSEL = 0x0;		// SOC0 samples Phase A (ADCINA0)
	AdcRegs.ADCSOC1CTL.bit.CHSEL = 0x1;		// SOC1 samples Phase B	(ADCINA1)
	AdcRegs.ADCSOC2CTL.bit.CHSEL = 0x2;		// SOC2 samples Phase C	(ADCINA2)

	AdcRegs.ADCSOC0CTL.bit.ACQPS = 0x9;		// sample window is 9+1 cycles
	AdcRegs.ADCSOC1CTL.bit.ACQPS = 0x9;
	AdcRegs.ADCSOC2CTL.bit.ACQPS = 0x9;

	AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 0x5;	// trigger from ePWM1 ADCSOCA
	AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 0x6;	// trigger from ePWM1 ADCSOCB
	AdcRegs.ADCSOC2CTL.bit.TRIGSEL = 0x7;	// trigger from ePWM2 ADCSOCA

	AdcRegs.INTSEL3N4.bit.INT4E   = 1;		// ADC INT2 is enabled
	AdcRegs.INTSEL3N4.bit.INT4SEL = 2;		// EOC2 is trigger for ADCINT2

	AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;	// int pulse generation occurs 1 cycle prior to ADC result latching
}

void initQEP(){
	GpioCtrlRegs.GPAMUX2.bit.GPIO20    = 1;	// QEP1A input
	GpioCtrlRegs.GPAMUX2.bit.GPIO21    = 1;	// QEP1B input
	SysCtrlRegs.PCLKCR1.bit.EQEP1ENCLK = 1;
	asm(" NOP"); asm(" NOP");				// wait two system clock cycles
	EQep1Regs.QPOSMAX         = 0xFFFFFFFF;
	EQep1Regs.QEPCTL.bit.QPEN = 1;			// Enable QEP1
	EQep1Regs.QEPCTL.bit.SWI  = 1;			// Reset to zero position
}

interrupt void timerISR(void) {

	// reference command
	if(k<-t_rest/T){
		back = 0;
		r = r;
	}
	if(k <= 0){
		r = r;
	}
	else if(k <= (t_accel)/T) {// acceleration phase
		r = theta_i + 0.5*accel_max*(k*T)*(k*T);
	}
	else if(k <= (t_accel+t_cruise)/T) { // cruise phase
		r = 0.5*(theta_i + theta_f)+vel_max*(k*T - 0.5*(t_f));
	}
	else if((k <= (t_f)/T)){ // deceleration phase
		r = theta_f - .5*accel_max*(t_f - k*T)*(t_f - k*T);
	}
	else if(k<=(t_f+t_rest)/T) { // dwell phase
		r = r;
	}
	else if(k>(t_f+t_rest)/T){ // reverse reference command
		back = 1;
	}
	if(back){
		k--;
	}
	else{
		k++;
	}


	// integral controller
	x1hat_new = x1_hat + T*x2_hat - T*L1*(x1_hat - y);
	x2hat_new = x2_hat - T*alpha*x2_hat + T*beta*u - T*L2*(x1_hat - y);
	sigma_new = sigma + T*(y - r);

	u_new = -K11*x1_hat - K12*x2_hat - K2*sigma;
	if(u_new > V_max){
		u_new = V_max;
	} else if(u_new < -V_max) {
		u_new = -V_max;
	}

	u      = u_new;
	x1_hat = x1hat_new;
	x2_hat = x2hat_new;
	sigma  = sigma_new;

	// voltage vector generator
	if(fabs(u) <= V_max){
		v = fabs(u);
	}
	else{
		v = V_max;
	}
	if(u>=0){
		phi = pi/2.0;
	}
	else{
		phi = -pi/2.0;
	}

	voltageA = 0.5*Vdc + 0.4714 * v * cos(N*x1_hat + phi - (pi/6.0));
	voltageB = 0.5*Vdc + 0.4714 * v * cos(N*x1_hat + phi - (5.0*pi/6.0));
	voltageC = 0.5*Vdc + 0.4714 * v * cos(N*x1_hat + phi + (pi/2.0));

	setVoltage(voltageA, voltageB, voltageC);

	counter = (int32) EQep1Regs.QPOSCNT;
	y       = counter*2.0*pi/4000.0;
	// data logging
	if(!(count2 % size)){
		currentALog[count2/size] = currentA;
		currentBLog[count2/size] = currentB;
		currentCLog[count2/size] = currentC;
		posLog[count2/size]      = y;
	}
	if(count2 < 1000*size) {
		count2++;
	}
	PieCtrlRegs.PIEACK.bit.ACK1 = 1;		// Acknowledge the timer interrupt
}

interrupt void AdcISR(void) {

	currentA = AdcResult.ADCRESULT0;		// Read value for SOC0
	currentB = AdcResult.ADCRESULT1;		// Read value for SOC1
	currentC = AdcResult.ADCRESULT2;		// Read value for SOC2
	if ((currentA< low_thresh) || (currentA > hi_thresh)) {
		GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;		// turn off the PWM power
		asm(" NOP"); asm(" NOP");
	}
	if ((currentB< low_thresh) || (currentB > hi_thresh)) {
		GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;		// turn off the PWM power
		asm(" NOP"); asm(" NOP");
	}
	if ((currentC< low_thresh) || (currentC > hi_thresh)) {
		GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;		// turn off the PWM power
		asm(" NOP"); asm(" NOP");
	}
	currentA = currentA * 3.3/4095.0;
	currentB = currentB * 3.3/4095.0;
	currentC = currentC * 3.3/4095.0;

	AdcRegs.ADCINTFLGCLR.bit.ADCINT4 = 1;	// Clear interrupt 4 flag
	PieCtrlRegs.PIEACK.bit.ACK10     = 1;	// Acknowledge interrupt 10
}

void wait(float32 waitTime) {				// give a time (in seconds) to wait
	int32 cycles = 900000;			// adjusts clock cycles based on system frequency
	while (cycles > 0) {
		cycles--;
	}
}
