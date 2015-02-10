#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdarg.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ssi.h"
#include "inc/hw_uart.h"
#include "inc/hw_pwm.h"
#include "driverlib/debug.h"
#include "driverlib/timer.h"
#include "driverlib/fpu.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/ssi.h"
#include "math.h"
#include <string.h>

int SystemMode = 0;
long int currentIndex = 0x00;
void InitConsole(void)
{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	    GPIOPinConfigure(GPIO_PA0_U0RX);
	    GPIOPinConfigure(GPIO_PA1_U0TX);
	    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //enable GPIO port for LED
	    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2); //enable pin for LED PF2

	    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	    IntMasterEnable(); //enable processor interrupts
	    IntEnable(INT_UART0); //enable the UART interrupt
	    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT); //only enable RX and TX interru
	    UARTStdioConfig(0, 115200, 16000000);
}
unsigned char ifStatement(long int var1, unsigned char compareType, long int var2) {
	if (compareType == 0x01){ // ==
		if (var1 == var2) return 0x01;
	}
	if (compareType == 0x02){ // >
		if (var1 > var2) return 0x01;
	}
	if (compareType == 0x03){ // <
		if (var1 < var2) return 0x01;
	}
	if (compareType == 0x04){ // >=
		if (var1 >= var2) return 0x01;
	}
	if (compareType == 0x05){ // <=
		if (var1 <= var2) return 0x01;
	}
	if (compareType == 0x02){ // !=
		if (var1 != var2) return 0x01;
	}
	return 0x00;
}
void nextInstruction() {
	//TODO ADD INSTRUCTIONS HERE TO GRAB NEXT PART;
	currentIndex++;
}
void destroyInternalVariables(long int start, long int end) {
	//TODO ADD INSTRUCTIONS HERE TO KILL LOCAL VARIABLES;
}
void whileLoop(long int var1, unsigned char compareType, long int var2, long int endIndex) {
	long int startIndex = currentIndex;
	while (ifStatement(var1,compareType,var2) == 0x01) {
		int i;
		for (i = 0; i < endIndex - currentIndex; i++) {
			nextInstruction();
		}
		destroyInternalVariables(startIndex, endIndex);
		currentIndex = startIndex + 0x01;
	}

}
uint32_t getADC_Channel(uint32_t portFamily, uint32_t portNumber) {
	if (portFamily == GPIO_PORTE_BASE && portNumber == GPIO_PIN_3) {
		return ADC_CTL_CH0;
	}
	if (portFamily == GPIO_PORTE_BASE && portNumber == GPIO_PIN_2) {
		return ADC_CTL_CH1;
	}
	if (portFamily == GPIO_PORTE_BASE && portNumber == GPIO_PIN_1) {
		return ADC_CTL_CH2;
	}
	if (portFamily == GPIO_PORTE_BASE && portNumber == GPIO_PIN_0) {
		return ADC_CTL_CH3;
	}
	if (portFamily == GPIO_PORTD_BASE && portNumber == GPIO_PIN_3) {
		return ADC_CTL_CH4;
	}
	if (portFamily == GPIO_PORTD_BASE && portNumber == GPIO_PIN_2) {
		return ADC_CTL_CH5;
	}
	if (portFamily == GPIO_PORTD_BASE && portNumber == GPIO_PIN_1) {
		return ADC_CTL_CH6;
	}
	if (portFamily == GPIO_PORTD_BASE && portNumber == GPIO_PIN_0) {
		return ADC_CTL_CH7;
	}
	if (portFamily == GPIO_PORTE_BASE && portNumber == GPIO_PIN_4) {
		return ADC_CTL_CH8;
	}
	if (portFamily == GPIO_PORTE_BASE && portNumber == GPIO_PIN_5) {
		return ADC_CTL_CH9;
	}
	if (portFamily == GPIO_PORTB_BASE && portNumber == GPIO_PIN_4) {
		return ADC_CTL_CH10;
	}
	if (portFamily == GPIO_PORTB_BASE && portNumber == GPIO_PIN_5) {
		return ADC_CTL_CH11;
	}
	return 0xFFFFFF;
}
void setPort(uint32_t portFamily, uint32_t portNumber, char portType) {
	if (portFamily == GPIO_PORTA_BASE) SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	if (portFamily == GPIO_PORTB_BASE) SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	if (portFamily == GPIO_PORTC_BASE) SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	if (portFamily == GPIO_PORTD_BASE) SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	if (portFamily == GPIO_PORTE_BASE) SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	if (portFamily == GPIO_PORTF_BASE) SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	if (portType == 0x00) { //Digital Input
		GPIOPinTypeGPIOInput(portFamily, portNumber);
		GPIOPadConfigSet(portFamily,portNumber,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	}
	if (portType == 0x01) { //Digital Output
		GPIOPinTypeGPIOOutput(portFamily, portNumber);
	}
	if (portType == 0x02) { //Analog Input
		SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
		GPIOPinTypeADC(portFamily, portNumber);
		ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
		ADCSequenceStepConfigure(ADC0_BASE, 3, getADC_Channel(portFamily, portNumber), getADC_Channel(portFamily, portNumber) | ADC_CTL_IE | ADC_CTL_END);
		ADCSequenceEnable(ADC0_BASE, 3);
		ADCIntClear(ADC0_BASE, 3);
	}
}
void InitI2C0(void)
{
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}
void InitSSI(uint32_t SSIBase, uint32_t CSFamily, uint32_t CSPort, uint32_t bitRate, uint32_t bitWidth) {
	setPort(CSFamily, CSPort, 0x01);
	SSIConfigSetExpClk(SSIBase, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, bitRate, bitWidth);
	SSIEnable(SSIBase);
}
void SSISend(uint32_t SSIBase, uint32_t CSFamily, uint32_t CSPort, uint32_t values[]){
		int k=0, i=0;
		GPIOPinWrite(CSFamily, CSPort, 0x00);
	  while (values[k]){
	    for( i=23; i >= 0; i--){
	      volatile uint8_t convert = 0xC0 ;//0b11000000;
	      if((values[k] >> i) & 0x1 != 0){
	        convert = 0xF8;// 0b11111000;
	      }
	      SSIDataPut(SSIBase, convert);
	    }
	    k++;
	  }
}
void SSIReceive(uint32_t SSIBase, uint32_t CSFamily, uint32_t CSPort, uint32_t data[]){
	int idx=0;

	SSIIntClear(SSIBase, 0x3); //SSI0_BASE
	GPIOPinWrite(CSFamily, CSPort, 0x00);
	uint32_t v = SSIIntStatus(SSIBase, 1);
	while(data[idx]) {

		SSIDataGetNonBlocking(SSIBase, &data[idx]) ;
		idx++;
	}
	//GPIOPinWrite(CSFamily, CSPort, 0x00);
}
void endSSI(uint32_t CSFamily, uint32_t CSPort) {
	GPIOPinWrite(CSFamily, CSPort, CSPort);
}
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...)
{
    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);

    //stores list of variable number of arguments
    va_list vargs;

    //specifies the va_list to "open" and the last fixed argument
    //so vargs knows where to start looking
    va_start(vargs, num_of_args);

    //put data to be sent into FIFO
    I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));

    //if there is only one argument, we only need to use the
    //single send I2C function
    if(num_of_args == 1)
    {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));

        //"close" variable argument list
        va_end(vargs);
    }

    //otherwise, we start transmission of multiple bytes on the
    //I2C bus
    else
    {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));

        //send num_of_args-2 pieces of data, using the
        //BURST_SEND_CONT command of the I2C module
        uint8_t i;
        for(i = 1; i < (num_of_args - 1); i++)
        {
            //put next piece of data into I2C FIFO
            I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));
            //send next data that was just placed into FIFO
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);

            // Wait until MCU is done transferring.
            while(I2CMasterBusy(I2C0_BASE));
        }

        //put last piece of data into I2C FIFO
        I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));
        //send next data that was just placed into FIFO
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));

        //"close" variable args list
        va_end(vargs);
    }
}
//read specified register on slave device
uint32_t I2CReceive(uint32_t PORT, uint32_t slave_addr, uint8_t reg)
{
    //specify that we are writing (a register address) to the
    //slave device
    I2CMasterSlaveAddrSet(PORT, slave_addr, false);

    //specify register to be read
    I2CMasterDataPut(PORT, reg);

    //send control byte and register address byte to slave device
    I2CMasterControl(PORT, I2C_MASTER_CMD_BURST_SEND_START);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(PORT));

    //specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(PORT, slave_addr, true);

    //send control byte and read from the register we
    //specified
    I2CMasterControl(PORT, I2C_MASTER_CMD_SINGLE_RECEIVE);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(PORT));

    //return data pulled from the specified register
    return I2CMasterDataGet(PORT); //I2C0_BASE
}
void initPWM0(unsigned long Frequency){
		unsigned long ulPeriod;
	   //Configure PWM Clock to match system
	   SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

	   // Enable the peripherals used by this program.
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	    ulPeriod = SysCtlClockGet() / Frequency; //PWM frequency 4MHz
	    //Configure PF1,PF2,PF3 Pins as PWM
	    GPIOPinConfigure(GPIO_PB6_M0PWM0);
	    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_0);
	    //Configure PWM Options
	    //PWM_GEN_2 Covers M1PWM4 and M1PWM5
	    //PWM_GEN_3 Covers M1PWM6 and M1PWM7 See page 207 4/11/13 DriverLib doc
	    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
	    //Set the Period (expressed in clock ticks)
	    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ulPeriod);
	    //Set PWM duty-50% (Period /2)
	    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,ulPeriod/2);
	    // Enable the PWM generator
	    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
	    // Turn on the Output pins
	    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
}
//*****************************************************************************
//
//! \brief Float to ASCII
//!
//! Converts a floating point number to ASCII. Note that buf must be
//! large enough to hold
//!
//! \param f is the floating point number.
//! \param buf is the buffer in which the resulting string is placed.
//! \return None.
//!
//! \par Example:
//! ftoa(3.14) returns "3.14"
//!
//
//*****************************************************************************
void ftoa(float f,char *buf)
{
    int pos=0,ix,dp,num;
    if (f<0)
    {
        buf[pos++]='-';
        f = -f;
    }
    dp=0;
    while (f>=10.0)
    {
        f=f/10.0;
        dp++;
    }
    for (ix=1;ix<8;ix++)
    {
            num = (int)f;
            f=f-num;
            if (num>9)
                buf[pos++]='#';
            else
                buf[pos++]='0'+num;
            if (dp==0) buf[pos++]='.';
            f=f*10.0;
            dp--;
    }
}
void pollLinearArrayT(){
	//sh pin TX1
	//ICG pin RX1
	//phiM pin TX3
	//OS pin A0
	uint32_t pui32ADC0Value[1];
	uint32_t Spectra[1546];
	float tempnm = 305.30;
	int k =1596;
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5); //sh pin
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7); //ICG pin
	initPWM0(4000000); //phiM pin (PWM0_N = PB6)
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0); //sh pin
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 1); //ICG pin
	SysCtlDelay(100); //1250ns
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0); //ICG LOW 100ns before SH
	SysCtlDelay(8); //100ns
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 1); // SH HIGH
	SysCtlDelay(80); //1000ns
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0); // SH LOW
	SysCtlDelay(4); //50ns
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0); //ICG HIGH duration of DATA
	int i = 0;
	for (i = 0; i < 1546; i++){
	SysCtlDelay(20); //250ns: will take two 4MHz clock cycles before data can be appended.
	ADCProcessorTrigger(ADC0_BASE, 3);
	 while(!ADCIntStatus(ADC0_BASE, 3, false))
		        {
		        }
	 ADCIntClear(ADC0_BASE, 3);
	 ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);
	 Spectra[i] = pui32ADC0Value[0];
	}
	//2650ns passed
	//1546 elements
	//each element is 20 cycles or 250ns
	//386500ns for all the elements
	//391800ns total time -> 0.0003918 s -> 2552.3226135783563042368555385401 Hz total time
	//while ()
	UARTprintf("ALMOST DONE!");
	for (i = 0; i < k; i++) {
	char j[8];
	ftoa(tempnm,j);
	//int tnm = (int)(tempnm*100);
	UARTprintf("%s,%4d\n", j,(Spectra[i]));
	}
}

char UART_Receive(uint32_t COM) {
	if (UARTCharsAvail(COM)){
	    		return UARTCharGet(COM);
	    	}
	return 0x00;
}
char readDigitalPort(uint32_t portFamily, uint32_t portNumber) {
	if ( (GPIOPinRead(portFamily,portNumber) & portNumber) == 0x00) return 0x00;
	return 0x01;
}
uint32_t readAnalogPort(uint32_t portFamily, uint32_t portNumber) {
	uint32_t pui32ADC0Value[12];
	ADCProcessorTrigger(ADC0_BASE, 3);
	while(!ADCIntStatus(ADC0_BASE, 3, false)){ }
	ADCIntClear(ADC0_BASE, 3);
	ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);
	return pui32ADC0Value[getADC_Channel(portFamily, portNumber)];
}
void writePort(uint32_t portFamily, uint32_t portNumber, char state) {
	if (state == 0x01) {
	GPIOPinWrite(portFamily,portNumber, portNumber);
	}
	else {
		GPIOPinWrite(portFamily,portNumber, 0x00);
	}
}
void forLoop(long int start, long int end, long int increment, long int endIndex) {
	long int startIndex = currentIndex;
	for (start = start; start != end; start+=increment) {
		int i;
		for (i = 0; i < endIndex - currentIndex; i++) {
			nextInstruction();
		}
		destroyInternalVariables(startIndex, endIndex);
		currentIndex = startIndex + 0x01;
	}
}
void delay(long ns125) {
	SysCtlDelay(ns125); //Delay in intervals of 12.5ns
}


void pollMovable(){
	SystemMode = 2;
	const double nm = 405; // Known Calibration source Frequency
	const double x = 37.65; // Distance of point above the "Screen"
	const double D = 50.00; // distance to the "Screen"
	const double degreesperstep = 18; // Degrees per step for the stepper motor
	const double microstepping = 4; // This allows for you to determine how often you want the Sample to be taken (inverse microstep)
	const double mmPerRotation = 2.72; // This is the distance between the two teeth of the spindle which moves the sensor up and down
	const double heightofTray = 37.6; // Under estimate this a bit
	const double stepsperrotation = 360/degreesperstep;
	const double microstepsuntilend = heightofTray/mmPerRotation * stepsperrotation * (microstepping);
	const double starty = 32.35;
	const double d = (nm)/sin(atan(x/D));
	//int datafile = 0;
	double cnt = 0;
	///char buf[30];
	float tempnm = 0;
	double currenty = 0;
	const double zero = starty-x;
	//char ksk = '0';
	uint32_t pui32ADC0Value[1];
	//UARTprintf(" Starting to Move\n");
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlDelay(3);
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5); //STEPPER
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4); //DIR
	SysCtlDelay(40);
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4); // STEPPER_DIR
	while (cnt < microstepsuntilend) {
		cnt++;
		ADCProcessorTrigger(ADC0_BASE, 3);
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0); // STEPPER

		SysCtlDelay(10000); //125000ns = 125us
	        while(!ADCIntStatus(ADC0_BASE, 3, false))
	        {
	        }
	        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5); // STEPPER
	        SysCtlDelay(10000); //125000ns = 125us
	        ADCIntClear(ADC0_BASE, 3);
	        ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);
	        currenty = zero+((cnt/microstepsuntilend)  * heightofTray);
	        tempnm = (float)d*(float)sin(atan((x+currenty)/D));
	        char j[8];
	        ftoa(tempnm,j);
	        //int tnm = (int)(tempnm*100);
	        UARTprintf("%s,%4d\n", j,(pui32ADC0Value[0]));
	        //UARTprintf(",", pui32ADC0Value[0]);
	        //UARTprintf("%4d\n", );
	}

	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0); // STEPPER_DIR
	int i = 0;
	for (i = 0; i < microstepsuntilend; i++) {
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5); // STEPPER
		SysCtlDelay(10000); //125000ns = 125us
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0); // STEPPER
		SysCtlDelay(10000); //125000ns = 125us
	}
	UARTprintf("$\n");
	SystemMode = 0;

}
void startStepper(void){
		const double degreesperstep = 18; // Degrees per step for the stepper motor
		const double microstepping = 4; // This allows for you to determine how often you want the Sample to be taken (inverse microstep)
		const double mmPerRotation = 2.72; // This is the distance between the two teeth of the spindle which moves the sensor up and down
		const double heightofTray = 37.6; // Under estimate this a bit
		const double stepsperrotation = 360/degreesperstep;
		const double microstepsuntilend = heightofTray/mmPerRotation * stepsperrotation * (microstepping);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
			SysCtlDelay(3);
		GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5); //STEPPER
			GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4); //DIR
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0); // STEPPER_DIR
	int i = 0;
	for (i = 0; i < microstepsuntilend; i++) {
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5); // STEPPER
		SysCtlDelay(20000); //125000ns = 125us*2
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0); // STEPPER
		SysCtlDelay(20000); //125000ns = 125us*2
	}
}
void SPITEST(){
	uint32_t val2 = {0x9F};
	uint32_t vals[32];
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinConfigure(GPIO_PB4_SSI2CLK);
	GPIOPinConfigure(GPIO_PB5_SSI2FSS);
	GPIOPinConfigure(GPIO_PB6_SSI2RX);
	GPIOPinConfigure(GPIO_PB7_SSI2TX);
	GPIOPinTypeSSI(GPIO_PORTB_BASE,GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
	SSIConfigSetExpClk(SSI2_BASE,SysCtlClockGet(),SSI_FRF_MOTO_MODE_3,SSI_MODE_MASTER,25000000,8);
	SSIEnable(SSI2_BASE);

	while(1) {
		SSIDataPutNonBlocking(SSI2_BASE, 0x0000009F);
		SSIDataGetNonBlocking(SSI0_BASE, &vals[0]);
		SSIDataGetNonBlocking(SSI0_BASE, &vals[1]);
		SSIDataGetNonBlocking(SSI0_BASE, &vals[2]);
		UARTprintf("Responce 1: %d\n",vals[0]);
		UARTprintf("Responce 2: %d\n",vals[1]);
		UARTprintf("Responce 3: %d\n",vals[2]);
		SysCtlDelay(2000000); //12500000ns = 12500us*2
	}

}
main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
    InitConsole();
    //InitI2C0();
    UARTprintf(" Startup!\n");
    //startStepper();
   // SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
   // 			SysCtlDelay(3);
   // 		GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5); //STEPPER
   // while(1)
   //     {
   // 	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5); // STEPPER
   // 			SysCtlDelay(200000); //125000ns = 125us*2
   // 			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0); // STEPPER
   // 			SysCtlDelay(200000); //125000ns = 125us*2
   //     }
    //pollLinearArrayT(); //For Linear Array
    SPITEST();
    UARTprintf(" :)\n");
    while(1)
    {
    	if (UARTCharsAvail(UART0_BASE)){
    		if (UARTCharGet(UART0_BASE) == '1') {
    			UARTprintf(" Starting to Move\n");
    			SystemMode = 1;
    		}
    	}
    	if (SystemMode == 1) {
    		//pollMovable();
    		pollLinearArrayT(); //For Linear Array
    	}
    }
}

/*ADC read:
 	 	ADCProcessorTrigger(ADC0_BASE, 3);

        while(!ADCIntStatus(ADC0_BASE, 3, false))
        {
        }

        ADCIntClear(ADC0_BASE, 3);
        ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);
        UARTprintf("AIN0 = %4d\r", pui32ADC0Value[0]);

        SysCtlDelay(SysCtlClockGet() / 120);
 */
