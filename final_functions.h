//includes from libraries provided by TI
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "f2802x_common/include/adc.h"
#include "f2802x_common/include/clk.h"
#include "f2802x_common/include/flash.h"
#include "f2802x_common/include/gpio.h"
#include "f2802x_common/include/pie.h"
#include "f2802x_common/include/pll.h"
#include "f2802x_common/include/pwm.h"
#include "f2802x_common/include/timer.h"
#include "f2802x_common/include/sci.h"
#include "f2802x_common/include/wdog.h"
#include "f2802x_common/include/f2802x_examples.h"       // f2802x Examples Headerfile
#include "f2802x_epwm_defines.h"    // useful defines for initialization

//Prototype statements for functions found within this project.
__interrupt void xint1_isr(void);
__interrupt void xint2_isr(void);
__interrupt void xint3_isr(void);
__interrupt void adc_isr(void);
void HRPWM1_Config(Uint16);
void HRPWM2_Config(Uint16);
void HRPWM3_Config(Uint16);
void HRPWM4_Config(Uint16);
void Adc_Config(void);
void scia_echoback_init(SCI_Handle mySci, CLK_Handle myClk);
void scia_fifo_init(SCI_Handle mySci);
void scia_xmit(int a);
void scia_msg(char *msg);
void pinSetup(GPIO_Handle myGpio);
void timerSetup(TIMER_Handle myTimer0);
void adcSetup(ADC_Handle myAdc);
void interruptSetup(GPIO_Handle myGpio, PIE_Handle myPie, TIMER_Handle myTimer0, CPU_Handle myCpu);
void closedControlLoop(void);
short int digitFlag = 0;
void printHomeScreen(char * msgGlobal);
void printMeasurements(void);
void printMeasurementLayout(char * msgGlobal);
void dutyRampStartupQSC(int period);
void MeasureBatVoltage(void);
void printBatVoltage(void);

// Test 1,SCIA  DLB, 8-bit word, baud rate 0x000F, default, 1 STOP bit, no parity
void scia_echoback_init(SCI_Handle mySci, CLK_Handle myClk)
{
    CLK_enableSciaClock(myClk);

    // 1 stop bit,  No loopback
    // No parity,8 char bits,
    // async mode, idle-line protocol
    SCI_disableParity(mySci);
    SCI_setNumStopBits(mySci, SCI_NumStopBits_One);
    SCI_setCharLength(mySci, SCI_CharLength_8_Bits);

    SCI_enableTx(mySci);
    SCI_enableRx(mySci);
    SCI_enableTxInt(mySci);
    SCI_enableRxInt(mySci);

    // SCI BRR = LSPCLK/(SCI BAUDx8) - 1
#if (CPU_FRQ_60MHZ)
    SCI_setBaudRate(mySci, (SCI_BaudRate_e)194);
#elif (CPU_FRQ_50MHZ)
    SCI_setBaudRate(mySci, (SCI_BaudRate_e)162);
#elif (CPU_FRQ_40MHZ)
    SCI_setBaudRate(mySci, (SCI_BaudRate_e)129);
#endif

    SCI_enable(mySci);

    return;
}

void scia_fifo_init(SCI_Handle mySci)
{
    SCI_enableFifoEnh(mySci);
    SCI_resetTxFifo(mySci);
    SCI_clearTxFifoInt(mySci);
    SCI_resetChannels(mySci);
    SCI_setTxFifoIntLevel(mySci, SCI_FifoLevel_Empty);

    SCI_resetRxFifo(mySci);
    SCI_clearRxFifoInt(mySci);
    SCI_setRxFifoIntLevel(mySci, SCI_FifoLevel_4_Words);

    return;
}

void timerSetup(TIMER_Handle myTimer0)
{
    TIMER_stop(myTimer0);

    //1000 Hz (60 MHz SYSCLK)
    TIMER_setPeriod(myTimer0, 6000);

    TIMER_setPreScaler(myTimer0, 0);
    TIMER_reload(myTimer0);
    TIMER_setEmulationMode(myTimer0, TIMER_EmulationMode_StopAfterNextDecrement);

    //timer is started after button press to begin closed loop control
    //TIMER_start(myTimer0);
}

void pinSetup(GPIO_Handle myGpio)
{
    //init GPIO pins for EPwm1, EPwm2, EPwm3, EPwm4
    GPIO_setPullUp(myGpio, GPIO_Number_0, GPIO_PullUp_Disable);
    GPIO_setMode(myGpio, GPIO_Number_0, GPIO_0_Mode_EPWM1A);

    GPIO_setPullUp(myGpio, GPIO_Number_2, GPIO_PullUp_Disable);
    GPIO_setMode(myGpio, GPIO_Number_2, GPIO_2_Mode_EPWM2A);

    GPIO_setPullUp(myGpio, GPIO_Number_4, GPIO_PullUp_Disable);
    GPIO_setMode(myGpio, GPIO_Number_4, GPIO_4_Mode_EPWM3A);

    GPIO_setPullUp(myGpio, GPIO_Number_6, GPIO_PullUp_Disable);
    GPIO_setMode(myGpio, GPIO_Number_6, GPIO_6_Mode_EPWM4A);

    // Initialize GPIO for SCI TX
    GPIO_setPullUp(myGpio, GPIO_Number_29, GPIO_PullUp_Disable);
    GPIO_setMode(myGpio, GPIO_Number_29, GPIO_29_Mode_SCITXDA);

//    // GPIO19 (FLT_RESET) is output
//    GPIO_setMode(myGpio, GPIO_Number_19, GPIO_19_Mode_GeneralPurpose);
//    GPIO_setDirection(myGpio, GPIO_Number_19, GPIO_Direction_Output);
//
//    //set FLT_RESET low initially
//    GPIO_setLow(myGpio, GPIO_Number_19);

    // GPIO3 is CCL_EN
    GPIO_setMode(myGpio, GPIO_Number_3, GPIO_3_Mode_GeneralPurpose);
    GPIO_setDirection(myGpio, GPIO_Number_3, GPIO_Direction_Output);

    //set the CCL_EN low
    GPIO_setLow(myGpio, GPIO_Number_3);

    // GPIO1 is CVS_EN
    GPIO_setMode(myGpio, GPIO_Number_1, GPIO_1_Mode_GeneralPurpose);
    GPIO_setDirection(myGpio, GPIO_Number_1, GPIO_Direction_Output);

    //set the CVS_EN low
    GPIO_setLow(myGpio, GPIO_Number_1);

    // GPIO19 is debug pin
    GPIO_setMode(myGpio, GPIO_Number_19, GPIO_19_Mode_GeneralPurpose);
    GPIO_setDirection(myGpio, GPIO_Number_19, GPIO_Direction_Output);
    //set low
    GPIO_setLow(myGpio, GPIO_Number_19);


    // GPIO5 is input for button 1
    GPIO_setMode(myGpio, GPIO_Number_5, GPIO_5_Mode_GeneralPurpose);
    GPIO_setDirection(myGpio, GPIO_Number_5, GPIO_Direction_Input);
    GPIO_setPullUp(myGpio, GPIO_Number_5, GPIO_PullUp_Enable);

    // GPIO16/32 is input for button 2
    GPIO_setMode(myGpio, GPIO_Number_16, GPIO_16_Mode_GeneralPurpose);
    GPIO_setDirection(myGpio, GPIO_Number_16, GPIO_Direction_Input);
    GPIO_setPullUp(myGpio, GPIO_Number_16, GPIO_PullUp_Enable);

    GPIO_setQualificationPeriod(myGpio, GPIO_Number_5, 0); //changed from 0xff to 0
    GPIO_setQualificationPeriod(myGpio, GPIO_Number_16, 0);
}

void adcSetup(ADC_Handle myAdc) {

    // Initialize the ADC
    ADC_enableBandGap(myAdc);
    ADC_enableRefBuffers(myAdc);
    ADC_powerUp(myAdc);
    ADC_enable(myAdc);
    ADC_setVoltRefSrc(myAdc, ADC_VoltageRefSrc_Int);

    //HV+ ADCB3 HV- B7
    //IHV B6
    //LV+ A3, LV- A0
    //ILV A7

    //only other IO 2 buttons, LCD, Gates, CVS_EN, CCL_En
    //Global En, Not necessary

    //IHV
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_0, ADC_SocChanNumber_B6);    //set SOC0 channel select to ADCIN
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_1, ADC_SocChanNumber_B6);    //set SOC1 channel select to ADCIN

    //VHV+, diff, gain is 8.2
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_2, ADC_SocChanNumber_B3);    //set SOC2 channel select to ADCIN

    //VHV -
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_3, ADC_SocChanNumber_B7);    //set SOC3 channel select to ADCIN

    //VLV +
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_4, ADC_SocChanNumber_A3);    //set SOC4 channel select to ADCIN

    //VLV -
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_5, ADC_SocChanNumber_A0);    //set SOC5 channel select to ADCIN

    //ILV
    ADC_setSocChanNumber (myAdc, ADC_SocNumber_6, ADC_SocChanNumber_A7);    //set SOC5 channel select to ADCIN

    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_0, ADC_SocTrigSrc_CpuTimer_0);    //set SOC0 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_1, ADC_SocTrigSrc_CpuTimer_0);    //set SOC1 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_2, ADC_SocTrigSrc_CpuTimer_0);    //set SOC2 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1, then SOC2
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_3, ADC_SocTrigSrc_CpuTimer_0);    //set SOC2 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1, then SOC2
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_4, ADC_SocTrigSrc_CpuTimer_0);    //set SOC2 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1, then SOC2
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_5, ADC_SocTrigSrc_CpuTimer_0);    //set SOC2 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1, then SOC2
    ADC_setSocTrigSrc(myAdc, ADC_SocNumber_6, ADC_SocTrigSrc_CpuTimer_0);    //set SOC2 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1, then SOC2

    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_0, ADC_SocSampleWindow_7_cycles);   //set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_1, ADC_SocSampleWindow_7_cycles);   //set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_2, ADC_SocSampleWindow_7_cycles);   //set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_3, ADC_SocSampleWindow_7_cycles);   //set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_4, ADC_SocSampleWindow_7_cycles);   //set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_5, ADC_SocSampleWindow_7_cycles);   //set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    ADC_setSocSampleWindow(myAdc, ADC_SocNumber_6, ADC_SocSampleWindow_7_cycles);   //set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)

}

void interruptSetup(GPIO_Handle myGpio, PIE_Handle myPie, TIMER_Handle myTimer0, CPU_Handle myCpu) {

    // Register external interrupt handlers in the PIE vector table
    PIE_registerPieIntHandler(myPie, PIE_GroupNumber_1, PIE_SubGroupNumber_4, (intVec_t)&xint1_isr); //XINT1 button 1
    PIE_registerPieIntHandler(myPie, PIE_GroupNumber_1, PIE_SubGroupNumber_5, (intVec_t)&xint2_isr); //XINT2 button 2
    // Register ADC interrupt handlers in the PIE vector table
//    PIE_registerPieIntHandler(myPie, PIE_GroupNumber_10, PIE_SubGroupNumber_1,
//                              (intVec_t)&adc_isr);

    // Enable XINT1, XINT2 and XINT3 in the PIE
    PIE_enableInt(myPie, PIE_GroupNumber_1, PIE_InterruptSource_XINT_1);
    PIE_enableInt(myPie, PIE_GroupNumber_1, PIE_InterruptSource_XINT_2);
    //PIE_enableInt(myPie, PIE_GroupNumber_12, PIE_InterruptSource_XINT_3);
    CPU_enableInt(myCpu, CPU_IntNumber_1);
    CPU_enableInt(myCpu, CPU_IntNumber_12);

    // Enable ADCINT1 in PIE
    PIE_enableAdcInt(myPie, ADC_IntNumber_1);
    // Enable CPU Interrupt 1
    CPU_enableInt(myCpu, CPU_IntNumber_10);

     //Enable global Interrupts and higher priority real-time debug events:
     CPU_enableGlobalInts(myCpu);
     //Enable Global realtime interrupt DBGM
     CPU_enableDebugInt(myCpu);

     // XINT1 is GPIO5
     GPIO_setExtInt(myGpio, GPIO_Number_5, CPU_ExtIntNumber_1);  // XINT1 is GPIO18 (PWM enable)
     // Configure XINT1
     PIE_setExtIntPolarity(myPie, CPU_ExtIntNumber_1,
                 PIE_ExtIntPolarity_FallingEdge);
     // Enable XINT1
     PIE_enableExtInt(myPie, CPU_ExtIntNumber_1);

     // XINT2 is GPIO16
     GPIO_setExtInt(myGpio, GPIO_Number_16, CPU_ExtIntNumber_2);
     // Configure XINT2
     PIE_setExtIntPolarity(myPie, CPU_ExtIntNumber_2,
                 PIE_ExtIntPolarity_FallingEdge);
     // Enable XINT2
     PIE_enableExtInt(myPie, CPU_ExtIntNumber_2);

     //timer begin
     TIMER_enableInt(myTimer0);

     PIE_registerPieIntHandler(myPie, PIE_GroupNumber_1, PIE_SubGroupNumber_7, (intVec_t)&closedControlLoop);

     // Enable TINT0 in the PIE: Group 1 interrupt 7
     PIE_enableTimer0Int(myPie);

     // Enable CPU int1 which is connected to CPU-Timer 0
     CPU_enableInt(myCpu, CPU_IntNumber_1);

     //timer end
}

void printHomeScreen(char * msgGlobal) {

    //send control char 254
    scia_xmit(254);
    //clear display
    scia_xmit(1);

    //set up the outline of the LCD screen
    msgGlobal = "1: Mode Select\0";
    scia_msg(msgGlobal);

    //send control char 254
    scia_xmit(254);
    //set cursor to first char on second line
    scia_xmit(192);

    //set up the outline of the LCD screen
    msgGlobal = "2: Change Limits\0";
    scia_msg(msgGlobal);
}

void printMeasurementLayout(char * msgGlobal) {

    //send control char 254
    scia_xmit(254);
    //clear display
    scia_xmit(1);

    //send control char 254
    scia_xmit(254);
    //set cursor to position 1
    scia_xmit(128);

    //set up the outline of the LCD screen
    msgGlobal = "ILV:\0";
    scia_msg(msgGlobal);

    //send control char 254
    scia_xmit(254);
    //set cursor to first char on second line
    scia_xmit(192);

    //set up the outline of the LCD screen
    msgGlobal = "ILV:\0";
    scia_msg(msgGlobal);

    //send control char 254
    scia_xmit(254);
    //set cursor to first char on third line
    scia_xmit(148);

    //set up the outline of the LCD screen
    msgGlobal = "ILV: \0";
    scia_msg(msgGlobal);
}
