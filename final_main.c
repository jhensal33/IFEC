/*
 * Author: Jeff Hensal
 *
 * Purpose: Control bidirectional dc/dc converter using
 * single phase-shift control scheme to compete in the 2018
 * IEEE International Future Energy Challenge. Includes a
 * user interface involving buttons and LCD display. This
 * was the code used for the final competition in Beijing, China.
 *
 * Last modified: 07/20/2017
 */

/*
 * this is the main program including interface, timer
 * interrupts, and the main control loop code
 * functions write_volt write_current, and adc_isr  are deprecated in v2
 * TODO: further subdivide this file
*/

//user defined includes
#include "final_functions.h"
#include "final_variables.h"

void main(void)
{
//----------------------handler setup: DO NOT MODIFY--------------------------------------------------

    // Initialize all the handles needed for PWM_setPhase(myPwm2this application
    myAdc = ADC_init((void *)ADC_BASE_ADDR, sizeof(ADC_Obj));
    myClk = CLK_init((void *)CLK_BASE_ADDR, sizeof(CLK_Obj));
    myFlash = FLASH_init((void *)FLASH_BASE_ADDR, sizeof(FLASH_Obj));
    myGpio = GPIO_init((void *)GPIO_BASE_ADDR, sizeof(GPIO_Obj));
    myPie = PIE_init((void *)PIE_BASE_ADDR, sizeof(PIE_Obj));
    myPll = PLL_init((void *)PLL_BASE_ADDR, sizeof(PLL_Obj));
    myPwm1 = PWM_init((void *)PWM_ePWM1_BASE_ADDR, sizeof(PWM_Obj));
    myPwm2 = PWM_init((void *)PWM_ePWM2_BASE_ADDR, sizeof(PWM_Obj));
    myPwm3 = PWM_init((void *)PWM_ePWM3_BASE_ADDR, sizeof(PWM_Obj));
    myPwm4 = PWM_init((void *)PWM_ePWM4_BASE_ADDR, sizeof(PWM_Obj));
    mySci = SCI_init((void *)SCIA_BASE_ADDR, sizeof(SCI_Obj));
    myTimer0 = TIMER_init((void *)TIMER0_BASE_ADDR, sizeof(TIMER_Obj));
    myWDog = WDOG_init((void *)WDOG_BASE_ADDR, sizeof(WDOG_Obj));

    //Perform basic system initialization
    WDOG_disable(myWDog);
    CLK_enableAdcClock(myClk);
    (*Device_cal)();

    //Select the internal oscillator 1 as the clock source
    CLK_setOscSrc(myClk, CLK_OscSrc_Internal);

    // Setup the PLL for x12 /2 which will yield 60Mhz = 10Mhz * 12 / 2
    PLL_setup(myPll, PLL_Multiplier_12, PLL_DivideSelect_ClkIn_by_2);

    //Disable the PIE and all interrupts
    PIE_disable(myPie);
    PIE_disableAllInts(myPie);
    CPU_disableGlobalInts(myCpu);
    CPU_clearIntFlags(myCpu);

    //Setup a debug vector table and enable the PIE
    PIE_setDebugIntVectorTable(myPie);
    PIE_enable(myPie);

#ifdef _FLASH
    // Copy time critical code and Flash setup code to RAM
    // This includes the following ISR functions: EPwm1_timer_isr(), EPwm2_timer_isr()
    // and FLASH_setup();
    // The  RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart
    // symbols are created by the linker. Refer to the F2280270.cmd file.
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);

    // Call Flash Initialization to setup flash waitstates
    // This function must reside in RAM
    FLASH_setup(myFlash);
#endif // end #ifdef _FLASH

//----------------------------------end of DO NOT MODIFY---------------------------------------------------------------------

    //configure pins for PWM or GPIOs
    pinSetup(myGpio);
    //configure ADC and choose pins to be converted
    adcSetup(myAdc);

    //initialize SCI module
    scia_echoback_init(mySci, myClk);
    //Initialize the SCI FIFO
    scia_fifo_init(mySci);

    //send control char 254
    scia_xmit(254);
    //clear display
    scia_xmit(1);

    //send control char 254
    scia_xmit(124);
    //set backlight level between 128-157
    scia_xmit(150);

    //send control char 254
    scia_xmit(254);
    //set cursor to position 1
    scia_xmit(128);

    //set the default state
    state = standby;

    CLK_disableTbClockSync(myClk);
    //configure the PWM signals @ 500 kHz. AQs still need set to generate signals after this configuration (done in ISR)
    //phase and deadbands are configured to 0 intially
    HRPWM1_Config(170);
    HRPWM2_Config(170);
    HRPWM3_Config(170);
    HRPWM4_Config(170);
    CLK_enableHrPwmClock(myClk);

    //do not modify
    PWM_Obj *pwm1 = (PWM_Obj *)myPwm1;
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;
    pwm1->HRMSTEP = 255;
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;
    //do not modify

    //set PWM parameters
    PWM_Obj *pwm3 = (PWM_Obj *)myPwm3;
    pwm3->TBPHSHR = maxHRPWM << 8;

    PWM_Obj *pwm4 = (PWM_Obj *)myPwm4;
    pwm4->TBPHSHR = maxHRPWM << 8;

    PWM_setPhase(myPwm2, 2);     // ePWM2 is in sync with ePWM1 when phase is set to 2

    PWM_setPhase(myPwm3, initialEPWMcharging);   //Phase for FB side
    PWM_setPhase(myPwm4, initialEPWMcharging);   //Phase for FB side

    PWM_setDeadBandRisingEdgeDelay(myPwm1, 7);  //DB for QSC // Q2 turn off to Q1 turn on
    PWM_setDeadBandRisingEdgeDelay(myPwm2, 10);  //DB for QSC
    PWM_setDeadBandRisingEdgeDelay(myPwm3, 13);  //DB for FB side
    PWM_setDeadBandRisingEdgeDelay(myPwm4, 13);  //DB for FB side
    //end of PWM parameters

    CLK_enableTbClockSync(myClk);

    //GDEN and FLT_RESET
    GPIO_setHigh(myGpio, GPIO_Number_19);

    t = 0;
    buttonCount = 0;
    stack[3] = 0;
    stack[2] = 0;
    stack[1] = 0;
    stack[0] = 0;

    interruptSetup(myGpio, myPie, myTimer0, myCpu);
    timerSetup(myTimer0);
    GPIO_setLow(myGpio, GPIO_Number_1);

    while(1){}
}

__interrupt void xint1_isr(void)
{
    if(state == standby) {

        state = modeSelect;

        //send control char 254
        scia_xmit(254);
        //clear display
        scia_xmit(1);

        msgGlobal = "1: Charge Mode\0";
        scia_msg(msgGlobal);

        //send control char 254
        scia_xmit(254);
        //set cursor to first char on second line
        scia_xmit(192);

        msgGlobal = "2: Discharge Mode\0";
        scia_msg(msgGlobal);

    }
    else if(state == modeSelect) { //enter charge mode, set desired icharge

        parameter = icharge;
        state = changeLimits;

        //send control char 254
        scia_xmit(254);
        //clear display
        scia_xmit(1);

        msgGlobal = "Set Icharge\0";
        scia_msg(msgGlobal);

        //send control char 254
        scia_xmit(254);
        //set cursor to first char on second line
        scia_xmit(192);

        msgGlobal = "Icharge: \0";
        scia_msg(msgGlobal);

        digitFlag = FIRSTDIGIT; //position of the first digit
        digitOne = 0;

        //send control char 254
        scia_xmit(254);
        //set cursor to first digit
        scia_xmit(digitFlag);

        //wait for button clicks
        scia_xmit(digitOne + 48);
    }
    else if(state == changeLimits) {

        switch(parameter) {

            case vbatMin:

            break;

            case vbatMax:

            break;

            case icharge: //this case will fall through to the ichareMax

            case ichargeMax:

                //TODO change this to restrict the ILV (output current) once the sensors added to circuit (2.5 is the limit for IHV)
                if(buttonCount == 0) {

                    digitOne++;

                    if(digitOne == 3) {
                        digitOne = 0;
                    }
                }

                if(buttonCount == 1) {

                    digitOne++;

                    if(digitOne == 10) {
                        digitOne = 0;
                    }
                }

            break;

            case idischargeMax:

            break;

            case idischarge:

            break;

        }

//        digitOne++;
//
//        if(digitOne == 10) {
//            digitOne = 0;
//        }
//
//        if(digitOne == -1) {
//            digitOne = 9;
//        }

        //send control char 254
        scia_xmit(254);
        //set cursor to the current digit being changed
        scia_xmit(digitFlag);

        //display the digit after it has been changed
        scia_xmit(digitOne + 48);
    }
    else if(state == chargeDisplay) {

        buttonCount++;

        if(buttonCount == 1) {

        currentSetPointCharging = Icharge;

        printMeasurementLayout(msgGlobal);

        //Measure the battery voltage to determine initial PS
        MeasureBatVoltage();
        //InitialLV = 48;

        //Determine initial PS
        if(InitialLV < 40.5){
            initialEPWMcharging = 8;
            startingHRcharging = 0;
        }
        else if(InitialLV >= 40.5 && InitialLV < 41.5){
            initialEPWMcharging = 6;
            startingHRcharging = 80;
        }
        else if(InitialLV >= 41.5 && InitialLV < 43){
            initialEPWMcharging = 2;
            startingHRcharging = 119;
        }
        else if(InitialLV >= 43 && InitialLV < 45){
            initialEPWMcharging = 0;
            startingHRcharging = 119;
        }
        else if(InitialLV >= 45 && InitialLV < 47){
            initialEPWMcharging = 168;
            startingHRcharging = 50;
        }
        else if(InitialLV >= 47){
            initialEPWMcharging = 168;
            startingHRcharging = 90;
        }

        //turn on CVS_EN
        GPIO_toggle(myGpio, GPIO_Number_19);
        GPIO_setHigh(myGpio, GPIO_Number_1);

        count = 0;
        for(; count < 3000; count++) {}

        CLK_disableTbClockSync(myClk);
        //set PWM parameters
        PWM_setPhase(myPwm2, 2);     // ePWM2 is in sync with ePWM1 when phase is set to 2
        PWM_setPhase(myPwm3, initialEPWMcharging);   //Phase for FB side
        PWM_setPhase(myPwm4, initialEPWMcharging);   //Phase for FB side

        PWM_Obj *pwm3 = (PWM_Obj *)myPwm3;
        pwm3->TBPHSHR = startingHRcharging << 8;
        PWM_Obj *pwm4 = (PWM_Obj *)myPwm4;
        pwm4->TBPHSHR = startingHRcharging << 8;

        ePWMphase = initialEPWMcharging;
        integral = 0;
        //end of PWM parameters

        //charging startup sequence begins, QSC ramps up to 50% duty incrementally
        GPIO_toggle(myGpio, GPIO_Number_19);
        dutyRampStartupQSC(period);

        //Delay b/w ramp up and FB turn on
        count = 0;
        for(; count < 600; count++) {}
        }

        else if (buttonCount == 2) {
            //FB turns on
            PWM_setActionQual_Zero_PwmA(myPwm3, PWM_ActionQual_Clear);
            PWM_setActionQual_CntUp_CmpA_PwmA(myPwm4, PWM_ActionQual_Clear);
            PWM_setActionQual_CntUp_CmpB_PwmA(myPwm3, PWM_ActionQual_Clear);
            PWM_setCounterMode(myPwm3, PWM_CounterMode_Up);     // Count up
            PWM_setCounterMode(myPwm4, PWM_CounterMode_Up);     // Count up

            count = 0;
            for(; count < 20; count++) {}

            GPIO_toggle(myGpio, GPIO_Number_19);
            PWM_setActionQual_Zero_PwmA(myPwm3, PWM_ActionQual_Set);
            PWM_setActionQual_CntUp_CmpA_PwmA(myPwm4, PWM_ActionQual_Set);
            PWM_setActionQual_CntUp_CmpB_PwmA(myPwm3, PWM_ActionQual_Set);
        }

        else if (buttonCount == 3) {

            //reset closed loop control terms
            ePWMphase = initialEPWMcharging;
            HRphase = maxHRPWM;
            integral = 0;

            //ramp up the HRPS
            GPIO_toggle(myGpio, GPIO_Number_19);
            while(startingHRcharging < 119) {

                newFlag = 0;
                for(; newFlag < 300; newFlag++) {}

                startingHRcharging++;

                PWM_Obj *pwm3 = (PWM_Obj *)myPwm3;
                pwm3->TBPHSHR = startingHRcharging << 8;
                PWM_Obj *pwm4 = (PWM_Obj *)myPwm4;
                pwm4->TBPHSHR = startingHRcharging << 8;
            }

            //begin the closedControlLoop function
            //GPIO_toggle(myGpio, GPIO_Number_19);
            measureIndex = 0;
            first32Flag = 0;
            TIMER_start(myTimer0);

        }

        //Phase Rampdown, Turn off Full Bridge, Turn of CVS Enable, Turn off QSC
        else if(buttonCount == 4) {

            //stop control loop
            //GPIO_toggle(myGpio, GPIO_Number_19);
            TIMER_stop(myTimer0);

            //phase ramp down to ensure mitigate transient effects
            if(ePWMphase >= initialEPWMcharging && ePWMphase <= phaseShiftMin) {
                //do nothing
                GPIO_toggle(myGpio, GPIO_Number_19);
                GPIO_toggle(myGpio, GPIO_Number_19);
            }
            else {
                GPIO_toggle(myGpio, GPIO_Number_19);

                // RAMPDOWN //
                while(ePWMphase != initialEPWMcharging) {

                    //Delay to determine speed of rampdown
                    testCount = 0;
                    for(; testCount < 300; testCount++) {}

                    ePWMphase++;

                    if(ePWMphase > 169) {
                        ePWMphase = 0;
                    }

                    EPwm3Regs.TBPHS.half.TBPHS = ePWMphase;
                    EPwm4Regs.TBPHS.half.TBPHS = ePWMphase;

                    PWM_Obj *pwm3 = (PWM_Obj *)myPwm3;
                    pwm3->TBPHSHR = startingHRcharging << 8;
                    PWM_Obj *pwm4 = (PWM_Obj *)myPwm4;
                    pwm4->TBPHSHR = startingHRcharging << 8;
                } // end phase ramp down

            }

            //Delay between end of rampdown and turn off of FB
            testCount = 0;
            for(; testCount < 200; testCount++) {}

            //turn off FB PWM below
            //clear AQs and delay 1 PWM cycle to allow the pins to fall low
            PWM_setActionQual_Zero_PwmA(myPwm3, PWM_ActionQual_Clear);
            PWM_setActionQual_CntUp_CmpA_PwmA(myPwm4, PWM_ActionQual_Clear);
            PWM_setActionQual_CntUp_CmpB_PwmA(myPwm3, PWM_ActionQual_Clear);
            //GPIO_toggle(myGpio, GPIO_Number_19);

            //Delay to let FB gates turn off and b/w FB turn off and CVS turn off
            testCount = 0;
            for(; testCount < 100; testCount++) {}

            //stop FB PWM clocks
            PWM_setCounterMode(myPwm3, PWM_CounterMode_Stop);
            PWM_setCounterMode(myPwm4, PWM_CounterMode_Stop);

            //turn off CVS_EN
            GPIO_setLow(myGpio, GPIO_Number_1);
            GPIO_toggle(myGpio, GPIO_Number_19);

            //Delay between turning CVS off and turning QSC PWM off
            testCount = 0;
            for(; testCount < 1200000; testCount++) {}

            //turn off QSC PWM
            //clear AQs and delay 1 PWM cycle to allow the pins to fall low
            PWM_setActionQual_Zero_PwmA(myPwm1, PWM_ActionQual_Clear);
            PWM_setActionQual_CntUp_CmpA_PwmA(myPwm1, PWM_ActionQual_Clear);

            PWM_setActionQual_Zero_PwmA(myPwm2, PWM_ActionQual_Clear);
            PWM_setActionQual_CntUp_CmpA_PwmA(myPwm2, PWM_ActionQual_Clear);

            testCount = 0;
            for(; testCount < 30000; testCount++) {}  //in theory, 100 us delay

            //stop PWM on QSC
            PWM_setCounterMode(myPwm1, PWM_CounterMode_Stop);
            PWM_setCounterMode(myPwm2, PWM_CounterMode_Stop);

            CLK_disableTbClockSync(myClk);
            //ensure the counters are not at 0 or CmpA
            PWM_setCount(myPwm1, 2);
            PWM_setCount(myPwm2, 2);
            PWM_setCount(myPwm3, 2);
            PWM_setCount(myPwm4, 2);

            //once sure PWM pins are low, return AQs to normal while counter is stopped
            PWM_setActionQual_CntUp_CmpA_PwmA(myPwm3, PWM_ActionQual_Clear);
            PWM_setActionQual_Zero_PwmA(myPwm4, PWM_ActionQual_Clear);
            //PWM_setActionQual_CntUp_CmpA_PwmA(myPwm4, PWM_ActionQual_Set);
            //PWM_setActionQual_CntUp_CmpB_PwmA(myPwm3, PWM_ActionQual_Set);
            CLK_enableTbClockSync(myClk);

            state = standby;
            parameter = vbatMin;

            printHomeScreen(msgGlobal);

            buttonCount = 0;
        }

    }
    else if(state == dischargeDisplay) {

        buttonCount++;

        if(buttonCount == 1) {

            //currentSetPointDischarging = Idischarge;

            //printMeasurementLayout(msgGlobal);

            //set PWM parameters
            CLK_disableTbClockSync(myClk);
            PWM_setCounterMode(myPwm1, PWM_CounterMode_Up);     // Count up
            PWM_setPhase(myPwm2, 2);     // ePWM2 is in sync with ePWM1 when phase is set to 2
            PWM_setPhase(myPwm3, initialEPWMdischarging);   //Phase for FB side
            PWM_setPhase(myPwm4, initialEPWMdischarging);   //Phase for FB side
            PWM_Obj *pwm3 = (PWM_Obj *)myPwm3;
            pwm3->TBPHSHR = startingHRdischarging << 8;
            PWM_Obj *pwm4 = (PWM_Obj *)myPwm4;
            pwm4->TBPHSHR = startingHRdischarging << 8;
            //CLK_enableTbClockSync(myClk);

            PWM_setDeadBandRisingEdgeDelay(myPwm1, 7);  //DB for QSC // Q2 turn off to Q1 turn on
            PWM_setDeadBandRisingEdgeDelay(myPwm2, 10);  //DB for QSC
            PWM_setDeadBandRisingEdgeDelay(myPwm3, 13);  //DB for FB side
            PWM_setDeadBandRisingEdgeDelay(myPwm4, 13);  //DB for FB side

            //turn on CCL_EN
            GPIO_toggle(myGpio, GPIO_Number_19);
            GPIO_setHigh(myGpio, GPIO_Number_3);

            scia_xmit(254);
            scia_xmit(1);
            msgGlobal = "Turn on CCL_EN\0";
            scia_msg(msgGlobal);
        }
        else if(buttonCount == 2) {

            //S1 duty ramp up
            int FBduty = 17;

            //CLK_disableTbClockSync(myClk);
            //PWM_setShadowMode_CmpA(myPwm3, PWM_ShadowMode_Immediate);  //Load registers every ZERO
            PWM_setCmpA(myPwm3, FBduty);
            PWM_setCounterMode(myPwm3, PWM_CounterMode_Up);     // Count up
            //PWM_setShadowMode_CmpA(myPwm3, PWM_ShadowMode_Shadow);  //Load registers every ZERO

            CLK_enableTbClockSync(myClk);

            count = 0;
            for(; count < 200; count++) {}

            PWM_setActionQual_Zero_PwmA(myPwm3, PWM_ActionQual_Set);
            PWM_setActionQual_CntUp_CmpB_PwmA(myPwm3, PWM_ActionQual_Set);

            count = 0;
            for(; count < 100; count++) {}

            while(FBduty != period/2) {

                //Delay to determine ramp up speed
                count = 0;
                for(; count < 100; count++) {}

                FBduty += 1;

                GPIO_toggle(myGpio, GPIO_Number_19);
                PWM_setCmpA(myPwm3, FBduty);

            }

        scia_xmit(254);
        scia_xmit(1);
        msgGlobal = "S1 duty ramp up\0";
        scia_msg(msgGlobal);

        }
        else if(buttonCount == 3) {

            //S2 turns on
            PWM_setActionQual_CntUp_CmpA_PwmA(myPwm4, PWM_ActionQual_Clear);
            PWM_setCounterMode(myPwm4, PWM_CounterMode_Up);     // Count up
            count = 0;
            for(; count < 20; count++) {}
            GPIO_toggle(myGpio, GPIO_Number_19);
            //PWM_setActionQual_Zero_PwmA(myPwm3, PWM_ActionQual_Set);
            PWM_setActionQual_CntUp_CmpA_PwmA(myPwm4, PWM_ActionQual_Set);

            scia_xmit(254);
            scia_xmit(1);
            msgGlobal = "S2 turns on\0";
            scia_msg(msgGlobal);

        }
        else if(buttonCount == 4) {

            //turn on QSC
            //CLK_disableTbClockSync(myClk);
            //set to 50% duty


//            PWM_setCount(myPwm1, 2);
//            PWM_setCount(myPwm2, 2);
//            PWM_setCount(myPwm3, 2);
//            PWM_setCount(myPwm4, 2);

            PWM_setActionQual_Zero_PwmA(myPwm1, PWM_ActionQual_Clear);
            PWM_setActionQual_CntUp_CmpA_PwmA(myPwm2, PWM_ActionQual_Clear);

            //PWM_setCounterMode(myPwm1, PWM_CounterMode_Up);     // Count up
            PWM_setCounterMode(myPwm2, PWM_CounterMode_Up);     // Count up

            PWM_setCmpA(myPwm1, period/2);
            PWM_setCmpA(myPwm2, period/2);

            count = 0;
            for(; count < 200; count++) {}

            //CLK_disableTbClockSync(myClk);
            PWM_setActionQual_Zero_PwmA(myPwm1, PWM_ActionQual_Set);
            PWM_setActionQual_CntUp_CmpA_PwmA(myPwm2, PWM_ActionQual_Set);
            //CLK_enableTbClockSync(myClk);
            //marker

            count = 0;
            for(; count < 200; count++) {}

           GPIO_toggle(myGpio, GPIO_Number_19);
           //CLK_enableTbClockSync(myClk);

           scia_xmit(254);
           scia_xmit(1);
           msgGlobal = "QSC turns on\0";
           scia_msg(msgGlobal);
        }
        else if(buttonCount == 5) {

            //begin the closedControlLoop function
            GPIO_toggle(myGpio, GPIO_Number_19);
            measureIndex = 0;
            first32Flag = 0;
            ePWMphase = initialEPWMdischarging;
            integral = 0;
            //TIMER_start(myTimer0);

            scia_xmit(254);
            scia_xmit(1);
            msgGlobal = "Closed Loop Control\0";
            scia_msg(msgGlobal);

        }
        else if(buttonCount == 6) {

            //stop closed loop control
            //TIMER_stop(myTimer0);

            //phase ramp down to ensure mitigate transient effects
            GPIO_toggle(myGpio, GPIO_Number_19);

            /*
            // RAMPDOWN //
            if (ePWMphase > initialEPWMdischarging){
                while(ePWMphase > initialEPWMdischarging) {

                    //Delay to determine speed of rampdown
                    testCount = 0;
                    for(; testCount < 300; testCount++) {}

                    ePWMphase--;

                    GPIO_toggle(myGpio, GPIO_Number_19);

                    PWM_setPhase(myPwm3, ePWMphase);   // Phase for FB side
                    PWM_setPhase(myPwm4, ePWMphase);   // Phase for FB side
                }

                PWM_Obj *pwm3 = (PWM_Obj *)myPwm3;
                pwm3->TBPHSHR = startingHRdischarging << 8;
                PWM_Obj *pwm4 = (PWM_Obj *)myPwm4;
                pwm4->TBPHSHR = startingHRdischarging << 8;
            }
            */

            scia_xmit(254);
            scia_xmit(1);
            msgGlobal = "Stop ClC, phase   rampdown\0";
            scia_msg(msgGlobal);
        }
        else if(buttonCount == 7) {

            //turn off FB PWM below
            //clear AQs and delay 1 PWM cycle to allow the pins to fall low
            PWM_setActionQual_Zero_PwmA(myPwm3, PWM_ActionQual_Clear);
            PWM_setActionQual_CntUp_CmpA_PwmA(myPwm4, PWM_ActionQual_Clear);
            PWM_setActionQual_CntUp_CmpB_PwmA(myPwm3, PWM_ActionQual_Clear);

            testCount = 0;
            for(; testCount < 100; testCount++) {}

            //stop FB PWM clocks
            PWM_setCounterMode(myPwm3, PWM_CounterMode_Stop);
            PWM_setCounterMode(myPwm4, PWM_CounterMode_Stop);

            testCount = 0;
            for(; testCount < 100; testCount++) {}

            //turn off CCL_EN
            GPIO_toggle(myGpio, GPIO_Number_19);
            GPIO_setLow(myGpio, GPIO_Number_3);

            scia_xmit(254);
            scia_xmit(1);
            msgGlobal = "Turn off FB, CCL\0";
            scia_msg(msgGlobal);
        }
        else if(buttonCount == 8) {

            GPIO_toggle(myGpio, GPIO_Number_19);
            //turn off QSC PWM
            //clear AQs and delay 1 PWM cycle to allow the pins to fall low
            PWM_setActionQual_Zero_PwmA(myPwm1, PWM_ActionQual_Clear);
            PWM_setActionQual_CntUp_CmpA_PwmA(myPwm1, PWM_ActionQual_Clear);

            PWM_setActionQual_Zero_PwmA(myPwm2, PWM_ActionQual_Clear);
            PWM_setActionQual_CntUp_CmpA_PwmA(myPwm2, PWM_ActionQual_Clear);
            //GPIO_toggle(myGpio, GPIO_Number_19);

            testCount = 0;
            for(; testCount < 30000; testCount++) {}  //in theory, 100 us delay

            //stop PWM on QSC
            //PWM_setCounterMode(myPwm1, PWM_CounterMode_Stop);
            PWM_setCounterMode(myPwm2, PWM_CounterMode_Stop);

            CLK_disableTbClockSync(myClk);
            //ensure the counters are not at 0 or CmpA
            PWM_setCount(myPwm1, 2);
            PWM_setCount(myPwm2, 2);
            PWM_setCount(myPwm3, 2);
            PWM_setCount(myPwm4, 2);

            //once sure PWM pins are low, return AQs to normal while counter is stopped
            PWM_setActionQual_CntUp_CmpA_PwmA(myPwm3, PWM_ActionQual_Clear);
            PWM_setActionQual_Zero_PwmA(myPwm4, PWM_ActionQual_Clear);
            CLK_enableTbClockSync(myClk);

            state = standby;
            parameter = vbatMin;

            printHomeScreen(msgGlobal);

            buttonCount = 0;
        }


    }

    // Acknowledge interrupt to PIE
    PIE_clearInt(myPie, PIE_GroupNumber_1);
}

__interrupt void xint2_isr(void)
{
    if(state == standby) {

        state = changeLimits;

        //send control char 254
        scia_xmit(254);
        //clear display
        scia_xmit(1);

        msgGlobal = "Change Limits\0";
        scia_msg(msgGlobal);

        //send control char 254
        scia_xmit(254);
        //set cursor to first char on second line
        scia_xmit(192);

        msgGlobal = "VbatMin: \0";
        scia_msg(msgGlobal);

        digitFlag = FIRSTDIGIT; //position of the first digit

        //send control char 254
        scia_xmit(254);
        //set cursor to first digit
        scia_xmit(digitFlag);

        //wait for button clicks
        scia_xmit(digitOne + 48);
    }
    else if(state == modeSelect) { //enter charge mode, set desired idischarge

        parameter = idischarge;
        state = changeLimits;

        //send control char 254
        scia_xmit(254);
        //clear display
        scia_xmit(1);

        msgGlobal = "Set Idischarge\0";
        scia_msg(msgGlobal);

        //send control char 254
        scia_xmit(254);
        //set cursor to first char on second line
        scia_xmit(192);

        msgGlobal = "Idischr: \0";
        scia_msg(msgGlobal);

        digitFlag = FIRSTDIGIT; //position of the first digit
        digitOne = 0;

        //send control char 254
        scia_xmit(254);
        //set cursor to first digit
        scia_xmit(digitFlag);

        //wait for button clicks
        scia_xmit(digitOne + 48);
    }
    else if(state == changeLimits) {

            switch(parameter) {

                case vbatMin:

                    buttonCount++;

                        if(buttonCount == 1) {

                        VbatMin = digitOne;
                        digitOne = 0;
                        digitFlag++;

                        //send control char 254
                        scia_xmit(254);
                        //set cursor to second digit
                        scia_xmit(digitFlag);

                        //display zero, wait for button clicks for changes
                        scia_xmit(digitOne + 48);

                    }
                    else if(buttonCount == 2) {
                        parameter = vbatMax;

                        VbatMin = VbatMin * 10 + digitOne;

                        //send control char 254
                         scia_xmit(254);
                        //set cursor to first char on second line
                        scia_xmit(192);

                        msgGlobal = "VbatMax: \0";
                        scia_msg(msgGlobal);

                        //send control char 254
                        scia_xmit(254);
                        //set cursor to first digit
                        scia_xmit(FIRSTDIGIT);
                        //send zero
                        scia_xmit(48);

                        //send control char 254
                        scia_xmit(254);
                        //set cursor to second digit
                        scia_xmit(SECONDDIGIT);
                        //clear second digit
                        scia_xmit(32);

                        digitFlag = FIRSTDIGIT;

                        //send control char 254
                        scia_xmit(254);
                        //set cursor to first digit
                        scia_xmit(digitFlag);

                        buttonCount = 0;
                        digitOne = 0;
                    }

                break;

                case vbatMax:

                    buttonCount++;

                    if(buttonCount == 1) {

                        VbatMax = digitOne;
                        digitOne = 0;
                        digitFlag++;

                        //send control char 254
                        scia_xmit(254);
                        //set cursor to second digit
                        scia_xmit(digitFlag);

                        //display zero, wait for button clicks for changes
                        scia_xmit(digitOne + 48);

                    }
                    else if(buttonCount == 2) {

                        parameter = ichargeMax;

                        VbatMax = VbatMax * 10 + digitOne;

                        //send control char 254
                        scia_xmit(254);
                        //set cursor to first char on second line
                        scia_xmit(192);

                        msgGlobal = "IchrMax: \0";
                        scia_msg(msgGlobal);

                        //send control char 254
                        scia_xmit(254);
                        //set cursor to first digit
                        scia_xmit(FIRSTDIGIT);
                        //send zero
                        scia_xmit(48);

                        //send control char 254
                        scia_xmit(254);
                        //set cursor to second digit
                        scia_xmit(SECONDDIGIT);
                        //clear second digit
                        scia_xmit(32);

                        digitFlag = FIRSTDIGIT;

                        //send control char 254
                        scia_xmit(254);
                        //set cursor to first digit
                        scia_xmit(digitFlag);

                        buttonCount = 0;
                        digitOne = 0;
                    }

                break;

                case ichargeMax:

                    buttonCount++;

                    if(buttonCount == 1) {

                        IchargeMax = digitOne;
                        digitOne = 0;
                        //this one gets double incremented because of the decimal point
                        digitFlag+= 2;

                        //send control char 254
                        scia_xmit(254);
                        //set cursor to second digit
                        scia_xmit(SECONDDIGIT);
                        //send a decimal point
                        scia_xmit(46);

                        //send control char 254
                        scia_xmit(254);
                        //set cursor to second digit (1st decimal place)
                        scia_xmit(digitFlag);

                        //display zero, wait for button clicks for changes
                        scia_xmit(digitOne + 48);

                    }
                    else if(buttonCount == 2) {

                        parameter = idischargeMax;

                        IchargeMax = IchargeMax + digitOne * 0.1;

                        //send control char 254
                        scia_xmit(254);
                        //set cursor to first char on second line
                        scia_xmit(192);

                        msgGlobal = "IdisMax: \0";
                        scia_msg(msgGlobal);

                        //send control char 254
                        scia_xmit(254);
                        //set cursor to second digit
                        scia_xmit(FIRSTDIGIT);
                        //send zero
                        scia_xmit(48);

                        //send control char 254
                        scia_xmit(254);
                        //set cursor to second digit
                        scia_xmit(SECONDDIGIT);
                        //clear second digit
                        scia_xmit(32);

                        //send control char 254
                        scia_xmit(254);
                        //set cursor to third digit
                        scia_xmit(SECONDDIGIT + 1);
                        //clear third digit
                        scia_xmit(32);


                        digitFlag = FIRSTDIGIT;

                        //send control char 254
                        scia_xmit(254);
                        //set cursor to first digit
                        scia_xmit(digitFlag);

                        buttonCount = 0;
                        digitOne = 0;
                    }

                break;

                case idischargeMax:

                    buttonCount++;

                    if(buttonCount == 1) {

                        IdischargeMax = digitOne;
                        digitOne = 0;
                        digitFlag++;

                        //send control char 254
                        scia_xmit(254);
                        //set cursor to second digit
                        scia_xmit(digitFlag);

                        //display zero, wait for button clicks for changes
                        scia_xmit(digitOne + 48);

                    }
                    else if(buttonCount == 2) {

                        state = standby;
                        parameter = vbatMin;

                        //send control char 254
                        scia_xmit(254);
                        //clear display
                        scia_xmit(1);

                        printHomeScreen(msgGlobal);

                        IdischargeMax = IdischargeMax * 10 + digitOne;

                        digitFlag = FIRSTDIGIT;
                        buttonCount = 0;
                        digitOne = 0;
                    }

                break;

                case icharge:

                    buttonCount++;

                    if(buttonCount == 1) {

                        Icharge = digitOne;
                        digitOne = 0;
                        //this one gets double incremented because of the decimal point, not anymore
                        digitFlag += 1;

//                        //send control char 254
//                        scia_xmit(254);
//                        //set cursor to second digit
//                        scia_xmit(SECONDDIGIT);
//                        //send a decimal point
//                        scia_xmit(46);

                        //send control char 254
                        scia_xmit(254);
                        //set cursor to second digit (1st decimal place)
                        scia_xmit(digitFlag);

                        //display zero, wait for button clicks for changes
                        scia_xmit(digitOne + 48);
                    }
                    else if(buttonCount == 2) {

                        state = chargeDisplay;
                        parameter = vbatMin;

                        Icharge = Icharge*10 + digitOne;

                        //send control char 254
                        scia_xmit(254);
                        //clear display
                        scia_xmit(1);

                        //set up the outline of the LCD screen
                        msgGlobal = "Button1: charge \0";
                        scia_msg(msgGlobal);

                        //send control char 254
                        scia_xmit(254);
                        //set cursor to first char on second line
                        scia_xmit(192);

                        //set up the outline of the LCD screen
                        msgGlobal = "Button2: standby\0";
                        scia_msg(msgGlobal);

                        //display InitialLV
                        MeasureBatVoltage();

                        //send control char 254
                        scia_xmit(254);
                        //set cursor to first char on third line
                        scia_xmit(148);

                        //set up the outline of the LCD screen
                        msgGlobal = "Battery(V):\0";
                        scia_msg(msgGlobal);
                        printBatVoltage();

                        digitFlag = FIRSTDIGIT;
                        buttonCount = 0;
                        digitOne = 0;
                    }

                    break;

                case idischarge:

                    buttonCount++;

                    if(buttonCount == 1) {

                        Idischarge = digitOne;
                        digitOne = 0;
                        digitFlag++;

                        //send control char 254
                        scia_xmit(254);
                        //set cursor to second digit
                        scia_xmit(digitFlag);

                        //display zero, wait for button clicks for changes
                        scia_xmit(digitOne + 48);
                    }
                    else if(buttonCount == 2) {

                        state = dischargeDisplay;
                        parameter = vbatMin;

                        Idischarge = Idischarge * 10 + digitOne;

                        //send control char 254
                        scia_xmit(254);
                        //clear display
                        scia_xmit(1);

                        //set up the outline of the LCD screen
                        msgGlobal = "Button1: dischrg \0";
                        scia_msg(msgGlobal);

                        //send control char 254
                        scia_xmit(254);
                        //set cursor to first char on second line
                        scia_xmit(192);

                        //set up the outline of the LCD screen
                        msgGlobal = "Button2: standby\0";
                        scia_msg(msgGlobal);

                        digitFlag = FIRSTDIGIT;
                        buttonCount = 0;
                        digitOne = 0;
                    }

                break;

                default:
                    state = standby;
            }

        }
    else if(state == chargeDisplay) {

        printHomeScreen(msgGlobal);
        state = standby;

    }
    else if(state == dischargeDisplay) {

        printHomeScreen(msgGlobal);
        state = standby;

    }
    // Acknowledge interrupt to PIE
    PIE_clearInt(myPie, PIE_GroupNumber_1);
}

__interrupt void xint3_isr(void)
{

    // Acknowledge interrupt to PIE
    PIE_clearInt(myPie, PIE_GroupNumber_12);
}

//this ISR is entered when timer0  is started
__interrupt void closedControlLoop(void)
{
    switch(state) {

    case chargeDisplay:


        //this root IF populates the array with the first 32 values
        if(first32Flag == 0) {

            if(measureIndex == 0) {
                sumOfILV = 0;
            }

            //current measurement for control
            measuredILV[measureIndex] = AdcResult.ADCRESULT6 >> 5;
            sumOfILV += measuredILV[measureIndex];

            if(measureIndex == 31) {

                //this flag will be set after the array is populated with the first 32 elements
                //closed loop control calculations begin after this flag is set
                first32Flag = 1;

                measureIndex = 0;
            }
            else {
                measureIndex++;
            }

        }
        //this root ELSE handles closed loop control after the first 32 measurements are taken
        else {

            //subtract the oldest measurement and replace it with the newest (running average)
            sumOfILV -= measuredILV[measureIndex];
            measuredILV[measureIndex] = AdcResult.ADCRESULT6 >> 5;
            sumOfILV += measuredILV[measureIndex];

            if(measureIndex == 31) {
                measureIndex = 0;
            }
            else {
                measureIndex++;
            }

        //closed loop control below, needs cleaned up====================================================================

            adcVoltage = sumOfILV;
            //equation relating ADC value (0-4095) to a voltage on the pin (0-3.3V)
            //actualIHV = ((adcVoltage / 4095.0) * 3.3);
            actualILV = (adcVoltage * 0.000805860806);

            //stuff for displaying at 0-3.3V
            actualILV_33 = actualILV * 100.0;
            iLVcast = (int) actualILV_33;

            //VLVpluscast = (int) ((sumOfVLVplus / 4095.0) * 330.0);
            //VLVminuscast = (int) ((sumOfVLVminus / 4095.0) * 330.0);
            VLVpluscast = (int) (sumOfVLVplus * 0.0805860806);
            VLVminuscast = (int) (sumOfVLVminus * 0.0805860806);

            //actualIHV += - 1.4553;
            //actualIHV *= 53.41;

            actualILV += -1.5732;
            actualILV *= -39.5257;

            //watch this value as it changes in real time
            displayCurr = actualILV;

            //TODO verify this limit implementation
            //if (actualIHV < IchargeMax) {

            //PI control effort begins here
            error = currentSetPointCharging - actualILV;
            integral += error;
            PIfactor = kp*error + ki*integral;

            //convert phase to ePMWM=0-170,HRPWM=0-120s
            phaseStore = initialEPWMcharging - PIfactor;
            wholeNum = phaseStore; // course Phase
            phaseLeftover = phaseStore - wholeNum; //phaseLeftover is a decimal representing % of HR phase out of 120 (120 is how many HR MEP steps make up 1 coarse step)

            if(phaseLeftover <= 0 && wholeNum <= 0) {
                phaseLeftover += 1;
                wholeNum += 169;
            }

            //get the HR phase number from a decimal to a whole number between 0-120
            HRphase = 120 - (120 * phaseLeftover);
            ePWMphase = wholeNum;

            //TODO make sure to fix the limit setting and handling

            // Make max and min phase limits
            if(ePWMphase > 60 && ePWMphase < phaseShiftMax) {
                ePWMphase = phaseShiftMax;
            }

            if(ePWMphase <= 59 && ePWMphase > (phaseShiftMin - 1)) {
                ePWMphase = phaseShiftMin;
            }

            //error checking/limit setting
            if(HRphase < 0) {
                HRphase = 0;
            }

            if(HRphase > 120) {
                HRphase = 120;
            }

            if(ePWMphase < 0) {
                ePWMphase = 169;
            }

            PWM_Obj *pwm3 = (PWM_Obj *)myPwm3;
            PWM_Obj *pwm4 = (PWM_Obj *)myPwm4;

            //set PWM high resolution and course phase value
            pwm3->TBPHSHR = HRphase << 8;
            pwm4->TBPHSHR = HRphase << 8;
            PWM_setPhase(myPwm3, ePWMphase);   // Phase for FB side
            PWM_setPhase(myPwm4, ePWMphase);   // Phase for FB side

        //closed loop control needs cleaned up====================================================================
        }

        count++;

        //delay to display the numbers slow enough for human eyes
        if(count == 1000) {

            //TODO write a function that displays all the averages to the screen

            //this prints the current with 2 decimal places
            printMeasurements();

            //write_num men
            count = 0;
        }

    break;

    case dischargeDisplay:

        //this root IF populates the array with the first 32 values
        if(first32Flag == 0) {

            if(measureIndex == 0) {
                sumOfIHV = 0;
            }

            measuredIHV[measureIndex] = AdcResult.ADCRESULT1 >> 5;
            sumOfIHV += measuredIHV[measureIndex];

            if(measureIndex == 31) {

                //this flag will be set after the array is populated with the first 32 elements
                //closed loop control calculations begin after this flag is set
                first32Flag = 1;

                measureIndex = 0;
            }
            else {
                measureIndex++;
            }

        }
        //this root ELSE handles closed loop control after the first 32 measurements are taken
        else {

            //subtract the oldest value and replace it with the newest value (running average)
            sumOfIHV -= measuredIHV[measureIndex];
            measuredIHV[measureIndex] = AdcResult.ADCRESULT1 >> 5;
            sumOfIHV += measuredIHV[measureIndex];

            if(measureIndex == 31) {
                measureIndex = 0;
            }
            else {
                measureIndex++;
            }

//closed loop control below, needs cleaned up====================================================================

            adcVoltage = sumOfIHV;
            //equation relating ADC value (0-4095) to a voltage on the pin (0-3.3V)
            //actualIHV = ((adcVoltage / 4095.0) * 3.3);
            actualIHV = (adcVoltage * 0.000805860806);

            //stuff for displaying at 0-3.3V
            actualIHV_33 = actualIHV * 100.0;
            iHVcast = (int) actualIHV_33;

            //actualIHV += -1.5732;
            //actualIHV *= -39.5257;

            //watch this value as it changes in real time
            displayCurr = actualIHV;

            //PI control effort begins here
            error = currentSetPointDischarging - actualIHV;
            integral += error;
            PIfactor = kp*error + ki*integral;

            //convert phase to ePMWM=0-170,HRPWM=0-120s
            phaseStore = initialEPWMdischarging + PIfactor;
            wholeNum = phaseStore; // course Phase
            phaseLeftover = phaseStore - wholeNum; //phaseLeftover is a decimal representing % of HR phase out of 120 (120 is how many HR MEP steps make up 1 coarse step)

            if(phaseLeftover <= 0 && wholeNum <= 0) {
                phaseLeftover += 1;
                wholeNum += 169;
            }

            //get the HR phase number from a decimal to a whole number between 0-120
            HRphase = 120 - (120 * phaseLeftover);
            ePWMphase = wholeNum;


            PWM_Obj *pwm3 = (PWM_Obj *)myPwm3;
            PWM_Obj *pwm4 = (PWM_Obj *)myPwm4;
            //GPIO_toggle(myGpio, GPIO_Number_19);
            //pwm3->TBPHSHR = HRphase << 8;
            //pwm4->TBPHSHR = HRphase << 8;
            //PWM_setPhase(myPwm3, ePWMphase);   // Phase for FB side
            //PWM_setPhase(myPwm4, ePWMphase);   // Phase for FB side

        }


    break;

    default:
        //TODO something is wrong, implement error handling
    break;


    }

    // Acknowledge interrupt to PIE
    PIE_clearInt(myPie, PIE_GroupNumber_1);

    // Clear ADCINT1 flag reinitialize for next SOC
    ADC_clearIntFlag(myAdc, ADC_IntNumber_1);
}

void dutyRampStartupQSC(int period) {

    int duty = period - 16;

    CLK_disableTbClockSync(myClk);
    PWM_setShadowMode_CmpA(myPwm1, PWM_ShadowMode_Immediate);  //Load registers every ZERO
    PWM_setShadowMode_CmpA(myPwm2, PWM_ShadowMode_Immediate);  //Load registers every ZERO
    PWM_setCmpA(myPwm1, duty);
    PWM_setCmpA(myPwm2, duty);
    PWM_setCounterMode(myPwm1, PWM_CounterMode_Up);     // Count up
    PWM_setCounterMode(myPwm2, PWM_CounterMode_Up);     // Count up
    PWM_setShadowMode_CmpA(myPwm1, PWM_ShadowMode_Shadow);  //Load registers every ZERO
    PWM_setShadowMode_CmpA(myPwm2, PWM_ShadowMode_Shadow);  //Load registers every ZERO
    //CLK_enableTbClockSync(myClk); //KEEP THIS COMMENTED OUT

    count = 0;
    for(; count < 200; count++) {}

    PWM_setActionQual_Zero_PwmA(myPwm1, PWM_ActionQual_Set);
    PWM_setActionQual_CntUp_CmpA_PwmA(myPwm2, PWM_ActionQual_Set);


    while(duty != period/2) {

        //Delay to determine ramp up speed
        count = 0;
        for(; count < 100; count++) {}

        duty -= 1;

        PWM_setCmpA(myPwm1, duty);
        PWM_setCmpA(myPwm2, duty);

        CLK_enableTbClockSync(myClk);
    }
}


void HRPWM1_Config(Uint16 period)   //Q1a, Q1b GATE
{
    CLK_enablePwmClock(myClk, PWM_Number_1);

    PWM_setPeriodLoad(myPwm1, PWM_PeriodLoad_Immediate);
    PWM_setPeriod(myPwm1, period-1);    //Set timer period
    PWM_setCmpA(myPwm1, 0);  //set to 0 originally. will be ramped up to period/2 in dutyRampStartupQSC

    PWM_setPhase(myPwm1, 0x0000);   //Phase is 0 initially
    PWM_setCount(myPwm1, 0x001);   //start counter at 1 so the pin is not originally high

    PWM_setCounterMode(myPwm1, PWM_CounterMode_Stop);   //Counter will be enabled after button press ISR
    PWM_disableCounterLoad(myPwm1);                     //Disable phase loading
    PWM_setSyncMode(myPwm1, PWM_SyncMode_CounterEqualZero);
    PWM_setHighSpeedClkDiv(myPwm1, PWM_HspClkDiv_by_1); //Clock ratio to SYSCLKOUT
    PWM_setClkDiv(myPwm1, PWM_ClkDiv_by_1);

    PWM_setShadowMode_CmpA(myPwm1, PWM_ShadowMode_Shadow);  //Load registers every ZERO
    PWM_setLoadMode_CmpA(myPwm1, PWM_LoadMode_Zero);

    //Action set up
    PWM_setActionQual_Zero_PwmA(myPwm1, PWM_ActionQual_Set);
    PWM_setActionQual_CntUp_CmpA_PwmA(myPwm1, PWM_ActionQual_Clear);

    //HR setup
    PWM_setHrEdgeMode(myPwm1, PWM_HrEdgeMode_Both);
    PWM_enableAutoConvert(myPwm1);
    PWM_setHrControlMode(myPwm1, PWM_HrControlMode_Phase);
    PWM_setHrShadowMode(myPwm1, PWM_HrShadowMode_CTR_EQ_0);
    PWM_enableHrPhaseSync(myPwm1);

    //Active high complementary PWMs - Setup the deadband
    PWM_setDeadBandOutputMode(myPwm1, PWM_DeadBandOutputMode_EPWMxA_Rising_EPWMxB_Disable);
    PWM_enableDeadBandHalfCycle(myPwm1);
    PWM_setDeadBandPolarity(myPwm1, PWM_DeadBandPolarity_EPWMxA_EPWMxB);
}
void HRPWM2_Config(Uint16 period)   //Q2 GATE
{
    CLK_enablePwmClock(myClk, PWM_Number_2);

    PWM_setPeriodLoad(myPwm2, PWM_PeriodLoad_Immediate);
    PWM_setPeriod(myPwm2, period-1);    //Set timer period
    PWM_setCmpA(myPwm2, 0);  ////////set to 0 originally. will be ramped up to period/2 in dutyRampStartupQSC
//    PWM_setCmpB(myPwm2, period / 2);

    PWM_setPhase(myPwm2, 0);   //Phase is 0 initially
    PWM_setCount(myPwm2, 0x0001);   //start counter at 1 so the pin is not originally high

    PWM_setCounterMode(myPwm2, PWM_CounterMode_Stop);   //Counter will be enabled after button press ISR
    PWM_enableCounterLoad(myPwm2);
    PWM_setSyncMode(myPwm2, PWM_SyncMode_EPWMxSYNC);
    PWM_setHighSpeedClkDiv(myPwm2, PWM_HspClkDiv_by_1); //Clock ratio to SYSCLKOUT
    PWM_setClkDiv(myPwm2, PWM_ClkDiv_by_1);

    //Load registers every ZERO
    PWM_setShadowMode_CmpA(myPwm2, PWM_ShadowMode_Shadow);
    PWM_setLoadMode_CmpA(myPwm2, PWM_LoadMode_Zero);

    //Action set up
    PWM_setActionQual_Zero_PwmA(myPwm2, PWM_ActionQual_Clear);
    //PWM_setActionQual_CntUp_CmpB_PwmA(myPwm2, PWM_ActionQual_Set);
    PWM_setActionQual_CntUp_CmpA_PwmA(myPwm2, PWM_ActionQual_Clear);

    //HR setup
    PWM_setHrEdgeMode(myPwm2, PWM_HrEdgeMode_Both);
    PWM_setHrControlMode(myPwm2, PWM_HrControlMode_Phase);
    PWM_setHrShadowMode(myPwm2, PWM_HrShadowMode_CTR_EQ_0);
    PWM_enableHrPhaseSync(myPwm2);
    PWM_enableAutoConvert(myPwm2);

    //Active high complementary PWMs - Setup the deadband
    PWM_setDeadBandOutputMode(myPwm2, PWM_DeadBandOutputMode_EPWMxA_Rising_EPWMxB_Disable);
    PWM_enableDeadBandHalfCycle(myPwm2);
    PWM_setDeadBandPolarity(myPwm2, PWM_DeadBandPolarity_EPWMxA_EPWMxB);
}
void HRPWM3_Config(Uint16 period)   //S1, S3 GATE
{
    CLK_enablePwmClock(myClk, PWM_Number_3);

    PWM_setPeriodLoad(myPwm3, PWM_PeriodLoad_Immediate);
    PWM_setPeriod(myPwm3, period-1);    //Set timer period
    PWM_setCmpA(myPwm3, period / 2);
    PWM_setCmpB(myPwm3, 4);

    PWM_setPhase(myPwm3, 0);   // Phase is 0
    PWM_setCount(myPwm3, 0x0001);   //Start counter at 1 so the pin is not originally high

    PWM_setCounterMode(myPwm3, PWM_CounterMode_Stop); //Counter will be turned on later to start PWM
    PWM_enableCounterLoad(myPwm3);
    PWM_setSyncMode(myPwm3, PWM_SyncMode_EPWMxSYNC);
    PWM_setHighSpeedClkDiv(myPwm3, PWM_HspClkDiv_by_1); //Clock ratio to SYSCLKOUT
    PWM_setClkDiv(myPwm3, PWM_ClkDiv_by_1);

    PWM_setShadowMode_CmpA(myPwm3, PWM_ShadowMode_Shadow);  //Load registers every ZERO
    PWM_setLoadMode_CmpA(myPwm3, PWM_LoadMode_Zero);

    PWM_setShadowMode_CmpB(myPwm3, PWM_ShadowMode_Shadow);  //Load registers every ZERO
    PWM_setLoadMode_CmpB(myPwm3, PWM_LoadMode_Zero);

    //Action setup
    PWM_setActionQual_Zero_PwmA(myPwm3, PWM_ActionQual_Set);
    PWM_setActionQual_CntUp_CmpB_PwmA(myPwm3, PWM_ActionQual_Set);
    PWM_setActionQual_CntUp_CmpA_PwmA(myPwm3, PWM_ActionQual_Clear);

    //HR setup
    PWM_setHrEdgeMode(myPwm3, PWM_HrEdgeMode_Both);
    PWM_setHrControlMode(myPwm3, PWM_HrControlMode_Phase);
    PWM_setHrShadowMode(myPwm3, PWM_HrShadowMode_CTR_EQ_0);
    PWM_enableHrPhaseSync(myPwm3);
    PWM_enableAutoConvert(myPwm3);

    //Active high complementary PWMs - Setup the deadband
    PWM_setDeadBandOutputMode(myPwm3, PWM_DeadBandOutputMode_EPWMxA_Rising_EPWMxB_Disable);
    PWM_enableDeadBandHalfCycle(myPwm3);
    PWM_setDeadBandPolarity(myPwm3, PWM_DeadBandPolarity_EPWMxA_EPWMxB);
}
void HRPWM4_Config(Uint16 period)   //S2, S4 GATE
{
    CLK_enablePwmClock(myClk, PWM_Number_4);

    PWM_setPeriodLoad(myPwm4, PWM_PeriodLoad_Immediate);
    PWM_setPeriod(myPwm4, period-1);    //Set timer period
    PWM_setCmpA(myPwm4, period / 2);
    PWM_setCmpB(myPwm4, 4);

    PWM_setPhase(myPwm4, 0);   //Phase is 0
    PWM_setCount(myPwm4, 0x0001);  //Start counter at 1 so the pin is not originally high

    PWM_setCounterMode(myPwm4, PWM_CounterMode_Stop);     //Counter will be turned on later to start PWM
    PWM_enableCounterLoad(myPwm4);
    PWM_setSyncMode(myPwm4, PWM_SyncMode_EPWMxSYNC);
    PWM_setHighSpeedClkDiv(myPwm4, PWM_HspClkDiv_by_1); //Clock ratio to SYSCLKOUT
    PWM_setClkDiv(myPwm4, PWM_ClkDiv_by_1);

    //EPwm4Regs.TBCTL.bit.PRDLD = TB_SHADOW;

    PWM_setShadowMode_CmpA(myPwm4, PWM_ShadowMode_Shadow);  //Load registers every ZERO
    PWM_setLoadMode_CmpA(myPwm4, PWM_LoadMode_Zero);

    PWM_setShadowMode_CmpB(myPwm4, PWM_ShadowMode_Shadow);  //Load registers every ZERO
    PWM_setLoadMode_CmpB(myPwm4, PWM_LoadMode_Zero);

    //Action setup
    PWM_setActionQual_Zero_PwmA(myPwm4, PWM_ActionQual_Clear);
    PWM_setActionQual_CntUp_CmpB_PwmA(myPwm4, PWM_ActionQual_Clear);
    PWM_setActionQual_CntUp_CmpA_PwmA(myPwm4, PWM_ActionQual_Set);

    //HR setup
    PWM_setHrEdgeMode(myPwm4, PWM_HrEdgeMode_Both);
    PWM_setHrControlMode(myPwm4, PWM_HrControlMode_Phase);
    PWM_setHrShadowMode(myPwm4, PWM_HrShadowMode_CTR_EQ_0);
    PWM_enableHrPhaseSync(myPwm4);
    PWM_enableAutoConvert(myPwm4);

    //Active high complementary PWMs - Setup the deadband
    PWM_setDeadBandOutputMode(myPwm4, PWM_DeadBandOutputMode_EPWMxA_Rising_EPWMxB_Disable);
    PWM_enableDeadBandHalfCycle(myPwm4);
    PWM_setDeadBandPolarity(myPwm4, PWM_DeadBandPolarity_EPWMxA_EPWMxB);
}

// Transmit a character from the SCI
void scia_xmit(int a)
{
    while(SCI_getTxFifoStatus(mySci) != SCI_FifoStatus_Empty)
    {
    }

    SCI_putDataBlocking(mySci, a);
}

void scia_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scia_xmit(msg[i]);
        i++;
    }
}

void printMeasurements() {

    stack[0] = 0;
    stack[1] = 0;
    stack[2] = 0;
    stack[3] = 0;

    //print avgVLVplus
    //print avgVLVminus
    //print avgIVH

    printCount = 0;

    for(; printCount < 3; printCount++) {

        if(printCount == 0) {
            cast = iLVcast;
            //send control char 254
            scia_xmit(254);
            //set cursor to position 6 first line
            scia_xmit(134);

        }
        if(printCount == 1) {
            cast = VLVpluscast;
            //send control char 254
            scia_xmit(254);
            //set cursor to position 6 second line
            scia_xmit(198);
        }
        if(printCount == 2) {
            cast = VLVminuscast;
            //send control char 254
            scia_xmit(254);
            //set cursor to position 6 third line
            scia_xmit(154);
        }

        i = 0;
        while(cast > 0)
        {
            t = cast % 10;
            cast = cast * 0.1;
            stack[i] = t;
            i++;
        }

        SCI_putDataBlocking(mySci, stack[3] + 48);
        SCI_putDataBlocking(mySci, stack[2] + 48);
        SCI_putDataBlocking(mySci, 46);
        SCI_putDataBlocking(mySci, stack[1] + 48);
        SCI_putDataBlocking(mySci, stack[0] + 48);
    }
}

void printBatVoltage() {

    stack[0] = 0;
    stack[1] = 0;
    stack[2] = 0;
    stack[3] = 0;

    cast = (InitialLV * 10);

    i = 0;
    while(cast > 0)
    {
        t = cast % 10;
        cast = cast * 0.1;
        stack[i] = t;
        i++;
    }

    SCI_putDataBlocking(mySci, stack[2] + 48);
    SCI_putDataBlocking(mySci, stack[1] + 48);
    SCI_putDataBlocking(mySci, 46);
    SCI_putDataBlocking(mySci, stack[0] + 48);
}

void MeasureBatVoltage(){

//    double VLVsumPlus = 0, VLVsumMinus = 0;
//
//    int i = 0;
//    for(; i < 10; i++) {
//        testCount = 0;
//        for(; testCount < 100; testCount++) {}
//        VLVsumPlus += AdcResult.ADCRESULT4;
//        VLVsumMinus += AdcResult.ADCRESULT5;
//    }
//
//    VLVsumPlus *= 0.1;
//    VLVsumMinus *= 0.1;
//
//    return (VLVsumPlus - VLVsumMinus) * 0.0805860806;

    VLVplustemp = 0;
    VLVminustemp = 0;
    int i = 0;
    for(; i < 10; i++) {
        ADC_forceConversion(myAdc, ADC_SocNumber_4);
        ADC_forceConversion(myAdc, ADC_SocNumber_5);
        testCount = 0;
        for(; testCount < 100; testCount++) {}
        VLVplustemp += AdcResult.ADCRESULT4;
        VLVminustemp += AdcResult.ADCRESULT5;
    }

    VLVplustemp *= 0.1;
    VLVminustemp *= 0.1;
    VLVplustemp *= 0.000805860806;
    VLVminustemp *= 0.000805860806;

    VLVsumPlus = VLVplustemp;
    VLVsumMinus = VLVminustemp;

    InitialLV = 32.4675 * (VLVsumPlus - VLVsumMinus - .0598);
}

// -----------------------------------------------------------------------------------------------------------------------------------------------
