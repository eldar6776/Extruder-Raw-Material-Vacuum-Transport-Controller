/******************************************************************************
 *
 *           Kontroler vakum transporta sirovine linije 5
 *
 *                          Glavni program
 *
 *                       Firmware verzija 1.02
 *
 ******************************************************************************
 *
 *  Ime fajla:      trans_ctrl.c
 *
 *  Procesor:       PIC16F1517
 *
 *  Kompajler:      Microchip XC8 v1,13
 *
 *  IDE:            MPLAB X IDE v2.10 (java 1.7.0_25)
 *
 *  Datum:          20. jun 2014.
 *
 *  Autor:          eldar6776@hotmail.com
 *
 ******************************************************************************/
//
//
/** I N C L U D E D  **********************************************************/
//
//
#include <xc.h>
#include "trans_ctrl.h"
#include "typedefs.h"
#include "delay.c"
#include "io_cfg.h"

// <editor-fold defaultstate="collapsed" desc="interrupt service">

void interrupt timer_int(void) {
    if (TMR1IE && TMR1IF) {
        TMR1IF = FALSE;
        TMR1H = 0xd8; // reload timer for 10 ms interrupt
        TMR1L = 0xf2; // with 8 MHz crystal oscilator

        SetSysClockTick();

        // <editor-fold defaultstate="collapsed" desc="100ms clock tick">
        if (++clock_tick_100ms == 10) {
            clock_tick_100ms = 0;
            if (!SYSCLK_TICK_100MS) SYSCLK_TICK_100MS = TRUE;
            else SYSCLK_TICK_100MS = FALSE;
        }// </editor-fold>

        // <editor-fold defaultstate="collapsed" desc="500ms clock tick">
        if (++clock_tick_500ms == 50) {
            clock_tick_500ms = 0;
            if (!SYSCLK_TICK_500MS) SYSCLK_TICK_500MS = TRUE;
            else SYSCLK_TICK_500MS = FALSE;
        }// </editor-fold>

        // <editor-fold defaultstate="collapsed" desc="1s clock tick">
        if (++clock_tick_1s == 100) {
            clock_tick_1s = 0;
            if (!SYSCLK_TICK_1S) SYSCLK_TICK_1S = TRUE;
            else SYSCLK_TICK_1S = FALSE;
        }// </editor-fold>

        // <editor-fold defaultstate="collapsed" desc="2s clock tick">
        if (++clock_tick_2s == 200) {
            clock_tick_2s = 0;
            if (!SYSCLK_TICK_2S) SYSCLK_TICK_2S = TRUE;
            else SYSCLK_TICK_2S = FALSE;
        }// </editor-fold>

        // <editor-fold defaultstate="collapsed" desc="status led">
        if (LED_STATUS_ERROR) {
            if (!SYSCLK_TICK_100MS) StatusLED_On();
            else StatusLED_Off();
        } else if (LED_STATUS_BOOT) {
            if (!SYSCLK_TICK_500MS) StatusLED_On();
            else StatusLED_Off();
        } else if (LED_STATUS_STOP) {
            if (!SYSCLK_TICK_1S) StatusLED_On();
            else StatusLED_Off();
        } else if (LED_STATUS_RUN) {
            if (!SYSCLK_TICK_2S) StatusLED_On();
            else StatusLED_Off();
        }// </editor-fold>
    }// End of tmr1 interrupt
}// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="main">

void main(void) {

    InitRAM();
    InitSYS();

    while (1) {
        CheckTimer();
        CheckSafety();
        CheckDigitalInput();
        CheckAnalogInput();
        ProcessFeeder1();
        ProcessFeeder2();
        ProcessFeeder3();
        ProcessFeeder4();
        ProcessFeeder5();
        ProcessFeederCoex();
        ProcessCyclon1();
        ProcessVacumPump1();
        SetRelayOutput();
        SetAnalogOutput();
        SetSignalOutput();
    }// End of while(1)
}// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="init ram">

void InitRAM(void) {
    vacum_pump1_state = VP1_OFF;
    activ_unit = FEEDER1;
    feeder1_state = FEEDER1_OFF;
    feeder2_state = FEEDER2_OFF;
    feeder3_state = FEEDER3_OFF;
    feeder4_state = FEEDER4_OFF;
    feeder5_state = FEEDER5_OFF;
    feeder_coex_state = FEEDER_COEX_OFF;
    hc595_shift._byte = 0xcf;
    ClearStatusLED();
    an0_out = 0;
    an1_out = 0;
}// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="init sys">

void InitSYS(void) {
    InitPORTA();
    InitPORTB();
    InitPORTC();
    InitPORTD();
    InitPORTE();
    DisableDIN_0_7();
    DisableDIN_8_15();
    DisableDIN_16_23();
    DisableDIN_24_31();
    PWM_Init();
    VoltageReference_Init();
    TMR1Interrupt_Init(); // init timer 1 for 10 ms interrupt with 8 MHz oscilator
    TMR1Interrupt_Disable();
    //TMR0Interrupt_Init();
    //TMR0Interrupt_Enable();
    GlobalInterrupt_Enable();
    if (!DIN_DRIVE_OPTION) SetDriveOptionInverter();
    else SetDriveOptionStarDeltaStarter();
    an_in_pcnt = 0;
    SetSysClockTick();
    for (temp = 0; temp != 7; temp++) {
        CheckAnalogInput();
    }// End of for...
    TMR1Interrupt_Enable(); // enable timer 1 interrupt
    StatusLED_Booting();
    DelayS(5);
}// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="check timers">

void CheckTimer(void) {

    if (!IsSysClockTicked()) return;
    else ResetSysClockTick();

    if (vp1_tmr) {
        --vp1_tmr;
        VP1_TEX = FALSE;
    } else VP1_TEX = TRUE;

    if (error_display_tmr) {
        --error_display_tmr;
        ERROR_DISPLAY_TEX = FALSE;
    } else ERROR_DISPLAY_TEX = TRUE;

    if (max_charging_tmr) {
        --max_charging_tmr;
        MAX_CHARGING_TEX = FALSE;
    } else MAX_CHARGING_TEX = TRUE;

    if (piston_tmr) {
        --piston_tmr;
        PISTON_TEX = FALSE;
    } else PISTON_TEX = TRUE;

    if (feeder4_charging_tmr) {
        --feeder4_charging_tmr;
        FEEDER4_CHARGING_TEX = FALSE;
    } else FEEDER4_CHARGING_TEX = TRUE;

    if (mill_shutdown_delay_tmr) {
        --mill_shutdown_delay_tmr;
        MILL_SHUTDOWN_DELAY_TEX = FALSE;
    } else MILL_SHUTDOWN_DELAY_TEX = TRUE;

    if (feeder2_cleaning_piston_tmr) {
        --feeder2_cleaning_piston_tmr;
        FEEDER2_CLEANING_PISTON_TEX = FALSE;
    } else FEEDER2_CLEANING_PISTON_TEX = TRUE;

    if (feeder_state_change_tmr) {
        --feeder_state_change_tmr;
        FEEDER_STATE_CHANGE_TEX = FALSE;
    } else FEEDER_STATE_CHANGE_TEX = TRUE;

    if (vp1_control_delay_tmr) {
        --vp1_control_delay_tmr;
        VP1_CONTROL_DELAY_TEX = FALSE;
    } else VP1_CONTROL_DELAY_TEX = TRUE;

    if (feeder1_control_delay_tmr) {
        --feeder1_control_delay_tmr;
        FEEDER1_CONTROL_DELAY_TEX = FALSE;
    } else FEEDER1_CONTROL_DELAY_TEX = TRUE;

    if (feeder2_control_delay_tmr) {
        --feeder2_control_delay_tmr;
        FEEDER2_CONTROL_DELAY_TEX = FALSE;
    } else FEEDER2_CONTROL_DELAY_TEX = TRUE;

    if (feeder3_control_delay_tmr) {
        --feeder3_control_delay_tmr;
        FEEDER3_CONTROL_DELAY_TEX = FALSE;
    } else FEEDER3_CONTROL_DELAY_TEX = TRUE;

    if (feeder4_control_delay_tmr) {
        --feeder4_control_delay_tmr;
        FEEDER4_CONTROL_DELAY_TEX = FALSE;
    } else FEEDER4_CONTROL_DELAY_TEX = TRUE;

    if (feeder5_control_delay_tmr) {
        --feeder5_control_delay_tmr;
        FEEDER5_CONTROL_DELAY_TEX = FALSE;
    } else FEEDER5_CONTROL_DELAY_TEX = TRUE;

    if (feeder_coex_control_delay_tmr) {
        --feeder_coex_control_delay_tmr;
        FEEDER_COEX_CONTROL_DELAY_TEX = FALSE;
    } else FEEDER_COEX_CONTROL_DELAY_TEX = TRUE;
}// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="check digital input">

void CheckDigitalInput(void) {
    TMR1Interrupt_Disable();
    SetDataPortAsInput();
    EnableDIN_0_7();
    input_0_7._byte = DataRead();
    DisableDIN_0_7();
    EnableDIN_8_15();
    input_8_15._byte = DataRead();
    DisableDIN_8_15();
    EnableDIN_16_23();
    input_16_23._byte = DataRead();
    DisableDIN_16_23();
    EnableDIN_24_31();
    input_24_31._byte = DataRead();
    DisableDIN_24_31();
    TMR1Interrupt_Enable();
    if (EXTRUDER_OLD_STATE != DIN_EXTRUDER_RUN) { // check for extruder start condition
        EXTRUDER_OLD_STATE = DIN_EXTRUDER_RUN; // save new state in flag
        if (IsExtruderRunInputActiv()) SetExtruderStartFlag(); // if iput changed from 0 -> 1 extruder started, set flag
        else ResetExtruderStartFlag(); // if input changed from 1 -> 0 extruder stoped, clear flag
    } else ResetExtruderStartFlag(); // if no change on input, clear flag
    //------------------------------------------- TASTER VACUM PUMP 1 ON / OFF CONTROL
    if (TST_VP1_FL != DIN_VP1_CTRL_SW) {
        TST_VP1_FL = DIN_VP1_CTRL_SW;
        if (IsVacumPump1ControlSwitchActiv()) SetVacumPump1ControlSwitch();
        else ResetVacumPump1ControlSwitch();
    } else ResetVacumPump1ControlSwitch();
    //------------------------------------------- TASTER FEEDER 1 ON / OFF CONTROL
    if (TST_FEEDER1_FL != DIN_FEEDER1_CTRL_SW) {
        TST_FEEDER1_FL = DIN_FEEDER1_CTRL_SW;
        if (IsFeeder1ControlSwitchActiv()) SetFeeder1ControlSwitch();
        else ResetFeeder1ControlSwitch();
    } else ResetFeeder1ControlSwitch();
    //------------------------------------------- TASTER FEEDER 2 ON / OFF CONTROL
    if (TST_FEEDER2_FL != DIN_FEEDER2_CTRL_SW) {
        TST_FEEDER2_FL = DIN_FEEDER2_CTRL_SW;
        if (IsFeeder2ControlSwitchActiv()) SetFeeder2ControlSwitch();
        else ResetFeeder2ControlSwitch();
    } else ResetFeeder2ControlSwitch();
    //------------------------------------------- TASTER FEEDER 3 ON / OFF CONTROL
    if (TST_FEEDER3_FL != DIN_FEEDER3_CTRL_SW) {
        TST_FEEDER3_FL = DIN_FEEDER3_CTRL_SW;
        if (IsFeeder3ControlSwitchActiv()) SetFeeder3ControlSwitch();
        else ResetFeeder3ControlSwitch();
    } else ResetFeeder3ControlSwitch();
    //------------------------------------------- TASTER FEEDER 4 ON / OFF CONTROL
    if (TST_FEEDER4_FL != DIN_FEEDER4_CTRL_SW) {
        TST_FEEDER4_FL = DIN_FEEDER4_CTRL_SW;
        if (IsFeeder4ControlSwitchActiv()) SetFeeder4ControlSwitch();
        else ResetFeeder4ControlSwitch();
    } else ResetFeeder4ControlSwitch();
    //------------------------------------------- TASTER FEEDER 5 ON / OFF CONTROL
    if (TST_FEEDER5_FL != DIN_FEEDER5_CTRL_SW) {
        TST_FEEDER5_FL = DIN_FEEDER5_CTRL_SW;
        if (IsFeeder5ControlSwitchActiv()) SetFeeder5ControlSwitch();
        else ResetFeeder5ControlSwitch();
    } else ResetFeeder5ControlSwitch();
    //------------------------------------------- TASTER FEEDER COEX. ON / OFF CONTROL
    if (TST_FEEDER_COEX_FL != DIN_FEEDER_COEX_CTRL_SW) {
        TST_FEEDER_COEX_FL = DIN_FEEDER_COEX_CTRL_SW;
        if (IsFeederCoexControlSwitchActiv()) SetFeederCoexControlSwitch();
        else ResetFeederCoexControlSwitch();
    } else ResetFeederCoexControlSwitch();
}// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="check analog input">

void CheckAnalogInput(void) {
    //------------------------------------------- wait for sys clock flag
    if (!IsSysClockTicked()) return;
    //-------------------------------------------
    TMR1Interrupt_Disable();
    if (an_in_pcnt == 0) {
        ADC_InitCH0();
        DelayUs(4);
        ADC_Convert();
        while (ADC_Busy()) continue;
        vp1_delay_time = ADRESH;
        vp1_delay_time = vp1_delay_time << 8;
        vp1_delay_time += ADRESL;
        vp1_delay_time = (1023 - vp1_delay_time);
        if (IsDriveOptionInverter()) vp1_delay_time = vp1_delay_time * 6; // 0 - 60s vp1 speed reduction delay for inverter drive
        else if (IsDriveOptionStarDeltaStarter()) vp1_delay_time = vp1_delay_time * 60; // 0 - 10min speed reduction delay for DY starter
        ++an_in_pcnt;
    } else if (an_in_pcnt == 1) {
        ADC_InitCH1();
        DelayUs(4);
        ADC_Convert();
        while (ADC_Busy()) continue;
        vp1_production_speed = ADRESH;
        vp1_production_speed = vp1_production_speed << 8;
        vp1_production_speed += ADRESL;
        vp1_production_speed = (1023 - vp1_production_speed);
        ++an_in_pcnt;
    } else if (an_in_pcnt == 2) {
        ADC_InitCH2();
        DelayUs(4);
        ADC_Convert();
        while (ADC_Busy()) continue;
        vp1_idle_speed = ADRESH;
        vp1_idle_speed = vp1_idle_speed << 8;
        vp1_idle_speed += ADRESL;
        vp1_idle_speed = (1023 - vp1_idle_speed);
        if(vp1_idle_speed > vp1_production_speed) vp1_idle_speed = vp1_production_speed;
        ++an_in_pcnt;
    } else if (an_in_pcnt == 3) {
        ADC_InitCH3();
        DelayUs(4);
        ADC_Convert();
        while (ADC_Busy()) continue;
        max_charging_time = ADRESH;
        max_charging_time = max_charging_time << 8;
        max_charging_time += ADRESL;
        max_charging_time = (1023 - max_charging_time);
        max_charging_time = max_charging_time * 30;
        ++an_in_pcnt;
    } else if (an_in_pcnt == 4) {
        ADC_InitCH4();
        DelayUs(4);
        ADC_Convert();
        while (ADC_Busy()) continue;
        mill_shutdown_delay = ADRESH;
        mill_shutdown_delay = mill_shutdown_delay << 8;
        mill_shutdown_delay += ADRESL;
        mill_shutdown_delay = (1023 - mill_shutdown_delay);
        mill_shutdown_delay = mill_shutdown_delay * 30;
        ++an_in_pcnt;
    } else if (an_in_pcnt == 5) {
        ADC_InitCH5();
        DelayUs(4);
        ADC_Convert();
        while (ADC_Busy()) continue;
        feeder4_charging_time = ADRESH;
        feeder4_charging_time = feeder4_charging_time << 8;
        feeder4_charging_time += ADRESL;
        feeder4_charging_time = (1023 - feeder4_charging_time);
        feeder4_charging_time = feeder4_charging_time * 18;
        ++an_in_pcnt;
    } else if (an_in_pcnt == 6) {
        ADC_InitCH6();
        DelayUs(4);
        ADC_Convert();
        while (ADC_Busy()) continue;
        vp1_cleaning_cycles = ADRESH;
        vp1_cleaning_cycles = vp1_cleaning_cycles << 8;
        vp1_cleaning_cycles += ADRESL;
        vp1_cleaning_cycles = (1023 - vp1_cleaning_cycles);
        vp1_cleaning_cycles = (vp1_cleaning_cycles / 2);
        an_in_pcnt = 0;
    }// End of analog
    TMR1Interrupt_Enable();
}// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="set relay output">

void SetRelayOutput(void) {
    //------------------------------------------- wait for sys clock flag
    if (!IsSysClockTicked()) return;
    else ++relay_pcnt;
    if(relay_pcnt != 5) return;
    else relay_pcnt = 0;
    //------------------------------------------- refresh relay ouptut
    TMR1Interrupt_Disable();
    DisableDIN_0_7();
    DisableDIN_8_15();
    DisableDIN_16_23();
    DisableDIN_24_31();
    SetDataPortAsOutput();
    DataWrite() = output_0_7._byte;
    ClockOUT1();
    DataWrite() = output_8_15._byte;
    ClockOUT2();
    EnableDO();
    SetDataPortAsInput();
    TMR1Interrupt_Enable();
}// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="set analog output">

void SetAnalogOutput(void) {
    //------------------------------------------- wait for sys clock flag
    if (!IsSysClockTicked()) return;
    //-------------------------------------------
    CCPR1L = (an0_out >> 2);
    if (an0_out & 0x0001) DC1B0 = 1;
    else DC1B0 = 0;
    if (an0_out & 0x0002) DC1B1 = 1;
    else DC1B1 = 0;
    CCPR2L = (an1_out >> 2);
    if (an1_out & 0x0001) DC2B0 = 1;
    else DC2B0 = 0;
    if (an1_out & 0x0002) DC2B1 = 1;
    else DC2B1 = 0;
}// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="set signal output">

void SetSignalOutput(void) {
    //------------------------------------------- wait for sys clock flag
    if (!IsSysClockTicked()) return;
    else ++sigout_pcnt;
    if(sigout_pcnt != 5) return;
    else sigout_pcnt = 0;
    //-------------------------------------------
    // <editor-fold defaultstate="collapsed" desc="led signal feeder 1 state">

    switch (feeder1_state) {

        case FEEDER1_OFF:
            Feeder1StatusLED_Off();
            break;

        case FEEDER1_DISCHARGING:
            Feeder1StatusLED_On();
            break;

        case FEEDER1_CHARGING:
            if ((activ_unit != FEEDER1) || !IsVacumPump1Ready()) {
                if (SYSCLK_TICK_1S) Feeder1StatusLED_On();
                else Feeder1StatusLED_Off();
            } else {
                if (SYSCLK_TICK_1S || SYSCLK_TICK_500MS) Feeder1StatusLED_On();
                else Feeder1StatusLED_Off();
            }// End of else
            break;

        case FEEDER1_ERROR:
            if (SYSCLK_TICK_100MS) Feeder1StatusLED_On();
            else Feeder1StatusLED_Off();
            break;
    }// </editor-fold>

    // <editor-fold defaultstate="collapsed" desc="led signal feeder 2 state">

    switch (feeder2_state) {

        case FEEDER2_OFF:
            Feeder2StatusLED_Off();
            break;

        case FEEDER2_DISCHARGING:
            Feeder2StatusLED_On();
            break;

        case FEEDER2_CHARGING:
            if ((activ_unit != FEEDER2) || !IsVacumPump1Ready()) {
                if (SYSCLK_TICK_1S) Feeder2StatusLED_On();
                else Feeder2StatusLED_Off();
            } else {
                if (SYSCLK_TICK_1S || SYSCLK_TICK_500MS) Feeder2StatusLED_On();
                else Feeder2StatusLED_Off();
            }// End of else
            break;

        case FEEDER2_ERROR:
            if (SYSCLK_TICK_100MS) Feeder2StatusLED_On();
            else Feeder2StatusLED_Off();
            break;
    }// </editor-fold>

    // <editor-fold defaultstate="collapsed" desc="led signal feeder 3 state">

    switch (feeder3_state) {

        case FEEDER3_OFF:
            Feeder3StatusLED_Off();
            break;

        case FEEDER3_DISCHARGING:
            Feeder3StatusLED_On();
            break;

        case FEEDER3_CHARGING:
            if ((activ_unit != FEEDER3) || !IsVacumPump1Ready()) {
                if (SYSCLK_TICK_1S) Feeder3StatusLED_On();
                else Feeder3StatusLED_Off();
            } else {
                if (SYSCLK_TICK_1S || SYSCLK_TICK_500MS) Feeder3StatusLED_On();
                else Feeder3StatusLED_Off();
            }// End of else
            break;

        case FEEDER3_ERROR:
            if (SYSCLK_TICK_100MS) Feeder3StatusLED_On();
            else Feeder3StatusLED_Off();
            break;
    }// </editor-fold>

    // <editor-fold defaultstate="collapsed" desc="led signal feeder 4 state">

    switch (feeder4_state) {

        case FEEDER4_OFF:
            Feeder4StatusLED_Off();
            break;

        case FEEDER4_DISCHARGING:
            Feeder4StatusLED_On();
            break;

        case FEEDER4_CHARGING:
            if ((activ_unit != FEEDER4) || !IsVacumPump1Ready()) {
                if (SYSCLK_TICK_1S) Feeder4StatusLED_On();
                else Feeder4StatusLED_Off();
            } else {
                if (SYSCLK_TICK_1S || SYSCLK_TICK_500MS) Feeder4StatusLED_On();
                else Feeder4StatusLED_Off();
            }// End of else
            break;

        case FEEDER4_ERROR:
            if (SYSCLK_TICK_100MS) Feeder4StatusLED_On();
            else Feeder4StatusLED_Off();
            break;
    }// </editor-fold>

    // <editor-fold defaultstate="collapsed" desc="led signal feeder 5 state">

    switch (feeder5_state) {

        case FEEDER5_OFF:
            Feeder5StatusLED_Off();
            break;

        case FEEDER5_DISCHARGING:
            Feeder5StatusLED_On();
            break;

        case FEEDER5_CHARGING:
            if ((activ_unit != FEEDER5) || !IsVacumPump1Ready()) {
                if (SYSCLK_TICK_1S) Feeder5StatusLED_On();
                else Feeder5StatusLED_Off();
            } else {
                if (SYSCLK_TICK_1S || SYSCLK_TICK_500MS) Feeder5StatusLED_On();
                else Feeder5StatusLED_Off();
            }// End of else
            break;

        case FEEDER5_ERROR:
            if (SYSCLK_TICK_100MS) Feeder5StatusLED_On();
            else Feeder5StatusLED_Off();
            break;
    }// </editor-fold>

    // <editor-fold defaultstate="collapsed" desc="led signal feeder coex. state">

    switch (feeder_coex_state) {

        case FEEDER_COEX_OFF:
            FeederCoexStatusLED_Off();
            break;

        case FEEDER_COEX_DISCHARGING:
            FeederCoexStatusLED_On();
            break;

        case FEEDER_COEX_CHARGING:
            if ((activ_unit != FEEDER_COEX) || !IsVacumPump1Ready()) {
                if (SYSCLK_TICK_1S) FeederCoexStatusLED_On();
                else FeederCoexStatusLED_Off();
            } else {
                if (SYSCLK_TICK_1S || SYSCLK_TICK_500MS) FeederCoexStatusLED_On();
                else FeederCoexStatusLED_Off();
            }// End of else
            break;

        case FEEDER_COEX_ERROR:
            if (SYSCLK_TICK_100MS) FeederCoexStatusLED_On();
            else FeederCoexStatusLED_Off();
            break;
    }// </editor-fold>

    // <editor-fold defaultstate="collapsed" desc="led signal vacum pump 1 state">
    switch (vacum_pump1_state) {

        case VP1_OFF:
            VacumPump1StatusLED_Off();
            break;

        case VP1_START:
            if (SYSCLK_TICK_1S) VacumPump1StatusLED_On();
            else VacumPump1StatusLED_Off();
            break;

        case VP1_IDLE:
            if (SYSCLK_TICK_1S) VacumPump1StatusLED_On();
            else VacumPump1StatusLED_Off();
            break;

        case VP1_RUN:
            if (!IsVacumPump1Ready()) {
                if (SYSCLK_TICK_1S) VacumPump1StatusLED_On();
                else VacumPump1StatusLED_Off();
            } else VacumPump1StatusLED_On();
            break;

        case VP1_ERROR:
            if (SYSCLK_TICK_100MS) VacumPump1StatusLED_On();
            else VacumPump1StatusLED_Off();
            break;
    }// </editor-fold>

    // <editor-fold defaultstate="collapsed" desc="led control state">
    if (!IsEmergencyProtectionActiv()) {
        if (SYSCLK_TICK_100MS) ControlStatusLED_On();
        else ControlStatusLED_Off();
    } else ControlStatusLED_On();
    // </editor-fold>

    // <editor-fold defaultstate="collapsed" desc="7 segment led error display">

    /*  ----      ----
     * |    |    |    |
     *  ----      ----
     * |    |    |    |
     *  ----      ----
     *   |          |----------- 1 CHARGING TIMER TIMEOUT
     *   |          |----------- 2 FLAP SENSOR
     *   |          |----------- 3 RAW LEVEL MAKSIMUM
     *   |          |----------- 4 OVERCURRENT PROTECTION ACTIVATE
     *   |          |----------- 5 THERMAL OVERHEAT PROTECTION ACTIVATE
     *   |          |----------- 6 RAW LEVEL MINIMUM
     *   |          |----------- 7 MILL DISABLED
     *   |          |----------- 8 EMERGENCY ACTIVATED
     *   |          |----------- 9
     *   |          |----------- 0
     *   |---------------------- 1 FEEDER 1
     *   |---------------------- 2 FEEDER 2
     *   |---------------------- 3 FEEDER 3
     *   |---------------------- 4 FEEDER 4
     *   |---------------------- 5 FEEDER 5
     *   |---------------------- 6 FEEDER COEX.
     *   |---------------------- 7 VACUM PUMP 1
     *   |---------------------- 8 CYCLON
     *   |---------------------- 9 CONTROLER
     *   |---------------------- 0
     */
    if (!IsVacumPump1FlapSensorErrorActiv() && !IsVacumPump1OvercurrentErrorActiv() && !IsVacumPump1OverheatErrorActiv() \
                && !IsFeeder1ChargingTimeErrorActiv() && !IsFeeder1FlapSensorErrorActiv() && !IsFeeder2ChargingTimeErrorActiv() \
                && !IsFeeder2FlapSensorErrorActiv() && !IsFeeder3ChargingTimeErrorActiv() && !IsFeeder3FlapSensorErrorActiv() \
                && !IsFeeder4FlapSensorErrorActiv() && !IsFeeder5ChargingTimeErrorActiv() && !IsFeeder5FlapSensorErrorActiv() \
                && !IsFeederCoexChargingTimeErrorActiv() && !IsFeederCoexFlapSensorErrorActiv() && !IsFeeder4RawLevelMinErrorActiv() \
                && !IsCyclon1RawLevelMaxErrorActiv() && !IsMillDisabled() && IsEmergencyProtectionActiv()) {
        ClearErrorDisplay();
    }// End of if...
    if (!IsErrorDisplayItemTimerExpired()) {
        /* do nothing with error display until display timer expired */
    } else {
        if (error_display_pcnt == 0) {
            if (IsVacumPump1FlapSensorErrorActiv()) {
                DisplayTens_7();
                DisplayUnits_2();
                SetTimer(ERROR_DISPLAY_TMR, ERROR_ITEM_DISPLAY_TIME);
            }// End of if...
            ++error_display_pcnt;
        } else if (error_display_pcnt == 1) {
            if (IsVacumPump1OvercurrentErrorActiv()) {
                DisplayTens_7();
                DisplayUnits_4();
                SetTimer(ERROR_DISPLAY_TMR, ERROR_ITEM_DISPLAY_TIME);
            }// end of if...
            ++error_display_pcnt;
        } else if (error_display_pcnt == 2) {
            if (IsVacumPump1OverheatErrorActiv()) {
                DisplayTens_7();
                DisplayUnits_5();
                SetTimer(ERROR_DISPLAY_TMR, ERROR_ITEM_DISPLAY_TIME);
            }// end of if...
            ++error_display_pcnt;
        } else if (error_display_pcnt == 3) {
            if (IsFeeder1ChargingTimeErrorActiv()) {
                DisplayTens_1();
                DisplayUnits_1();
                SetTimer(ERROR_DISPLAY_TMR, ERROR_ITEM_DISPLAY_TIME);
            }// end of if...
            ++error_display_pcnt;
        } else if (error_display_pcnt == 4) {
            if (IsFeeder1FlapSensorErrorActiv()) {
                DisplayTens_1();
                DisplayUnits_2();
                SetTimer(ERROR_DISPLAY_TMR, ERROR_ITEM_DISPLAY_TIME);
            }// end of if...
            ++error_display_pcnt;
        } else if (error_display_pcnt == 5) {
            if (IsFeeder2ChargingTimeErrorActiv()) {
                DisplayTens_2();
                DisplayUnits_1();
                SetTimer(ERROR_DISPLAY_TMR, ERROR_ITEM_DISPLAY_TIME);
            }// end of if...
            ++error_display_pcnt;
        } else if (error_display_pcnt == 6) {
            if (IsFeeder2FlapSensorErrorActiv()) {
                DisplayTens_2();
                DisplayUnits_2();
                SetTimer(ERROR_DISPLAY_TMR, ERROR_ITEM_DISPLAY_TIME);
            }// end of if...
            ++error_display_pcnt;
        } else if (error_display_pcnt == 7) {
            if (IsFeeder3ChargingTimeErrorActiv()) {
                DisplayTens_3();
                DisplayUnits_1();
                SetTimer(ERROR_DISPLAY_TMR, ERROR_ITEM_DISPLAY_TIME);
            }// end of if...
            ++error_display_pcnt;
        } else if (error_display_pcnt == 8) {
            if (IsFeeder3FlapSensorErrorActiv()) {
                DisplayTens_3();
                DisplayUnits_2();
                SetTimer(ERROR_DISPLAY_TMR, ERROR_ITEM_DISPLAY_TIME);
            }// end of if...
            ++error_display_pcnt;
        } else if (error_display_pcnt == 9) {
            if (IsFeeder4FlapSensorErrorActiv()) {
                DisplayTens_4();
                DisplayUnits_2();
                SetTimer(ERROR_DISPLAY_TMR, ERROR_ITEM_DISPLAY_TIME);
            }// end of if...
            ++error_display_pcnt;
        } else if (error_display_pcnt == 10) {
            if (IsFeeder5ChargingTimeErrorActiv()) {
                DisplayTens_5();
                DisplayUnits_1();
                SetTimer(ERROR_DISPLAY_TMR, ERROR_ITEM_DISPLAY_TIME);
            }// end of if...
            ++error_display_pcnt;
        } else if (error_display_pcnt == 11) {
            if (IsFeeder5FlapSensorErrorActiv()) {
                DisplayTens_5();
                DisplayUnits_2();
                SetTimer(ERROR_DISPLAY_TMR, ERROR_ITEM_DISPLAY_TIME);
            }// end of if...
            ++error_display_pcnt;
        } else if (error_display_pcnt == 12) {
            if (IsFeederCoexChargingTimeErrorActiv()) {
                DisplayTens_6();
                DisplayUnits_1();
                SetTimer(ERROR_DISPLAY_TMR, ERROR_ITEM_DISPLAY_TIME);
            }// end of if...
            ++error_display_pcnt;
        } else if (error_display_pcnt == 13) {
            if (IsFeederCoexFlapSensorErrorActiv()) {
                DisplayTens_6();
                DisplayUnits_2();
                SetTimer(ERROR_DISPLAY_TMR, ERROR_ITEM_DISPLAY_TIME);
            }// end of if...
            ++error_display_pcnt;
        } else if (error_display_pcnt == 14) {
            if (IsFeeder4RawLevelMinErrorActiv()) {
                DisplayTens_4();
                DisplayUnits_6();
                SetTimer(ERROR_DISPLAY_TMR, ERROR_ITEM_DISPLAY_TIME);
            }// end of if...
            ++error_display_pcnt;
        } else if (error_display_pcnt == 15) {
            if (IsCyclon1RawLevelMaxErrorActiv()) {
                DisplayTens_8();
                DisplayUnits_3();
                SetTimer(ERROR_DISPLAY_TMR, ERROR_ITEM_DISPLAY_TIME);
            }// End of if...
            ++error_display_pcnt;
        } else if (error_display_pcnt == 16) {
            if (IsMillDisabled()) {
                DisplayTens_8();
                DisplayUnits_7();
                SetTimer(ERROR_DISPLAY_TMR, ERROR_ITEM_DISPLAY_TIME);
            }// End of if...
            ++error_display_pcnt;
        } else if (error_display_pcnt == 17) {
            if (!IsEmergencyProtectionActiv()) {
                DisplayTens_9();
                DisplayUnits_8();
                SetTimer(ERROR_DISPLAY_TMR, ERROR_ITEM_DISPLAY_TIME);
            }// End of if...
            error_display_pcnt = 0;
        }// End of else if... 
    }// End of else...
    // </editor-fold>

    // <editor-fold defaultstate="collapsed" desc="signal output refresh">
    DisableDIN_0_7();
    DisableDIN_8_15();
    DisableDIN_16_23();
    DisableDIN_24_31();
    SetDataPortAsOutput();
    SetSig1Clk_Low();
    SetSig2Clk_Low();
    DataWrite() = signal_0_7._byte;
    SetSig1Clk_High();
    SetSig1Clk_Low();
    DataWrite() = signal_8_15._byte;
    SetSig2Clk_High();
    SetSig2Clk_Low();
    SetDataPortAsInput();
    // </editor-fold>
}// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="set timer">

void SetTimer(int timer_for_setup, unsigned int period) {

    switch (timer_for_setup) {

        case VP1_TMR:
            vp1_tmr = period;
            VP1_TEX = FALSE;
            break;

        case PISTON_TMR:
            piston_tmr = period;
            PISTON_TEX = FALSE;
            break;

        case MAX_CHARGING_TMR:
            max_charging_tmr = period;
            MAX_CHARGING_TEX = FALSE;
            break;

        case ERROR_DISPLAY_TMR:
            error_display_tmr = period;
            ERROR_DISPLAY_TEX = FALSE;
            break;

        case FEEDER4_CHARGING_TMR:
            feeder4_charging_tmr = period;
            FEEDER4_CHARGING_TEX = FALSE;
            break;

        case MILL_SHUTDOWN_DELAY_TMR:
            mill_shutdown_delay_tmr = period;
            MILL_SHUTDOWN_DELAY_TEX = FALSE;
            break;

        case FEEDER2_CLEANING_PISTON_TMR:
            feeder2_cleaning_piston_tmr = period;
            FEEDER2_CLEANING_PISTON_TEX = FALSE;
            break;

        case FEEDER_STATE_CHANGE_TMR:
            feeder_state_change_tmr = period;
            FEEDER_STATE_CHANGE_TEX = FALSE;
            break;

        case VP1_CONTROL_DELAY_TMR:
            vp1_control_delay_tmr = period;
            VP1_CONTROL_DELAY_TEX = FALSE;
            break;

        case FEEDER1_CONTROL_DELAY_TMR:
            feeder1_control_delay_tmr = period;
            FEEDER1_CONTROL_DELAY_TEX = FALSE;
            break;

        case FEEDER2_CONTROL_DELAY_TMR:
            feeder2_control_delay_tmr = period;
            FEEDER2_CONTROL_DELAY_TEX = FALSE;
            break;

        case FEEDER3_CONTROL_DELAY_TMR:
            feeder3_control_delay_tmr = period;
            FEEDER3_CONTROL_DELAY_TEX = FALSE;
            break;

        case FEEDER4_CONTROL_DELAY_TMR:
            feeder4_control_delay_tmr = period;
            FEEDER4_CONTROL_DELAY_TEX = FALSE;
            break;

        case FEEDER5_CONTROL_DELAY_TMR:
            feeder5_control_delay_tmr = period;
            FEEDER5_CONTROL_DELAY_TEX = FALSE;
            break;

        case FEEDER_COEX_CONTROL_DELAY_TMR:
            feeder_coex_control_delay_tmr = period;
            FEEDER_COEX_CONTROL_DELAY_TEX = FALSE;
            break;
    }// End of switch
}// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="hc595 shift">

void HC595_Shift(void) {
    if (STATUS_LED) DOUT_SERIAL_DATA = HIGH;
    else DOUT_SERIAL_DATA = LOW;
    ClockShift();
    if (OUTPUT_ENABLE) DOUT_SERIAL_DATA = HIGH;
    else DOUT_SERIAL_DATA = LOW;
    ClockShift();
    if (OUT2_CLK) DOUT_SERIAL_DATA = HIGH;
    else DOUT_SERIAL_DATA = LOW;
    ClockShift();
    if (OUT1_CLK) DOUT_SERIAL_DATA = HIGH;
    else DOUT_SERIAL_DATA = LOW;
    ClockShift();
    if (INPUT_24_31) DOUT_SERIAL_DATA = HIGH;
    else DOUT_SERIAL_DATA = LOW;
    ClockShift();
    if (INPUT_16_23) DOUT_SERIAL_DATA = HIGH;
    else DOUT_SERIAL_DATA = LOW;
    ClockShift();
    if (INPUT_8_15) DOUT_SERIAL_DATA = HIGH;
    else DOUT_SERIAL_DATA = LOW;
    ClockShift();
    if (INPUT_0_7) DOUT_SERIAL_DATA = HIGH;
    else DOUT_SERIAL_DATA = LOW;
    ClockShift();
    ClockLatch();
    EnableShiftOutput();
}// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="check safety">

void CheckSafety(void) {

    if (!IsEmergencyProtectionActiv()) {
        vacum_pump1_state = VP1_OFF;
        feeder1_state = FEEDER1_OFF;
        feeder2_state = FEEDER2_OFF;
        feeder3_state = FEEDER3_OFF;
        feeder4_state = FEEDER4_OFF;
        feeder5_state = FEEDER5_OFF;
        feeder_coex_state = FEEDER_COEX_OFF;
        if (SYSCLK_TICK_2S && SYSCLK_TICK_1S) AlarmSignal_On();
        else AlarmSignal_Off();
        StatusLED_Stop();
    } else if (IsVacumPump1FlapSensorErrorActiv() || IsVacumPump1OverheatErrorActiv() || IsVacumPump1OvercurrentErrorActiv() \
            || IsFeeder1FlapSensorErrorActiv() || IsFeeder2FlapSensorErrorActiv() || IsFeeder3FlapSensorErrorActiv() \
            || IsFeeder4FlapSensorErrorActiv() || IsFeeder5FlapSensorErrorActiv() || IsFeederCoexFlapSensorErrorActiv()) {
        AlarmSignal_On();
        StatusLED_Error();
    } else if (IsCyclon1RawLevelMaxErrorActiv() || IsFeeder1ChargingTimeErrorActiv() || IsFeeder2ChargingTimeErrorActiv() \
            || IsFeeder3ChargingTimeErrorActiv() || IsFeeder4RawLevelMinErrorActiv() || IsFeeder5ChargingTimeErrorActiv() \
            || IsFeederCoexChargingTimeErrorActiv()) {
        if (SYSCLK_TICK_2S) AlarmSignal_On();
        else AlarmSignal_Off();
        StatusLED_Error();
    } else {
        StatusLED_Run();
        AlarmSignal_Off();
    }// End of else...

    if (IsFeeder1ChargingTimeErrorActiv() || IsFeeder1FlapSensorErrorActiv()) {
        if (SYSCLK_TICK_1S) Feeder1AlarmSignal_On();
        else Feeder1AlarmSignal_Off();
    } else Feeder1AlarmSignal_Off();

    if (IsFeeder4FlapSensorErrorActiv() || IsFeeder4RawLevelMinErrorActiv()) {
        if (SYSCLK_TICK_1S) Feeder4AlarmSignal_On();
        else Feeder4AlarmSignal_Off();
    } else Feeder4AlarmSignal_Off();
}// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="process feeder 1">

void ProcessFeeder1(void) {

    switch (feeder1_state) {

        case FEEDER1_OFF:
            if (activ_unit == FEEDER1) activ_unit = FEEDER2;
            if (IsFeeder1ControlSwitchPressed() && IsFeeder1ControlDelayTimerExpired()) {
                SetTimer(FEEDER1_CONTROL_DELAY_TMR, FEEDER_CONTROL_DELAY_TIME);
                feeder1_state = FEEDER1_DISCHARGING;
            }// End of if...
            Feeder1VacumReleaseValve_Off();
            ClearFeeder1Error();
            break;

        case FEEDER1_DISCHARGING:
            if (IsFeeder1ControlSwitchPressed() && IsFeeder1ControlDelayTimerExpired()) {
                SetTimer(FEEDER1_CONTROL_DELAY_TMR, FEEDER_CONTROL_DELAY_TIME);
                feeder1_state = FEEDER1_OFF;
            }// End of if...
            if (IsFeeder1SensorRequestActiv() && !IsFeeder1SensorRawLevelMaxActiv() && IsFeederStateChangeTimerExpired()) {
                feeder1_state = FEEDER1_CHARGING;
            } else if (activ_unit == FEEDER1) activ_unit = FEEDER2;
            feeder1_pcnt = 0;
            break;

        case FEEDER1_CHARGING:
            if (IsFeeder1ControlSwitchPressed() && IsFeeder1ControlDelayTimerExpired()) {
                SetTimer(FEEDER1_CONTROL_DELAY_TMR, FEEDER_CONTROL_DELAY_TIME);
                feeder1_state = FEEDER1_OFF;
            }// End of if...
            if (activ_unit != FEEDER1) {
                break;
            } else if (feeder1_pcnt == 0) {
                SetVacumPump1Request();
                ++feeder1_pcnt;
            } else if ((feeder1_pcnt == 1) && IsVacumPump1Ready()) {
                Feeder1VacumReleaseValve_On();
                SetTimer(PISTON_TMR, PISTON_MOVE_TIME);
                ++feeder1_pcnt;
            } else if ((feeder1_pcnt == 2) && IsPistonMoveTimerExpired()) {
                SetTimer(MAX_CHARGING_TMR, max_charging_time);
                ++feeder1_pcnt;
            } else if (feeder1_pcnt == 3) {
                if (IsFeeder1SensorRawLevelMaxActiv() || !IsVacumPump1Ready()) {
                    Feeder1VacumReleaseValve_Off();
                    ResetFeeder1ChargingTimeError();
                    SetTimer(FEEDER_STATE_CHANGE_TMR, FEEDER_STATE_CHANGE_TIME);
                    feeder1_state = FEEDER1_DISCHARGING;
                    feeder1_pcnt = 0;
                } else if (!IsFeeder1SensorRequestActiv()) {
                    Feeder1VacumReleaseValve_Off();
                    SetFeeder1FlapSensorError();
                    feeder1_state = FEEDER1_ERROR;
                } else if (IsFeederMaxChargingTimeTimerExpired()) {
                    Feeder1VacumReleaseValve_Off();
                    SetFeeder1ChargingTimeError();
                    SetTimer(FEEDER_STATE_CHANGE_TMR, FEEDER_STATE_CHANGE_TIME);
                    feeder1_state = FEEDER1_DISCHARGING;
                    feeder1_pcnt = 0;
                }// End of else if...
            }// End of else if...
            break;

        case FEEDER1_ERROR:
            if (activ_unit == FEEDER1) activ_unit = FEEDER2;
            if (IsFeeder1ControlSwitchPressed() && IsFeeder1ControlDelayTimerExpired()) {
                SetTimer(FEEDER1_CONTROL_DELAY_TMR, FEEDER_CONTROL_DELAY_TIME);
                feeder1_state = FEEDER1_OFF;
            }// End of if...
            Feeder1VacumReleaseValve_Off();
            break;
    }// End of switch...
}// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="process feeder 2">

void ProcessFeeder2(void) {

    switch (feeder2_state) {

        case FEEDER2_OFF:
            if (activ_unit == FEEDER2) activ_unit = FEEDER3;
            if (IsFeeder2ControlSwitchPressed() && IsFeeder2ControlDelayTimerExpired()) {
                SetTimer(FEEDER2_CONTROL_DELAY_TMR, FEEDER_CONTROL_DELAY_TIME);
                feeder2_state = FEEDER2_DISCHARGING;
            }// End of if...
            Feeder2CleaningPistonValve_Off();
            Feeder2VacumReleaseValve_Off();
            ClearFeeder2Error();
            feeder2_pcnt = 0;
            break;

        case FEEDER2_DISCHARGING:
            if (IsFeeder2ControlSwitchPressed() && IsFeeder2ControlDelayTimerExpired()) {
                SetTimer(FEEDER2_CONTROL_DELAY_TMR, FEEDER_CONTROL_DELAY_TIME);
                feeder2_state = FEEDER2_OFF;
            }// End of if...
            if ((feeder2_pcnt == 0) && IsFeederStateChangeTimerExpired()) {
                if (activ_unit == FEEDER2) activ_unit = FEEDER3;
                SetTimer(FEEDER2_CLEANING_PISTON_TMR, FEEDER2_CLEANING_PISTON_CYCLUS_TIME);
                if (!IsFeeder2SensorRequestActiv()) {
                    SetTimer(PISTON_TMR, PISTON_MOVE_TIME);
                    Feeder2CleaningPistonValve_On();
                    ++feeder2_pcnt;
                } else feeder2_pcnt = 2;
            } else if ((feeder2_pcnt == 1) && IsPistonMoveTimerExpired()) {
                if (activ_unit == FEEDER2) activ_unit = FEEDER3;
                SetTimer(PISTON_TMR, PISTON_MOVE_TIME);
                Feeder2CleaningPistonValve_Off();
                ++feeder2_pcnt;
            } else if (feeder2_pcnt == 2) {
                if (IsFeeder2CleaningPistonTimerExpired()) feeder2_pcnt = 0;
                else if (IsFeeder2SensorRequestActiv() && !IsFeeder2SensorRawLevelMaxActiv() && IsFeederStateChangeTimerExpired()) {
                    feeder2_state = FEEDER2_CHARGING;
                    feeder2_pcnt = 0;
                } else if (activ_unit == FEEDER2) activ_unit = FEEDER3;
            }// End of else if... 
            break;

        case FEEDER2_CHARGING:
            if (IsFeeder2ControlSwitchPressed() && IsFeeder2ControlDelayTimerExpired()) {
                SetTimer(FEEDER2_CONTROL_DELAY_TMR, FEEDER_CONTROL_DELAY_TIME);
                feeder2_state = FEEDER2_OFF;
            }// End of if...
            if (activ_unit != FEEDER2) {
                break;
            } else if (feeder2_pcnt == 0) {
                SetVacumPump1Request();
                ++feeder2_pcnt;
            } else if ((feeder2_pcnt == 1) && IsVacumPump1Ready()) {
                Feeder2VacumReleaseValve_On();
                SetTimer(PISTON_TMR, PISTON_MOVE_TIME);
                ++feeder2_pcnt;
            } else if ((feeder2_pcnt == 2) && IsPistonMoveTimerExpired()) {
                SetTimer(MAX_CHARGING_TMR, max_charging_time);
                ++feeder2_pcnt;
            } else if (feeder2_pcnt == 3) {
                if (IsFeeder2SensorRawLevelMaxActiv() || !IsVacumPump1Ready()) {
                    Feeder2VacumReleaseValve_Off();
                    ResetFeeder2ChargingTimeError();
                    SetTimer(FEEDER_STATE_CHANGE_TMR, FEEDER_STATE_CHANGE_TIME);
                    feeder2_state = FEEDER2_DISCHARGING;
                    feeder2_pcnt = 0;
                } else if (!IsFeeder2SensorRequestActiv()) {
                    Feeder2VacumReleaseValve_Off();
                    feeder2_state = FEEDER2_ERROR;
                    SetFeeder2FlapSensorError();
                } else if (IsFeederMaxChargingTimeTimerExpired()) {
                    Feeder2VacumReleaseValve_Off();
                    SetTimer(FEEDER_STATE_CHANGE_TMR, FEEDER_STATE_CHANGE_TIME);
                    feeder2_state = FEEDER2_DISCHARGING;
                    SetFeeder2ChargingTimeError();
                    feeder2_pcnt = 0;
                }// End of else if...
            }// End of else if...
            break;

        case FEEDER2_ERROR:
            if (activ_unit == FEEDER2) activ_unit = FEEDER3;
            if (IsFeeder2ControlSwitchPressed() && IsFeeder2ControlDelayTimerExpired()) {
                SetTimer(FEEDER2_CONTROL_DELAY_TMR, FEEDER_CONTROL_DELAY_TIME);
                feeder2_state = FEEDER2_OFF;
            }// End of if...
            Feeder2CleaningPistonValve_Off();
            Feeder2VacumReleaseValve_Off();
            break;
    }// End of switch...
}// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="process feeder 3">

void ProcessFeeder3(void) {

    switch (feeder3_state) {

        case FEEDER3_OFF:
            if (activ_unit == FEEDER3) activ_unit = FEEDER4;
            if (IsFeeder3ControlSwitchPressed() && IsFeeder3ControlDelayTimerExpired()) {
                SetTimer(FEEDER3_CONTROL_DELAY_TMR, FEEDER_CONTROL_DELAY_TIME);
                feeder3_state = FEEDER3_DISCHARGING;
            }// End of if...
            Feeder3VacumReleaseValve_Off();
            ClearFeeder3Error();
            break;

        case FEEDER3_DISCHARGING:
            if (IsFeeder3ControlSwitchPressed() && IsFeeder3ControlDelayTimerExpired()) {
                SetTimer(FEEDER3_CONTROL_DELAY_TMR, FEEDER_CONTROL_DELAY_TIME);
                feeder3_state = FEEDER3_OFF;
            }// End of if...
            if (IsFeeder3SensorRequestActiv() && !IsFeeder3SensorRawLevelMaxActiv() && IsFeederStateChangeTimerExpired()) {
                feeder3_state = FEEDER3_CHARGING;
            } else if (activ_unit == FEEDER3) activ_unit = FEEDER4;
            feeder3_pcnt = 0;
            break;

        case FEEDER3_CHARGING:
            if (IsFeeder3ControlSwitchPressed() && IsFeeder3ControlDelayTimerExpired()) {
                SetTimer(FEEDER3_CONTROL_DELAY_TMR, FEEDER_CONTROL_DELAY_TIME);
                feeder3_state = FEEDER3_OFF;
            }// End of if...
            if (activ_unit != FEEDER3) {
                break;
            } else if (feeder3_pcnt == 0) {
                SetVacumPump1Request();
                ++feeder3_pcnt;
            } else if ((feeder3_pcnt == 1) && IsVacumPump1Ready()) {
                Feeder3VacumReleaseValve_On();
                SetTimer(PISTON_TMR, PISTON_MOVE_TIME);
                ++feeder3_pcnt;
            } else if ((feeder3_pcnt == 2) && IsPistonMoveTimerExpired()) {
                SetTimer(MAX_CHARGING_TMR, max_charging_time);
                ++feeder3_pcnt;
            } else if (feeder3_pcnt == 3) {
                if (IsFeeder3SensorRawLevelMaxActiv() || !IsVacumPump1Ready()) {
                    Feeder3VacumReleaseValve_Off();
                    ResetFeeder3ChargingTimeError();
                    SetTimer(FEEDER_STATE_CHANGE_TMR, FEEDER_STATE_CHANGE_TIME);
                    feeder3_state = FEEDER3_DISCHARGING;
                    feeder3_pcnt = 0;
                } else if (!IsFeeder3SensorRequestActiv()) {
                    Feeder3VacumReleaseValve_Off();
                    SetFeeder3FlapSensorError();
                    feeder3_state = FEEDER3_ERROR;
                } else if (IsFeederMaxChargingTimeTimerExpired()) {
                    Feeder3VacumReleaseValve_Off();
                    SetTimer(FEEDER_STATE_CHANGE_TMR, FEEDER_STATE_CHANGE_TIME);
                    feeder3_state = FEEDER3_DISCHARGING;
                    SetFeeder3ChargingTimeError();
                    feeder3_pcnt = 0;
                }// End of else if...
            }// End of else if...
            break;

        case FEEDER3_ERROR:
            if (activ_unit == FEEDER3) activ_unit = FEEDER4;
            if (IsFeeder3ControlSwitchPressed() && IsFeeder3ControlDelayTimerExpired()) {
                SetTimer(FEEDER3_CONTROL_DELAY_TMR, FEEDER_CONTROL_DELAY_TIME);
                feeder3_state = FEEDER3_OFF;
            }// End of if...
            Feeder3VacumReleaseValve_Off();
            break;
    }// End of switch...
}// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="process feeder 4">

void ProcessFeeder4(void) {

    switch (feeder4_state) {

        case FEEDER4_OFF:
            if (activ_unit == FEEDER4) activ_unit = FEEDER5;
            if (IsFeeder4ControlSwitchPressed() && IsFeeder4ControlDelayTimerExpired()) {
                SetTimer(FEEDER4_CONTROL_DELAY_TMR, FEEDER_CONTROL_DELAY_TIME);
                feeder4_state = FEEDER4_DISCHARGING;
            }// End of if...
            Feeder4VacumReleaseValve_Off();
            ClearFeeder4Error();
            break;

        case FEEDER4_DISCHARGING:
            if (IsFeeder4ControlSwitchPressed() && IsFeeder4ControlDelayTimerExpired()) {
                SetTimer(FEEDER4_CONTROL_DELAY_TMR, FEEDER_CONTROL_DELAY_TIME);
                feeder4_state = FEEDER4_OFF;
            }// End of if...
            if (!IsFeeder4SensorRawLevelMinActiv()) SetFeeder4RawLevelMinError();
            if (IsFeeder4SensorRequestActiv() && IsFeederStateChangeTimerExpired()) {
                feeder4_state = FEEDER4_CHARGING;
            } else if (activ_unit == FEEDER4) activ_unit = FEEDER5;
            feeder4_pcnt = 0;
            break;

        case FEEDER4_CHARGING:
            if (IsFeeder4ControlSwitchPressed() && IsFeeder4ControlDelayTimerExpired()) {
                SetTimer(FEEDER4_CONTROL_DELAY_TMR, FEEDER_CONTROL_DELAY_TIME);
                feeder4_state = FEEDER4_OFF;
            }// End of if...
            if (!IsFeeder4SensorRawLevelMinActiv()) SetFeeder4RawLevelMinError();
            if (activ_unit != FEEDER4) {
                break;
            } else if (feeder4_pcnt == 0) {
                SetVacumPump1Request();
                ++feeder4_pcnt;
            } else if ((feeder4_pcnt == 1) && IsVacumPump1Ready()) {
                Feeder4VacumReleaseValve_On();
                SetTimer(PISTON_TMR, PISTON_MOVE_TIME);
                ++feeder4_pcnt;
            } else if ((feeder4_pcnt == 2) && IsPistonMoveTimerExpired()) {
                SetTimer(MAX_CHARGING_TMR, max_charging_time);
                SetTimer(FEEDER4_CHARGING_TMR, feeder4_charging_time);
                ++feeder4_pcnt;
            } else if (feeder4_pcnt == 3) {
                if (IsFeeder4ChargingTimeTimerExpired() || !IsVacumPump1Ready()) {
                    Feeder4VacumReleaseValve_Off();
                    SetTimer(FEEDER_STATE_CHANGE_TMR, FEEDER_STATE_CHANGE_TIME);
                    feeder4_state = FEEDER4_DISCHARGING;
                    feeder4_pcnt = 0;
                } else if (!IsFeeder4SensorRequestActiv()) {
                    Feeder4VacumReleaseValve_Off();
                    SetFeeder4FlapSensorError();
                    feeder4_state = FEEDER4_ERROR;
                }// End of else if...
            }// End of else if...
            break;

        case FEEDER4_ERROR:
            if (activ_unit == FEEDER4) activ_unit = FEEDER5;
            if (IsFeeder4ControlSwitchPressed() && IsFeeder4ControlDelayTimerExpired()) {
                SetTimer(FEEDER4_CONTROL_DELAY_TMR, FEEDER_CONTROL_DELAY_TIME);
                feeder4_state = FEEDER4_OFF;
            }// End of if...
            if (!IsFeeder4SensorRawLevelMinActiv()) SetFeeder4RawLevelMinError();
            Feeder4VacumReleaseValve_Off();
            break;
    }// End of switch...
}// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="process feeder 5">

void ProcessFeeder5(void) {

    switch (feeder5_state) {

        case FEEDER5_OFF:
            if (activ_unit == FEEDER5) activ_unit = FEEDER_COEX;
            if (IsFeeder5ControlSwitchPressed() && IsFeeder5ControlDelayTimerExpired()) {
                SetTimer(FEEDER5_CONTROL_DELAY_TMR, FEEDER_CONTROL_DELAY_TIME);
                feeder5_state = FEEDER5_DISCHARGING;
            }// End of if...
            Feeder5VacumReleaseValve_Off();
            ClearFeeder5Error();
            break;

        case FEEDER5_DISCHARGING:
            if (IsFeeder5ControlSwitchPressed() && IsFeeder5ControlDelayTimerExpired()) {
                SetTimer(FEEDER5_CONTROL_DELAY_TMR, FEEDER_CONTROL_DELAY_TIME);
                feeder5_state = FEEDER5_OFF;
            }// End of if...
            if (IsFeeder5SensorRequestActiv() && !IsFeeder5SensorRawLevelMaxActiv() && IsFeederStateChangeTimerExpired()) {
                feeder5_state = FEEDER5_CHARGING;
            } else if (activ_unit == FEEDER5) activ_unit = FEEDER_COEX;
            feeder5_pcnt = 0;
            break;

        case FEEDER5_CHARGING:
            if (IsFeeder5ControlSwitchPressed() && IsFeeder5ControlDelayTimerExpired()) {
                SetTimer(FEEDER5_CONTROL_DELAY_TMR, FEEDER_CONTROL_DELAY_TIME);
                feeder5_state = FEEDER5_OFF;
            }// End of if...
            if (activ_unit != FEEDER5) {
                break;
            } else if (feeder5_pcnt == 0) {
                SetVacumPump1Request();
                ++feeder5_pcnt;
            } else if ((feeder5_pcnt == 1) && IsVacumPump1Ready()) {
                Feeder5VacumReleaseValve_On();
                SetTimer(PISTON_TMR, PISTON_MOVE_TIME);
                ++feeder5_pcnt;
            } else if ((feeder5_pcnt == 2) && IsPistonMoveTimerExpired()) {
                SetTimer(MAX_CHARGING_TMR, max_charging_time);
                ++feeder5_pcnt;
            } else if (feeder5_pcnt == 3) {
                if (IsFeeder5SensorRawLevelMaxActiv() || !IsVacumPump1Ready()) {
                    Feeder5VacumReleaseValve_Off();
                    ResetFeeder5ChargingTimeError();
                    SetTimer(FEEDER_STATE_CHANGE_TMR, FEEDER_STATE_CHANGE_TIME);
                    feeder5_state = FEEDER5_DISCHARGING;
                    feeder5_pcnt = 0;
                } else if (!IsFeeder5SensorRequestActiv()) {
                    Feeder5VacumReleaseValve_Off();
                    SetFeeder5FlapSensorError();
                    feeder5_state = FEEDER5_ERROR;
                } else if (IsFeederMaxChargingTimeTimerExpired()) {
                    Feeder5VacumReleaseValve_Off();
                    SetTimer(FEEDER_STATE_CHANGE_TMR, FEEDER_STATE_CHANGE_TIME);
                    feeder5_state = FEEDER5_DISCHARGING;
                    SetFeeder5ChargingTimeError();
                    feeder5_pcnt = 0;
                }// End of else if...
            }// End of else if...
            break;

        case FEEDER5_ERROR:
            if (activ_unit == FEEDER5) activ_unit = FEEDER_COEX;
            if (IsFeeder5ControlSwitchPressed() && IsFeeder5ControlDelayTimerExpired()) {
                SetTimer(FEEDER5_CONTROL_DELAY_TMR, FEEDER_CONTROL_DELAY_TIME);
                feeder5_state = FEEDER5_OFF;
            }// End of if...
            Feeder5VacumReleaseValve_Off();
            break;
    }// End of switch...
}// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="process feeder coextruder">

void ProcessFeederCoex(void) {

    switch (feeder_coex_state) {

        case FEEDER_COEX_OFF:
            if (activ_unit == FEEDER_COEX) activ_unit = VACUM_PUMP1;
            if (IsFeederCoexControlSwitchPressed() && IsFeederCoexControlDelayTimerExpired()) {
                SetTimer(FEEDER_COEX_CONTROL_DELAY_TMR, FEEDER_CONTROL_DELAY_TIME);
                feeder_coex_state = FEEDER_COEX_DISCHARGING;
            }// End of if...
            FeederCoexVacumReleaseValve_Off();
            ClearFeederCoexError();
            break;

        case FEEDER_COEX_DISCHARGING:
            if (IsFeederCoexControlSwitchPressed() && IsFeederCoexControlDelayTimerExpired()) {
                SetTimer(FEEDER_COEX_CONTROL_DELAY_TMR, FEEDER_CONTROL_DELAY_TIME);
                feeder_coex_state = FEEDER_COEX_OFF;
            }// End of if...
            if (IsFeederCoexSensorRequestActiv() && !IsFeederCoexSensorRawLevelMaxActiv() && IsFeederStateChangeTimerExpired()) {
                feeder_coex_state = FEEDER_COEX_CHARGING;
            } else if (activ_unit == FEEDER_COEX) activ_unit = VACUM_PUMP1;
            feeder_coex_pcnt = 0;
            break;

        case FEEDER_COEX_CHARGING:
            if (IsFeederCoexControlSwitchPressed() && IsFeederCoexControlDelayTimerExpired()) {
                SetTimer(FEEDER_COEX_CONTROL_DELAY_TMR, FEEDER_CONTROL_DELAY_TIME);
                feeder_coex_state = FEEDER_COEX_OFF;
            }// End of if...
            if (activ_unit != FEEDER_COEX) {
                break;
            } else if (feeder_coex_pcnt == 0) {
                SetVacumPump1Request();
                ++feeder_coex_pcnt;
            } else if ((feeder_coex_pcnt == 1) && IsVacumPump1Ready()) {
                FeederCoexVacumReleaseValve_On();
                SetTimer(PISTON_TMR, PISTON_MOVE_TIME);
                ++feeder_coex_pcnt;
            } else if ((feeder_coex_pcnt == 2) && IsPistonMoveTimerExpired()) {
                SetTimer(MAX_CHARGING_TMR, max_charging_time);
                ++feeder_coex_pcnt;
            } else if (feeder_coex_pcnt == 3) {
                if (IsFeederCoexSensorRawLevelMaxActiv() || !IsVacumPump1Ready()) {
                    FeederCoexVacumReleaseValve_Off();
                    ResetFeederCoexChargingTimeError();
                    SetTimer(FEEDER_STATE_CHANGE_TMR, FEEDER_STATE_CHANGE_TIME);
                    feeder_coex_state = FEEDER_COEX_DISCHARGING;
                    feeder_coex_pcnt = 0;
                } else if (!IsFeederCoexSensorRequestActiv()) {
                    FeederCoexVacumReleaseValve_Off();
                    SetFeederCoexFlapSensorError();
                    feeder_coex_state = FEEDER_COEX_ERROR;
                } else if (IsFeederMaxChargingTimeTimerExpired()) {
                    FeederCoexVacumReleaseValve_Off();
                    SetFeederCoexChargingTimeError();
                    SetTimer(FEEDER_STATE_CHANGE_TMR, FEEDER_STATE_CHANGE_TIME);
                    feeder_coex_state = FEEDER_COEX_DISCHARGING;
                    feeder_coex_pcnt = 0;
                }// End of else if...
            }// End of else if...
            break;

        case FEEDER_COEX_ERROR:
            if (activ_unit == FEEDER_COEX) activ_unit = VACUM_PUMP1;
            if (IsFeederCoexControlSwitchPressed() && IsFeederCoexControlDelayTimerExpired()) {
                SetTimer(FEEDER_COEX_CONTROL_DELAY_TMR, FEEDER_CONTROL_DELAY_TIME);
                feeder_coex_state = FEEDER_COEX_OFF;
            }// End of if...  
            FeederCoexVacumReleaseValve_Off();
            break;
    }// End of switch...
}// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="process cyclon 1">

void ProcessCyclon1(void) {
    if (cyc1_pcnt == 0) {
        if (IsCyclone1SensorRawLevelMaxActiv()) {
            SetTimer(MILL_SHUTDOWN_DELAY_TMR, mill_shutdown_delay);
            SetCyclon1RawLevelMaxError();
            ++cyc1_pcnt;
        } else MillEnabled();
    } else if (cyc1_pcnt == 1) {
        if (!IsCyclone1SensorRawLevelMaxActiv()) {
            ResetCyclon1RawLevelMaxError();
            cyc1_pcnt = 0;
        } else if (MILL_SHUTDOWN_DELAY_TEX) MillDisabled();
    }// End of else if...
}// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="process vacum pump 1">

void ProcessVacumPump1(void) {

    switch (vacum_pump1_state) {

        case VP1_OFF:
            if (activ_unit == VACUM_PUMP1) activ_unit = FEEDER1; // enable feeders to go into charging state
            if ((IsVacumPump1ControlSwitchPressed() || IsExtruderStarted()) && IsVacumPump1ControlDelayTimerExpired()) {
                vacum_pump1_state = VP1_START; // check control switch for start
                SetTimer(VP1_CONTROL_DELAY_TMR, VP1_CONTROL_DELAY_TIME);
            }// End of if...
            VacumPump1StartContactor_Off();
            VacumPump1StarContactor_Off();
            VacumPump1DeltaContactor_Off();
            VacumPump1CleaningAirValve_Off();
            VacumPump1VacumReleaseValve_Off();
            ClearVacumPump1Flags();
            ClearVacumPump1Error();
            an0_out = 0;
            vp1_pcnt = 0;
            break;

        case VP1_START:
            if ((activ_unit == VACUM_PUMP1) && !IsVacumPump1CleaningRequested()) activ_unit = FEEDER1; // control forward
            if (IsVacumPump1ControlSwitchPressed() && IsVacumPump1ControlDelayTimerExpired()) {
                vacum_pump1_state = VP1_OFF; // check control switch
                SetTimer(VP1_CONTROL_DELAY_TMR, VP1_CONTROL_DELAY_TIME);
            }// End of if...
            if (IsVacumPump1OvercurrentProtectionActiv() || IsVacumPump1ThermalProtectionActiv()) { // check error flags
                if (IsVacumPump1OvercurrentProtectionActiv()) SetVacumPump1OvercurrentError();
                if (IsVacumPump1ThermalProtectionActiv()) SetVacumPump1OverheatError();
                vacum_pump1_state = VP1_ERROR;
                break;
            } else if (vp1_pcnt == 0) {
                if (IsDriveOptionStarDeltaStarter()) {
                    VacumPump1DeltaContactor_Off();
                    VacumPump1StarContactor_On();
                    SetTimer(VP1_TMR, 50);
                    ++vp1_pcnt;
                } else if (IsDriveOptionInverter()) {
                    VacumPump1StartContactor_On();
                    an0_out = vp1_idle_speed;
                    SetTimer(VP1_TMR, VP1_ACCELERATION_TIME);
                    vp1_pcnt = 2;
                }// End of else...
            } else if ((vp1_pcnt == 1) && IsVacumPump1TimerExpired()) {
                VacumPump1StartContactor_On();
                SetTimer(VP1_TMR, VP1_ACCELERATION_TIME);
                ++vp1_pcnt;
            } else if ((vp1_pcnt == 2) && IsVacumPump1TimerExpired()) {
                if (!IsVacumPump1BottomFlapClosed()) {
                    SetVacumPump1FlapSensorError();
                    vacum_pump1_state = VP1_ERROR;
                } else {
                    SetVacumPump1CleaningRequest();
                    ResetVacumPump1Ready();
                    ++vp1_pcnt;
                }// End of else...
            } else if ((vp1_pcnt == 3) && IsVacumPump1CleaningFinished()) {
                ResetVacumPump1CleaningFinished();
                vacum_pump1_state = VP1_IDLE;
                vp1_pcnt = 3; // in IDLE case
            }// End of else if...
            break;

        case VP1_IDLE:
            if (activ_unit == VACUM_PUMP1) activ_unit = FEEDER1;
            if (IsVacumPump1ControlSwitchPressed() && IsVacumPump1ControlDelayTimerExpired()) {
                vacum_pump1_state = VP1_OFF;
                SetTimer(VP1_CONTROL_DELAY_TMR, VP1_CONTROL_DELAY_TIME);
            }// End of if...
            if (IsVacumPump1OvercurrentProtectionActiv() || IsVacumPump1ThermalProtectionActiv()) { // check error flags
                if (IsVacumPump1OvercurrentProtectionActiv()) SetVacumPump1OvercurrentError();
                if (IsVacumPump1ThermalProtectionActiv()) SetVacumPump1OverheatError();
                vacum_pump1_state = VP1_ERROR;
                break;
            } else if (vp1_pcnt == 0) {
                if (IsDriveOptionStarDeltaStarter()) {
                    VacumPump1StartContactor_Off();
                    VacumPump1StarContactor_Off();
                    VacumPump1DeltaContactor_Off();
                    SetTimer(VP1_TMR, VP1_DECCELERATION_TIME);
                    ++vp1_pcnt;
                } else if (IsDriveOptionInverter()) {
                    an0_out = vp1_idle_speed;
                    SetTimer(VP1_TMR, VP1_DECCELERATION_TIME);
                    vp1_pcnt = 3;
                }// End of else...
            } else if ((vp1_pcnt == 1) && IsVacumPump1TimerExpired()) {
                VacumPump1StarContactor_On();
                SetTimer(VP1_TMR, 50);
                ++vp1_pcnt;
            } else if ((vp1_pcnt == 2) && IsVacumPump1TimerExpired()) {
                VacumPump1StartContactor_On();
                ++vp1_pcnt;
            } else if ((vp1_pcnt == 3) && IsVacumPump1TimerExpired()) {
                SetTimer(VP1_TMR, vp1_delay_time);
                ++vp1_pcnt;
            } else if (vp1_pcnt == 4) {
                if (IsDriveOptionInverter()) an0_out = vp1_idle_speed;
                if (IsVacumPump1Requested() && IsDriveOptionStarDeltaStarter()) {
                    ResetVacumPump1Request();
                    VacumPump1StarContactor_Off();
                    VacumPump1DeltaContactor_Off();
                    SetTimer(VP1_TMR, 50);
                    ++vp1_pcnt;
                } else if (IsVacumPump1Requested() && IsDriveOptionInverter()) {
                    ResetVacumPump1Request();
                    an0_out = vp1_production_speed;
                    SetTimer(VP1_TMR, VP1_ACCELERATION_TIME);
                    vp1_pcnt = 6;
                } else if (IsVacumPump1TimerExpired()) {
                    an0_out = 0;
                    VacumPump1StartContactor_Off();
                    VacumPump1StarContactor_Off();
                    VacumPump1DeltaContactor_Off();
                    VacumPump1VacumReleaseValve_Off();
                    vp1_pcnt = 7;
                }// End of else if...
            } else if ((vp1_pcnt == 5) && IsVacumPump1TimerExpired()) {
                VacumPump1DeltaContactor_On();
                SetTimer(VP1_TMR, VP1_ACCELERATION_TIME);
                ++vp1_pcnt;
            } else if ((vp1_pcnt == 6) && IsVacumPump1TimerExpired()) {
                SetTimer(VP1_TMR, vp1_delay_time);
                vacum_pump1_state = VP1_RUN;
                SetVacumPump1Ready();
                vp1_pcnt = 0;
            } else if (vp1_pcnt == 7) {
                if (IsExtruderStarted() || IsVacumPump1Requested()) {
                    if (IsVacumPump1Requested()) ResetVacumPump1Request();
                    if (IsDriveOptionStarDeltaStarter()) {
                        VacumPump1StarContactor_On();
                        SetTimer(VP1_TMR, 50);
                        ++vp1_pcnt;
                    } else if (IsDriveOptionInverter()) {
                        VacumPump1StartContactor_On();
                        an0_out = vp1_production_speed;
                        SetTimer(VP1_TMR, VP1_ACCELERATION_TIME);
                        vp1_pcnt = 9;
                    }// End of else...
                }// End of if...
            } else if ((vp1_pcnt == 8) && IsVacumPump1TimerExpired()) {
                VacumPump1StartContactor_On();
                SetTimer(VP1_TMR, VP1_ACCELERATION_TIME);
                ++vp1_pcnt;
            } else if ((vp1_pcnt == 9) && IsVacumPump1TimerExpired()) {
                if (IsDriveOptionStarDeltaStarter()) {
                    VacumPump1StarContactor_Off();
                    VacumPump1DeltaContactor_On();
                }// End of if...
                SetTimer(VP1_TMR, vp1_delay_time);
                vacum_pump1_state = VP1_RUN;
                VacumPump1VacumReleaseValve_On();
                SetVacumPump1Ready();
                vp1_pcnt = 0;
            }// End of else if...
            break;

        case VP1_RUN:
            if (IsVacumPump1ControlSwitchPressed() && IsVacumPump1ControlDelayTimerExpired()) {
                SetTimer(VP1_CONTROL_DELAY_TMR, VP1_CONTROL_DELAY_TIME);
                vacum_pump1_state = VP1_OFF;
            }// End of if....
            if ((activ_unit == VACUM_PUMP1) && !IsVacumPump1CleaningRequested()) activ_unit = FEEDER1; // control forward
            if (!IsVacumPump1BottomFlapClosed()) {
                    SetVacumPump1FlapSensorError();
                    vacum_pump1_state = VP1_ERROR;
                }// End of if...
            if (IsVacumPump1OvercurrentProtectionActiv() || IsVacumPump1ThermalProtectionActiv()) { // check error flags
                if (IsVacumPump1OvercurrentProtectionActiv()) SetVacumPump1OvercurrentError();
                if (IsVacumPump1ThermalProtectionActiv()) SetVacumPump1OverheatError();
                vacum_pump1_state = VP1_ERROR;
                break;
            } else if (vp1_pcnt == 0) {
                if (IsVacumPump1Requested()) {
                    ResetVacumPump1Request();
                    if (vp1_cleaning_cnt) {
                        --vp1_cleaning_cnt;
                        ResetVacumPump1CleaningRequest();
                    } else {
                        SetVacumPump1CleaningRequest();
                        if (IsDriveOptionInverter()) an0_out = vp1_idle_speed;
                        ResetVacumPump1Ready();
                        ++vp1_pcnt;
                    }// End of else...
                } else if ((feeder1_state == FEEDER1_CHARGING) || (feeder2_state == FEEDER2_CHARGING) ||      \
                        (feeder3_state == FEEDER3_CHARGING) || (feeder4_state == FEEDER4_CHARGING) ||      \
                        (feeder5_state == FEEDER5_CHARGING) || (feeder_coex_state == FEEDER_COEX_CHARGING)) {
                    SetTimer(VP1_TMR, vp1_delay_time);
                } else if (IsVacumPump1TimerExpired()) {
                    ResetVacumPump1Ready();
                    vacum_pump1_state = VP1_IDLE;
                } else an0_out = vp1_production_speed;
            } else if ((vp1_pcnt == 1) && IsVacumPump1CleaningFinished()) {
                ResetVacumPump1CleaningFinished();
                if (IsDriveOptionInverter()) an0_out = vp1_production_speed;
                SetVacumPump1Ready();
                vp1_pcnt = 0;
            }// End of else if...
            break;

        case VP1_ERROR:
            if (activ_unit == VACUM_PUMP1) activ_unit = FEEDER1;
            if (IsVacumPump1ControlSwitchPressed() && IsVacumPump1ControlDelayTimerExpired()) {
                SetTimer(VP1_CONTROL_DELAY_TMR, VP1_CONTROL_DELAY_TIME);
                vacum_pump1_state = VP1_OFF;
            }// End of if....
            if (IsVacumPump1OvercurrentProtectionActiv()) SetVacumPump1OvercurrentError();
            if (IsVacumPump1ThermalProtectionActiv()) SetVacumPump1OverheatError();
            VacumPump1StartContactor_Off();
            VacumPump1StarContactor_Off();
            VacumPump1DeltaContactor_Off();
            VacumPump1CleaningAirValve_Off();
            VacumPump1VacumReleaseValve_Off();
            ResetVacumPump1Ready();
            an0_out = 0;
            break;
    }// End of switch;

    if (IsVacumPump1CleaningRequested() && !IsVacumPump1Ready()) {
        if (vp1_cl_pcnt == 0) {
            VacumPump1VacumReleaseValve_Off(); // disable vacum input
            SetTimer(PISTON_TMR, PISTON_MOVE_TIME); // set vacum piston movement timer
            ++vp1_cl_pcnt;
        } else if ((vp1_cl_pcnt == 1) && IsPistonMoveTimerExpired()) {
            VacumPump1CleaningAirValve_On(); // energize cleaning air valve
            SetTimer(VP1_TMR, VP1_CLEANING_AIR_BLOW_TIME); // set cleaning time
            ++vp1_cl_pcnt;
        } else if ((vp1_cl_pcnt == 2) && IsVacumPump1TimerExpired()) {
            VacumPump1CleaningAirValve_Off(); // shutdown cleaning air valve
            SetTimer(VP1_TMR, VP1_AFTER_CLEANING_VACUM_DELAY); // wait for dust to settle
            ++vp1_cl_pcnt;
        } else if ((vp1_cl_pcnt == 3) && IsVacumPump1TimerExpired()) {
            VacumPump1VacumReleaseValve_On(); // enable vacum
            SetTimer(PISTON_TMR, PISTON_MOVE_TIME);
            ++vp1_cl_pcnt;
        } else if ((vp1_cl_pcnt == 4) && IsPistonMoveTimerExpired()) {
            vp1_cleaning_cnt = vp1_cleaning_cycles; // reset cycles counter
            ResetVacumPump1CleaningRequest();
            SetVacumPump1CleaningFinished();
            vp1_cl_pcnt = 0;
        }// End of else if...
    }// End of if...
}// </editor-fold>

