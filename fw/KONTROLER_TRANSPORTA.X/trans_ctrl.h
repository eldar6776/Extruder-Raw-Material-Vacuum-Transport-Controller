/******************************************************************************
 *
 *           Kontroler vakum transporta sirovine linije 5
 *
 *                     Zaglavlje glavnog programa
 *
 *                       Firmware versija 1.02
 *
 ******************************************************************************
 *
 *  Ime fajla:      trans_ctrl.h
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
//
#ifndef TRANS_CTRL_H
#define TRANS_CTRL_H
//
#include <xc.h>
#include "typedefs.h"
#include "io_cfg.h"
//
//
/** K O N F I G U R A C I J A    P R O C E S O R A  ***************************/
//
//
#pragma config FOSC = HS        // Oscillator Selection (HS Oscillator, High-speed crystal/resonator connected between OSC1 and OSC2 pins)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config VCAPEN = OFF     // Voltage Regulator Capacitor Enable bit (VCAP pin function disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)
//
//
/** K O N S T A N T E   P R O G R A M A   ***************************************/
//
//
#define ERROR_ITEM_DISPLAY_TIME             500     // x 10ms (5s error item display time)
#define VP1_CLEANING_AIR_BLOW_TIME          500     // 5s air blow time for tank cleaning
#define VP1_AFTER_CLEANING_VACUM_DELAY      1000    // 10s for dust to settle
#define VP1_CONTROL_DELAY_TIME              500     // 5s min time
#define VP1_ACCELERATION_TIME               700     // x 10ms for vacum pump to acceleratte to set speed
#define VP1_DECCELERATION_TIME              500     // x 10ms for vp do deccelerate to set speed
#define PISTON_MOVE_TIME                    100     // 1s max time for piston movement
#define FEEDER2_CLEANING_PISTON_CYCLUS_TIME 12000   // 1min cleaning piston strike interval
#define FEEDER_STATE_CHANGE_TIME            500     // 5s for raw to fall
#define FEEDER_CONTROL_DELAY_TIME           100     // 1s control switch response delay time
//
//
/** R A M   V A R I J A B L E *************************************************/
//
//
//----------------------------------------------- char ram variable
//
unsigned char clock_tick_100ms;
unsigned char clock_tick_500ms;
unsigned char clock_tick_1s;
unsigned char clock_tick_2s;
unsigned char anin_tmr;
unsigned char dout_tmr;
unsigned char anout_tmr;
unsigned char sigout_tmr;
unsigned char test_cnt;
unsigned char temp;
//
//----------------------------------------------- char process counters
//
unsigned char error_display_pcnt;
unsigned char feeder1_pcnt;
unsigned char feeder2_pcnt;
unsigned char feeder3_pcnt;
unsigned char feeder4_pcnt;
unsigned char feeder5_pcnt;
unsigned char feeder_coex_pcnt;
unsigned char vp1_pcnt;
unsigned char an_in_pcnt;
unsigned char relay_pcnt;
unsigned char sigout_pcnt;
unsigned char vp1_cl_pcnt;
unsigned char cyc1_pcnt;
//
//----------------------------------------------- int timers
//
unsigned int vp1_tmr;
unsigned int mill_shutdown_delay_tmr;
unsigned int piston_tmr;
unsigned int max_charging_tmr;
unsigned int error_display_tmr;
unsigned int feeder4_charging_tmr;
unsigned int feeder2_cleaning_piston_tmr;
unsigned int feeder_state_change_tmr;
unsigned int vp1_control_delay_tmr;
unsigned int feeder1_control_delay_tmr;
unsigned int feeder2_control_delay_tmr;
unsigned int feeder3_control_delay_tmr;
unsigned int feeder4_control_delay_tmr;
unsigned int feeder5_control_delay_tmr;
unsigned int feeder_coex_control_delay_tmr;
//
//----------------------------------------------- int ram variable
//
unsigned int vp1_delay_time;
unsigned int vp1_production_speed;
unsigned int vp1_idle_speed;
unsigned int max_charging_time;
unsigned int mill_shutdown_delay;
unsigned int feeder4_charging_time;
unsigned int vp1_cleaning_cycles;
unsigned int vp1_cleaning_cnt;
unsigned int an0_out, an1_out;
//
//
/** E N U M E R A T O R S  ****************************************************/
//
//
enum timers {
    VP1_TMR, MILL_SHUTDOWN_DELAY_TMR, PISTON_TMR, MAX_CHARGING_TMR,
    ERROR_DISPLAY_TMR, FEEDER4_CHARGING_TMR, FEEDER2_CLEANING_PISTON_TMR,
    FEEDER_STATE_CHANGE_TMR, VP1_CONTROL_DELAY_TMR, FEEDER1_CONTROL_DELAY_TMR,
    FEEDER2_CONTROL_DELAY_TMR, FEEDER3_CONTROL_DELAY_TMR, FEEDER4_CONTROL_DELAY_TMR,
    FEEDER5_CONTROL_DELAY_TMR, FEEDER_COEX_CONTROL_DELAY_TMR
} timer;
//
enum vacum_pump1_states {
    VP1_OFF, VP1_START, VP1_RUN, VP1_IDLE, VP1_ERROR
} vacum_pump1_state;
//
enum activ_units {
    FEEDER1, FEEDER2, FEEDER3, FEEDER4, FEEDER5, FEEDER_COEX, VACUM_PUMP1
} activ_unit;
//
enum feeder1_states {
    FEEDER1_OFF, FEEDER1_DISCHARGING, FEEDER1_CHARGING, FEEDER1_ERROR
} feeder1_state;
//
enum feeder2_states {
    FEEDER2_OFF, FEEDER2_DISCHARGING, FEEDER2_CHARGING, FEEDER2_ERROR
} feeder2_state;
//
enum feeder3_states {
    FEEDER3_OFF, FEEDER3_DISCHARGING, FEEDER3_CHARGING, FEEDER3_ERROR
} feeder3_state;
//
enum feeder4_states {
    FEEDER4_OFF, FEEDER4_DISCHARGING, FEEDER4_CHARGING, FEEDER4_ERROR
} feeder4_state;
//
enum feeder5_states {
    FEEDER5_OFF, FEEDER5_DISCHARGING, FEEDER5_CHARGING, FEEDER5_ERROR
} feeder5_state;
//
enum feeder_coex_states {
    FEEDER_COEX_OFF, FEEDER_COEX_DISCHARGING, FEEDER_COEX_CHARGING, FEEDER_COEX_ERROR
} feeder_coex_state;
//
//
/********* M A R K E R I  ****************************************************/
//
//
BYTE timer_flags1;
#define VP1_TEX                             timer_flags1.b0
#define MILL_SHUTDOWN_DELAY_TEX             timer_flags1.b1
#define FEEDER_STATE_CHANGE_TEX             timer_flags1.b2
#define PISTON_TEX                          timer_flags1.b3
#define MAX_CHARGING_TEX                    timer_flags1.b4
#define ERROR_DISPLAY_TEX                   timer_flags1.b5
#define FEEDER4_CHARGING_TEX                timer_flags1.b6
#define FEEDER2_CLEANING_PISTON_TEX         timer_flags1.b7
#define IsVacumPump1TimerExpired()          (VP1_TEX == TRUE)
#define IsMillDisableDelayTimerExpired()    (MILL_SHUTDOWN_DELAY_TEX == TRUE)
#define IsFeederStateChangeTimerExpired()   (FEEDER_STATE_CHANGE_TEX == TRUE)
#define IsPistonMoveTimerExpired()          (PISTON_TEX == TRUE)
#define IsFeederMaxChargingTimeTimerExpired()   (MAX_CHARGING_TEX == TRUE)
#define IsErrorDisplayItemTimerExpired()    (ERROR_DISPLAY_TEX == TRUE)
#define IsFeeder4ChargingTimeTimerExpired() (FEEDER4_CHARGING_TEX == TRUE)
#define IsFeeder2CleaningPistonTimerExpired()   (FEEDER2_CLEANING_PISTON_TEX == TRUE)
//
BYTE timer_flags2;
//#define ANIN_TEX                            timer_flags2.b0
//#define DOUT_TEX                            timer_flags2.b1
//#define ANOUT_TEX                           timer_flags2.b2
//#define SIGOUT_TEX                          timer_flags2.b3
#define CLOCK_TICK                          timer_flags2.b4
#define VP1_CONTROL_DELAY_TEX               timer_flags2.b5
#define FEEDER1_CONTROL_DELAY_TEX           timer_flags2.b6
#define FEEDER2_CONTROL_DELAY_TEX           timer_flags2.b7
#define IsSysClockTicked()                  (CLOCK_TICK == TRUE)
#define SetSysClockTick()                   (CLOCK_TICK = TRUE)
#define ResetSysClockTick()                 (CLOCK_TICK = FALSE)
#define IsVacumPump1ControlDelayTimerExpired()  (VP1_CONTROL_DELAY_TEX == TRUE)
#define IsFeeder1ControlDelayTimerExpired() (FEEDER1_CONTROL_DELAY_TEX == TRUE)
#define IsFeeder2ControlDelayTimerExpired() (FEEDER2_CONTROL_DELAY_TEX == TRUE)
//
BYTE timer_flags3;
#define FEEDER3_CONTROL_DELAY_TEX           timer_flags3.b0
#define FEEDER4_CONTROL_DELAY_TEX           timer_flags3.b1
#define FEEDER5_CONTROL_DELAY_TEX           timer_flags3.b2
#define FEEDER_COEX_CONTROL_DELAY_TEX       timer_flags3.b3
#define IsFeeder3ControlDelayTimerExpired() (FEEDER3_CONTROL_DELAY_TEX == TRUE)
#define IsFeeder4ControlDelayTimerExpired() (FEEDER4_CONTROL_DELAY_TEX == TRUE)
#define IsFeeder5ControlDelayTimerExpired() (FEEDER5_CONTROL_DELAY_TEX == TRUE)
#define IsFeederCoexControlDelayTimerExpired()  (FEEDER_COEX_CONTROL_DELAY_TEX == TRUE)
//
BYTE systeml_flags1;
#define SYSCLK_TICK_100MS                   systeml_flags1.b0
#define SYSCLK_TICK_500MS                   systeml_flags1.b1
#define SYSCLK_TICK_1S                      systeml_flags1.b2
#define SYSCLK_TICK_2S                      systeml_flags1.b3
#define LED_STATUS_BOOT                     systeml_flags1.b4
#define LED_STATUS_RUN                      systeml_flags1.b5
#define LED_STATUS_STOP                     systeml_flags1.b6
#define LED_STATUS_ERROR                    systeml_flags1.b7
#define StatusLED_Run()                     ((systeml_flags1._byte &= 0x0f),LED_STATUS_RUN = TRUE)
#define StatusLED_Stop()                    ((systeml_flags1._byte &= 0x0f),LED_STATUS_STOP = TRUE)
#define StatusLED_Error()                   ((systeml_flags1._byte &= 0x0f),LED_STATUS_ERROR = TRUE)
#define StatusLED_Booting()                 ((systeml_flags1._byte &= 0x0f),LED_STATUS_BOOT = TRUE)
#define ClearStatusLED()                    (systeml_flags1._byte &= 0x0f)
//
BYTE system_flags2;
#define EXTRUDER_STATE                      system_flags2.b0
#define EXTRUDER_OLD_STATE                  system_flags2.b1
#define SetExtruderStartFlag()              (EXTRUDER_STATE = TRUE)
#define ResetExtruderStartFlag()            (EXTRUDER_STATE = FALSE)
#define IsExtruderStarted()                 (EXTRUDER_STATE == TRUE)
#define IsExtruderRunInputActiv()           (DIN_EXTRUDER_RUN == HIGH)
//
//=============================================== VACUM PUMP 1 FLAGS
//
BYTE vacum_pump1_flags1;
#define VP1_CLEANING_REQUEST                vacum_pump1_flags1.b1
#define VP1_CLEANING_FINISHED               vacum_pump1_flags1.b2
#define VP1_READY                           vacum_pump1_flags1.b3
#define VP1_REQUEST                         vacum_pump1_flags1.b4
#define VP1_DRIVE_OPTION                    vacum_pump1_flags1.b5
#define SetVacumPump1Request()              (VP1_REQUEST = TRUE)
#define ResetVacumPump1Request()            (VP1_REQUEST = FALSE)
#define IsVacumPump1Requested()             (VP1_REQUEST == TRUE)
#define SetVacumPump1Ready()                (VP1_READY = TRUE)
#define ResetVacumPump1Ready()              (VP1_READY = FALSE)
#define IsVacumPump1Ready()                 (VP1_READY == TRUE)
#define SetDriveOptionInverter()            (VP1_DRIVE_OPTION = FALSE)
#define SetDriveOptionStarDeltaStarter()    (VP1_DRIVE_OPTION = TRUE)
#define IsDriveOptionInverter()             (VP1_DRIVE_OPTION == FALSE)
#define IsDriveOptionStarDeltaStarter()     (VP1_DRIVE_OPTION == TRUE)
#define SetVacumPump1CleaningRequest()      (VP1_CLEANING_REQUEST = TRUE)
#define ResetVacumPump1CleaningRequest()    (VP1_CLEANING_REQUEST = FALSE)
#define IsVacumPump1CleaningRequested()     (VP1_CLEANING_REQUEST == TRUE)
#define SetVacumPump1CleaningFinished()     (VP1_CLEANING_FINISHED = TRUE)
#define ResetVacumPump1CleaningFinished()   (VP1_CLEANING_FINISHED = FALSE)
#define IsVacumPump1CleaningFinished()      (VP1_CLEANING_FINISHED == TRUE)
#define ClearVacumPump1Flags()              (vacum_pump1_flags1._byte &= 0xf0)
//
BYTE vacum_pump1_flags2;
#define VP1_FLAP_SENSOR_ERROR               vacum_pump1_flags2.b0
#define VP1_THERMAL_OVERHEAT_ERROR          vacum_pump1_flags2.b1
#define VP1_OVERCURENT_ERROR                vacum_pump1_flags2.b2
#define SetVacumPump1FlapSensorError()      (VP1_FLAP_SENSOR_ERROR = TRUE)
#define ResetVacumPump1FlapSensorError()    (VP1_FLAP_SENSOR_ERROR = FALSE)
#define IsVacumPump1FlapSensorErrorActiv()  (VP1_FLAP_SENSOR_ERROR == TRUE)
#define SetVacumPump1OverheatError()        (VP1_THERMAL_OVERHEAT_ERROR = TRUE)
#define ResetVacumPump1OverheatError()      (VP1_THERMAL_OVERHEAT_ERROR = FALSE)
#define IsVacumPump1OverheatErrorActiv()    (VP1_THERMAL_OVERHEAT_ERROR == TRUE)
#define SetVacumPump1OvercurrentError()     (VP1_OVERCURENT_ERROR = TRUE)
#define ResetVacumPump1OvercurrentError()   (VP1_OVERCURENT_ERROR = FALSE)
#define IsVacumPump1OvercurrentErrorActiv() (VP1_OVERCURENT_ERROR == TRUE)
#define ClearVacumPump1Error()              (vacum_pump1_flags2._byte = 0x00)
//
//=============================================== CYCLON 1 FLAGS
//
BYTE cyclon_flags1;
#define CYCLON_RAW_LEVEL_MAX_ERROR  cyclon_flags1.b0
#define SetCyclon1RawLevelMaxError()        (CYCLON_RAW_LEVEL_MAX_ERROR = TRUE)
#define ResetCyclon1RawLevelMaxError()      (CYCLON_RAW_LEVEL_MAX_ERROR = FALSE)
#define IsCyclon1RawLevelMaxErrorActiv()    (CYCLON_RAW_LEVEL_MAX_ERROR == TRUE)
#define ClearCyclonError()          cyclon_flags1._byte = 0x00
//
//=============================================== FEEDER 1 FLAGS
//
BYTE feeder1_flags1;
#define FEEDER1_CHARGING_TIME_ERROR     feeder1_flags1.b0
#define FEEDER1_FLAP_SENSOR_ERROR       feeder1_flags1.b1
#define SetFeeder1ChargingTimeError()   (FEEDER1_CHARGING_TIME_ERROR = TRUE)
#define ResetFeeder1ChargingTimeError() (FEEDER1_CHARGING_TIME_ERROR = FALSE)
#define IsFeeder1ChargingTimeErrorActiv()(FEEDER1_CHARGING_TIME_ERROR == TRUE)
#define SetFeeder1FlapSensorError()     (FEEDER1_FLAP_SENSOR_ERROR = TRUE)
#define ResetFeeder1FlapSensorError()   (FEEDER1_FLAP_SENSOR_ERROR = FALSE)
#define IsFeeder1FlapSensorErrorActiv() (FEEDER1_FLAP_SENSOR_ERROR == TRUE)
#define ClearFeeder1Error()             feeder1_flags1._byte = 0x00
//
//=============================================== FEEDER 2 FLAGS
//
BYTE feeder2_flags1;
#define FEEDER2_CHARGING_TIME_ERROR     feeder2_flags1.b0
#define FEEDER2_FLAP_SENSOR_ERROR       feeder2_flags1.b1
#define SetFeeder2ChargingTimeError()   (FEEDER2_CHARGING_TIME_ERROR = TRUE)
#define ResetFeeder2ChargingTimeError() (FEEDER2_CHARGING_TIME_ERROR = FALSE)
#define IsFeeder2ChargingTimeErrorActiv()(FEEDER2_CHARGING_TIME_ERROR == TRUE)
#define SetFeeder2FlapSensorError()     (FEEDER2_FLAP_SENSOR_ERROR = TRUE)
#define ResetFeeder2FlapSensorError()   (FEEDER2_FLAP_SENSOR_ERROR = FALSE)
#define IsFeeder2FlapSensorErrorActiv() (FEEDER2_FLAP_SENSOR_ERROR == TRUE)
#define ClearFeeder2Error()             feeder2_flags1._byte = 0x00
//
//=============================================== FEEDER 3 FLAGS
//
BYTE feeder3_flags1;
#define FEEDER3_CHARGING_TIME_ERROR     feeder3_flags1.b0
#define FEEDER3_FLAP_SENSOR_ERROR       feeder3_flags1.b1
#define SetFeeder3ChargingTimeError()   (FEEDER3_CHARGING_TIME_ERROR = TRUE)
#define ResetFeeder3ChargingTimeError() (FEEDER3_CHARGING_TIME_ERROR = FALSE)
#define IsFeeder3ChargingTimeErrorActiv()(FEEDER3_CHARGING_TIME_ERROR == TRUE)
#define SetFeeder3FlapSensorError()     (FEEDER3_FLAP_SENSOR_ERROR = TRUE)
#define ResetFeeder3FlapSensorError()   (FEEDER3_FLAP_SENSOR_ERROR = FALSE)
#define IsFeeder3FlapSensorErrorActiv() (FEEDER3_FLAP_SENSOR_ERROR == TRUE)
#define ClearFeeder3Error()             feeder3_flags1._byte = 0x00
//
//=============================================== FEEDER 4 FLAGS
//
BYTE feeder4_flags1;
#define FEEDER4_FLAP_SENSOR_ERROR       feeder4_flags1.b0
#define FEEDER4_RAW_LEVEL_MIN_ERROR     feeder4_flags1.b1
#define SetFeeder4FlapSensorError()     (FEEDER4_FLAP_SENSOR_ERROR = TRUE)
#define ResetFeeder4FlapSensorError()   (FEEDER4_FLAP_SENSOR_ERROR = FALSE)
#define IsFeeder4FlapSensorErrorActiv() (FEEDER4_FLAP_SENSOR_ERROR == TRUE)
#define SetFeeder4RawLevelMinError()    (FEEDER4_RAW_LEVEL_MIN_ERROR = TRUE)
#define ResetFeeder4RawLevelMinError()  (FEEDER4_RAW_LEVEL_MIN_ERROR = FALSE)
#define IsFeeder4RawLevelMinErrorActiv()(FEEDER4_RAW_LEVEL_MIN_ERROR == TRUE)
#define ClearFeeder4Error()             (feeder4_flags1._byte = 0x00)
//
//=============================================== FEEDER 5 FLAGS
//
BYTE feeder5_flags1;
#define FEEDER5_CHARGING_TIME_ERROR     feeder5_flags1.b0
#define FEEDER5_FLAP_SENSOR_ERROR       feeder5_flags1.b1
#define SetFeeder5ChargingTimeError()   (FEEDER5_CHARGING_TIME_ERROR = TRUE)
#define ResetFeeder5ChargingTimeError() (FEEDER5_CHARGING_TIME_ERROR = FALSE)
#define IsFeeder5ChargingTimeErrorActiv()(FEEDER5_CHARGING_TIME_ERROR == TRUE)
#define SetFeeder5FlapSensorError()     (FEEDER5_FLAP_SENSOR_ERROR = TRUE)
#define ResetFeeder5FlapSensorError()   (FEEDER5_FLAP_SENSOR_ERROR = FALSE)
#define IsFeeder5FlapSensorErrorActiv() (FEEDER5_FLAP_SENSOR_ERROR == TRUE)
#define ClearFeeder5Error()             feeder5_flags1._byte = 0x00
//
//=============================================== FEEDER COEKSTRUDER FLAGS
//
BYTE feeder_coex_flags1;
#define FEEDER_COEX_CHARGING_TIME_ERROR     feeder_coex_flags1.b0
#define FEEDER_COEX_FLAP_SENSOR_ERROR       feeder_coex_flags1.b1
#define SetFeederCoexChargingTimeError()    (FEEDER_COEX_CHARGING_TIME_ERROR = TRUE)
#define ResetFeederCoexChargingTimeError()  (FEEDER_COEX_CHARGING_TIME_ERROR = FALSE)
#define IsFeederCoexChargingTimeErrorActiv()(FEEDER_COEX_CHARGING_TIME_ERROR == TRUE)
#define SetFeederCoexFlapSensorError()      (FEEDER_COEX_FLAP_SENSOR_ERROR = TRUE)
#define ResetFeederCoexFlapSensorError()    (FEEDER_COEX_FLAP_SENSOR_ERROR = FALSE)
#define IsFeederCoexFlapSensorErrorActiv()  (FEEDER_COEX_FLAP_SENSOR_ERROR == TRUE)
#define ClearFeederCoexError()              feeder_coex_flags1._byte = 0x00
//
//=============================================== TASTER FLAGS
//
BYTE taster_flags1;
#define TST_VP1_STATE           taster_flags1.b0
#define TST_FEEDER_COEX_STATE   taster_flags1.b1
#define TST_FEEDER1_STATE       taster_flags1.b2
#define TST_FEEDER2_STATE       taster_flags1.b3
#define TST_FEEDER3_STATE       taster_flags1.b4
#define TST_FEEDER4_STATE       taster_flags1.b5
#define TST_FEEDER5_STATE       taster_flags1.b6
#define SetVacumPump1ControlSwitch()        (TST_VP1_STATE = TRUE)
#define ResetVacumPump1ControlSwitch()      (TST_VP1_STATE = FALSE)
#define IsVacumPump1ControlSwitchPressed()  (TST_VP1_STATE == TRUE)
#define SetFeederCoexControlSwitch()        (TST_FEEDER_COEX_STATE = TRUE)
#define ResetFeederCoexControlSwitch()      (TST_FEEDER_COEX_STATE = FALSE)
#define IsFeederCoexControlSwitchPressed()  (TST_FEEDER_COEX_STATE == TRUE)
#define SetFeeder1ControlSwitch()           (TST_FEEDER1_STATE = TRUE)
#define ResetFeeder1ControlSwitch()         (TST_FEEDER1_STATE = FALSE)
#define IsFeeder1ControlSwitchPressed()     (TST_FEEDER1_STATE == TRUE)
#define SetFeeder2ControlSwitch()           (TST_FEEDER2_STATE = TRUE)
#define ResetFeeder2ControlSwitch()         (TST_FEEDER2_STATE = FALSE)
#define IsFeeder2ControlSwitchPressed()     (TST_FEEDER2_STATE == TRUE)
#define SetFeeder3ControlSwitch()           (TST_FEEDER3_STATE = TRUE)
#define ResetFeeder3ControlSwitch()         (TST_FEEDER3_STATE = FALSE)
#define IsFeeder3ControlSwitchPressed()     (TST_FEEDER3_STATE == TRUE)
#define SetFeeder4ControlSwitch()           (TST_FEEDER4_STATE = TRUE)
#define ResetFeeder4ControlSwitch()         (TST_FEEDER4_STATE = FALSE)
#define IsFeeder4ControlSwitchPressed()     (TST_FEEDER4_STATE == TRUE)
#define SetFeeder5ControlSwitch()           (TST_FEEDER5_STATE = TRUE)
#define ResetFeeder5ControlSwitch()         (TST_FEEDER5_STATE = FALSE)
#define IsFeeder5ControlSwitchPressed()     (TST_FEEDER5_STATE == TRUE)
//
BYTE taster_flags2;
#define TST_VP1_FL              taster_flags2.b0
#define TST_FEEDER_COEX_FL      taster_flags2.b1
#define TST_FEEDER1_FL          taster_flags2.b2
#define TST_FEEDER2_FL          taster_flags2.b3
#define TST_FEEDER3_FL          taster_flags2.b4
#define TST_FEEDER4_FL          taster_flags2.b5
#define TST_FEEDER5_FL          taster_flags2.b6
//
//=============================================== INPUT 0 - 7
//
BYTE input_0_7;
#define DIN_VP1_OVERCURRENT             input_0_7.b0
#define DIN_VP1_THERMAL_SW              input_0_7.b1
#define DIN_VP1_FLAP                    input_0_7.b2
#define DIN_VP1_CTRL_SW                 input_0_7.b3
#define DIN_EMERGENCY                   input_0_7.b4
#define DIN_CYCLONE_SEN_MAX             input_0_7.b5
#define DIN_FEEDER_COEX_SEN_MAX         input_0_7.b6
#define DIN_FEEDER_COEX_SEN_REQ         input_0_7.b7
#define IsVacumPump1OvercurrentProtectionActiv()(DIN_VP1_OVERCURRENT == HIGH)
#define IsVacumPump1ThermalProtectionActiv()    (DIN_VP1_THERMAL_SW == HIGH)
#define IsVacumPump1BottomFlapClosed()          (DIN_VP1_FLAP == LOW)
#define IsVacumPump1ControlSwitchActiv()        (DIN_VP1_CTRL_SW == LOW)
#define IsEmergencyProtectionActiv()            (DIN_EMERGENCY == LOW)
#define IsCyclone1SensorRawLevelMaxActiv()      (DIN_CYCLONE_SEN_MAX == LOW)
#define IsFeederCoexSensorRawLevelMaxActiv()    (DIN_FEEDER_COEX_SEN_MAX == LOW)
#define IsFeederCoexSensorRequestActiv()        (DIN_FEEDER_COEX_SEN_REQ == LOW)
//
//=============================================== INPUT 8 - 15
//
BYTE input_8_15;
#define DIN_FEEDER_COEX_CTRL_SW         input_8_15.b0
#define DIN_FEEDER1_CTRL_SW             input_8_15.b1
#define DIN_FEEDER2_CTRL_SW             input_8_15.b2
#define DIN_FEEDER3_CTRL_SW             input_8_15.b3
#define DIN_FEEDER4_CTRL_SW             input_8_15.b4
#define DIN_FEEDER5_CTRL_SW             input_8_15.b5
#define DIN_FEEDER1_SEN_REQ             input_8_15.b6
#define DIN_FEEDER1_SEN_MAX             input_8_15.b7
#define IsFeederCoexControlSwitchActiv()        (DIN_FEEDER_COEX_CTRL_SW == LOW)
#define IsFeeder1ControlSwitchActiv()           (DIN_FEEDER1_CTRL_SW == LOW)
#define IsFeeder2ControlSwitchActiv()           (DIN_FEEDER2_CTRL_SW == LOW)
#define IsFeeder3ControlSwitchActiv()           (DIN_FEEDER3_CTRL_SW == LOW)
#define IsFeeder4ControlSwitchActiv()           (DIN_FEEDER4_CTRL_SW == LOW)
#define IsFeeder5ControlSwitchActiv()           (DIN_FEEDER5_CTRL_SW == LOW)
#define IsFeeder1SensorRequestActiv()           (DIN_FEEDER1_SEN_REQ == LOW)
#define IsFeeder1SensorRawLevelMaxActiv()       (DIN_FEEDER1_SEN_MAX == LOW)
//
//=============================================== INPUT 16 - 23
//
BYTE input_16_23;
#define DIN_FEEDER2_SEN_REQ             input_16_23.b0
#define DIN_FEEDER2_SEN_MAX             input_16_23.b1
#define DIN_FEEDER3_SEN_REQ             input_16_23.b2
#define DIN_FEEDER3_SEN_MAX             input_16_23.b3
#define DIN_FEEDER4_SEN_REQ             input_16_23.b4
#define DIN_FEEDER4_SEN_MIN             input_16_23.b5
#define DIN_FEEDER5_SEN_REQ             input_16_23.b6
#define DIN_FEEDER5_SEN_MAX             input_16_23.b7
#define IsFeeder2SensorRequestActiv()           (DIN_FEEDER2_SEN_REQ == LOW)
#define IsFeeder2SensorRawLevelMaxActiv()       (DIN_FEEDER2_SEN_MAX == LOW)
#define IsFeeder3SensorRequestActiv()           (DIN_FEEDER3_SEN_REQ == LOW)
#define IsFeeder3SensorRawLevelMaxActiv()       (DIN_FEEDER3_SEN_MAX == LOW)
#define IsFeeder4SensorRequestActiv()           (DIN_FEEDER4_SEN_REQ == LOW)
#define IsFeeder4SensorRawLevelMinActiv()       (DIN_FEEDER4_SEN_MIN == LOW)
#define IsFeeder5SensorRequestActiv()           (DIN_FEEDER5_SEN_REQ == LOW)
#define IsFeeder5SensorRawLevelMaxActiv()       (DIN_FEEDER5_SEN_MAX == LOW)
//
//=============================================== INPUT 24 - 31
//
BYTE input_24_31;
#define DIN_RESERVE1                    input_24_31.b0
#define DIN_RESERVE2                    input_24_31.b1
#define DIN_RESERVE3                    input_24_31.b2
#define DIN_RESERVE4                    input_24_31.b3
#define DIN_RESERVE5                    input_24_31.b4
#define DIN_RESERVE6                    input_24_31.b5
#define DIN_RESERVE7                    input_24_31.b6
#define DIN_RESERVE8                    input_24_31.b7
//
//=============================================== OUTPUT 0 - 7
//
BYTE output_0_7;
#define OUT_VP1_START_CONTACTOR         output_0_7.b0
#define OUT_FEEDER1_VACUM_VALVE         output_0_7.b1
#define OUT_FEEDER2_VACUM_VALVE         output_0_7.b2
#define OUT_FEEDER3_VACUM_VALVE         output_0_7.b3
#define OUT_FEEDER4_VACUM_VALVE         output_0_7.b4
#define OUT_FEEDER5_VACUM_VALVE         output_0_7.b5
#define OUT_FEEDER_COEX_VACUM_VALVE     output_0_7.b6
#define OUT_VP1_STAR_CONTACTOR          output_0_7.b7
#define VacumPump1StartContactor_On()   (OUT_VP1_START_CONTACTOR = HIGH)
#define VacumPump1StartContactor_Off()  (OUT_VP1_START_CONTACTOR = LOW)
#define Feeder1VacumReleaseValve_On()   (OUT_FEEDER1_VACUM_VALVE = HIGH)
#define Feeder1VacumReleaseValve_Off()  (OUT_FEEDER1_VACUM_VALVE = LOW)
#define Feeder2VacumReleaseValve_On()   (OUT_FEEDER2_VACUM_VALVE = HIGH)
#define Feeder2VacumReleaseValve_Off()  (OUT_FEEDER2_VACUM_VALVE = LOW)
#define Feeder3VacumReleaseValve_On()   (OUT_FEEDER3_VACUM_VALVE = HIGH)
#define Feeder3VacumReleaseValve_Off()  (OUT_FEEDER3_VACUM_VALVE = LOW)
#define Feeder4VacumReleaseValve_On()   (OUT_FEEDER4_VACUM_VALVE = HIGH)
#define Feeder4VacumReleaseValve_Off()  (OUT_FEEDER4_VACUM_VALVE = LOW)
#define Feeder5VacumReleaseValve_On()   (OUT_FEEDER5_VACUM_VALVE = HIGH)
#define Feeder5VacumReleaseValve_Off()  (OUT_FEEDER5_VACUM_VALVE = LOW)
#define FeederCoexVacumReleaseValve_On()(OUT_FEEDER_COEX_VACUM_VALVE = HIGH)
#define FeederCoexVacumReleaseValve_Off()(OUT_FEEDER_COEX_VACUM_VALVE = LOW)
#define VacumPump1StarContactor_On()    (OUT_VP1_STAR_CONTACTOR = HIGH)
#define VacumPump1StarContactor_Off()   (OUT_VP1_STAR_CONTACTOR = LOW)
//
//=============================================== OUTPUT 8 - 15
//
BYTE output_8_15;
#define OUT_FEEDER2_CLEANING_PISTON     output_8_15.b0
#define OUT_VP1_DELTA_CONTACTOR         output_8_15.b1
#define OUT_VP1_CLEANING_VALVE          output_8_15.b2
#define OUT_VP1_VACUM_CONTROL_VALVE     output_8_15.b3
#define OUT_ALARM_FEEDER1               output_8_15.b4
#define OUT_MILL_ENABLE                 output_8_15.b5
#define OUT_ALARM                       output_8_15.b6
#define OUT_ALARM_FEEDER4               output_8_15.b7
#define Feeder2CleaningPistonValve_On() (OUT_FEEDER2_CLEANING_PISTON = HIGH)
#define Feeder2CleaningPistonValve_Off()(OUT_FEEDER2_CLEANING_PISTON = LOW)
#define VacumPump1DeltaContactor_On()   (OUT_VP1_DELTA_CONTACTOR = HIGH)
#define VacumPump1DeltaContactor_Off()  (OUT_VP1_DELTA_CONTACTOR = LOW)
#define VacumPump1CleaningAirValve_On() (OUT_VP1_CLEANING_VALVE = HIGH)
#define VacumPump1CleaningAirValve_Off()(OUT_VP1_CLEANING_VALVE = LOW)
#define VacumPump1VacumReleaseValve_On()(OUT_VP1_VACUM_CONTROL_VALVE = HIGH)
#define VacumPump1VacumReleaseValve_Off()(OUT_VP1_VACUM_CONTROL_VALVE = LOW)
#define Feeder1AlarmSignal_On()         (OUT_ALARM_FEEDER1 = HIGH)
#define Feeder1AlarmSignal_Off()        (OUT_ALARM_FEEDER1 = LOW)
#define MillEnabled()                   (OUT_MILL_ENABLE = HIGH)
#define MillDisabled()                  (OUT_MILL_ENABLE = LOW)
#define IsMillDisabled()                (OUT_MILL_ENABLE == LOW)
#define AlarmSignal_On()                (OUT_ALARM = HIGH)
#define AlarmSignal_Off()               (OUT_ALARM = LOW)
#define Feeder4AlarmSignal_On()         (OUT_ALARM_FEEDER4 = HIGH)
#define Feeder4AlarmSignal_Off()        (OUT_ALARM_FEEDER4 = LOW)
//
//=============================================== SIGNAL 0 - 7
//
BYTE signal_0_7;
#define SIG_FEEDER1_STATE           signal_0_7.b0
#define SIG_FEEDER2_STATE           signal_0_7.b1
#define SIG_FEEDER3_STATE           signal_0_7.b2
#define SIG_FEEDER4_STATE           signal_0_7.b3
#define SIG_FEEDER5_STATE           signal_0_7.b4
#define SIG_FEEDER_COEX_STATE       signal_0_7.b5
#define SIG_VP1_STATE               signal_0_7.b6
#define SIG_CONTROL_STATE           signal_0_7.b7
#define Feeder1StatusLED_On()       (SIG_FEEDER1_STATE = HIGH)
#define Feeder1StatusLED_Off()      (SIG_FEEDER1_STATE = LOW)
#define Feeder2StatusLED_On()       (SIG_FEEDER2_STATE = HIGH)
#define Feeder2StatusLED_Off()      (SIG_FEEDER2_STATE = LOW)
#define Feeder3StatusLED_On()       (SIG_FEEDER3_STATE = HIGH)
#define Feeder3StatusLED_Off()      (SIG_FEEDER3_STATE = LOW)
#define Feeder4StatusLED_On()       (SIG_FEEDER4_STATE = HIGH)
#define Feeder4StatusLED_Off()      (SIG_FEEDER4_STATE = LOW)
#define Feeder5StatusLED_On()       (SIG_FEEDER5_STATE = HIGH)
#define Feeder5StatusLED_Off()      (SIG_FEEDER5_STATE = LOW)
#define FeederCoexStatusLED_On()    (SIG_FEEDER_COEX_STATE = HIGH)
#define FeederCoexStatusLED_Off()   (SIG_FEEDER_COEX_STATE = LOW)
#define VacumPump1StatusLED_On()    (SIG_VP1_STATE = HIGH)
#define VacumPump1StatusLED_Off()   (SIG_VP1_STATE = LOW)
#define ControlStatusLED_On()       (SIG_CONTROL_STATE = HIGH)
#define ControlStatusLED_Off()      (SIG_CONTROL_STATE = LOW)
//
//=============================================== SIGNAL 8 - 15
//
BYTE signal_8_15;
#define SIG_TENS_1                  signal_8_15.b0
#define SIG_TENS_2                  signal_8_15.b1
#define SIG_TENS_4                  signal_8_15.b2
#define SIG_TENS_8                  signal_8_15.b3
#define SIG_UNITS_1                 signal_8_15.b4
#define SIG_UNITS_2                 signal_8_15.b5
#define SIG_UNITS_4                 signal_8_15.b6
#define SIG_UNITS_8                 signal_8_15.b7
#define ClearErrorDisplay()         (signal_8_15._byte = 0x00)
#define DisplayTens_0()             (signal_8_15._byte &= 0xf0)
#define DisplayTens_1()             ((signal_8_15._byte &= 0xf0), (SIG_TENS_1 = HIGH))
#define DisplayTens_2()             ((signal_8_15._byte &= 0xf0), (SIG_TENS_2 = HIGH))
#define DisplayTens_3()             ((signal_8_15._byte &= 0xf0), (SIG_TENS_1 = HIGH),(SIG_TENS_2 = HIGH))
#define DisplayTens_4()             ((signal_8_15._byte &= 0xf0), (SIG_TENS_4 = HIGH))
#define DisplayTens_5()             ((signal_8_15._byte &= 0xf0), (SIG_TENS_1 = HIGH),(SIG_TENS_4 = HIGH))
#define DisplayTens_6()             ((signal_8_15._byte &= 0xf0), (SIG_TENS_2 = HIGH),(SIG_TENS_4 = HIGH))
#define DisplayTens_7()             ((signal_8_15._byte &= 0xf0), (SIG_TENS_1 = HIGH),(SIG_TENS_2 = HIGH),(SIG_TENS_4 = HIGH))
#define DisplayTens_8()             ((signal_8_15._byte &= 0xf0), (SIG_TENS_8 = HIGH))
#define DisplayTens_9()             ((signal_8_15._byte &= 0xf0), (SIG_TENS_1 = HIGH),(SIG_TENS_8 = HIGH))
#define DisplayTens_A()             ((signal_8_15._byte &= 0xf0), (SIG_TENS_2 = HIGH),(SIG_TENS_8 = HIGH))
#define DisplayTens_b()             ((signal_8_15._byte &= 0xf0), (SIG_TENS_1 = HIGH),(SIG_TENS_2 = HIGH),(SIG_TENS_8 = HIGH))
#define DisplayTens_C()             ((signal_8_15._byte &= 0xf0), (SIG_TENS_4 = HIGH),(SIG_TENS_8 = HIGH))
#define DisplayTens_d()             ((signal_8_15._byte &= 0xf0), (SIG_TENS_1 = HIGH),(SIG_TENS_4 = HIGH),(SIG_TENS_8 = HIGH))
#define DisplayTens_E()             ((signal_8_15._byte &= 0xf0), (SIG_TENS_2 = HIGH),(SIG_TENS_4 = HIGH),(SIG_TENS_8 = HIGH))
#define DisplayTens_F()             ((signal_8_15._byte &= 0xf0), (SIG_TENS_1 = HIGH),(SIG_TENS_2 = HIGH),(SIG_TENS_4 = HIGH),(SIG_TENS_8 = HIGH))
#define DisplayUnits_0()            (signal_8_15._byte &= 0x0f)
#define DisplayUnits_1()            ((signal_8_15._byte &= 0x0f), (SIG_UNITS_1 = HIGH))
#define DisplayUnits_2()            ((signal_8_15._byte &= 0x0f), (SIG_UNITS_2 = HIGH))
#define DisplayUnits_3()            ((signal_8_15._byte &= 0x0f), (SIG_UNITS_1 = HIGH),(SIG_UNITS_2 = HIGH))
#define DisplayUnits_4()            ((signal_8_15._byte &= 0x0f), (SIG_UNITS_4 = HIGH))
#define DisplayUnits_5()            ((signal_8_15._byte &= 0x0f), (SIG_UNITS_1 = HIGH),(SIG_UNITS_4 = HIGH))
#define DisplayUnits_6()            ((signal_8_15._byte &= 0x0f), (SIG_UNITS_2 = HIGH),(SIG_UNITS_4 = HIGH))
#define DisplayUnits_7()            ((signal_8_15._byte &= 0x0f), (SIG_UNITS_1 = HIGH),(SIG_UNITS_2 = HIGH),(SIG_UNITS_4 = HIGH))
#define DisplayUnits_8()            ((signal_8_15._byte &= 0x0f), (SIG_UNITS_8 = HIGH))
#define DisplayUnits_9()            ((signal_8_15._byte &= 0x0f), (SIG_UNITS_1 = HIGH),(SIG_UNITS_8 = HIGH))
#define DisplayUnits_A()            ((signal_8_15._byte &= 0x0f), (SIG_UNITS_2 = HIGH),(SIG_UNITS_8 = HIGH))
#define DisplayUnits_b()            ((signal_8_15._byte &= 0x0f), (SIG_UNITS_1 = HIGH),(SIG_UNITS_2 = HIGH),(SIG_UNITS_8 = HIGH))
#define DisplayUnits_C()            ((signal_8_15._byte &= 0x0f), (SIG_UNITS_4 = HIGH),(SIG_UNITS_8 = HIGH))
#define DisplayUnits_d()            ((signal_8_15._byte &= 0x0f), (SIG_UNITS_1 = HIGH),(SIG_UNITS_4 = HIGH),(SIG_UNITS_8 = HIGH))
#define DisplayUnits_E()            ((signal_8_15._byte &= 0x0f), (SIG_UNITS_2 = HIGH),(SIG_UNITS_4 = HIGH),(SIG_UNITS_8 = HIGH))
#define DisplayUnits_F()            ((signal_8_15._byte &= 0x0f), (SIG_UNITS_1 = HIGH),(SIG_UNITS_2 = HIGH),(SIG_UNITS_4 = HIGH),(SIG_UNITS_8 = HIGH))
//
//=============================================== SHIFT REGISTER
//
BYTE hc595_shift;
#define STATUS_LED                  hc595_shift.b7
#define OUTPUT_ENABLE               hc595_shift.b6
#define OUT2_CLK                    hc595_shift.b5
#define OUT1_CLK                    hc595_shift.b4
#define INPUT_24_31                 hc595_shift.b3
#define INPUT_16_23                 hc595_shift.b2
#define INPUT_8_15                  hc595_shift.b1
#define INPUT_0_7                   hc595_shift.b0
#define StatusLED_On()              (STATUS_LED = LOW, HC595_Shift())
#define StatusLED_Off()             (STATUS_LED = HIGH, HC595_Shift())
#define IsStatusLED_On()            (STATUS_LED == LOW)
#define EnableDO()                  (OUTPUT_ENABLE = LOW, HC595_Shift())
#define DisableDO()                 (OUTPUT_ENABLE = HIGH, HC595_Shift())
#define ClockOUT1()                 (OUT1_CLK = HIGH, HC595_Shift(), OUT1_CLK = LOW, HC595_Shift())
#define ClockOUT2()                 (OUT2_CLK = HIGH, HC595_Shift(), OUT2_CLK = LOW, HC595_Shift())
#define EnableDIN_0_7()             (INPUT_0_7 = LOW, HC595_Shift())
#define DisableDIN_0_7()            (INPUT_0_7 = HIGH, HC595_Shift())
#define EnableDIN_8_15()            (INPUT_8_15 = LOW, HC595_Shift())
#define DisableDIN_8_15()           (INPUT_8_15 = HIGH, HC595_Shift())
#define EnableDIN_16_23()           (INPUT_16_23 = LOW, HC595_Shift())
#define DisableDIN_16_23()          (INPUT_16_23 = HIGH, HC595_Shift())
#define EnableDIN_24_31()           (INPUT_24_31 = LOW, HC595_Shift())
#define DisableDIN_24_31()          (INPUT_24_31 = HIGH, HC595_Shift())
//
//
/** M A C R O S ***************************************************************/
//
//
#define	TMR0Interrupt_Init()        (OPTION = 0x82)
#define	TMR0Interrupt_Enable()      (T0IE = HIGH)
#define	TMR0Interrupt_Disable()     (T0IE = LOW)
#define TMR1Interrupt_Init()        (T1CON = 0x11, TMR1IF = TRUE, TMR1IE = TRUE)
#define TMR1Interrupt_Enable()      (PEIE = TRUE)
#define TMR1Interrupt_Disable()     (PEIE = FALSE)
#define GlobalInterrupt_Enable()    (GIE = HIGH)
#define GlobalInterrupt_Disable()   (GIE = LOW)
#define	PWM_Init() (PR2 = 0xff, CCPR1L = 0, CCPR2L = 0, CCP1CON = 0x0c, CCP2CON = 0x0c, T2CON = 0x07)
#define VoltageReference_Init()     (FVRCON = 0xb2)
#define ADC_InitCH0()   (ADCON0 = 0b00000001, ADCON1 = 0xe0)
#define ADC_InitCH1()   (ADCON0 = 0b00000101, ADCON1 = 0xe0)
#define ADC_InitCH2()   (ADCON0 = 0b00001001, ADCON1 = 0xe0)
#define ADC_InitCH3()   (ADCON0 = 0b00001101, ADCON1 = 0xe0)
#define ADC_InitCH4()   (ADCON0 = 0b00010001, ADCON1 = 0xe0)
#define ADC_InitCH5()   (ADCON0 = 0b00010101, ADCON1 = 0xe0)
#define ADC_InitCH6()   (ADCON0 = 0b00011001, ADCON1 = 0xe0)
#define ADC_Convert()   (GO_nDONE = HIGH)
#define ADC_Busy()      (GO_nDONE == HIGH)
#define SetDataPortAsInput()    (DataDir() = 0xff)
#define SetDataPortAsOutput()   (DataDir() = 0x00)
#define SetSig1Clk_High()       (DOUT_SIG1_CLK = HIGH)
#define SetSig1Clk_Low()        (DOUT_SIG1_CLK = LOW)
#define SetSig2Clk_High()       (DOUT_SIG2_CLK = HIGH)
#define SetSig2Clk_Low()        (DOUT_SIG2_CLK = LOW)
#define SetDac1Latch_High()     (DOUT_DAC1_LATCH = HIGH)
#define SetDac1Latch_Low()      (DOUT_DAC1_LATCH = LOW)
#define SetDac1CS_High()        (DOUT_DAC1_CS = HIGH)
#define SetDac1CS_Low()         (DOUT_DAC1_CS = LOW)
#define EnableShiftOutput()     (DOUT_SHIFT_OE = LOW)
#define DisableShiftOutput()    (DOUT_SHIFT_OE = HIGH)
#define SetRS485DataDir_High()  (RS485_DATA_DIR = HIGH)
#define SetRS485DataDir_Low()   (RS485_DATA_DIR = LOW)
#define ClockShift()            (DOUT_SERIAL_CLOCK = HIGH, DOUT_SERIAL_CLOCK = LOW)
#define ClockLatch()            (DOUT_SHIFT_LATCH = HIGH, DOUT_SHIFT_LATCH = LOW)
//
//
/** F U N C T I O N   P R O T O T Y P E S   ***********************************/
//
//
void low_isr(void);
void high_isr(void);
void InitRAM(void);
void InitSYS(void);
void CheckTimer(void);
void CheckDigitalInput(void);
void CheckAnalogInput(void);
void ProcessFeeder1(void);
void ProcessFeeder2(void);
void ProcessFeeder3(void);
void ProcessFeeder4(void);
void ProcessFeeder5(void);
void ProcessFeederCoex(void);
void ProcessCyclon1(void);
void ProcessVacumPump1(void);
void SetRelayOutput(void);
void SetAnalogOutput(void);
void SetSignalOutput(void);
void SetTimer(int timer_for_setup, unsigned int period);
void HC595_Shift(void);
void CheckSafety(void);
//
//
//
#endif // Razlaz sijela
