// This tab allows to configure the GCode interpreter in order to match it to your machine
#ifndef CONFIG4AXIS_H 
#define CONFIG4AXIS_H


// **************************** Steps per tour ******************
#define X_STEP_PER_TOUR 200
#define Y_STEP_PER_TOUR 200
#define Z_STEP_PER_TOUR 200
#define U_STEP_PER_TOUR 200

// **************************** Micropas  ******************
#define X_MICROPAS 16
#define Y_MICROPAS 16
#define Z_MICROPAS 16
#define U_MICROPAS 16

// *************************** mm per tour ******************
#define X_MM_PER_TOUR 4
#define Y_MM_PER_TOUR 4
#define Z_MM_PER_TOUR 4
#define U_MM_PER_TOUR 4

// *************************** Reverse rotation ******************
#define X_ROTATION NORMAL                // choose between NORMAL and REVERSE
#define Y_ROTATION NORMAL                // choose between NORMAL and REVERSE
#define Z_ROTATION NORMAL                // choose between NORMAL and REVERSE
#define U_ROTATION NORMAL                // choose between NORMAL and REVERSE

//*************************** Endstop logic
#define ENDSTOP_LOGIC CLOSED_WHEN_EOC     // choose between CLOSED_WHEN_EOC and OPEN_WHEN_EOC
#define X_DIR_ENDSTOP_MIN  NORMAL         // choose between NORMAL and REVERSE
#define Y_DIR_ENDSTOP_MIN  NORMAL         // choose between NORMAL and REVERSE
#define Z_DIR_ENDSTOP_MIN  NORMAL         // choose between NORMAL and REVERSE
#define U_DIR_ENDSTOP_MIN  NORMAL         // choose between NORMAL and REVERSE

 
//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
#define MAX_HEATRATE         (80) // max % of heating

#define MAX_FEEDRATE         (720) // mm/min  12mm/sec = 720 mm/min
#define MIN_FEEDRATE         (1)

#define CLOCK_FREQ           (16000000L)

#define MAX_SEGMENTS         (100)    // can be increased to about 20 or more; 3 could be used just for testing purpose


#define VERSION              (1)  // firmware version

#define BAUD                 (115200)  // How fast is the Arduino talking?
#define MAX_BUF              (64)  // What is the longest line Arduino can get?


//**************************************   Here the pin being used for a mega 2560 with a ramp 1.4 board. ******************
// Define step pulse output pins.    
#define X_STEP_DDR      DDRF
#define X_STEP_PORT     PORTF
#define X_STEP_BIT      0  // Mega PF0 (A0)

#define Y_STEP_DDR      DDRF
#define Y_STEP_PORT     PORTF
#define Y_STEP_BIT      6  // Mega PF6 (A6)

#define Z_STEP_DDR      DDRA
#define Z_STEP_PORT     PORTA
#define Z_STEP_BIT      4  // Mega PA4 (D26)

#define U_STEP_DDR      DDRC
#define U_STEP_PORT     PORTC
#define U_STEP_BIT      1  // Mega PC1 (D36)

#define X_STEP_MASK     (1<<X_STEP_BIT) 
#define Y_STEP_MASK     (1<<Y_STEP_BIT)
#define Z_STEP_MASK     (1<<Z_STEP_BIT)
#define U_STEP_MASK     (1<<U_STEP_BIT)  
 
// Define direction output pins. 
#define X_DIRECTION_DDR     DDRF
#define X_DIRECTION_PORT    PORTF
#define X_DIRECTION_BIT   1  // Mega PF1 (A1)

#define Y_DIRECTION_DDR     DDRF
#define Y_DIRECTION_PORT    PORTF
#define Y_DIRECTION_BIT   7  // Mega PF7 (A7)

#define Z_DIRECTION_DDR     DDRA
#define Z_DIRECTION_PORT    PORTA
#define Z_DIRECTION_BIT   6  // Mega PA6 (D28)

#define U_DIRECTION_DDR     DDRC
#define U_DIRECTION_PORT    PORTC
#define U_DIRECTION_BIT   3  // Mega PC3 (D30)

#define X_DIRECTION_MASK    (1<<X_DIRECTION_BIT)
#define Y_DIRECTION_MASK    (1<<Y_DIRECTION_BIT)
#define Z_DIRECTION_MASK    (1<<Z_DIRECTION_BIT)
#define U_DIRECTION_MASK    (1<<U_DIRECTION_BIT)

// Define stepper driver enable/disable output pin.
#define X_STEPPERS_ENABLE_DDR    DDRD
#define X_STEPPERS_ENABLE_PORT   PORTD
#define X_STEPPERS_ENABLE_BIT    7  // Mega PD7 (D38) 

#define Y_STEPPERS_ENABLE_DDR    DDRF
#define Y_STEPPERS_ENABLE_PORT   PORTF
#define Y_STEPPERS_ENABLE_BIT    2  // Mega PF2 (A2)

#define Z_STEPPERS_ENABLE_DDR    DDRA
#define Z_STEPPERS_ENABLE_PORT   PORTA
#define Z_STEPPERS_ENABLE_BIT    2  // Mega PA2 (D24)

#define U_STEPPERS_ENABLE_DDR    DDRC
#define U_STEPPERS_ENABLE_PORT   PORTC
#define U_STEPPERS_ENABLE_BIT    7  // Mega PC7 (D30) 

#define X_STEPPERS_ENABLE_MASK   (1<<X_STEPPERS_ENABLE_BIT)
#define Y_STEPPERS_ENABLE_MASK   (1<<Y_STEPPERS_ENABLE_BIT) 
#define Z_STEPPERS_ENABLE_MASK   (1<<Z_STEPPERS_ENABLE_BIT) 
#define U_STEPPERS_ENABLE_MASK   (1<<U_STEPPERS_ENABLE_BIT) 

// Heating on UNO uses PWM on timer 2 so it must be on arduino pin 3 or 11
//  #define HEATING_DDR    DDRB
//  #define HEATING_PORT   PORTB
//  #define HEATING_PWM_BIT    3  // Uno Digital Pin 11 = output of OC2A in pwm mode

// Heating on Mega uses PWM on timer 4 (????) and is on pin PH5 = D8 = OC4C
  #define HEATING_DDR    DDRH
  #define HEATING_PORT   PORTH
  #define HEATING_PWM_BIT    5  // Uno Digital Pin 11 = output of OC2A in pwm mode

// Define endstop input pins.
#define X_END_MIN_DDR      DDRE
#define X_END_MIN_PORT     PORTE
#define X_END_MIN_PIN     PINE
#define X_END_MIN_BIT      5  // Mega PE5 (D3) = INT5

#define Y_END_MIN_DDR      DDRE
#define Y_END_MIN_PORT     PORTE
#define Y_END_MIN_PIN     PINE
#define Y_END_MIN_BIT      4  // Mega PE4 (D2) = INT4

#define Z_END_MIN_DDR      DDRD
#define Z_END_MIN_PORT     PORTD
#define Z_END_MIN_PIN     PIND
#define Z_END_MIN_BIT      3  // Mega PD3 (TX1) = INT3

#define U_END_MIN_DDR      DDRD
#define U_END_MIN_PORT     PORTD
#define U_END_MIN_PIN     PIND
#define U_END_MIN_BIT      2  // Mega PD2 (RX1) = INT2
   
#define CLOSED_WHEN_EOC 0
#define OPEN_WHEN_EOC 1

#define NORMAL         0
#define REVERSE        1

#endif
