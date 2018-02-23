//------------------------------------------------------------------------------
// This code reuses a big part of code from 6 Axis CNC Demo Rumba - supports raprapdiscount RUMBA controller
// dan@marginallycelver.com 2013-10-28
// RUMBA should be treated like a MEGA 2560 Arduino.
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/GcodeCNCDemo for more information.

#include "config4axis.h"

// for debug 
//#define VERBOSE              (1)  // add to get a lot more serial output.  

//------------------------------------------------------------------------------
// STRUCTS
//------------------------------------------------------------------------------
typedef struct {
  int32_t absdeltaX ;
  int32_t absdeltaY ;
  int32_t absdeltaZ ;
  int32_t absdeltaU ;
  uint8_t dirX ;
  uint8_t dirY ;
  uint8_t dirZ ;
  uint8_t dirU ;
  volatile int32_t steps_count;
  volatile uint8_t prescaler ; // prescaler: for timer 1
  volatile uint16_t cycles_per_tick ; // value for compA in timer1
  volatile int8_t segment_heat_rate ;
} Segment;


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------


char buffer[MAX_BUF];  // where we store the message until we get a CR/LF
int sofar;  // how much is in the buffer

volatile Segment line_segments[MAX_SEGMENTS]; // buffer where segments are stored waiting for execution
static volatile uint8_t current_segment_idx;
static volatile uint8_t last_segment_idx;

// Used to avoid ISR nesting of the "Stepper Driver Interrupt". Should never occur though.
static volatile uint8_t busy;   

static float position_mmX , position_mmY , position_mmZ , position_mmU;  // position after last line having been added from serial stream

static int32_t position_stepsX , position_stepsY , position_stepsZ , position_stepsU;  // position after last line having been added from serial stream
static int32_t overX, overY, overZ, overU ;// used by breselman algo
static volatile int32_t steps_left;
static int32_t steps_in_executed_segment;

static volatile uint8_t X_dir_outbit ;
static volatile uint8_t Y_dir_outbit ;
static volatile uint8_t Z_dir_outbit ;
static volatile uint8_t U_dir_outbit ;

static volatile uint8_t X_step_outbit ;
static volatile uint8_t Y_step_outbit ;
static volatile uint8_t Z_step_outbit ;
static volatile uint8_t U_step_outbit ;

static volatile uint8_t X_end_outbit ;
static volatile uint8_t Y_end_outbit ;
static volatile uint8_t Z_end_outbit ;
static volatile uint8_t U_end_outbit ;

static volatile uint8_t X_end_dir_outbit ;
static volatile uint8_t Y_end_dir_outbit ;
static volatile uint8_t Z_end_dir_outbit ;
static volatile uint8_t U_end_dir_outbit ;

static uint8_t prev_X_end_outbit ; // for debuging (used in main loop)

static volatile int32_t pX , pY, pZ, pU ; // store the real position of each motor (updated during stepping) (in micro steps)

volatile Segment *exec_segment = NULL ;
volatile uint8_t test1; // just to test interrupts (debug purpose)
volatile uint8_t test2; // just to test interrupts
volatile uint8_t test3; // just to test interrupts
volatile uint8_t test4; // just to test interrupts
volatile uint8_t test5; // just to test interrupts
volatile uint8_t test6; // just to test interrupts

// settings
#define REL (0)
#define ABS (1)
uint8_t mode_abs = REL ;  // mode is Relatif per default

// default speeds
float feed_rate= MAX_FEEDRATE / 4 ;  // mm/min (value per default)

// default heating
int heat_rate = 0 ;
boolean heat_enabled = false ; // default disabled

float steps_per_mmX ;
float steps_per_mmY ; 
float steps_per_mmZ ; 
float steps_per_mmU ; 
 
float mm_per_stepX ;
float mm_per_stepY ;
float mm_per_stepZ ;
float mm_per_stepU ;

float mm_per_stepX2 ;
float mm_per_stepY2 ;
float mm_per_stepZ2 ;
float mm_per_stepU2 ;

//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

/**
 * delay for the appropriate number of microseconds
 * @input ms how many milliseconds to wait
 */
void pause(long ms) {
  delay(ms/1000);
  delayMicroseconds(ms%1000);  // delayMicroseconds doesn't work for values > ~16k.
}


/**
 * Update the feedrate with the value received from parser (if any) (= steps per second if only one motor moves )
 * @input nfr the new speed in steps/second
 */
void feedrate(float nfr) {
  if(feed_rate==nfr) return ;  // same as last time?  quit now.
  if(nfr>MAX_FEEDRATE) {
    Serial.print(F("Feedrate set to maximum ("));
    Serial.print(MAX_FEEDRATE);
    Serial.println(F("steps/s)"));
    feed_rate=MAX_FEEDRATE; return ;
  } 
  if(nfr<MIN_FEEDRATE) {  // don't allow crazy feed rates
    Serial.print(F("Feedrate set to minimum ("));
    Serial.print(MIN_FEEDRATE);
    Serial.println(F("steps/s)"));
    feed_rate=MIN_FEEDRATE; return ;
  }
  feed_rate=nfr;
  return ;
}

void applyHeat( void) {                                       // to do : use another timer
  if ( ( heat_enabled ) && ( heat_rate > 0 )) { 
// for Uno using timer 2
//    TCCR2A |= (1 << COM2A1) ; // activate output of compA
//    OCR2A = heat_rate ; 
//  } else {
//    TCCR2A &= ~(1 << COM2A1) ; // de activate output of compA 
//  }     
//For Mega using timer 4 compare C
    TCCR4A |= (1 << COM4C1) ; // activate output of compC of timer 4 (output D8 = PH5)
    OCR4C = heat_rate ; 
  } else {
    TCCR4A &= ~(1 << COM4C1) ; // de activate output of compC 
  }     
  
}

void heatrate(float nfr) {
  if(heat_rate==nfr) return ;  // same as last time?  quit now.
  if(nfr>MAX_HEATRATE) nfr = MAX_FEEDRATE;   
  if(nfr<0) { 
    heat_rate = 0 ; 
  } else {
    heat_rate = nfr * 2.55 ; }
  applyHeat( );
#ifdef VERBOSE
    Serial.print(F("Heat applied= "));  Serial.println(heat_rate);
#endif
  
  return ;
}

void heat_enable() {
  heat_enabled = true ;  
  applyHeat() ;
}

void heat_disable() {                                  // to do : use another timer
  heat_enabled = false ;
// on Uno use timer 2
//  TCCR2A &= ~(1 << COM2A1) ; // de activate output of compA    
// on mega use timer 4 Compare 
  TCCR4A &= ~(1 << COM4C1) ; // de activate output of compC    
}

/**
 * Set the logical position
 * @input npx new position x
 * @input npy new position y
 */
void position(float npx,float npy,float npz,float npu) {
  // here is a good place to add sanity tests
  position_mmX=npx;
  position_mmY=npy;
  position_mmZ=npz;
  position_mmU=npu;
}


int8_t get_next_segment(int8_t i) {
  return ( i + 1 ) % MAX_SEGMENTS;
}


int8_t get_prev_segment(int8_t i) {
  return ( i + MAX_SEGMENTS - 1 ) % MAX_SEGMENTS;
}


/**********************************************************************************************************
 * Process all line segments in the ring buffer.  Uses bresenham's line algorithm to move all motors.
 * is called for the first time by line() when segment buffer is empty with a short time and direction/strps = 0 
 */
ISR(TIMER1_COMPA_vect) {
  if (busy) { return ; } 
  blink_led() ;
  // Set the direction pins a couple of nanoseconds before we step the steppers
  //DIRECTION_PORT = (DIRECTION_PORT & ~DIRECTION_MASK) | (dir_outbits & DIRECTION_MASK);
  X_DIRECTION_PORT = (X_DIRECTION_PORT & ~X_DIRECTION_MASK) | (X_dir_outbit) ;       // only 1 bit is set in the byte (done when segment is filled)         
  Y_DIRECTION_PORT = (Y_DIRECTION_PORT & ~Y_DIRECTION_MASK) | (Y_dir_outbit) ;                
  Z_DIRECTION_PORT = (Z_DIRECTION_PORT & ~Z_DIRECTION_MASK) | (Z_dir_outbit) ;                
  U_DIRECTION_PORT = (U_DIRECTION_PORT & ~U_DIRECTION_MASK) | (U_dir_outbit) ;                
  delayMicroseconds(1) ; // wait 1 usec before applying step pulse
  // in one direction, we don't take care of endstop to generate the pulse, in the other wel.
#if (defined X_DIR_ENDSTOP_MIN) && ( X_DIR_ENDSTOP_MIN == REVERSE)
  if (X_dir_outbit ) {                           
    X_STEP_PORT = (X_STEP_PORT & ~X_STEP_MASK) | ( X_step_outbit ) ;  // first time we come in, step_outbit = 0 and so there is no step.
  } else {
    X_STEP_PORT = (X_STEP_PORT & ~X_STEP_MASK) | ( X_step_outbit & X_end_outbit) ;  // first time we come in, step_outbit = 0 and so there is no step.  
  }
#else
  if (X_dir_outbit ) {                           
    X_STEP_PORT = (X_STEP_PORT & ~X_STEP_MASK) | ( X_step_outbit & X_end_outbit) ;  // first time we come in, step_outbit = 0 and so there is no step.  
  } else {
    X_STEP_PORT = (X_STEP_PORT & ~X_STEP_MASK) | ( X_step_outbit ) ;  // first time we come in, step_outbit = 0 and so there is no step.
  }
#endif
#if (defined Y_DIR_ENDSTOP_MIN) && ( Y_DIR_ENDSTOP_MIN == REVERSE)  
  if (Y_dir_outbit ) {                           
    Y_STEP_PORT = (Y_STEP_PORT & ~Y_STEP_MASK) | ( Y_step_outbit ) ;  // first time we come in, step_outbit = 0 and so there is no step.
  } else {
    Y_STEP_PORT = (Y_STEP_PORT & ~Y_STEP_MASK) | ( Y_step_outbit & Y_end_outbit) ;  // first time we come in, step_outbit = 0 and so there is no step.
  }
#else
  if (Y_dir_outbit ) {                           
    Y_STEP_PORT = (Y_STEP_PORT & ~Y_STEP_MASK) | ( Y_step_outbit & Y_end_outbit) ;  // first time we come in, step_outbit = 0 and so there is no step.
  } else {
    Y_STEP_PORT = (Y_STEP_PORT & ~Y_STEP_MASK) | ( Y_step_outbit ) ;  // first time we come in, step_outbit = 0 and so there is no step.
  }
#endif
#if (defined Z_DIR_ENDSTOP_MIN) && ( Z_DIR_ENDSTOP_MIN == REVERSE)
  if (Z_dir_outbit ) {
    Z_STEP_PORT = (Z_STEP_PORT & ~Z_STEP_MASK) | ( Z_step_outbit ) ;  // first time we come in, step_outbit = 0 and so there is no step.
  } else {
    Z_STEP_PORT = (Z_STEP_PORT & ~Z_STEP_MASK) | ( Z_step_outbit & Z_end_outbit) ;  // first time we come in, step_outbit = 0 and so there is no step.
  }
#else
  if (Z_dir_outbit ) {
    Z_STEP_PORT = (Z_STEP_PORT & ~Z_STEP_MASK) | ( Z_step_outbit & Z_end_outbit) ;  // first time we come in, step_outbit = 0 and so there is no step.
  } else {
    Z_STEP_PORT = (Z_STEP_PORT & ~Z_STEP_MASK) | ( Z_step_outbit ) ;  // first time we come in, step_outbit = 0 and so there is no step.
  }
#endif
#if (defined U_DIR_ENDSTOP_MIN) && ( U_DIR_ENDSTOP_MIN == REVERSE)
  if (U_dir_outbit ) {
    U_STEP_PORT = (U_STEP_PORT & ~U_STEP_MASK) | ( U_step_outbit ) ;  // first time we come in, step_outbit = 0 and so there is no step.
  } else {
    U_STEP_PORT = (U_STEP_PORT & ~U_STEP_MASK) | ( U_step_outbit & U_end_outbit) ;  // first time we come in, step_outbit = 0 and so there is no step.
  }
#else
  if (U_dir_outbit ) {
    U_STEP_PORT = (U_STEP_PORT & ~U_STEP_MASK) | ( U_step_outbit & U_end_outbit) ;  // first time we come in, step_outbit = 0 and so there is no step.
  } else {
    U_STEP_PORT = (U_STEP_PORT & ~U_STEP_MASK) | ( U_step_outbit ) ;  // first time we come in, step_outbit = 0 and so there is no step.
  }
#endif  
  
  delayMicroseconds(1) ; // wait 1 usec before resetting step pulse
  X_STEP_PORT = (X_STEP_PORT & ~X_STEP_MASK) ; //reset the steps pins
  Y_STEP_PORT = (Y_STEP_PORT & ~Y_STEP_MASK) ; //reset the steps pins
  Z_STEP_PORT = (Z_STEP_PORT & ~Z_STEP_MASK) ; //reset the steps pins
  U_STEP_PORT = (U_STEP_PORT & ~U_STEP_MASK) ; //reset the steps pins
  busy = true ;
  
  if ( exec_segment == NULL) {        // Is this segment done?
    
    if( current_segment_idx != last_segment_idx ) {   // try to find next segment
        
        exec_segment = &line_segments[current_segment_idx];

        // set frequency to segment feed rate
        TCNT1 = 0 ;
        TIFR1 = (1 << OCIE1A) ; // reset interrupt flag wrinting 1 in interrupt flag
        TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (exec_segment->prescaler <<CS10); // Update prescaler 
        OCR1A = exec_segment->cycles_per_tick ;  // set up timer         
        
        steps_in_executed_segment = steps_left = exec_segment->steps_count ;    //preload number of steps
        
        X_dir_outbit = exec_segment->dirX  ; // load the direction pins (there is only one bit set; which bit depends on setup; job is done in line() )
        Y_dir_outbit = exec_segment->dirY  ;
        Z_dir_outbit = exec_segment->dirZ  ;
        U_dir_outbit = exec_segment->dirU ;
            
        
        overX = overY = overZ = overU = steps_left >> 1; //prepare besenmans algo    
    } else { // stop interrupt because there is no segment to prepare
      //test3++;
    // Disable Stepper Driver Interrupt. Allow Stepper Port Reset Interrupt to finish, if active.
      TIMSK1 &= ~(1<<OCIE1A); // Disable Timer1 interrupt
      TCCR1B = (TCCR1B & ~((1<<CS12) | (1<<CS11))) | (1<<CS10); // Reset clock to no prescaling = no clock tick anymore.
      busy = false;
      return;
    }
  }
    //test4++;
    // make a step
    overX += exec_segment->absdeltaX;
    if(overX >= steps_in_executed_segment) {
        X_step_outbit = (1<<X_STEP_BIT);
        overX -= steps_in_executed_segment ;
        pX += ( X_dir_outbit ? 1 : -1 ) ;
    } else { X_step_outbit = 0 ;} ;
    overY += exec_segment->absdeltaY;
    if(overY >= steps_in_executed_segment) {
        Y_step_outbit = (1<<Y_STEP_BIT) ;
        overY -= steps_in_executed_segment ;
        pY += ( Y_dir_outbit ? 1 : -1 ) ;
    } else { Y_step_outbit = 0 ;} ;
    overX += exec_segment->absdeltaZ;
    if(overZ >= steps_in_executed_segment) {
        Z_step_outbit = (1<<Z_STEP_BIT);
        overZ -= steps_in_executed_segment ;
        pZ += ( Z_dir_outbit ? 1 : -1 ) ;
    } else { Z_step_outbit = 0 ;} ;
    overU += exec_segment->absdeltaU;
    if(overU >= steps_in_executed_segment) {
        U_step_outbit |= (1<<U_STEP_BIT);
        overU -= steps_in_executed_segment ;
        pU += ( U_dir_outbit ? 1 : -1 ) ;
    } else { U_step_outbit = 0 ;} ;
    steps_left-- ;
    
    if (steps_left == 0) {
      //test6++ ;
      exec_segment = NULL;
      // Segment is complete. Discard current segment and advance segment indexing.
      current_segment_idx++ ;
      if ( current_segment_idx == MAX_SEGMENTS) { current_segment_idx = 0; }
    }
  busy = false;
}



/********************************************
 * Line
 * Uses bresenham's line algorithm to move both motors
 *********************************************
 **/
void line(float n0,float n1,float n2,float n3 ) {

  int32_t _target_steps  ;
  int32_t _delta_steps ;
  
  int8_t next_segment_idx = last_segment_idx + 1;
  if ( next_segment_idx == MAX_SEGMENTS) next_segment_idx = 0 ;
   
#ifdef VERBOSE
  Serial.println(F("process line ") ) ;
  Serial.print(F(" test1=") ) ;Serial.println(test1 ) ;
  Serial.print(F(" test2=") ) ;Serial.println(test2 ) ;
  Serial.print(F(" test3=") ) ;Serial.println(test3 ) ;
  Serial.print(F(" test4=") ) ;Serial.println(test4 ) ;
  Serial.print(F(" test5=") ) ;Serial.println(test5 ) ;
  Serial.print(F(" test6=") ) ;Serial.println(test6 ) ;
  Serial.print(F(" X=") ) ;Serial.print(n0 ) ;
  Serial.print(F(" Y=") ) ;Serial.print(n1 ) ;
  Serial.print(F(" Z=") ) ;Serial.print(n2 ) ;
  Serial.print(F(" U=") ) ;Serial.println(n3 ) ;
  Serial.print(F("currentSeg= ") ) ; Serial.println(current_segment_idx ) ; 
  Serial.print(F("lastSegIdx= ") ) ; Serial.println(last_segment_idx) ;
  Serial.print(F("nextSegIdx= ") ) ; Serial.println(next_segment_idx) ;
#endif  
  
  
  while( next_segment_idx == current_segment_idx ) {
    // the buffer is full, we are way ahead of the motion system
    delay(1);
  }

  volatile Segment &new_seg = line_segments[last_segment_idx];
  //Segment &old_seg = line_segments[old_segment_idx];
 
  if ( mode_abs ) {
    position_mmX = n0 ;
    position_mmY = n1 ;
    position_mmZ = n2 ;
    position_mmU = n3 ;
  } else {  
    position_mmX += n0 ;
    position_mmY += n1 ;
    position_mmZ += n2 ;
    position_mmU += n3 ;
  }  // from here Target new expected position in float
  _target_steps = lround(position_mmX * steps_per_mmX) ;
  _delta_steps = _target_steps - position_stepsX ;
  position_stepsX = _target_steps ;  
  new_seg.absdeltaX = labs(_delta_steps);
#if defined (X_DIR_ENDSTOP_MIN) && ( X_DIR_ENDSTOP_MIN == REVERSE)  
  new_seg.dirX = (_delta_steps < 0 ? HIGH : LOW) << X_DIRECTION_BIT;   // direction LOW = dir positif
#else
  new_seg.dirX = (_delta_steps < 0 ? LOW : HIGH) << X_DIRECTION_BIT;   // direction LOW = dir negatif
#endif  
  //if (delta_mm < 0 ) { block->direction_bits |= get_direction_pin_mask(idx); }
  _target_steps = lround(position_mmY * steps_per_mmY) ;
  _delta_steps = _target_steps - position_stepsY ;
  position_stepsY = _target_steps ;  
  new_seg.absdeltaY = labs(_delta_steps);
#if defined (Y_DIR_ENDSTOP_MIN) && ( Y_DIR_ENDSTOP_MIN == REVERSE)  
  new_seg.dirY = (_delta_steps < 0 ? HIGH : LOW) << Y_DIRECTION_BIT;   // direction LOW = dir positif
#else
  new_seg.dirY = (_delta_steps < 0 ? LOW : HIGH) << Y_DIRECTION_BIT;   // direction LOW = dir negatif
#endif  
  //if (delta_mm < 0 ) { block->direction_bits |= get_direction_pin_mask(idx); }
  _target_steps = lround(position_mmZ * steps_per_mmZ) ;
  _delta_steps = _target_steps - position_stepsZ ;
  position_stepsZ = _target_steps ;  
  new_seg.absdeltaZ = labs(_delta_steps);
#if defined (Z_DIR_ENDSTOP_MIN) && ( Z_DIR_ENDSTOP_MIN == REVERSE)  
  new_seg.dirZ = (_delta_steps < 0 ? HIGH : LOW) << Z_DIRECTION_BIT;   // direction LOW = dir positif
#else
  new_seg.dirZ = (_delta_steps < 0 ? LOW : HIGH) << Z_DIRECTION_BIT;   // direction LOW = dir negatif
#endif  
  //if (delta_mm < 0 ) { block->direction_bits |= get_direction_pin_mask(idx); }
  _target_steps = lround(position_mmU * steps_per_mmU) ;
  _delta_steps = _target_steps - position_stepsU ;
  position_stepsU = _target_steps ;  
  new_seg.absdeltaU = labs(_delta_steps);
#if defined (U_DIR_ENDSTOP_MIN) && ( U_DIR_ENDSTOP_MIN == REVERSE)  
  new_seg.dirU = (_delta_steps < 0 ? HIGH : LOW) << U_DIRECTION_BIT;   // direction LOW = dir positif
#else
  new_seg.dirU = (_delta_steps < 0 ? LOW : HIGH) << U_DIRECTION_BIT;   // direction LOW = dir negatif
#endif  
  //if (delta_mm < 0 ) { block->direction_bits |= get_direction_pin_mask(idx); }
  
  // take the max number of steps
  new_seg.steps_count= new_seg.absdeltaX ;
  if (new_seg.steps_count < new_seg.absdeltaY) new_seg.steps_count= new_seg.absdeltaY ;
  if (new_seg.steps_count < new_seg.absdeltaZ) new_seg.steps_count= new_seg.absdeltaZ ;
  if (new_seg.steps_count < new_seg.absdeltaU) new_seg.steps_count= new_seg.absdeltaU ; 

  if ( new_seg.steps_count == 0 ) return; 
  if ( heat_enabled) {                         // store heat rate in segment
    new_seg.segment_heat_rate = heat_rate ;
  } else {
    new_seg.segment_heat_rate = 0 ;
  }

  // calculate the max distance for XY and ZU axis (4 axis foam cutter)
  float distance_mm =  ((float) new_seg.absdeltaX) * ((float) new_seg.absdeltaX )* mm_per_stepX2 + ((float)new_seg.absdeltaY) * ((float) new_seg.absdeltaY) * mm_per_stepY2;  
  float distanceZU = ((float) new_seg.absdeltaZ) * ((float)new_seg.absdeltaZ) * mm_per_stepZ2 + ((float)new_seg.absdeltaU) * ((float)new_seg.absdeltaU) * mm_per_stepU2;  
  if (distanceZU > distance_mm ) distance_mm = distanceZU ; // keep max of both distances
  distance_mm = sqrt( distance_mm) ; 
  // calculate the number of cycles between 2 steps (based on the distance, the feedrate and the max number of steps in the segment)
  // store the value for the timer and the prescaler in the segment buffer.
  // time for the whole segment in min = distance_mm / feedrate (mm/min) = time for steps_count
  // time for the whole segment in sec = distance_mm / feedrate (mm/min) * 60 = time for steps_count
  // time for 1 step_ count in second =  distance_mm / feedrate (mm/min) * 60 / steps_count
  // cycles = CLOCK_FREQ * distance_mm / feedrate (mm/min) * 60 / steps_count
    uint32_t cycles = ceil( (CLOCK_FREQ * 60 * distance_mm / (new_seg.steps_count * feed_rate)  ) ); // (cycles/step_count)   
    
    // Compute step timing and timer prescalar for normal step generation.
    if (cycles < (1UL << 16)) { // < 65536  (4.1ms @ 16MHz)
      new_seg.prescaler = 1; // prescaler: 0
      new_seg.cycles_per_tick = cycles;
    } else if (cycles < (1UL << 19)) { // < 524288 (32.8ms@16MHz)
      new_seg.prescaler = 2; // prescaler: 8
      new_seg.cycles_per_tick = cycles >> 3;
    } else { 
      new_seg.prescaler = 3; // prescaler: 64
      if (cycles < (1UL << 22)) { // < 4194304 (262ms@16MHz)
        new_seg.cycles_per_tick =  cycles >> 6;
      } else { // Just set the slowest speed possible. (Around 4 step/sec.)
        new_seg.cycles_per_tick = 0xffff;
      }
    }

#ifdef VERBOSE
  Serial.print(F("steps= "));  Serial.println(new_seg.steps_count);
  Serial.print(F("dist_mm= "));  Serial.println(distance_mm);
  Serial.print(F("cycles= "));  Serial.println(cycles);
  Serial.print(F("new_cy/tick= ") ) ; Serial.println( new_seg.cycles_per_tick) ; 
  Serial.print(F("prescaler=") ) ; Serial.println(new_seg.prescaler ) ; 
  Serial.print(F("Adding in Idx= "));  Serial.println(last_segment_idx);
  Serial.print(F("absdeltaX= "));  Serial.println(new_seg.absdeltaX);
  Serial.print(F("dirX= "));  Serial.println(new_seg.dirX);
  Serial.print(F("steps= "));  Serial.println(new_seg.steps_count);
#endif

  if( current_segment_idx==last_segment_idx ) { // If the added block has to be executed immediately)
    last_segment_idx = next_segment_idx;   // add the new block (as last segment)
#ifdef VERBOSE
    Serial.print(F("last seg= "));  Serial.println(last_segment_idx);
#endif
    noInterrupts();
    X_dir_outbit = Y_dir_outbit = Z_dir_outbit = U_dir_outbit = 0 ;
    X_step_outbit = Y_step_outbit = Z_step_outbit = U_step_outbit = 0 ;  
    steps_left = 0 ; // so it will force interrupt to upload a segment
    TCNT1  = 0;                   // set the timer clock to 0
    OCR1A = 100 ;                 // let timer fire within a short time
    TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (1 <<CS10); // Update prescaler to 1 (prescaler = 1)
    TIFR1 = (1 << OCIE1A) ;        // reset interrupt flag by writing 1 in interrupt flag
    TIMSK1 |= (1 << OCIE1A);       // enable timer compare interrupt
    interrupts();  // enable global interrupts
  } else { 
    last_segment_idx = next_segment_idx;   // add the new block (as last segment)
  }  
}



/**
 * Look for character /code/ in the buffer and read the float that immediately follows it.
 * @return the value found.  If nothing is found, /val/ is returned.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 **/
float parsenumber(char code,float val) {
  char *ptr=buffer;  // start at the beginning of buffer
  for ( uint8_t i = 0 ; i < sofar ; i++ ) {  // walk to the end  
    if(*ptr==code) {  // if you find code on your walk,
      return atof(ptr+1);  // convert the digits that follow into a float and return it
    }
    ptr++ ;
  }
  return val;  // end reached, nothing found, return default val.
}


/**
 * write a string followed by a float to the serial line.  Convenient for debugging.
 * @input code the string.
 * @input val the float.
 */
void output(char *code,float val) {
  Serial.print(code);
  Serial.println(val);
}


/**
 * print the current position, feedrate, and absolute mode.
 */
void where() {
  output("X",position_mmX);
  output("Y",position_mmY);
  output("Z",position_mmZ);
  output("U",position_mmU);
  output("F",feed_rate);
  Serial.println(mode_abs?"ABS":"REL");
} 


/**
 * display helpful information
 */
void help() {
  Serial.print(F("Grbl v0.10 ['$' for help]"));
  Serial.println(VERSION);
  Serial.println(F("Commands:"));
  Serial.println(F("G00/G01 [X/Y/Z/U (steps)] [F(feedrate)]; - linear move"));
  Serial.println(F("G04 P[seconds]; - delay"));
  Serial.println(F("G90; - absolute mode"));
  Serial.println(F("G91; - relative mode"));
  Serial.println(F("G92 [X/Y/Z/U(steps)]; - change logical position"));
  Serial.println(F("M3; - ensable heat"));
  Serial.println(F("M5; - disable heat"));
  Serial.println(F("M17; - enable motors"));
  Serial.println(F("M18; - disable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M114; - report position and feedrate"));
  Serial.println(F("S; - set heating"));
  Serial.println(F("All commands must end with a newline."));
}


void wait_all_segments_executed( void) {
  while ( ( current_segment_idx != last_segment_idx ) && (exec_segment == NULL ) ) {  // wait that all segment in buffer have been executed.
        delay(1) ;
      }
}

/**
 * Read the input buffer and find any recognized commands.  One G or M command per line.
 */
void processCommand() {

  int cmd = parsenumber('G',-1);
#ifdef VERBOSE
//  Serial.print(F("Cmd= "));  Serial.println(cmd);
#endif
  switch(cmd) {
    case  0:
    case  1: { // line
        feedrate(parsenumber('F',feed_rate));
        if ( mode_abs ) {
          line( parsenumber('X',position_mmX),
              parsenumber('Y',position_mmY),
              parsenumber('Z',position_mmZ),
              parsenumber('U',position_mmU) );
        } else {
          line( parsenumber('X',0),
              parsenumber('Y',0),
              parsenumber('Z',0),
              parsenumber('U',0) );      
        }
        break;
      }
    case  4:  {                           // G04 = pause
      wait_all_segments_executed() ;
      pause(parsenumber('P',0)*1000000);  break;  // convert sec in usec
    }
    case 90:   { mode_abs=1;  break; }  // absolute mode
    case 91:   {mode_abs=0;  break;  }// relative mode
    case 92:  {// set logical position
      wait_all_segments_executed() ;
      position( parsenumber('X',0),
                parsenumber('Y',0),
                parsenumber('Z',0),
                parsenumber('U',0) );
      break;
      }
    default:  break;
  return ;
  }
// Handle M commands *******************************
  cmd = parsenumber('M',-1);
  switch(cmd) {
  case 3:  {     // M3 = enable heat
    wait_all_segments_executed() ;
    heat_enable();
#ifdef VERBOSE
    Serial.print(F("Heat enabled= "));  Serial.println(heat_rate);
#endif
    break;  
  }
  case 5:   {     // M5 = disable heat
    wait_all_segments_executed() ;
    heat_disable();
#ifdef VERBOSE
    Serial.print(F("Heat disabled= "));  Serial.println(heat_rate);
#endif    
    break;  }
  case 17:  {wait_all_segments_executed() ; motor_enable(); break;  } // M17 (enable motor)
  case 18:  {wait_all_segments_executed() ; motor_disable();  break; } // M18 (disable motor)
  case 100:  help();  break;                                           // M100 (send help message on serial port)
  case 114:  {wait_all_segments_executed() ;where();  break; }         // M114 (send the current logical position)
  
  default:  break;
  return ;
  }
// Handle F commands *******************************
  feedrate(parsenumber('F',feed_rate)); // update feedrate if F is present on a line without G command
// Handle S commands  for setting the heating (does not enable heating if it was not enabled ) *******************************  
  if (buffer[0] == 'S') {
    wait_all_segments_executed() ;
    heatrate(parsenumber('S',heat_rate)); // update heatrate if S is present
  }
  if (buffer[0] == '$') {
    if (buffer[1] == '\n') {
      help() ;
    }  
  } 
}


/**
 * prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
 */
void ready() {
  sofar=0;  // clear input buffer
  //Serial.print(F(">"));  // signal ready to receive input
}


/**
 * set up the pins for each motor
 */


void motor_enable() {
  X_STEPPERS_ENABLE_PORT  &=  ~(1<<X_STEPPERS_ENABLE_BIT) ;
  Y_STEPPERS_ENABLE_PORT  &=  ~(1<<Y_STEPPERS_ENABLE_BIT) ;
  Z_STEPPERS_ENABLE_PORT  &=  ~(1<<Z_STEPPERS_ENABLE_BIT) ;
  U_STEPPERS_ENABLE_PORT  &=  ~(1<<U_STEPPERS_ENABLE_BIT) ;
}


void motor_disable() {
  X_STEPPERS_ENABLE_PORT  |=  (1<<X_STEPPERS_ENABLE_BIT) ;
  Y_STEPPERS_ENABLE_PORT  |=  (1<<Y_STEPPERS_ENABLE_BIT) ;
  Z_STEPPERS_ENABLE_PORT  |=  (1<<Z_STEPPERS_ENABLE_BIT) ;
  U_STEPPERS_ENABLE_PORT  |=  (1<<U_STEPPERS_ENABLE_BIT) ;
  
}


void check_limits() { // read the input pins for endstops and activates masks accordingly
// here a logic for normally open contact ; so level 1 = endstop non activated
// When mask is set to 0, the interrupt handler will not generate a step pulse anymore. 
#if defined( ENDSTOP_LOGIC ) && ( ENDSTOP_LOGIC == OPEN_WHEN_EOC )
  #define ENDSTOP_ACTIVATED 0xFF
  #define ENDSTOP_DEACTIVATED 0x00
#else  
  #define ENDSTOP_ACTIVATED 0x00
  #define ENDSTOP_DEACTIVATED 0xFF
#endif  
  if ( X_END_MIN_PIN & ( 1 << X_END_MIN_BIT ) ) {   
    X_end_outbit = ENDSTOP_DEACTIVATED ;
  } else {
    X_end_outbit = ENDSTOP_ACTIVATED ;
  }
  if ( Y_END_MIN_PIN & ( 1 << Y_END_MIN_BIT ) ) {
    Y_end_outbit = ENDSTOP_DEACTIVATED ;
  } else {
    Y_end_outbit = ENDSTOP_ACTIVATED ;
  }
  if ( Z_END_MIN_PIN & ( 1 << Z_END_MIN_BIT ) ) {
    Z_end_outbit = ENDSTOP_DEACTIVATED ;
  } else {
    Z_end_outbit = ENDSTOP_ACTIVATED ;
  }
  if ( U_END_MIN_PIN & ( 1 << U_END_MIN_BIT ) ) {
    U_end_outbit = ENDSTOP_DEACTIVATED ;
  } else {
    U_end_outbit = ENDSTOP_ACTIVATED ;
  }
}



ISR(INT5_vect){          // use for Endstop on INT (X axis)
  check_limits() ;
}

ISR(INT4_vect){          // use for Endstop on INT (Y axis)
  check_limits() ;
}

ISR(INT3_vect){          // use for Endstop on INT (Z axis)
  check_limits() ;
}

ISR(INT2_vect){          // use for Endstop on INT (U axis)
  check_limits() ;
}


/**
 * First thing this machine does on startup.  Runs only once.
 */
void setup() {
  Serial.begin(BAUD);  // open coms 
  pinMode(13, OUTPUT); // set led as output 
  X_STEP_DDR |= X_STEP_MASK;   // pout port pin as output
  Y_STEP_DDR |= Y_STEP_MASK;
  Z_STEP_DDR |= Z_STEP_MASK;
  U_STEP_DDR |= U_STEP_MASK;

  X_DIRECTION_DDR |= X_DIRECTION_MASK ;  // pout port pin as output
  Y_DIRECTION_DDR |= Y_DIRECTION_MASK ;
  Z_DIRECTION_DDR |= Z_DIRECTION_MASK ;
  U_DIRECTION_DDR |= U_DIRECTION_MASK ;
  
  HEATING_DDR |= (1<<HEATING_PWM_BIT); // Configure as PWM output pin.
  HEATING_PORT &= ~(1<<HEATING_PWM_BIT); // force output to 0
  // for uno on timer 2
  //TCCR2A = ((1<<WGM20) | (1<<WGM21)); // Configure PWM output compare timer in fast PWM ; let COM2B1 and COM2B0 to 0 to disable output of PWM
  //TCCR2B = (1<<CS22) ;                 // 1/64 prescaler -> 0.98kHz; let WGM22 to zero for fast PWM

  // for Mega on timer 4
  TCCR4A = ((1<<WGM40) ); // Configure PWM output compare timer in fast PWM 8 bits (WGM4 (So WGM must be 5); let COM4C1 and COM4C0 to 0 to disable output of PWM
  TCCR4B =  ( (1<<WGM42) | (1<<CS41) | (1<<CS40) ) ;                 // WGM42 is set to apply a value 5 = fast PWM 8 bits; CS bits 3 -> 1/64 prescaler -> 0.98kHz; 

  X_STEPPERS_ENABLE_DDR |= X_STEPPERS_ENABLE_MASK ; // pout port pin as output
  Y_STEPPERS_ENABLE_DDR |= Y_STEPPERS_ENABLE_MASK ;
  Z_STEPPERS_ENABLE_DDR |= Z_STEPPERS_ENABLE_MASK ;
  U_STEPPERS_ENABLE_DDR |= U_STEPPERS_ENABLE_MASK ;
  motor_disable() ;
  
  TCCR1A = 0;                   // set entire TCCR1A register to 0
  TCCR1B = (1 << WGM12) | (1<<CS10)  ;       // set entire TCCR1B register to 0 EXCEPT that we apply CTC MODE (continue running on comparator) and prescaller = 1

  steps_per_mmX = 1.0 * X_STEP_PER_TOUR * X_MICROPAS / ((float) X_MM_PER_TOUR) ;
  steps_per_mmY = 1.0 * Y_STEP_PER_TOUR * Y_MICROPAS / ((float) Y_MM_PER_TOUR) ; 
  steps_per_mmZ = 1.0 * Z_STEP_PER_TOUR * Z_MICROPAS / ((float) Z_MM_PER_TOUR) ; 
  steps_per_mmU = 1.0 * U_STEP_PER_TOUR * U_MICROPAS / ((float) U_MM_PER_TOUR) ; 
    
  mm_per_stepX2 = 1.0/ ( steps_per_mmX * steps_per_mmX ) ;   // calculated once to increase performance
  mm_per_stepY2 = 1.0/ ( steps_per_mmY * steps_per_mmY ) ;
  mm_per_stepZ2 = 1.0/ ( steps_per_mmZ * steps_per_mmZ ) ;
  mm_per_stepU2 = 1.0/ ( steps_per_mmU * steps_per_mmU ) ;
    
  mm_per_stepX = 1/ ( steps_per_mmX ) ;   // calculated once to increase performance
  mm_per_stepY = 1/ ( steps_per_mmY ) ;
  mm_per_stepZ = 1/ ( steps_per_mmZ ) ;
  mm_per_stepU = 1/ ( steps_per_mmU ) ;
  
  Serial.println() ;
  Serial.println ("Grbl v1.1f ['$' for help]");  // send some text like GRBL do (version is a dummy one)
  position(0,0,0,0);  // set staring position
  feedrate(200);  // set default speed
  ready();
  
  // init of interrupt when some pin changes
  X_END_MIN_PORT |= (1 << X_END_MIN_BIT ) ; // set input pin with pull up
  Y_END_MIN_PORT |= (1 << Y_END_MIN_BIT ) ; // set input pin with pull up
  Z_END_MIN_PORT |= (1 << Z_END_MIN_BIT ) ; // set input pin with pull up
  U_END_MIN_PORT |= (1 << U_END_MIN_BIT ) ; // set input pin with pull up 
  
  // configure input pins for interrupt
  EICRA = (1<<ISC30) | (1<<ISC20) ; //   ISC21 ISC20 or ISC31 ISC30 = 0 1 means an interrupt on all edges
  EICRB = (1<<ISC50) | (1<<ISC40) ;  //   ISC51 ISC50 or ISC41 ISC40 = 0 1 means an interrupt on all edges
  EIMSK = (1<<INT5) | (1<<INT4) | (1<<INT3) | (1<<INT2) ; // Enable interrupt on all edges of INT 2, 3, 4, 5
  check_limits() ;                                        // read the level on pins used for limit sensors and set the mask accordingly to block step signal
  prev_X_end_outbit = X_end_outbit ;
}

void blink_led() {
  
  static uint32_t prev_millis ;
  if (millis() > ( prev_millis + 500) ) {
    digitalWrite( 13, digitalRead(13)   ? LOW : HIGH );
    prev_millis = millis() ; 
  }
}

/**
 * After setup() this machine will repeat loop() forever.
 */
void loop() {
  // listen for serial commands
  //static uint8_t countCar;
  //static uint8_t prevCar;
  while(Serial.available() > 0) {  // if something is available
    char c=Serial.read();  // get it
    //countCar++ ;
    //Serial.println(countCar) ;
    //Serial.print(c);  // repeat it back so I know you got the message
    if ( c == '?' ) {       // reply immediately to a ? with the current position of the stepper motors. format is <Idle,MPos:5.529,0.560,7.000,WPos:1.529,-5.440,-0.000>
      Serial.print("<Idle|MPos:");
      Serial.print( pX * mm_per_stepX , 3 );  Serial.print( ',' );   
      Serial.print( pY * mm_per_stepX , 3 );  Serial.print( ',' );
      Serial.print( pZ * mm_per_stepX , 3 );  Serial.print( ',' );
      Serial.print( pU * mm_per_stepX , 3 );  Serial.print( "|FS:0,0" );
      Serial.println(">");     // 
    } else if( c == 0x18 ) {  // send reset sequence 
      Serial.println() ;
      Serial.println ("Grbl v1.1f ['$' for help]");  // send some text like GRBL do (version is a dummy one)
      //Serial.println ("ok>") ;
    } else if( c != ' ' ) {  // skip spaces 
      if(sofar<MAX_BUF-1) buffer[sofar++]=c;  // store it
    }
    if(c=='\r') {
      // entire line received
      buffer[sofar]=0;  // end the buffer so string functions work right      
      //Serial.print(F("\r\n"));  // echo a return character for humans
      //Serial.println("Buffer=");
      //for(int i= 0 ; i<sofar ; i++) {
      //  Serial.print( buffer[i] );
      //}
      //Serial.println(" ") ;
      processCommand();  // do something with the command
      Serial.println("ok") ; 
      ready();
    }
  }
//  blink_led() ;
//  static uint32_t prev_millis ;
//  if ( millis() > ( prev_millis + 500) ) {
//    Serial.print("End X ") ;  Serial.println( X_END_MIN_PIN & (1<< X_END_MIN_BIT ) ) ;
//    prev_millis = millis() ;
//  }

//  if ( prev_X_end_outbit != X_end_outbit) {;
//    Serial.print("End X change ") ;  Serial.println( X_end_outbit ) ;
//    prev_X_end_outbit = X_end_outbit ;
//  }

}


/**
* This file is part of GcodeCNCDemo.
*
* GcodeCNCDemo is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* GcodeCNCDemo is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Foobar. If not, see <http://www.gnu.org/licenses/>.
*/

