#undef F_CPU
#define F_CPU 16000000UL // 16MHz clock frequency
#define PRESCALER 8UL
#define F_S 100UL  // 1Hz sampling frequency

volatile uint8_t countFlag = 0;
int counts = F_CPU/(PRESCALER*F_S) - 1; //
volatile bool Mode = 0;
unsigned int i;

void setup() {
  //Used for verification/debugging
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);


  /*
     Timers and interrupts
  */
  // Set a 1Hz Timer for clocking controller
  TCCR5A = 0;
  TCCR5B = (1 << WGM52) | (0 << CS52) | (1 << CS51) | (0 << CS50);
  OCR5A = F_CPU / (PRESCALER * F_S) - 1; //Set the number of counts to compare the counter with
  TIMSK5 |= (1 << OCIE5A);
  TCNT5 = 0;
}

void loop() {

  if (countFlag) {
    countFlag = 0;
    i = TCNT5;

    // Prints out counts and number of counts since last interrupt for debugging purposes
    Serial.print(counts);
    Serial.print(" ");
    Serial.println(i);
    digitalWrite(LED_BUILTIN, Mode);
  }
}

/* Interrupt Service Routine called by the counter
 * Be careful with what code you place in this routine! If this routine takes too long to execute it may cause some complicated errors.
 */
ISR(TIMER5_COMPA_vect) {  
  countFlag = 1;
  Mode = !Mode;
}