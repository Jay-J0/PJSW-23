/*
 * ZUMO.c
 *
 * Created: 25-4-2023 10:57:42
 * : Author Dream Team ZUMO
 */ 
#define F_SCL 100000UL // I2C serial bus standard mode 100kHz
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <math.h>
#define USART_BAUDRATE 9600
#define F_CPU 16000000UL
#define UBRR0_ASYNC_MODE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1) //Async mode, so clock signal is not sent

//I2C define porten en slave adressen
#define I2C_SDA PD1
#define I2C_SDA_PORT PORTD
#define I2C_SCL PD0
#define SlaveMag 0x1D // Slave adres

//Defines voor registers I2C
#define WHO_AM_I 0b0001111
#define CTRL5 0b0100100
#define CTRL6 0b0100101
#define CTRL7 0b0100110
#define OUT_X_L_M 0b0001000
#define OUT_X_H_M 0b0001001
#define OUT_Y_L_M 0b0001010
#define OUT_Y_H_M 0b0001011

// Functions for USART
void initUsart();
void writeChar(char);
void writeString(char st[]);
void writeInt(int i);
void writeInt16_t(int16_t x);
volatile unsigned char waarde_USART;

// Functions for motor control
void motorForward();
void motorBackward();
void motorLeft();
void motorRight();
void motorStop();
void initMotors();
void alertMotorControl();
void buzzerSetup();

// Front proximity sensor
void initProximitySensor();
void initADCFrontSensorInterrupt();
void readADCInterrupt(int LeftorRightE);
void startPulse(int emitterLorR);
volatile bool alert;

//Distance traveled
void initializeEncoderInterrupts();
volatile int16_t countLeft;
volatile int16_t countRight;
void giveCounts();
static volatile bool lastLeftA;
static volatile bool lastLeftB;
static volatile bool lastRightA;
static volatile bool lastRightB;
static volatile bool errorLeft;
static volatile bool errorRight;
volatile bool enableShowDistance;


//I2C
void writeMagReg(uint8_t reg, uint8_t waarde);
int readMagReg(uint8_t Sendbyte);
void i2c_master_init(void);
void i2c_send_byte(uint8_t data);
void i2c_start(void);
uint8_t i2c_read_ack(void);
uint8_t i2c_read_nack(void);
void i2c_stop(void);

//Kompas
int16_t Heading();
void showgraden(void);
volatile bool enableShowGraden;
void performMagnetometerCalibration();
int16_t returnMagDataX();
int16_t returnMagDataY();
static volatile int16_t offset_m_x;
static volatile int16_t offset_m_y;
int16_t Magheading;

int main(void){

  sei();
  
  i2c_master_init();
  
  //Calling the writeMagReg function and specifying the byte values for the registers where the settings should be applied
  writeMagReg(CTRL5, 0b01100100); //M_RES1, M_RES0, M_ODR0
  writeMagReg(CTRL6, 0b00100000); //MFS1, 4 gauss
  writeMagReg(CTRL7, 0b00000000); //Magnetic Sensor mode selection: Continues Conversion mode

  initUsart();
  initMotors();
  
  initProximitySensor();
  initADCFrontSensorInterrupt();

  initializeEncoderInterrupts();

  performMagnetometerCalibration();
  
  int16_t Snelheid;
  DDRC = 0b11111111;
  DDRD = 0b11111111;
  DDRB &= ~(1 << PB3);
  PORTB |= (1 << PB3);
  DDRD &= ~(1 << PD5);
  PORTD |= (1 << PD5);
  Snelheid = 100;
  alert = false;
  
  while(1){
    // assign returned heading value to Magheading
    Magheading = Heading();
    
    readADCInterrupt(0);
    readADCInterrupt(1);

    if(enableShowDistance){
      giveCounts();
    }

    if(enableShowGraden) {
    writeString(" Richting: ");
    writeInt16_t(Magheading);
    writeChar('\n');
    writeChar('\r');
    _delay_ms(500);
    }

   if (alert) {
     alertMotorControl();
     alert = false;
    }
  
    switch(waarde_USART) {
  case 'w':
    // ZUMO moves forward 
    writeString("Moving forward\n");
    writeChar('\n');
    writeChar('\r');
    OCR1A=Snelheid;
    OCR1B=Snelheid;
    motorForward();
    waarde_USART = 0; //stops the writeString() function from repeating
    break;
  case 'a':
   // ZUMO moves left
       OCR1A=Snelheid;
    OCR1B=Snelheid;
    writeString("Moving left\n");
    writeChar('\n');
    writeChar('\r');
    motorLeft();
    waarde_USART = 0; //stops the writeString() function from repeating
    break;
  case 'd':
   // ZUMO moves right
       OCR1A=Snelheid;
    OCR1B=Snelheid;
    writeString("Moving right\n");
    writeChar('\n');
    writeChar('\r');
    motorRight();
    waarde_USART = 0; //stops the writeString() function from repeating
    break;
  case 's':
   // ZUMO moves backwards
    OCR1A=Snelheid;
    OCR1B=Snelheid;
    writeString("Moving backwards\n");
    writeChar('\n');
    writeChar('\r');
    motorBackward(); 
    waarde_USART = 0; //stops the writeString() function from repeating
    break;
  case ' ':
   // ZUMO stops moving 
    writeString("Stop Moving\n");
    writeChar('\n');
    writeChar('\r');
    motorStop();
    waarde_USART = 0; //stops the writeString() function from repeating
    break;
  case 'z':
   //ZUMO slower
   Snelheid = Snelheid * 0.85;
   OCR1A=Snelheid;
   OCR1B=Snelheid;
   waarde_USART = 0;
   
   if(Snelheid < 100){
    Snelheid = 100;
    }
    writeString("Moving slower: ");
    writeInt(Snelheid);
    writeChar('\n');
    writeChar('\r');
    break;
    
  case 'x':
  // ZUMO faster
            Snelheid = Snelheid * 1.15;
            OCR1A=Snelheid;
            OCR1B=Snelheid;
            waarde_USART = 0;

            if(Snelheid > 512){
              Snelheid = 512;
              }
            writeString("Moving faster: ");
            writeInt(Snelheid);
            writeChar('\n');
            writeChar('\r');
        break;
  case 'q':
   //display distance travelled
   enableShowDistance = true;
   break;
   
  case 'e':
   //display distance travelled
   enableShowDistance = false;
   break;

  case 'k':
   //display graden
   enableShowGraden = true;
   break;

  case 'l':
   //display graden
   enableShowGraden = false;
  default:
    // handle unexpected input here
    break;

    
}
    }
    return 0;
}

// Interrupt service routine for receiving data from USART
ISR(USART1_RX_vect) {
  waarde_USART = UDR1;
  }

// Interrupt service routine for proximity sensor;
ISR(ADC_vect) {
   int adcValue = ADC; //assign ADC value to int adcValue
   
    if(adcValue<23) { //If the value is lower than 23, object detected.
      alert = true;
      writeString("Alert!\n");
      writeChar('\n');
      writeChar('\r');
    } 
  }

ISR(INT6_vect) { //Right encoder
   bool newRightB = PINF & (1 << PINF0); // Read the value on pin F0
   bool newRightA = (PINE & (1 << PINE6)) ^ newRightB; // Read the value on pin E6, XOR gives the original value of encoder A

   countRight += ((newRightA ^ lastRightB) - (lastRightA ^ newRightB));
// Comparing the new and last measured values using XOR determines the direction of the wheel.
// If the wheel is moving forward, the right counter is incremented. If the wheel is moving backward, the counter is decremented.

   lastRightA = newRightA; // Store the last measured value of encoder A in lastRightA for the next calculation
   lastRightB = newRightB; // Store the last measured value of encoder B in lastRightB for the next calculation
}

ISR(PCINT0_vect) { //Left encoder
   bool newLeftB = PINE & (1 << PINE2); // Read the value on pin E2
   bool newLeftA = (PINB & (1 << PINB4)) ^ newLeftB; // Read the value on pin B4, XOR gives the original value of encoder A

   countLeft += ((newLeftA ^ lastLeftB) - (lastLeftA ^ newLeftB));
// Comparing the new and last measured values using XOR determines the direction of the wheel.
// If the wheel is moving forward, the left counter is incremented. If the wheel is moving backward, the counter is decremented.

   lastLeftA = newLeftA; // Store the last measured value of encoder A in lastLeftA for the next calculation
   lastLeftB = newLeftB; // Store the last measured value of encoder B in lastLeftB for the next calculation 
}

// Function to initialize USART
void initUsart() {
  UBRR1H = (UBRR0_ASYNC_MODE >> 8); 
  UBRR1L = UBRR0_ASYNC_MODE;
  UCSR1B = (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1); //RXENn: Receiver Enable n, TXENn: Transmitter Enable n, RXCIEn: RX Complete Interrupt Enable n
  UCSR1C = (1 << UCSZ10) | (1 << UCSZ11); //8-bit character size, Asynchronous USART, No Parity, stop bit(s): 1-bit
}

// Function to write a single character to USART
void writeChar(char x) {
  while (~UCSR1A & (1 << UDRE1));
  UDR1 = x;
}

// Function to write a string to USART
void writeString(char st[]) {
  for (uint8_t i = 0; st[i] != 0; i++) {
    writeChar(st[i]);
  }
}

// Function to write an integer to USART
void writeInt(int i) {
  char buffer[8];
  itoa(i, buffer, 10);
  writeString(buffer);
}

// Function to write an int16_t to USART
void writeInt16_t(int16_t x) {
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%d", x);
  writeString(buffer);
}

//funtions motor control 
void motorForward(){
    PORTB = (0 << PORTB1)|(0 << PORTB2);
  }
void motorBackward(){
    PORTB = (1 << PORTB1)|(1 << PORTB2);
  }
void motorLeft(){
    PORTB = (1 << PORTB2);
  }
void motorRight(){
    PORTB = (1 << PORTB1);
  }
void motorStop() {
    PORTB = (0 << PORTB1)|(0 << PORTB2)|(0 << PORTB5) | (0 << PORTB6);
    OCR1A = 0;
    OCR1B = 0;
  }

void initMotors() {
  DDRB |= ((1 << PORTB1) | (1 << PORTB2) | (1 << PORTB5) | (1 << PORTB6)); // Set corresponding pins as output
  TCCR1A = (1 << COM1A1) | (1 << COM1B1);  // Set COM1A1, COM1B1 for non-inverted PWM on OC1A and OC1B
  TCCR1B = (1 << WGM13) | (1 << CS11);  // Set WGM13 for Phase Correct PWM mode with prescaler 8
  TCCR1B &= ~(1 << WGM12);  // Clear WGM12
  ICR1 = 512;  // Set TOP value to 512, max snelheid van motor 
}

//"initProximitySensor()" initialization
void initProximitySensor() {
    // Set up Proximity LED PWM
    DDRC |= (1 << PORTC6); // Set Proximity LED PWM pin as output
    PORTC &= ~(1 << PORTC6); // Set Proximity LED output to low
    
    // Set up ADC clock prescaler
    ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2); // Set ADC clock prescaler to 128
    
    // Enable ADC
    ADCSRA |= (1 << ADEN);
}

//The function initADCFrontSensorInterrupt() initializes the interrupt of the ADC and the front proximity sensor.
void initADCFrontSensorInterrupt() {
      // Front Proximity Sensor port. Set PORTF1 as an input with a pull-up resistor (the pull-up resistor blocks outside electrical interference.)
    DDRF &= ~(1 << PORTF1);
    PORTF |= (1 << PORTF1);
    
    // Enable ADC interrupt
    ADCSRA |= (1 << ADIE);
    
    // Set voltage reference to external and ADC input to ADC1 on PORTF1
    ADMUX = (1 << REFS0) | (1 << MUX0);
    
    // Set ADC to free running mode
    ADCSRB = 0;
}

// This function reads the output of the left or right emitter of the front proximity sensor using ADC interrupt.
void readADCInterrupt(int LeftorRightE) {
    // Start pulsing the selected IR emitter.
    startPulse(LeftorRightE);
       
    // Start ADC conversion.
    ADCSRA |= (1 << ADSC);
}

// This function pulses the selected IR emitter with the chosen brightness.
void startPulse(int emitterLorR) {
    // Set PORTF6 Proximity LED selection as output.
    DDRF |= (1 << PORTF6);
    
    // Determine which IR emitter to use (1 is right, 0 is left).
    if (emitterLorR == 1) { 
        // Set PORTF6 to 1, turning on the right IR LEDs.
        PORTF |= (1 << PORTF6);
    }
    if (emitterLorR == 0) {
        // Set PORTF6 to 0, turning on the left IR LEDs.
        PORTF &= ~(1 << PORTF6);
    }
    
    // Configure Timer 3 for Fast PWM mode with ICR3 as the top and prescaler of 1.
    TCCR3A |= (1 << WGM31) | (1 << COM3A1);
    TCCR3B |= (1 << WGM33) | (1 << WGM32) | (1 << CS30);
    
    ICR3 = 420; // 38 kHz infra red LED, so ICR3 = (F_CPU / (2 * N * F_PWM)) - 1, dus (16,000,000 / (1 * 38,000)) - 1 = 420
    
    OCR3A = 379; //The dutycycle controls the IR LEDs brightness lets take 90% brightness, so OCR3A = (dutycycle/100) * (ICR3 +1), so 0CR3A = (90/100) * (420 + 1)= 379
}

void alertMotorControl(){
  buzzerSetup();
  motorBackward();
  _delay_ms(500);
  motorStop();
}

void initializeEncoderInterrupts() {
  DDRE &= ~(1 << PORTE6); // Set the direction of PORTE6 (Right XOR) pin as input
  DDRF &= ~(1 << PORTF0); // Set the direction of PORTF0 (Right encoder channel B) pin as input
  DDRB &= ~(1 << PORTB4); // Set the direction of PORTB4 (Left XOR) pin as input
  DDRE &= ~(1 << PORTE2); // Set the direction of PORTE2 (Left encoder channel B) pin as input
  
  PORTB |= (1 << PORTB4); // Enable the pull-up resistor on PORTB4 (Left XOR) pin for PCINT4
  PORTF |= (1 << PORTF0); // Enable the pull-up resistor on PORTF0 (Right B) pin
  PORTE |= (1 << PORTE2) | (1 << PORTE6); // Enable the pull-up resistors on PORTE2 (Left B) and PORTE6 (Right XOR) pins

  PCICR |= (1 << PCIE0); // Enable pin change interrupt on PCINT[7:0]
  PCMSK0 |= (1 << PCINT4); // Enable pin change interrupt for PCINT4 (PORTB4)
  EIMSK |= (1 << INT6); // Enable interrupts for INT6
  EICRB |= (1 << ISC60); // Set interrupt on INT6 to trigger on any logical change
  PCIFR |= (1 << PCIF0); // Clear the interrupt flag by writing a 1 to it

  countLeft = 0;
  countRight = 0;
}

void giveCounts(){
  int16_t countsR = countRight;
  int16_t countsL = countLeft;
  int16_t distanceR = (((2 * 2 * 3.14) * countsR) / 1200) * 2; //The encoders provide a resolution of 12 counts per revolution, ZUMO has a RED sticker in battery compartment which means 100:1 gear ratio, the encoders provide 100 × 12 ≈ 1200 CPR.
  int16_t distanceL = (((2 * 2 *  3.14) * countsL) / 1200) * 2; //Distance = (2 * π * Radius * countLeft) * 2 / CPR
  writeString("Distance travelled left wheel: ");
  writeInt(distanceL);
  writeChar('\n');
  writeString("Distance travelled right wheel: ");
  writeInt(distanceR);
  writeChar('\n');
  writeChar('\r');
  _delay_ms(500);
}

// Function for buzzer with delay for moving backwards.
void buzzerSetup(){
      TCCR4B = 1 << CS43 | 1 << CS40;//prescaler- frequentie van de timer aanpassen
      DDRD |= 1 << PORTD7;           // make this pin output
      TCCR4C = ((1 << COM4D1) | (0 << COM4D0) | (1 << PWM4D)); // config timer
      OCR4D = 250; // Duty cycle PWM signal
      _delay_ms(250);
      OCR4D = 0; // Turn off buzzer
      _delay_ms(250);
}

// Function that writes to the magnet slave, specifying which registers to enable and their settings:
void writeMagReg(uint8_t reg, uint8_t waarde){
  i2c_start();
  i2c_send_byte((SlaveMag<<1)+0); //+0 is write, +1 is read
  i2c_send_byte(reg);
  i2c_send_byte(waarde);
  i2c_stop();
}

// Function to read the values from the magnet register
int readMagReg(uint8_t Sendbyte){
  i2c_start();  //i2c starts
  i2c_send_byte((SlaveMag<<1)+0); // Sending data to the magneto (slave)
  i2c_send_byte(Sendbyte);  // Sending data to the magneto
  
  i2c_start(); // Reset I2C
  i2c_send_byte((SlaveMag<<1)+1); // The master is requesting data from the magneto (slave)
  uint16_t MagWaarde = i2c_read_nack(); // Reading the values sent by the magnet
  i2c_stop(); //Stop I2C
  return MagWaarde;
}

// Function for giving MagY data
int16_t returnMagDataX(){
    int16_t MagX;
    //Magneto X low:
    int8_t X_L_M = readMagReg(OUT_X_L_M);
    //Magneto X high:
    int8_t X_H_M = readMagReg(OUT_X_H_M);

     //Combine High and Low
    //16 bit van Mag X
    MagX = (int16_t)(X_H_M << 8 | X_L_M);

    return MagX;
}

// Function for giving MagX data
int16_t returnMagDataY(){
    int16_t MagY;
      //Magneto Y low:
    int8_t Y_L_M = readMagReg(OUT_Y_L_M);
    //Magneto Y high:
    int8_t Y_H_M = readMagReg(OUT_Y_H_M);

    //Combine High and Low
    MagY = (int16_t)(Y_H_M << 8 | Y_L_M);

    return MagY;
}

// Fiunction for calibration
void performMagnetometerCalibration() {
    // Collect calibration samples
    for (uint16_t i = 0; i < 1000; i++) {
      offset_m_x += returnMagDataX();
      offset_m_y += returnMagDataY();
    }

    offset_m_x /= 1000;
    offset_m_y /= 1000;
}

// Function for determining the direction:
int16_t Heading(){
    int16_t MagX = returnMagDataX();
    int16_t MagY = returnMagDataY();

    //Hard-Iron callibration
    MagX -= offset_m_x;
    MagY -= offset_m_y;

//Soft-Iron callibration
    MagX = (MagX * 8.75) / 1000;
    MagY = (MagY * 8.75) / 1000;
    
    int16_t Magheading = atan2(MagY, MagX) * (180.0 / M_PI);

    if (Magheading < 0) {
        Magheading += 360.0;
    }
    return Magheading;
}


// Initialization of I2C
void i2c_master_init(void) {
  TWSR = 0;
  TWBR = (uint8_t)(((F_CPU / F_SCL) - 16 ) / 2); //original formula SCL freq. = CPU freq. / (16+2(TWBR)*4)
  TWCR = (1<<TWEN);
  I2C_SDA_PORT |= ((1 << I2C_SDA) | (1 << I2C_SCL));//Pullup SCL and SDA
}

// Starting the I2C operation
void i2c_start(void){
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // transmit START condition
  while( !(TWCR & (1<<TWINT)) ); // wait for end of transmission
}

// Sending data through I2C, including data and address for I2C slave
void i2c_send_byte(uint8_t data){
  TWDR = (data);
  TWCR = (1<<TWINT) | (1<<TWEN); // start transmission
  while( !(TWCR & (1<<TWINT)) ); // wait for end of transmission
}

// Reading with acknowledgment from the master
uint8_t i2c_read_ack(void) {
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA); // start TWI with ACK after reception
  while( !(TWCR & (1<<TWINT)) ); // wait for end of transmission
  return TWDR; // return received data from TWDR
}

// Reading without acknowledgment from the master
uint8_t i2c_read_nack(void) {
  TWCR = (1<<TWINT) | (1<<TWEN); // start receiving without acknowledging reception
  while( !(TWCR & (1<<TWINT)) ); // wait for end of transmission
  return TWDR; // return received data from TWDR
}

// Stops the I2C operation
void i2c_stop(void) {
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO); // transmit STOP condition
}