#include "msp.h"


/**
 * main.c
 */

// Making a buffer of 100 characters for serial to store to incoming serial data
#define BUFFER_SIZE 100
char INPUT_BUFFER[BUFFER_SIZE];
// initializing the starting position of used buffer and read buffer
uint8_t storage_location = 0; // used in the interrupt to store new data
uint8_t read_location = 0; // used in the main application to read valid data that hasn't been read yet
void setupSerial(); // Sets up serial for use and enables interrupts
void writeOutput(char *string); // write output charactrs to the serial port
void readInput(char* string); // read input characters from INPUT_BUFFER that are valid

void LCD_pin_init(void);
void butt_init(void);
int button_pressed(void);
int button_pressed2(void);
void Systick_us_delay(uint32_t microsecond);
void PortADC_init(void);
void ADC14_init(void);
void LCD_pin_init(void);
void write_command(uint8_t command);
void dataWrite(uint8_t data);
void LCD_init(void);
void Systick_ms_delay(uint16_t delay);
void SysTick_Init(void);
void Byte(uint8_t byte);
void Nibble(uint8_t nibble);
void main(void)
{

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
    char string[BUFFER_SIZE]; // Creates local char array to store incoming serial commands
    setupSerial();
    INPUT_BUFFER[0]= '\0';  // Sets the global buffer to initial condition of empty

    void LCD_pin_init();
    butt_init();
    SysTick_Init();
    PortADC_init();
    ADC14_init();
    void LCD_init();
    float volt =0;
    float cel = 0;
    float fare = 0;
    static volatile uint16_t result;

    NVIC->ISER[0] = 1 << ((ADC14_IRQn) & 31);   //enables ADC interrupt in  NVIC
while (1){
    ADC14->CTL0 |= ADC14_CTL0_SC;    //starts conversion
    while(!(ADC14->IFGR0));         //waits for it to complete
    result = ADC14->MEM[0];         //get value from the ADC
    volt = (result*3.3)/16384;      //calculate volts
    cel =  ((volt * 1000.0) - 500) / 10;
    fare = (cel * 9/5) + 32;
}
}
/********************************************************
 * Michael James     Bryanna Flowers
 * Written by whoever wrote the code in the Lab6 document
 * initializes,clears and homes cursor on LCD
 * Inputs: None
 * Outputs:None
 ***************************************************** */
void LCD_init()
{
    write_command(3);   //reset sequence
    Systick_ms_delay(10);
    write_command(3);
    Systick_us_delay(10000);
    write_command(3);
    Systick_ms_delay(10);

    write_command(2);       //home cursor
    Systick_us_delay(10000);
    write_command(2);
    Systick_us_delay(10000);

    write_command(8);   //black display
    Systick_us_delay(10000);
    write_command(0x0F);
    Systick_us_delay(10000);
    write_command(1);   //clear screen
    Systick_us_delay(10000);
    write_command(6);   //increment cursor
    Systick_ms_delay(10);
}
/********************************************************
 * Michael James     Bryanna Flowers
 * Written by MJ and BF
 * * sets rs to 0 and sends command to Byte
 * Inputs: NA
 * Outputs: rs
 ***************************************************** */
void write_command(uint8_t command)
{
    P2->OUT &= ~(BIT3); //set rs to 0
    Byte(command);  //send command to Byte
}
/********************************************************
 * Michael James     Bryanna Flowers
 * Written by MJ and BF
 * sets rs to 1 and sends data to Byte
 * Inputs: NA
 * Outputs: RS on LCD
 ***************************************************** */
void dataWrite(uint8_t data)
{
    P2->OUT |= (BIT3);  //set rs to 1

    Byte(data); //send data to Byte
}
/********************************************************
 * Michael James     Bryanna Flowers
 * Written by MJ and BF
 * pulse enables for LCD
 * Inputs: NA
 * Outputs: E on LCD
 ***************************************************** */
void PulseEnablePin(void)
{
    P3->OUT &=~(BIT0);  //turns enable low
    Systick_us_delay(10000);
    P3->OUT |= (BIT0);  //turns enable high
    Systick_us_delay(10000);
    P3->OUT &=~(BIT0);  //turns enable low
    Systick_us_delay(10000);
}
/********************************************************
 * Michael James     Bryanna Flowers
 * Written by MJ and BF
 * shifts bits for printing on LCD
 * Inputs: NA
 * Ouputs: NA
 ***************************************************** */
void Byte(uint8_t byte)
{
    uint8_t nibble;
    nibble = (byte & 0xF0) >> 4;    //shifts left most bits to right and set it to nibble
    Nibble(nibble);                 //calls nibble
    nibble = byte & 0x0F;           //sets nibble to right most bits
    Nibble(nibble);                 //calls nibble
    Systick_us_delay(10000);
}
/********************************************************
 * Michael James     Bryanna Flowers
 * Written by MJ and BF
 * shifts and sends out bits for printing on LCD
 * Inputs: NA
 * Outputs: DB input pins to LCD
 ***************************************************** */
void Nibble(uint8_t nibble)
{
    P2->OUT &= ~(BIT4|BIT5|BIT6|BIT7); //clears P2 needed
    P2->OUT |= (nibble & (0x0F)) << 4; //port pins wired to D4-D7
    PulseEnablePin();
}
/********************************************************
 * Michael James     Bryanna Flowers
 * Written by MJ and BF
 * Initializes pins for LCD and motor LEDs
 * Inputs: N/A
 * Outputs: E, DB, and rs on LCD, and LEDs for motor
 ***************************************************** */
void LCD_pin_init(void)
{
    P2->SEL0 &=  ~(BIT4|BIT5|BIT6|BIT7|BIT3); // Port 2, 4(DB pins) and rs
    P2->SEL1 &=  ~(BIT4|BIT5|BIT6|BIT7|BIT3); // Port 2, 4 (DB pins) and rs
    P2->DIR |= (BIT4|BIT5|BIT6|BIT7|BIT3); //Set all pins to outputs
    P3->SEL0 &=  ~(BIT0); // E on LCD
    P3->SEL1 &=  ~(BIT0); // E on LCD
    P3->DIR |= (BIT0);  //set to output

    P1->SEL0 &= ~(BIT6|BIT7);   //motor LEDs initialization
    P1->SEL0 &= ~(BIT6|BIT7);
    P1->DIR |= (BIT6|BIT7); //set to output
    P1->OUT &= ~BIT7;
    P1->OUT |= BIT6;

}
/********************************************************
 * Michaael James     Bryanna Flowers
 * Written by MJ and BF
 * initialize systick for microseconds
 * Inputs: NA
 * Outputs: NA
 ***************************************************** */
void Systick_us_delay(uint32_t microsecond)
{
    SysTick->LOAD = (microsecond*3 - 1);    //delay times 3 the value
    SysTick->VAL = 0;   //write to CVR clears it
    while ((SysTick -> CTRL & 0x00010000) == 0);
}
/********************************************************
 * Michael James     Bryanna Flowers
 * Written by MJ and BF
 * systick initialize
 * Inputs: NA
 * Outputs: NA
 ***************************************************** */
void SysTick_Init(void)
{
    SysTick -> CTRL = 0;
    SysTick -> LOAD = 0x00FFFFFF;
    SysTick -> VAL = 0;
    SysTick -> CTRL = 0x00000005;
}
/********************************************************
 * Michael James     Bryanna Flowers
 * Written by MJ and BF
 * systick millisecond setup
 * Inputs: NA
 * Outputs: NA
 ***************************************************** */
void Systick_ms_delay(uint16_t delay)
{
    SysTick -> LOAD = ((delay*3000) - 1);
    SysTick -> VAL = 0;
    while((SysTick -> CTRL & 0x00010000) == 0);
}

void PortADC_init(void)
{
    P5->SEL0 |= BIT5;   //sets pin 5.5 as A0 input
    P5->SEL1 |= BIT5;
}
void ADC14_init(void)
{
    ADC14->CTL0 &= ~ADC14_CTL0_ENC;    //turns off ADC converter while initializing
    ADC14->CTL0 |= 0x04200210;         //16 sample clocks, SMCLK, S/H pulse
    ADC14->CTL1 =  0x00000030;         //14 bit resolution
    ADC14->CTL1 |= 0x00000000;         //convert for mem0 register
    ADC14->MCTL[0]=0x00000000;         //mem[0] to ADC14INCHx = 0
    ADC14->CTL0 |= ADC14_CTL0_ENC;     //enables ADC14ENC and starts ADC after configuration
}

/********************************************************
 * Michael James     Bryanna Flowers
 * Written by MJ and BF
 * initializing buttons for interrupts
 * Inputs: buttons for emergency stop and light switch
 * Outputs: N/A
 ***************************************************** */
void butt_init(void)
{
    P6->DIR &= ~(BIT0); //input
    P6->REN = (BIT0);
    P6->OUT = (BIT0);
    P6->IE = (BIT0);    //enable interrupt
    P6->IES |= (BIT0);
    P6->IFG = 0;    //clear flag

    P5->DIR &= ~(BIT7); //input
    P5->REN = (BIT7);
    P5->OUT = (BIT7);
    P5->IE = (BIT7);    //enable interrupt
    P5->IES |= (BIT7);
    P5->IFG = 0;    //clear flag
}
/********************************************************
 * Michael James     Bryanna Flowers
 * Written by MJ and BF
 * debounce for light switch
 * Inputs: button for lights
 * Outputs: N/A
 ***************************************************** */
int button_pressed(void)
{
    int buttonDebounced = 0;
    if (!(P5->IN & BIT7))   //if button pressed
    {
        __delay_cycles(1500); // waits for 5 milliseconds
        while (!(P5->IN & BIT7)){}  //waits for button to be released
        buttonDebounced =1;
    }
    return buttonDebounced;
}
/********************************************************
 * Michael James     Bryanna Flowers
 * Written by MJ and BF
 * debounce for emergency stop
 * Inputs: button for emergency stop
 * Outputs: N/A
 ***************************************************** */
int button_pressed2(void)
{
    int buttonDebounced = 0;
    if (!(P6->IN & BIT0))   //if button pressed
    {
        __delay_cycles(1500); // waits for 5 milliseconds
        while (!(P6->IN & BIT0)){}  //wait for button to be released
        buttonDebounced =1;
    }
    return buttonDebounced;
}
void EUSCIA0_IRQHandler(void)
{
    if (EUSCI_A0->IFG & BIT0)  // Interrupt on the receive line
    {
        INPUT_BUFFER[storage_location] = EUSCI_A0->RXBUF; // store the new piece of data at the present location in the buffer
        EUSCI_A0->IFG &= ~BIT0; // Clear the interrupt flag right away in case new data is ready
        storage_location++; // update to the next position in the buffer
        if(storage_location == BUFFER_SIZE) // if the end of the buffer was reached, loop back to the start
            storage_location = 0;
    }
}
void writeOutput(char *string)
{
    int i = 0;  // Location in the char array "string" that is being written to
    while(string[i] != '\0') {
        EUSCI_A0->TXBUF = string[i];
        i++;
        while(!(EUSCI_A0->IFG & BIT1));
    }
}
void readInput(char *string)
{
    int i = 0;  // Location in the char array "string" that is being written to
    // One of the few do/while loops I've written, but need to read a character before checking to see if a \n has been read
    do
    {
        // If a new line hasn't been found yet, but we are caught up to what has been received, wait here for new data
        while(read_location == storage_location && INPUT_BUFFER[read_location] != 13);
        string[i] = INPUT_BUFFER[read_location];  // Manual copy of valid character into "string"
        INPUT_BUFFER[read_location] = '\0';
        i++; // Increment the location in "string" for next piece of data
        read_location++; // Increment location in INPUT_BUFFER that has been read
        if(read_location == BUFFER_SIZE)  // If the end of INPUT_BUFFER has been reached, loop back to 0
            read_location = 0;
    }
    while(string[i-1] != 13); // If a \n was just read, break out of the while loop
    string[i-1] = '\0'; // Replace the \n with a \0 to end the string when returning this function
}

void setupSerial()
{
    // Baud Rate Configuration
    // 3000000/(16*9600) = 19.531  (3 MHz at 9600 bps is fast enough to turn on over sampling (UCOS = /16))
    // UCOS16 = 1 (0ver sampling, /16 turned on)
    // UCBR  = 19 (Whole portion of the divide)
    // UCBRF = .531 * 16 = 9 (0x09) (Remainder of the divide)
    // UCBRS = 3000000/9600 remainder=0.5 -> 0xAA (look up table 22-4)
    P1->SEL0 |=  (BIT2 | BIT3); // P1.2 and P1.3 are EUSCI_A0 RX
    P1->SEL1 &= ~(BIT2 | BIT3); // and TX respectively.
    EUSCI_A0->CTLW0  = BIT0; // Disables EUSCI. Default configuration is 8N1
    EUSCI_A0->CTLW0 |= BIT7; // Connects to SMCLK BIT[7:6] = 10
    //EUSCI_A0->CTLW0 &= ~(BIT(15)|BIT(14)|BIT(11));  //BIT15 = Parity, BIT14 = Even, BIT11 = One Stop Bit
    EUSCI_A0->BRW = 19;  // UCBR Value from above
    EUSCI_A0->MCTLW = 0xAA81; //UCBRS (Bits 15-8) & UCBRF (Bits 7-4) & UCOS16 (Bit 0)
    EUSCI_A0->CTLW0 &= ~BIT0;  // Enable EUSCI
    EUSCI_A0->IFG &= ~BIT0;    // Clear interrupt
    EUSCI_A0->IE |= BIT0;      // Enable interrupt
    NVIC_EnableIRQ(EUSCIA0_IRQn);
}
