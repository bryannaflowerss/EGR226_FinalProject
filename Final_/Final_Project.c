#include "msp.h"
#include "stdio.h"

enum states{
    CLOCK,
    ALARM,
    SETTIME,
    SETALARM,
    SNOOZE,
    TURN,
};




void RTC_Init();
void LCD_pin_init(void);
void write_command(uint8_t command);
void dataWrite(uint8_t data);
void LCD_init(void);
void Systick_ms_delay(uint16_t delay);
void Systick_us_delay(uint32_t microsecond);
void SysTick_Init(void);
void Byte(uint8_t byte);
void Nibble(uint8_t nibble);


void setupSerial(); // Sets up serial for use and enables interrupts
void writeOutput(char *string); // write output charactrs to the serial port
void readInput(char* string); // read input characters from INPUT_BUFFER that are valid
// Making a buffer of 100 characters for serial to store to incoming serial data
#define BUFFER_SIZE 100
char INPUT_BUFFER[BUFFER_SIZE];
// initializing the starting position of used buffer and read buffer
uint8_t storage_location = 0; // used in the interrupt to store new data
uint8_t read_location = 0; // used in the main application to read valid data that hasn't been read yet
int newcom = 0;
int b = 0;
int time_update = 1, alarm_update = 1;
uint8_t hours, mins, secs, Ahours=12, Amins=00, Shours=12, Smins=00, Ssecs=00;

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    char string[BUFFER_SIZE]; // Creates local char array to store incoming serial commands
        setupSerial();
        INPUT_BUFFER[0]= '\0';  // Sets the global buffer to initial condition of empty


    int i;
    int validh = 0;
    int validm = 0;
    int valids =0;
    char time[10];
    char Atime[0];
    SysTick_Init();
    LCD_pin_init();
    LCD_init();
    RTC_Init();
    __enable_interrupt();


    enum states state = CLOCK;       //sets the state immediately to MENU
    while(1){

        if (newcom){
            readInput(string); // Read the input up to \n, store in string.  This function doesn't return until \n is received
            if(string[0] != '\0'){ // if string is not empty, check the inputted data.
                if (string[0] == 'S' && string[1] == 'E' && string[2] == 'T' && string[3] == 'T' && string[4] == 'I' && string[5] == 'M' && string[6] == 'E')
                {
                if      ((string[8] == '2' && string[9] <'4' && string[9] >= '0'))  validh = 1; //20-24
                else if (string[8] == '1' && string[9] <='9' && string[9] >= '0')  validh = 1; //10-19
                else if (string[8] == '0' && string[9] <='9' && string[9] >= '0')  validh = 1; //0-9
                if (string[11] >= '0' && string[11] <'6' && string[12] >= '0' && string[12] <='9')  validm = 1;  //valid minute
                if (string[14] >= '0' && string[14] <'6' && string[15] >= '0' && string[15] <='9')  valids = 1;  //valid minute
                if ((validh + validm + valids) == 3){
                    writeOutput("Time Set");
                    Shours = (string[8]-48)*10 + (string[9]-48);
                    Smins = (string[11]-48)*10 + (string[12]-48);
                    Ssecs  = (string[14]-48)*10 + (string[15]-48);
                    writeOutput(string);    //prints valid to serial
                    writeOutput("\n");
                    validh = 0;
                    validm = 0;
                    valids =0;
                    RTC_Init();
                    newcom = 0;
                }
                else{
                    validh = 0;
                    validm = 0;
                    valids =0;
                    writeOutput("Invalid time");
                    writeOutput(string);    //prints valid to serial
                    writeOutput("\n");
                    newcom = 0;
                }

            }
                else if (string[0] == 'S' && string[1] == 'E' && string[2] == 'T' && string[3] == 'A' && string[4] == 'L' && string[5] == 'A' && string[6] == 'R' && string[7] == 'M')
                {
                    if      ((string[9] == '2' && string[10] <'4' && string[10] >= '0'))  validh = 1; //20-24
                                           else if (string[9] == '1' && string[10] <='9' && string[10] >= '0')  validh = 1; //10-19
                                           else if (string[9] == '0' && string[10] <='9' && string[10] >= '0')  validh = 1; //0-9
                                           if (string[12] >= '0' && string[12] <'6' && string[13] >= '0' && string[12] <='9')  validm = 1;  //valid minute
                                           if ((validh + validm) == 2){
                                               writeOutput("ALARM Set");
                                               Ahours = (string[9]-48)*10 + (string[10]-48);
                                               Amins = (string[12]-48)*10 + (string[13]-48);
                                               writeOutput(string);    //prints valid to serial
                                               writeOutput("\n");
                                                validh = 0;
                                                validm = 0;
                                                RTC_Init();
                                                alarm_update = 1;
                                           }
                                           else{
                                                validh = 0;
                                                validm = 0;
                                               writeOutput("Invalid time, please renter.");
                                               writeOutput(string);    //prints valid to serial
                                               writeOutput("\n");
                                               newcom = 0;
                                           }
                                       }

            else if (string[0] == 'R' && string[1] == 'E' && string[2] == 'A' && string[3] == 'D' && string[4] == 'T' && string[5] == 'I' && string[6] == 'M' && string[7] == 'E')
            {
                if (hours == 0)                         sprintf(time, "%02d:%02d:%02d", hours+12,mins,secs);
                else if (hours>12 && hours<22)          sprintf(time, " %01d:%02d:%02d", hours-12,mins,secs);
                else if (hours==12)                     sprintf(time, "%02d:%02d:%02d", hours,mins,secs);
                else if(hours>=22)                      sprintf(time, "%02d:%02d:%02d", hours-12,mins,secs);
                else if (hours < 10)                    sprintf(time, " %01d:%02d:%02d", hours,mins,secs);
                else                                    sprintf(time, "%02d:%02d:%02d", hours,mins,secs);

                writeOutput(time);    //prints valid to serial
                writeOutput("\n");
            }
            else if (string[0] == 'R' && string[1] == 'E' && string[2] == 'A' && string[3] == 'D' && string[4] == 'A' && string[5] == 'L' && string[6] == 'A' && string[7] == 'R' && string[8] == 'M')
            {
                if (Ahours == 0)                        sprintf(Atime, "%02d:%02d", Ahours+12,Amins);
                else if (Ahours>12 && Ahours<22)        sprintf(Atime, " %01d:%02d", Ahours-12,Amins);
                else if (Ahours==12)                    sprintf(Atime, "%02d:%02d", Ahours,Amins);
                else if(Ahours>=22)                     sprintf(Atime, "%02d:%02d", Ahours-12,Amins);
                else if (Ahours < 10)                   sprintf(Atime, " %01d:%02d", Ahours,Amins);
                else                                    sprintf(Atime, "%02d:%02d", Ahours,Amins);
                writeOutput(Atime);    //prints valid to serial
                writeOutput("\n");
            }
            newcom = 0;
            }
        }

        switch (state){
        case CLOCK:
            if(time_update){
                time_update = 0;
                printf("%02d:%02d:%02d\n",hours,mins,secs);
                if (hours == 0)                         sprintf(time, "%02d:%02d:%02d AM", hours+12,mins,secs);
                else if (hours>12 && hours<22)          sprintf(time, " %01d:%02d:%02d PM", hours-12,mins,secs);
                else if (hours==12)                     sprintf(time, "%02d:%02d:%02d PM", hours,mins,secs);
                else if(hours>=22)                      sprintf(time, "%02d:%02d:%02d PM", hours-12,mins,secs);
                else if (hours < 10)                    sprintf(time, " %01d:%02d:%02d AM", hours,mins,secs);
                else                                    sprintf(time, "%02d:%02d:%02d AM", hours,mins,secs);
                write_command(0x82);
                for (i=0; i<11; i++)
                {
                    dataWrite(time[i]);    //prints time to LCD
                    Systick_us_delay(10);
                }
            }

            if(alarm_update){
                printf("ALARM\n");
                if (Ahours == 0)                        sprintf(Atime, "%02d:%02d AM", Ahours+12,Amins);
                else if (Ahours>12 && Ahours<22)        sprintf(Atime, " %01d:%02d PM", Ahours-12,Amins);
                else if (Ahours==12)                    sprintf(Atime, "%02d:%02d PM", Ahours,Amins);
                else if(Ahours>=22)                     sprintf(Atime, "%02d:%02d PM", Ahours-12,Amins);
                else if (Ahours < 10)                   sprintf(Atime, " %01d:%02d AM", Ahours,Amins);
                else                                    sprintf(Atime, "%02d:%02d AM", Ahours,Amins);
                write_command(0x93);
                for (i=0; i<8; i++)
                {
                    dataWrite(Atime[i]);    //prints time to LCD
                    Systick_us_delay(10);
                }

                alarm_update = 0;
            }
            //            if ( Ahours==hours && (Amins-mins) < 5) a = 1;
            //            else if ((Ahours > hours) && ((Ahours - hours) < 2) && (Amins <= 4) && (mins > 55) && ((mins - Amins) == 55)) a = 1;
            //            else if ((Ahours == 0) && (hours == 24) && (Amins <= 4) && (mins > 55) && ((mins - Amins) == 55)) a = 1;
            //            if (a)
            //            {
            //                b = 0;
            //                a = 0;
            //            }



        break;

        case SETTIME:

           state = SETALARM;

           break;

        case SETALARM:

                   break;

}
    }
}

void RTC_Init(){
    //Initialize time to 2:45:55 pm
    //    RTC_C->TIM0 = 0x2D00;  //45 min, 0 secs
    RTC_C->CTL0 = (0xA500);
    RTC_C->CTL13 = 0;

    RTC_C->TIM0 = Smins<<8 | Ssecs;//45 min, 55 secs
    RTC_C->TIM1 = 1<<8 | Shours;  //Monday, 2 pm
    RTC_C->YEAR = 2018;
    //Alarm at 2:46 pm
    RTC_C->AMINHR = Ahours<<8 | Amins | BIT(15) | BIT(7);  //bit 15 and 7 are Alarm Enable bits
    RTC_C->ADOWDAY = 0;
    RTC_C->PS1CTL = 0b11010;  //1/64 second interrupt

    RTC_C->CTL0 = (0xA500) | BIT5; //turn on interrupt
    RTC_C->CTL13 = 0;
    //TODO
    NVIC_EnableIRQ(RTC_C_IRQn);
}

void RTC_C_IRQHandler()
{
    if(RTC_C->PS1CTL & BIT0){
        hours = RTC_C->TIM1 & 0x00FF;
        mins = (RTC_C->TIM0 & 0xFF00) >> 8;
        secs = RTC_C->TIM0 & 0x00FF;

        static int light = 0;
        light++;
        if (light%3 == 0)
            b = 1;
        time_update = 1;
        RTC_C->PS1CTL &= ~BIT0;
    }
    if(RTC_C->CTL0 & BIT1)
    {
        alarm_update = 1;
        RTC_C->CTL0 = (0xA500) | BIT5;
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
    write_command(0x0C);   //hide cursor
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
    Systick_us_delay(1000);
    P3->OUT |= (BIT0);  //turns enable high
    Systick_us_delay(1000);
    P3->OUT &=~(BIT0);  //turns enable low
    Systick_us_delay(1000);
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
    Systick_us_delay(1000);
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
void EUSCIA0_IRQHandler(void)
{
    if (EUSCI_A0->IFG & BIT0)  // Interrupt on the receive line
    {
        INPUT_BUFFER[storage_location] = EUSCI_A0->RXBUF;
        if (EUSCI_A0->RXBUF == 13)// store the new piece of data at the present location in the buffer
        {
            newcom = 1;
        }
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

