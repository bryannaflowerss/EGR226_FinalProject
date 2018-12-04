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
void GeneralRTC_Init();
void ButtonRTC_Init();
void SetupTimerAlarm(void);
void PortADC_init(void);
void ADC14_init(void);

void LCD_pin_init(void);
void write_command(uint8_t command);
void dataWrite(uint8_t data);
void LCD_init(void);
void Systick_ms_delay(uint16_t delay);
void Systick_us_delay(uint32_t microsecond);
void SysTick_Init(void);
void Byte(uint8_t byte);
void Nibble(uint8_t nibble);
void brighter();
void initializePWMports();

void settime();
void setalarm();

//Button initialization
void butt_init(void);
int TIMEbutton_pressed();
int ALARMbutton_pressed();
int ONOFFbutton_pressed();
int SNOOZEbutton_pressed();
int speed = 0;
void P1_Init();

void setupSerial(); // Sets up serial for use and enables interrupts
void writeOutput(char *string); // write output charactrs to the serial port
void readInput(char* string); // read input characters from INPUT_BUFFER that are valid
// Making a buffer of 100 characters for serial to store to incoming serial data
#define BUFFER_SIZE 100
char INPUT_BUFFER[BUFFER_SIZE];
// initializing the starting position of used buffer and read buffer
uint8_t storage_location = 0; // used in the interrupt to store new data
uint8_t read_location = 0; // used in the main application to read valid data that hasn't been read yet
int OOU = 0; //global flag for on off button
int SD = 0; //global flag for snooze down button
int newcom = 0;
int al = 0;
int b = 0;
int c = 0;
int time_update = 1, alarm_update = 1;
int hours = 0, mins = 00, secs = 00;
int Ahours=12, Amins=6, Shours=12, Smins=00, Ssecs=00;
float brightness = 0;
int timebutton = 0; //flag for time button interrupt
int alarmbutton =0; //flag for alarm button interrupt
int setT=0;      //flag for adjusting the hours or mins for time
int setA=0;

void main(void){
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    char string[BUFFER_SIZE]; // Creates local char array to store incoming serial commands
    setupSerial();
    INPUT_BUFFER[0]= '\0';  // Sets the global buffer to initial condition of empty

    int status = 0; //flag sets alarm status to off
    int i;
    int sound = 0;
    int car = 0;
    int validh = 0;
    int validm = 0;
    int valids = 0;
    char time[10];
    char Atime[0];
    char temp[50];
    float volt =0;
    float cel = 0;
    float fare = 0;
    static volatile uint16_t result;

    SetupTimerAlarm();

    initializePWMports();       //lights
    SysTick_Init();             //delays
    LCD_pin_init();             //initializing the LCD
    LCD_init();                 //beginning the cursor at the top left corner
    P1_Init();
    RTC_Init();                 //initializing the real time clock
    butt_init();                //initializing each button used
    TIMEbutton_pressed();   //P6.0
    ALARMbutton_pressed();  //P5.7
    ONOFFbutton_pressed(); //P3.2
    SNOOZEbutton_pressed();  //P3.3
    PortADC_init();
    ADC14_init();

    NVIC->ISER[0] = 1 << ((ADC14_IRQn) & 31);   //enables ADC interrupt in  NVIC

    NVIC_EnableIRQ(PORT6_IRQn); //allowing the interrupt for the setT time button
    NVIC_EnableIRQ(PORT5_IRQn); //allowing the interrupt for the set alarm button
    NVIC_EnableIRQ(PORT3_IRQn); //allowing the interrupts for ON/OFF/Up and Snooze/Down buttons
    //Enables interrupts for each port that the buttons are on
    __enable_interrupt();

    enum states state = CLOCK;       //sets the state immediately to MENU
    while(1){
        ADC14->CTL0 |= ADC14_CTL0_SC;    //starts conversion
        while(!(ADC14->IFGR0));         //waits for it to complete
        result = ADC14->MEM[0];         //get value from the ADC
        volt = (result*3.3)/16384;      //calculate volts
        cel =  ((volt * 1000.0) - 500) / 10;
        fare = (cel * 9/5) + 32;
        write_command(0xD5);
        sprintf(temp, "%f", fare);
        Systick_us_delay(10);
        for (i=0; i<4; i++)
        {
            dataWrite(temp[i]);
            Systick_us_delay(10);
        }
        dataWrite(0xDF);
        Systick_us_delay(10);
        dataWrite(0x46);
        Systick_us_delay(10);
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
                int Ah = Ahours;
                int h = hours;
                int Am = Amins;
                int m = mins;

                if ( (Ah == h) && ( (Am - m) <= 5) && (Am - m) >= 0 && (status == 1 || status == 2))    car = 1;

                else if ((Ah > h) && ((Ah - h) < 2) && (Am <= 4) && (m > 55) && ((m - Am) == 55) && (status == 1 || status == 2))   car = 1;


                else if ((Ah == 0) && (h == 23) && (Am <= 4) && (m >= 55) && ((m - Am) == 55) && (status == 1 || status == 2))  car=1;

                else car = 0;

                if (b == 1 && car == 1)
                {
                    brightness += 100;
                    brighter();
                    b = 0;
                }
                else if (c == 1  && car == 1)
                {
                    brightness += 600;
                    brighter();
                    c = 0;
                }
                else if (status == 0)
                {
                    brightness = 0;
                    brighter();
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
                write_command(0x94);
                for (i=0; i<8; i++)
                {
                    dataWrite(Atime[i]);    //prints time to LCD
                    Systick_us_delay(10);
                }
                alarm_update = 0;
            }
            if(status == 1)
            {
                dataWrite(' ');
                Systick_us_delay(10);
                dataWrite(' ');
                Systick_us_delay(10);
                write_command(0xC7)  ;
                dataWrite('O');
                Systick_us_delay(10);
                dataWrite('N');
                Systick_us_delay(10);
                dataWrite(' ');
                Systick_us_delay(10);
                dataWrite(' ');
                Systick_us_delay(10);
            }
            else if(status == 0)
            {
                write_command(0xC5)  ;
                dataWrite(' ');
                Systick_us_delay(10);
                dataWrite(' ');
                Systick_us_delay(10);
                dataWrite('O');
                Systick_us_delay(10);
                dataWrite('F');
                Systick_us_delay(10);
                dataWrite('F');
                Systick_us_delay(10);
                dataWrite(' ');
                Systick_us_delay(10);
            }
            else if(status == 2)
            {
                write_command(0xC5)  ;
                dataWrite('S');
                Systick_us_delay(10);
                dataWrite('N');
                Systick_us_delay(10);
                dataWrite('O');
                Systick_us_delay(10);
                dataWrite('O');
                Systick_us_delay(10);
                dataWrite('Z');
                Systick_us_delay(10);
                dataWrite('E');
                Systick_us_delay(10);
            }
            if(timebutton ==1){     //checking flag for the time button
                timebutton=0;
                if(TIMEbutton_pressed())    state = SETTIME;
            }
            if(alarmbutton == 1){   //checking flag for the alarm button
                alarmbutton = 0;
                if(ALARMbutton_pressed())   state = SETALARM;
            }

            int Ah = Ahours;
            int h = hours;
            int Am = Amins;
            int m = mins;
            int s = secs;
            if (OOU == 1 && status == 0)
            {
                OOU = 0;
                if(ONOFFbutton_pressed())
                {
                    status = 1; //alarm turned on
                    sound = 0;
                }
            }
            else if (OOU == 1 && status == 1)
            {
                OOU = 0;
                if(ONOFFbutton_pressed())
                {
                    status = 0; // alarm turn off
                    sound = 0;
                    brightness = 0;
                    brighter();
                }
            }
            else if (OOU == 1 && status == 2)
            {
                OOU = 0;
                if(ONOFFbutton_pressed())
                {
                    status = 0; // alarm turn off
                    sound = 0;
                    brightness = 0;
                    brighter();
                }
            }
            else if (SD == 1 && sound == 1)
            {
                SD = 0;
                if(SNOOZEbutton_pressed())
                {
                    sound = 0;
                    status = 2;//snooze mode
                    car = 0;
                    alarm_update = 1;
                    brightness = 0;
                    brighter();
                    if (m < 50)
                    {
                        Ahours = hours;
                        Amins = mins + 10;
                        Smins = mins;
                        Shours = hours;
                        Ssecs = secs;
                        RTC_Init();
                    }
                    else if (m > 50 && h < 23)
                    {

                        Amins = mins - 50;
                        Ahours = hours + 1;
                        Smins = mins;
                        Shours = hours;
                        Ssecs = secs;
                        RTC_Init();
                    }
                    else if ( m > 50 && h == 23)
                    {
                        Amins = mins - 50;
                        Ahours = 0;
                        Smins = mins;
                        Shours = hours;
                        Ssecs = secs;
                        RTC_Init();
                    }
                }
            }
            if (h == Ah && Am == m && (s == 0 || speed == 1) && ((status == 1) || (status == 2)))
            {
                sound = 1;
            }
            if (sound == 1 && al == 1)
            {
                TIMER_A1->CCR[2]=1500000/450;
                al = 0;
            }
            else{
                TIMER_A1->CCR[2] = 0;
            }
            break;

        case SETTIME:
            if (setT==0)
                write_command(0x83);
            if (setT==1)
                write_command(0x86);
            settime();
            if(setT==2){
                setT=0;     //clearing so that if the button is pressed again it begins at 0
                RTC_Init();
                write_command(0x0F);
                state = CLOCK;
            }
            break;

        case SETALARM:
            if(setA==0)
                write_command(0x95);
            if(setA==1)
                write_command(0x98);
            setalarm();
            if(setA==2){
                setA = 0;
                write_command(0x0F);
                RTC_Init();
                state = CLOCK;
            }
            break;
        }
    }
}
void setalarm(){
    char butAtime[11];
    //when setA is 0 the hours are being adjusted
    //the set time button is pressed again setA = 1 and the minutes are being adjusted

    int j =0; //flag for writing to LCD

    if(alarmbutton ==1 ){ //flag for time button interrupt
        alarmbutton=0;   //time button flag
        if(ALARMbutton_pressed()){       //debouncing the button press
            setA++;
        }
    }
    if(ONOFFbutton_pressed()){  //if ON/OFF/Up button is pressed we are increasing the hours or minutes
        if(setA == 0){    //increasing the hours
            Ahours++;
            if (Ahours == 0)                         sprintf(butAtime, "%02d:%02d AM", Ahours+12,Amins);       //12 am
            else if (Ahours>12 && Ahours<22)      sprintf(butAtime, " %01d:%02d PM", Ahours-12,Amins);      //b/w 1 PM and 9 PM
            else if (Ahours==12)                     sprintf(butAtime, "%02d:%02d PM", Ahours,Amins);          //noon 12 PM
            else if(Ahours>21 && Ahours<24)     sprintf(butAtime, "%02d:%02d PM", Ahours-12,Amins);       //b/w 10 PM and 11 PM
            else if (Ahours < 10 && Ahours >0)                    sprintf(butAtime, " %01d:%02d AM", Ahours,Amins);         //b/w 12 AM and 9 AM
            else if(Ahours >23){
                Ahours=0;
                sprintf(butAtime, "%02d:%02d AM", Ahours+12,Amins);       //12 am
            }
            else if(Ahours < 0){
                Ahours =23;
                sprintf(butAtime, "%02d:%02d PM", Ahours-12,Amins);       //11 PM
            }
            else                                        sprintf(butAtime, "%02d:%02d AM", Ahours,Amins);
            write_command(0x94);
            write_command(0x0F);
            for (j=0; j<8; j++)
            {
                dataWrite(butAtime[j]);    //prints time to LCD
                Systick_us_delay(10);
            }
        }
        if (setA==1){       //increasing the minutes
            Amins++;
            if (Ahours == 0){     //12 AM
                if (Amins >=0 && Amins <= 59 )     sprintf(butAtime, "%02d:%02d AM", Ahours+12, Amins);    //12:00 to 12:59 AM
                else if (Amins == 60){
                    Amins=0;
                    sprintf(butAtime, "%02d:%02d AM", Ahours+12, Amins);    //12:00 goes back to zero
                }
            }
            else if (Ahours==12){
                if (Amins >0 && Amins <= 59 )     sprintf(butAtime, "%02d:%02d PM", Ahours,Amins);          //noon 12 PM
                else if (Amins == 60){
                    Amins=0;
                    sprintf(butAtime, "%02d:%02d PM", Ahours, Amins);    //12:00 goes back to zerow
                }
            }
            else if (Ahours>12 && Ahours<22){
                if (Amins > 0 && Amins <= 59 )     sprintf(butAtime, " %01d:%02d PM", Ahours-12,Amins);      //1-9PM
                else if (Amins == 60){
                    Amins=0;
                    sprintf(butAtime, " %01d:%02d PM", Ahours-12,Amins);
                }
            }
            else if(Ahours>21 && Ahours<24){
                if (Amins >= 0 && Amins <= 59 )    sprintf(butAtime, "%02d:%02d PM", Ahours-12,Amins);       //b/w 10 PM and 11 PM
                else if (Amins == 60){
                    Amins=0;
                    sprintf(butAtime, "%02d:%02d PM", Ahours-12,Amins);
                }
            }
            else if (Ahours < 10){
                if (Amins >= 0 && Amins <= 59 )    sprintf(butAtime, " %01d:%02d AM", Ahours,Amins);         //b/w 1 AM and 9 AM
                else if (Amins == 60){
                    Amins=0;
                    sprintf(butAtime, " %01d:%02d AM", Ahours,Amins);
                }
            }
            else if(Ahours >23){
                Ahours=0;
                if (Amins >= 0 && Amins <= 59 )     sprintf(butAtime, "%02d:%02d AM", Ahours+12, Amins);    //12:00 to 12:59 AM
                else if (Amins == 60){
                    Amins=0;
                    sprintf(butAtime, "%02d:%02d AM", Ahours+12, Amins);    //12:00 goes back to zerow
                }
            }
            else if(Ahours < 0){
                Ahours =23;
                if (Amins >=0 && Amins <= 59 )     sprintf(butAtime, "%02d:%02d PM", Ahours-12,Amins);       //11 PM
                else if(Amins == 60){
                    Amins =0;
                    sprintf(butAtime, "%02d:%02d PM", Ahours-12,Amins);       //11 PM
                }
            }
            else{
                if (Amins >=0 && Amins <= 59 )    sprintf(butAtime, "%02d:%02d AM", Ahours,Amins);
                else if (Amins == 60){
                    Amins=0;
                    sprintf(butAtime, "%02d:%02d AM", Ahours,Amins);
                }
            }
            write_command(0x94);
            write_command(0x0F);
            for (j=0; j<8; j++)
            {
                dataWrite(butAtime[j]);    //prints time to LCD
                Systick_us_delay(10);
            }
        }
    }

    if(SNOOZEbutton_pressed()){  // decreasing the hours or minutes
        if(setA == 0){    //decreasing the hours
            Ahours--;
            if (Ahours == 0)                         sprintf(butAtime, "%02d:%02d AM", Ahours+12,Amins);    //12 am
            else if (Ahours>12 && Ahours<22)      sprintf(butAtime, " %01d:%02d PM", Ahours-12,Amins);  //b/w 1 PM and 9 PM
            else if (Ahours==12)                     sprintf(butAtime, "%02d:%02d PM", Ahours,Amins);          //noon 12 PM
            else if(Ahours>21 && Ahours<24)       sprintf(butAtime, "%02d:%02d PM", Ahours-12,Amins);       //b/w 10 PM and 11 PM
            else if (Ahours < 10 && Ahours >0)                    sprintf(butAtime, " %01d:%02d AM", Ahours,Amins);         //b/w 1 AM and 9 AM
            else if(Ahours >23){
                Ahours=0;
                sprintf(butAtime, "%02d:%02d AM", Ahours+12,Amins);       //12 am
            }
            else if(Ahours<0){
                Ahours = 23;
                sprintf(butAtime, "%02d:%02d PM", Ahours-12,Amins);       //11 PM
            }
            else                                        sprintf(butAtime, "%02d:%02d AM", Ahours,Amins);
            write_command(0x94);
            write_command(0x0F);
            for (j=0; j<8; j++)
            {
                dataWrite(butAtime[j]);    //prints time to LCD
                Systick_us_delay(10);
            }
        }
        if (setA==1){       //decreasing the minutes
            Amins--;

            if (Ahours == 0){     //12 AM
                if (Amins >=0 && Amins <= 59 )     sprintf(butAtime, "%02d:%02d AM", Ahours+12, Amins);    //12:00 to 12:59 AM
                else if (Amins <= 0){
                    Amins=59;
                    sprintf(butAtime, "%02d:%02d AM", Ahours+12, Amins);    //12:00 goes back to zerow
                }
            }
            else if (Ahours>12 && Ahours<22){
                if (Amins >0 && Amins <= 59 )     sprintf(butAtime, " %01d:%02d PM", Ahours-12,Amins);
                else if (Amins <= 0){
                    Amins=59;
                    sprintf(butAtime, " %01d:%02d PM", Ahours-12,Amins);
                }
            }
            else if (Ahours==12){
                if (Amins >0 && Amins <= 59 )     sprintf(butAtime, "%02d:%02d PM", Ahours,Amins);          //noon 12 PM
                else if (Amins <= 0){
                    Amins=59;
                    sprintf(butAtime, "%02d:%02d PM", Ahours, Amins);    //12:00 goes back to zerow
                }
            }
            else if(Ahours>21 && Ahours<24){
                if (Amins >=0 && Amins <= 59 )    sprintf(butAtime, "%02d:%02d PM", Ahours-12,Amins);       //b/w 10 PM and midnight
                else if (Amins <= 0){
                    Amins=59;
                    sprintf(butAtime, "%02d:%02d PM", Ahours-12,Amins);
                }
            }
            else if (Ahours >0 && Ahours < 10){
                if (Amins >= 0 && Amins <= 59)      sprintf(butAtime, " %01d:%02d AM", Ahours, Amins);
                else if (Amins <= 0){
                    Amins=59;
                    sprintf(butAtime, " %01d:%02d AM", Ahours,Amins);
                }
            }
            else if(Ahours > 23){
                Ahours=0;
                if (Amins >=0 && Amins <= 59 )     sprintf(butAtime, "%02d:%02d AM", Ahours+12, Amins);    //12:00 to 12:59 AM
                else if (Amins <= 0){
                    Amins=59;
                    sprintf(butAtime, "%02d:%02d AM", Ahours+12, Amins);    //12:00 goes back to zerow
                }
            }
            else if(Ahours < 0){
                Ahours =23;
                if (Amins >=0 && Amins <= 59 )     sprintf(butAtime, "%02d:%02d PM", Ahours-12,Amins);       //11 PM
                else if(Amins <= 0){
                    Amins =59;
                    sprintf(butAtime, "%02d:%02d PM", Ahours-12,Amins);       //11 PM
                }
            }
            else{
                if (Amins >=0 && Amins <= 59 )    sprintf(butAtime, "%02d:%02d AM", Ahours,Amins);
                else if (Amins <= 0){
                    Amins=59;
                    sprintf(butAtime, "%02d:%02d AM", Ahours,Amins);
                }
            }
            write_command(0x94);
            write_command(0x0F);
            for (j=0; j<8; j++)
            {
                dataWrite(butAtime[j]);    //prints time to LCD
                Systick_us_delay(10);
            }
        }
    }
}

void settime(){
    char butStime[11];
    //when setT is 0 the hours are being adjusted
    //the set time button is pressed again setT = 1 and the minutes are being adjusted

    int j =0; //flag for writing to LCD

    if(timebutton ==1){ //flag for time button interrupt
        timebutton=0;   //time button flag
        if(TIMEbutton_pressed()){       //debouncing the button press
            setT++;
        }
    }

    if(ONOFFbutton_pressed()){  //if ON/OFF/Up button is pressed we are increasing the hours or minutes
        if(setT == 0){    //increasing the hours
            Shours++;
            if (Shours == 0)                         sprintf(butStime, "%02d:%02d:%02d AM", Shours+12,Smins, Ssecs);       //12 am
            else if (Shours>12 && Shours<22)      sprintf(butStime, " %01d:%02d:%02d PM", Shours-12,Smins, Ssecs);      //b/w 1 PM and 9 PM
            else if (Shours==12)                     sprintf(butStime, "%02d:%02d:%02d PM", Shours,Smins, Ssecs);          //noon 12 PM
            else if(Shours>21 && Shours<24)     sprintf(butStime, "%02d:%02d:%02d PM", Shours-12,Smins, Ssecs);       //b/w 10 PM and 11 PM
            else if (Shours < 10 && Shours >0)                    sprintf(butStime, " %01d:%02d:%02d AM", Shours,Smins, Ssecs);         //b/w 12 AM and 9 AM
            else if(Shours >23){
                Shours=0;
                sprintf(butStime, "%02d:%02d:%02d AM", Shours+12,Smins, Ssecs);       //12 am
            }
            else if(Shours<0){
                Shours =23;
                sprintf(butStime, "%02d:%02d:%02d PM", Shours-12,Smins,Ssecs);       //11 PM
            }
            else                                        sprintf(butStime, "%02d:%02d:%02d AM", Shours,Smins, Ssecs);
            write_command(0x82);
            write_command(0x0F);
            for (j=0; j<11; j++)
            {
                dataWrite(butStime[j]);    //prints time to LCD
                Systick_us_delay(10);
            }
        }
        if (setT==1){       //increasing the minutes
            Smins++;
            if (Shours == 0){     //12 AM
                if (Smins >=0 && Smins <= 59 )     sprintf(butStime, "%02d:%02d:%02d AM", Shours+12, Smins, Ssecs);    //12:00 to 12:59 AM
                else if (Smins == 60){
                    Smins=0;
                    sprintf(butStime, "%02d:%02d:%02d AM", Shours+12, Smins, Ssecs);    //12:00 goes back to zero
                }
            }
            else if (Shours==12){
                if (Smins >0 && Smins <= 59 )     sprintf(butStime, "%02d:%02d:%02d PM", Shours,Smins, Ssecs);          //noon 12 PM
                else if (Smins == 60){
                    Smins=0;
                    sprintf(butStime, "%02d:%02d:%02d PM", Shours, Smins, Ssecs);    //12:00 goes back to zerow
                }
            }
            else if (Shours>12 && Shours<22){
                if (Smins >0 && Smins <= 59 )     sprintf(butStime, " %01d:%02d:%02d PM", Shours-12,Smins, Ssecs);      //1-9PM
                else if (Smins == 60){
                    Smins=0;
                    sprintf(butStime, " %01d:%02d:%02d PM", Shours-12,Smins, Ssecs);
                }
            }
            else if(Shours>21 && Shours<24){
                if (Smins >=0 && Smins <= 59 )    sprintf(butStime, "%02d:%02d:%02d PM", Shours-12,Smins,Ssecs);       //b/w 10 PM and 11 PM
                else if (Smins == 60){
                    Smins=0;
                    sprintf(butStime, "%02d:%02d:%02d PM", Shours-12,Smins,Ssecs);
                }
            }
            else if (Shours < 10){
                if (Smins >=0 && Smins <= 59 )    sprintf(butStime, " %01d:%02d:%02d AM", Shours,Smins,Ssecs);         //b/w 1 AM and 9 AM
                else if (Smins == 60){
                    Smins=0;
                    sprintf(butStime, " %01d:%02d:%02d AM", Shours,Smins,Ssecs);
                }
            }
            else if(Shours >23){
                Shours=0;
                if (Smins >=0 && Smins <= 59 )     sprintf(butStime, "%02d:%02d:%02d AM", Shours+12, Smins, Ssecs);    //12:00 to 12:59 AM
                else if (Smins == 60){
                    Smins=0;
                    sprintf(butStime, "%02d:%02d:%02d AM", Shours+12, Smins, Ssecs);    //12:00 goes back to zerow
                }
            }
            else if(Shours < 0){
                Shours =23;
                if (Smins >=0 && Smins <= 59 )     sprintf(butStime, "%02d:%02d:%02d PM", Shours-12,Smins,Ssecs);       //11 PM
                else if(Smins == 60){
                    Smins =0;
                    sprintf(butStime, "%02d:%02d:%02d PM", Shours-12,Smins,Ssecs);       //11 PM
                }
            }
            else{
                if (Smins >=0 && Smins <= 59 )    sprintf(butStime, "%02d:%02d:%02d AM", Shours,Smins,Ssecs);
                else if (Smins == 60){
                    Smins=0;
                    sprintf(butStime, "%02d:%02d:%02d AM", Shours,Smins,Ssecs);
                }
            }
            write_command(0x82);
            write_command(0x0F);
            for (j=0; j<11; j++)
            {
                dataWrite(butStime[j]);    //prints time to LCD
                Systick_us_delay(10);
            }
        }
    }

    if(SNOOZEbutton_pressed()){  // decreasing the hours or minutes
        if(setT == 0){    //decreasing the hours
            Shours--;
            if (Shours == 0)                         sprintf(butStime, "%02d:%02d:%02d AM", Shours+12,Smins, Ssecs);    //12 am
            else if (Shours>12 && Shours<22)      sprintf(butStime, " %01d:%02d:%02d PM", Shours-12,Smins, Ssecs);  //b/w 1 PM and 9 PM
            else if (Shours==12)                     sprintf(butStime, "%02d:%02d:%02d PM", Shours,Smins,Ssecs);          //noon 12 PM
            else if(Shours>21 && Shours<24)       sprintf(butStime, "%02d:%02d:%02d PM", Shours-12,Smins,Ssecs);       //b/w 10 PM and 11 PM
            else if (Shours < 10 && Shours >0)                    sprintf(butStime, " %01d:%02d:%02d AM", Shours,Smins,Ssecs);         //b/w 1 AM and 9 AM
            else if(Shours >23){
                Shours=0;
                sprintf(butStime, "%02d:%02d:%02d AM", Shours+12,Smins, Ssecs);       //12 am
            }
            else if(Shours<0){
                Shours = 23;
                sprintf(butStime, "%02d:%02d:%02d PM", Shours-12,Smins,Ssecs);       //11 PM
            }
            else                                        sprintf(butStime, "%02d:%02d:%02d AM", Shours,Smins,Ssecs);
            write_command(0x82);
            write_command(0x0F);
            for (j=0; j<11; j++)
            {
                dataWrite(butStime[j]);    //prints time to LCD
                Systick_us_delay(10);
            }
        }
        if (setT==1){       //decreasing the minutes
            Smins--;
            if (Shours == 0){     //12 AM
                if (Smins >=0 && Smins <= 59 )     sprintf(butStime, "%02d:%02d:%02d AM", Shours+12, Smins, Ssecs);    //12:00 to 12:59 AM
                else if (Smins <= 0){
                    Smins=59;
                    sprintf(butStime, "%02d:%02d:%02d AM", Shours+12, Smins, Ssecs);    //12:00 goes back to zerow
                }
            }
            else if (Shours>12 && Shours<22){
                if (Smins >0 && Smins <= 59 )     sprintf(butStime, " %01d:%02d:%02d PM", Shours-12,Smins, Ssecs);
                else if (Smins <= 0){
                    Smins=59;
                    sprintf(butStime, " %01d:%02d:%02d PM", Shours-12,Smins, Ssecs);
                }
            }
            else if (Shours==12){
                if (Smins >0 && Smins <= 59 )     sprintf(butStime, "%02d:%02d:%02d PM", Shours,Smins, Ssecs);          //noon 12 PM
                else if (Smins <= 0){
                    Smins=59;
                    sprintf(butStime, "%02d:%02d:%02d PM", Shours, Smins, Ssecs);    //12:00 goes back to zerow
                }
            }
            else if(Shours>21 && Shours<24){
                if (Smins >=0 && Smins <= 59 )    sprintf(butStime, "%02d:%02d:%02d PM", Shours-12,Smins,Ssecs);       //b/w 10 PM and midnight
                else if (Smins <= 0){
                    Smins=59;
                    sprintf(butStime, "%02d:%02d:%02d PM", Shours-12,Smins,Ssecs);
                }
            }
            else if (Shours < 10){
                if (Smins >=0 && Smins <= 59 )    sprintf(butStime, " %01d:%02d:%02d AM", Shours,Smins,Ssecs);         //b/w 1 AM and 9 AM
                else if (Smins <= 0){
                    Smins=59;
                    sprintf(butStime, " %01d:%02d:%02d AM", Shours,Smins,Ssecs);
                }
            }
            else if(Shours > 23){
                Shours=0;
                if (Smins >=0 && Smins <= 59 )     sprintf(butStime, "%02d:%02d:%02d AM", Shours+12, Smins, Ssecs);    //12:00 to 12:59 AM
                else if (Smins <= 0){
                    Smins=59;
                    sprintf(butStime, "%02d:%02d:%02d AM", Shours+12, Smins, Ssecs);    //12:00 goes back to zerow
                }
            }
            else if(Shours < 0){
                Shours =23;
                if (Smins >=0 && Smins <= 59 )     sprintf(butStime, "%02d:%02d:%02d PM", Shours-12,Smins,Ssecs);       //11 PM
                else if(Smins <= 0){
                    Smins =59;
                    sprintf(butStime, "%02d:%02d:%02d PM", Shours-12,Smins,Ssecs);       //11 PM
                }
            }
            else{
                if (Smins >=0 && Smins <= 59 )    sprintf(butStime, "%02d:%02d:%02d AM", Shours,Smins,Ssecs);
                else if (Smins <= 0){
                    Smins=59;
                    sprintf(butStime, "%02d:%02d:%02d AM", Shours,Smins,Ssecs);
                }
            }
            write_command(0x82);
            write_command(0x0F);
            for (j=0; j<11; j++)
            {
                dataWrite(butStime[j]);    //prints time to LCD
                Systick_us_delay(10);
            }
        }
    }
}

void RTC_Init(){
    //Initialize time to input from serial
    RTC_C->CTL0 = (0xA500);
    RTC_C->CTL13 = 0;

    RTC_C->TIM0 = Smins<<8 | Ssecs;//input min, input secs
    RTC_C->TIM1 = 1<<8 | Shours;  //Monday, 2 pm
    RTC_C->YEAR = 2018;
    //Alarm at 2:46 pm
    RTC_C->AMINHR = Ahours<<8 | Amins | BIT(15) | BIT(7);  //bit 15 and 7 are Alarm Enable bits
    RTC_C->ADOWDAY = 0;

    if (speed)  RTC_C->PS1CTL = 0b00010;  //1/64 second interrupt
    else  RTC_C->PS1CTL = 0b11010;  //1/64 second interrupt

    RTC_C->CTL0 = (0xA500) | BIT5; //turn on interrupt
    RTC_C->CTL13 = 0;
    NVIC_EnableIRQ(RTC_C_IRQn);
}


void RTC_C_IRQHandler()
{
    if(RTC_C->PS1CTL & BIT0)
    {
        hours = RTC_C->TIM1 & 0x00FF;
        if (hours > 23) RTC_C->TIM1 = 0<<8;
        mins = (RTC_C->TIM0 & 0xFF00) >> 8;
        if(mins > 59) RTC_C->TIM0 = 0<<8;
        secs = RTC_C->TIM0 & 0x00FF;

        if (speed == 1)
        {
            static int alrm = 0;
            static int light = 0;
            c = 1;

            if (mins>59) RTC_C->TIM1 = ((RTC_C->TIM1 & 0x00FF)+1);
            if(secs != 59){                                 // If not  59 seconds, add 1 (otherwise 59+1 = 60 which doesn't work)
                RTC_C->TIM0 = RTC_C->TIM0 + 1;
            }
            else {
                RTC_C->TIM0 = (((RTC_C->TIM0 & 0xFF00) >> 8)+1)<<8;  // Add a minute if at 59 seconds.  This also resets seconds.
                // TODO: What happens if minutes are at 59 minutes as well?
                alrm++;
                if (alrm%2 == 0)
                    al = 1;
                time_update = 1;                                     // Send flag to main program to notify a time update occurred.
            }
            RTC_C->PS1CTL &= ~BIT0;
        }
        else if (speed == 0)
        {
            static int alrm = 0;
            static int light = 0;

            alrm++;
            if (alrm%2 == 0)
                al = 1;
            light++;
            if (light%3 == 0){
                b = 1;
            }
            time_update = 1;
            RTC_C->PS1CTL &= ~BIT0;
        }
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
    Systick_us_delay(1500);
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
/********************************************************
 * Michael James     Bryanna Flowers
 * Written by MJ and BF
 * initializing timers and PWM for both motors and LEDs
 * Inputs: N/A
 * Outputs: LED PWMs and both motor PWM
 ***************************************************** */
void initializePWMports(){

    P6->SEL0 |= (BIT6|BIT7);    //PWM for blue and red
    P6->SEL1 &= ~(BIT6|BIT7);
    P6->DIR |= (BIT6|BIT7);     //set as output
    P6->OUT &= ~(BIT6|BIT7);

    TIMER_A2->CCR[0] = 2999;  //1000 clocks = 0.333 ms.  This is the period of everything on Timer A0.  0.333 < 16.666 ms so the on/off shouldn't
    //be visible with the human eye.  1000 makes easy math to calculate duty cycle.  No particular reason to use 1000.

    TIMER_A2->CCTL[3] = 0b0000000011100000;
    TIMER_A2->CCR[3] = 0;  //P6.6 RED
    TIMER_A2->CCTL[4] = 0b0000000011100000;
    TIMER_A2->CCR[4] = 0;  //P6.7 BLUE

    //The next line turns on all of Timer A0.  None of the above will do anything until Timer A0 is started.
    TIMER_A2->CTL = 0b0000001000010100;  //up mode, smclk, taclr to load.  Up mode configuration turns on the output when CCR[1] is reached
    //and off when CCR[0] is reached. SMCLK is the master clock at 3,000,000 MHz.  TACLR must be set to load
    //in the changes to CTL register.

}
/********************************************************
 * Michael James     Bryanna Flowers
 * Written by MJ and BF
 * turns on and off the lights
 * Inputs: N/A
 * Outputs: N/A
 ***************************************************** */
void brighter()
{
    TIMER_A2->CCR[3] = brightness;
    TIMER_A2->CCR[4] = brightness;
}
/********************************************************
 * Michael James     Bryanna Flowers
 * Written by MJ and BF
 * initializing buttons for interrupts
 * Inputs: buttons for setting time and alarm, and snooze
 * and turning on and off the alarm
 * Outputs: N/A
 ***************************************************** */
void butt_init(void)
{
    P6->DIR &= ~(BIT0); //input time
    P6->REN = (BIT0);
    P6->OUT |= (BIT0);
    P6->IE = (BIT0);    //enable interrupt
    P6->IES |= (BIT0);
    P6->IFG = 0;    //clear flag

    P5->DIR &= ~(BIT7); //input alarm
    P5->REN = (BIT7);
    P5->OUT |= (BIT7);
    P5->IE = (BIT7);    //enable interrupt
    P5->IES |= (BIT7);
    P5->IFG = 0;    //clear flag

    P3->DIR &= ~(BIT2|BIT3); //input on/off/up and snooze/down
    P3->REN = (BIT2|BIT3);
    P3->OUT |= (BIT2|BIT3);
    P3->IE = (BIT2|BIT3);    //enable interrupt
    P3->IES |= (BIT2|BIT3);
    P3->IFG = 0;    //clear flag
}
/********************************************************
 * Michael James     Bryanna Flowers
 * Written by MJ and BF
 * debounce for alarm button
 * Inputs: button for setting the alarm P5.7
 * Outputs: N/A
 ***************************************************** */
int ALARMbutton_pressed(void)
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
 * debounce for time button
 * Inputs: debounce for button for setting the time P6.0
 * Outputs: N/A
 ***************************************************** */
int TIMEbutton_pressed(void)
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
/********************************************************
 * Michael James     Bryanna Flowers
 * Written by MJ and BF
 * debounce for the ON/OFF/UP button
 * Inputs: debounce for button for turning the alarm on
 * or off and increasing the hours and minutes when
 * setting alarm and time P3.2
 * Outputs: N/A
 ***************************************************** */
int ONOFFbutton_pressed(void)
{
    int buttonDebounced = 0;
    if (!(P3->IN & BIT2))   //if button pressed
    {
        __delay_cycles(1500); // waits for 5 milliseconds
        while (!(P3->IN & BIT2)){}  //wait for button to be released
        buttonDebounced =1;
    }
    return buttonDebounced;
}
/********************************************************
 * Michael James     Bryanna Flowers
 * Written by MJ and BF
 * debounce for Snooze/Down button
 * Inputs: button for snoozing the alarm and
 * decreasing the hours and minutes when setting alarm
 * and time P3.3
 * Outputs: N/A
 ***************************************************** */
int SNOOZEbutton_pressed(void)
{
    int buttonDebounced = 0;
    if (!(P3->IN & BIT3))   //if button pressed
    {
        __delay_cycles(1500); // waits for 5 milliseconds
        while (!(P3->IN & BIT3)){}  //wait for button to be released
        buttonDebounced =1;
    }
    return buttonDebounced;
}
/********************************************************
 * Michael James     Bryanna Flowers
 * Written by MJ and BF
 * interrupt for time button
 * Inputs: button for setting time
 * Outputs: N/A
 ***************************************************** */
void PORT6_IRQHandler(void){
    P6->IFG = 0x00; //clears intrrupt flag
    if(!(P6->IN & BIT0)){   // if interrupt flag triggered
        timebutton=1;
    }
}
/********************************************************
 * Michael James     Bryanna Flowers
 * Written by MJ and BF
 * interrupt for alarm button
 * Inputs: button for setting alarm
 * Outputs: N/A
 ***************************************************** */
void PORT5_IRQHandler(void){
    P5->IFG = 0x00; //clears interrupt flag
    if(!(P5->IN & BIT7)){   //if interrupt flag triggered
        alarmbutton =1;
    }
}
/********************************************************
 * Michael James     Bryanna Flowers
 * Written by MJ and BF
 * interrupt for ON/OFF/Up and Snooze/Down button
 * Inputs: buttons for turning on and off the alarm as well
 * as snoozing the alarm 3.2 (on/off/up) 3.3 (snooze/down)
 * Outputs: N/A
 ***************************************************** */

void PORT3_IRQHandler(void){
    P3->IFG = 0x00; //clears intrrupt flag
    if(!(P3->IN & BIT2)){   // if interrupt flag triggered
        OOU = 1;
    }
    if(!(P3->IN & BIT3)){   // if interrupt flag triggered
        SD = 1;
    }
}
void SetupTimerAlarm()
{
    P7->SEL0 |= BIT6;    // P7.6 to PWM Control for speaker
    P7->SEL1 &= ~BIT6;
    P7->DIR |= BIT6;

    TIMER_A1->CCR[0]=3000000/450;
    TIMER_A1->CCR[2]=0;
    TIMER_A1->CCTL[2] = 0b11100000;   //reset set
    TIMER_A1->CTL = 0x0214;  //count up clear register
}

void PORT1_IRQHandler(void)
{
    if(P1->IFG & BIT1) {                                //If P1.1 had an interrupt
        RTC_C->PS1CTL = 0b11010;  //1/64 second interrupt
        speed = 0;
    }
    if(P1->IFG & BIT4) {                                //If P1.4 had an interrupt
        RTC_C->PS1CTL = 0b00010;  //1/64 second interrupt
        speed = 1;
    }
    P1->IFG = 0;                                        //Clear all flags
}

void P1_Init() {
    P1->SEL0 &= ~(BIT1|BIT4);
    P1->SEL1 &= ~(BIT1|BIT4);
    P1->DIR  &= ~(BIT1|BIT4);
    P1->REN  |=  (BIT1|BIT4);
    P1->OUT  |=  (BIT1|BIT4);
    P1->IE   |=  (BIT1|BIT4);
    P1->IFG = 0;
    NVIC_EnableIRQ(PORT1_IRQn);
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
