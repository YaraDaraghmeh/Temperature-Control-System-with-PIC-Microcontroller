// CONFIG1H
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include<stdbool.h>
#include <xc.h>
#include <stdio.h>
#include "my_adc.h"
#include "my_pwm.h"
#include "lcd_x8.h"
#include "my_ser.h"
#include <string.h>
#include <stdint.h>   
#include  <stdlib.h>
#define WINTER_T 40
#define SUMMER_T 60
#define TMR3_START_VALUE 55536 

void setup_compare(void);
void Int0ISR(void);
void Int2ISR(void);
void setupInterpurts(void);
void initPorts(void);
void compare_isr(void);
void tmr_isr(void);
void OffMode(void);
void CoolMode(void);
void heatMode(void);
void autoCool(void);
void autoHeat(void);
void printingOnScreen(void);
void setupAllThings(void);
void autoModeControl(void);
typedef enum { OFF, COOL, HEAT, AUTO_COOL_HEAT } Mode;
Mode current_mode= OFF;
int mode_counter=-1;
int Hs = 0;
char Buffer[32];
volatile int interrupt_counter = 0;
volatile int prev_percent_heat_counter = -1;
int raw_value;
float AI0;
float AI1;
float AI2;
 int Percent_Heat_Counter_heat=0;
 int Percent_Heat_Counter_cool = 0;

char Buffer[32];
float AI0;
float AI1;
float AI2;
int percentheating;
int percentcooling;

char g;
int value_index = 0;
char value_buffer[10];
int flagtochangecoolingpercent=0;
int flagtochangeheatingpercent=0;
int readedcoolingpercentage=0;
int readedheatingpercentage=0;
int flagtochangebyreading=0;
int readedsetpoint=0;
int readedoutsidetemp=0;
float string_to_float(char* str) {
    float result = 0.0;
    float decimal = 0.0;
    float decimal_multiplier = 0.1;
    int decimal_point_seen = 0;
    
    // Handle negative numbers
    int negative = 0;
    if (*str == '-') {
        negative = 1;
        str++;
    }
    
    // Process each character
    while (*str) {
        if (*str == '.') {
            decimal_point_seen = 1;
        }
        else if (*str >= '0' && *str <= '9') {
            if (!decimal_point_seen) {
                result = result * 10.0 + (*str - '0');
            }
            else {
                decimal = decimal + (*str - '0') * decimal_multiplier;
                decimal_multiplier *= 0.1;
            }
        }
        str++;
    }
    
    result = result + decimal;
    if (negative) {
        result = -result;
    }
    return result;
}

void RX_isr(void) {
    if (PIR1bits.RCIF) { // Check if receive interrupt flag is set
        g = RCREG;

    if (g == 'S') {
        current_mode = OFF;
        mode_counter=-1;
        OffMode();
        send_string_no_lib((unsigned char *)"System is now OFF");

        return;
    }
        if (g=='M'){
            char status_response[100];
    sprintf(status_response, (unsigned char *)"Room Temperature (RM): %.2f C\r\n", (AI2 * 5.0 / 1023.0) * 100);
    send_string_no_lib(status_response);

    sprintf(status_response, (unsigned char *)"Outside Temperature (OT): %.2f C\r\n", (AI1*5.0*100.0)/1023.0/5.0);
    send_string_no_lib(status_response);

    sprintf(status_response, (unsigned char *)"SetPoint Temperature (SP): %.2f C\r\n",  (AI0*5.0*100.0)/1023.0/5.0);
    send_string_no_lib(status_response);

    // Cooler ON/OFF status
    if (PORTCbits.RC2 ==1) {
        sprintf(status_response, (unsigned char *)"Cooler: ON\r\n");
    } else {
        sprintf(status_response, (unsigned char *)"Cooler: OFF\r\n");
    }
    send_string_no_lib(status_response);

    // Heater ON/OFF status
    if (PORTCbits.RC5 == 1) {
        sprintf(status_response, (unsigned char *)"Heater: ON\r\n");
    } else {
        sprintf(status_response, (unsigned char *)"Heater: OFF\r\n");
    }
    send_string_no_lib(status_response);

    // Cooling Percentage
    sprintf(status_response, (unsigned char *)"Cooling Percentage: %d%%\r\n", percentcooling);
    send_string_no_lib(status_response);

    // Heating Percentage

    sprintf(status_response, (unsigned char *)"Heating Percentage: %d%%\r\n",percentheating);
    send_string_no_lib(status_response);
                return;
            }
        
       if (g == 'm') {
    // First display current mode
    switch(current_mode) {
        case OFF:
            send_string_no_lib((unsigned char *)"Current Mode: OFF\r\n");
            send_string_no_lib((unsigned char *)"You can change:\r\n");
            send_string_no_lib((unsigned char *)"1. Room Temperature (RM)\r\n");
            send_string_no_lib((unsigned char *)"2. Outside Temperature (OT)\r\n");
            send_string_no_lib((unsigned char *)"3. SetPoint Temperature (SP)\r\n");
            send_string_no_lib((unsigned char *)"Enter 'c' followed by number (1-3) and new value\r\n");
            break;
            
        case COOL:
            send_string_no_lib((unsigned char *)"Current Mode: COOL\r\n");
            send_string_no_lib((unsigned char *)"You can change:\r\n");
            send_string_no_lib((unsigned char *)"1. Cooling Percentage\r\n");
            send_string_no_lib((unsigned char *)"Enter 'c1' followed by new percentage (0-100)\r\n");
            break;
            
        case HEAT:
            send_string_no_lib((unsigned char *)"Current Mode: HEAT\r\n");
            send_string_no_lib((unsigned char *)"You can change:\r\n");
            send_string_no_lib((unsigned char *)"1. Heating Percentage\r\n");
            send_string_no_lib((unsigned char *)"Enter 'c1' followed by new percentage (0-100)\r\n");
            break;
            
        case AUTO_COOL_HEAT:
            send_string_no_lib((unsigned char *)"Current Mode: AUTO\r\n");
            send_string_no_lib((unsigned char *)"You can change:\r\n");
            send_string_no_lib((unsigned char *)"1. SetPoint Temperature (SP)\r\n");
            send_string_no_lib((unsigned char *)"2. Outside Temperature (OT)\r\n");
            send_string_no_lib((unsigned char *)"Enter 'c' followed by number (1-2) and new value\r\n");
            break;
    }
    return;
}

// Handle the change command 'c'
if (g == 'c') {
    value_index = 0;  // Reset value buffer index
    return;  // Wait for the next character
}

// If we're collecting a value (after 'c' was received)
if (value_index >= 0) {
    if (g >= '0' && g <= '9' || g == '.') {
        value_buffer[value_index++] = g;
        if (value_index >= 9) {  // Prevent buffer overflow
            value_index = -1;  // Reset collection
        }
        return;
    }
    else if (g == '\r' || g == '\n') {  // End of value input
    value_buffer[value_index] = '\0';  // Null terminate
    float new_value = string_to_float(value_buffer + 1); // Skip the first character which is the selection number
    
    switch(current_mode) {
        case OFF:
            // Handle OFF mode changes
            if (value_buffer[0] == '1') {
                AI2 = (new_value * 1023.0) / (100.0 * 5.0);  // Room temp
            } else if (value_buffer[0] == '2') {
                AI1 = (new_value * 1023.0) / (100.0 * 5.0);  // Outside temp
            } else if (value_buffer[0] == '3') {
                AI0 = (new_value * 1023.0) / (100.0 * 5.0);  // Setpoint
            }
            break;
            
        case COOL:
            readedcoolingpercentage = (int)new_value;
            flagtochangecoolingpercent=1;
            if (readedcoolingpercentage > 100) readedcoolingpercentage = 100;
            if (readedcoolingpercentage < 0) readedcoolingpercentage = 0;
            CoolMode();
            
            break;
            
        case HEAT:
            readedheatingpercentage = (int)new_value;
            flagtochangeheatingpercent=1;
            if (readedheatingpercentage > 100) readedheatingpercentage = 100;
            if (readedheatingpercentage < 0) readedheatingpercentage = 0;
            heatMode();
            break;
            
        case AUTO_COOL_HEAT:
            if (value_buffer[0] == '1') {
                readedsetpoint = new_value;  // Setpoint
                flagtochangebyreading=1;
               autoModeControl();
               printingOnScreen();
                
            } else if (value_buffer[0] == '2') {
                readedoutsidetemp = new_value ;  // Outside temp
                flagtochangebyreading=1;
                autoModeControl();
                printingOnScreen();
            }
            break;
    }
    
    send_string_no_lib((unsigned char *)"Value updated successfully\r\n");
    value_index = -1;  // Reset collection
    return;
    }}
}
}

void DecHs(void) {
    __delay_ms(70);
    INTCON3bits.INT1IF = 0;
    Hs--;
    if (Hs <= 0) {
        Hs = 0;
    }
}

void IncHs(void) {
    __delay_ms(70);
    INTCON3bits.INT2IF = 0;
    Hs++;
    if (Hs >= 3) {
        Hs = 3;
    }
}

void DecHeatPercentCounter(void) {
    __delay_ms(70);
    INTCON3bits.INT1IF = 0;
    if (Percent_Heat_Counter_heat > 0) {
        Percent_Heat_Counter_heat--;
    }
}

void IncHeatPercentCounter(void) {
    __delay_ms(70);
    INTCON3bits.INT2IF = 0;
    if (Percent_Heat_Counter_heat < 20) {
        Percent_Heat_Counter_heat++;
    }
}
void DecCoolPercentCounter(void) {
    
}
void IncCoolPercentCounter(void) {
    __delay_ms(250);
    INTCON3bits.INT2IF = 0;
    if (Percent_Heat_Counter_cool < 20) {
        Percent_Heat_Counter_cool++;
    }
}
void __interrupt(high_priority) highIsr(void) {
    if(PIR1bits.RCIF) RX_isr();
    // Check mode first, then handle appropriate interrupts
    if (INTCON3bits.INT1IF) {  // Changed from INT1F to INT1IF for consistency
    __delay_ms(100); // Debounce
    if (!PORTBbits.RB3) {
        current_mode = OFF;
        mode_counter=-1;
        OffMode();
    }
    INTCON3bits.INT1IF = 0;  // Clear interrupt flag/    -*/    
}


    if ( current_mode == AUTO_COOL_HEAT) {
        if (INTCON3bits.INT2IF) {
            IncHs();
            // Clear flag is handled in IncHs()
        } 
        else if (INTCON3bits.INT1IF) {
            DecHs();
            // Clear flag is handled in DecHs()
        }
    }
    if (current_mode == HEAT) {
        if (INTCON3bits.INT1IF) {
            DecHeatPercentCounter();
            // Clear flag is handled in DecHeatPercentCounter()
        } 
        if (INTCON3bits.INT2IF) {
            IncHeatPercentCounter();
            // Clear flag is handled in IncHeatPercentCounter()
        }
    }
    if (current_mode == COOL) {
        if (INTCON3bits.INT2IF) {
            IncCoolPercentCounter();
        } 
        if (INTCON3bits.INT1IF) {
            __delay_ms(100);
            INTCON3bits.INT1IF = 0;
             if (Percent_Heat_Counter_cool > 0) {
                Percent_Heat_Counter_cool--;
        
            }
            DecCoolPercentCounter();
        }
     }
    
    // Mode change interrupt should be checked independently
    if (INTCONbits.INT0IF) {
        Int0ISR();
        // Clear flag is handled in Int0ISR()
    }
    
    // Timer and Compare interrupts should also be independent of mode
    if (PIR2bits.CCP2IF) {
        compare_isr();
        // Clear flag is handled in compare_isr()
    }
    
    if (PIR2bits.TMR3IF) {
        tmr_isr();
        // Clear flag is handled in tmr_isr()
    }
    
    
    
    
}



void Int0ISR(void)
{
        __delay_ms(250);
        INTCONbits.INT0IF = 0;
         
        mode_counter++;
        if(mode_counter==0){
            current_mode=OFF;
            OffMode();}
        else if(mode_counter==1)
            current_mode=COOL;
        else if(mode_counter==2)
            current_mode=HEAT;
        else if(mode_counter==3)
        {
            current_mode=AUTO_COOL_HEAT;
             mode_counter=0;

        }
        

}

void setupInterpurts(void) {
     // Disable all interrupts during setup
    INTCONbits.GIE = 0;
    
    INTCON2 = 0b01010001;
    INTCON3 = 0b00110000;  // Enable both INT1 and INT2 interrupts
    INTCON = 0b11010000;   // Enable global and peripheral interrupts
    
    // Setup specific interrupt enables
    INTCON3bits.INT1IE = 1;  // Enable INT1
    INTCON3bits.INT2IE = 1;  // Enable INT2
    
    PIE2 = 0b00001001;
    PIR2 = 0b00000000;
    IPR2 = 0b00000000;
    
    // Re-enable global interrupts
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
   
    current_mode = OFF;         // Start in OFF mode
    OffMode();                  // Explicitly call OffMode to initialize in OFF state
    
    
} 
void setup_compare(void){
    CCP2CON = 9;
    T3CON = 0x00; 
    T3CONbits.TMR3CS = 0; 
    T3CONbits.T3CKPS = 0b00; 
    T3CONbits.T3CCP2 = 0;
    T3CONbits.T3CCP1 = 1;
    TMR3 = 0; 
    T3CONbits.TMR3ON = 1; 
}
void initPorts(void) {
    ADCON0 = 0;
    ADCON1 = 0b00001100; 
    LATA = LATB = LATC = LATD = LATE = 0;
     TRISA = 0xFF; 
    TRISB = 0xFF;
    TRISC = 0x80; 
    TRISD = 0; 
    TRISE = 0;   
}

void init_UART(void) {
    // Baud Rate Configuration
    TXSTAbits.SYNC = 0;    // Asynchronous mode
    TXSTAbits.BRGH = 1;    // High baud rate
    BAUDCONbits.BRG16 = 0; // 8-bit baud rate generator
    SPBRG = 25;            // Baud rate value
    
    // Transmitter Configuration
    TXSTAbits.TXEN = 1;    // Enable transmission
    
    // Receiver Configuration
    RCSTAbits.SPEN = 1;    // Enable serial port
    RCSTAbits.CREN = 1;    // Enable continuous reception
    
    // Interrupt Configuration
    PIE1bits.RCIE = 1;     // Enable UART receive interrupt
    IPR1bits.RCIP = 1;     // Set high priority for receive interrupt
    
    // Enable peripheral interrupts
    INTCONbits.PEIE = 1;   // Enable peripheral interrupts
    INTCONbits.GIE = 1;    // Enable global interrupts
}
void setupAllThings(void) {
    initPorts();        // Initialize ports first
    init_UART();        // Initialize UART before using it
    lcd_init();         // Initialize LCD
    __delay_ms(100);    // Add a small delay after LCD init
    init_adc_no_lib();  // Initialize ADC
    setupInterpurts();  // Setup interrupts
    setup_compare();    // Setup compare module
    setupSerial();
}



void compare_isr(void){
    PIR2bits.CCP2IF = 0;
    PORTCbits.RC5=0;
    PORTDbits.RD2 ^= 1;
}
void tmr_isr(void){
     PIR2bits.TMR3IF = 0;  // Clear interrupt flag
    PORTDbits.RD1 ^= 1;   // Toggle RD1 for debugging
    
    // Increment and check counter
    interrupt_counter++;
    if (interrupt_counter >= 20) {
        interrupt_counter = 0;
    }
    
    // Only set RC5 based on counter comparison
    if (interrupt_counter < Percent_Heat_Counter_heat) {
        PORTCbits.RC5 = 1;    // ON when counter is less than heat percentage
    } else {
        PORTCbits.RC5 = 0;    // OFF when counter exceeds heat percentage
    }
    
    TMR3 = 55536;  // Reset timer
}


void OffMode(void)
{
    lcd_gotoxy(1, 4);
    sprintf(Buffer,"Mode: OFF       ");
    lcd_puts(Buffer);
    PORTCbits.RC2 =0; 
    PORTCbits.RC5= 0;
    PIE2 = PIE1 = 0;
    PORTCbits.RC5 = 0;
    T3CONbits.TMR3ON = 0;
    PORTCbits.RC2 = 0;
    CCP1CON = 0x00;
    flagtochangecoolingpercent=0;
    flagtochangeheatingpercent=0;
    flagtochangebyreading=0;
    
    
    
}

 


void putch(char data) {
    while(!TXSTAbits.TRMT); 
    TXREG = data;           
}
void CoolMode(void) {
    // Display current mode
    lcd_gotoxy(1, 4);
    sprintf(Buffer, "Mode: Cool      ");
    lcd_puts(Buffer);
    
    // Initialize PWM for cooler
    init_pwm1();
  
    // Turn off heater
    PORTCbits.RC5 = 0;
    T3CONbits.TMR3ON = 0;
    // Calculate cooling percentage (5% per count)
    float cooling_percent;
    if (flagtochangecoolingpercent){
        cooling_percent=readedcoolingpercentage;
           
    }
    else{  cooling_percent = Percent_Heat_Counter_cool * 5.0;}
    
    // Set PWM duty cycle based on percentage
    set_pwm1_percent(cooling_percent);
    percentcooling=cooling_percent;
    lcd_gotoxy(1, 3);
    sprintf(Buffer, "OT:%2.1f  C:%2.f%%  ",(AI1*5.0*100.0)/1023.0/5.0,cooling_percent);
    lcd_puts(Buffer);
    // Debug output
   
}
void heatMode(void){
    // Display current mode
    lcd_gotoxy(1, 4);
    sprintf(Buffer, "Mode: Heat      ");
    lcd_puts(Buffer);
    
    // Turn off cooler
    PORTCbits.RC2 = 0;
    CCP1CON = 0x00;
    
    // Enable Timer3 and its interrupts
    T3CON = 0x00;
    T3CONbits.TMR3CS = 0;    // Use internal clock
    T3CONbits.T3CKPS = 0b10; // Prescaler
    T3CONbits.TMR3ON = 1;    // Turn on Timer3
    
    PIE2bits.TMR3IE = 1;     // Enable Timer3 interrupt
    PIE2bits.CCP2IE = 1;     // Enable CCP2 interrupt
    
    float percent_heat;
    // Calculate heating percentage from counter
    if (flagtochangeheatingpercent){
        percent_heat = readedheatingpercentage;
        Percent_Heat_Counter_heat = (int)(percent_heat / 5.0); // Convert percentage to counter value
    } else {
        percent_heat = Percent_Heat_Counter_heat * 5.0;
    }
    
    // Ensure percent_heat stays within valid range (0-100%)
    if (percent_heat > 100.0) percent_heat = 100.0;
    if (percent_heat < 0.0) percent_heat = 0.0;
    
    percentheating = percent_heat;
    lcd_gotoxy(1, 3);
    sprintf(Buffer, "OT:%2.1f  H:%2.f%%  ",(AI1*5.0*100.0)/1023.0/5.0,percent_heat);
    lcd_puts(Buffer);
}
void autoCool(void) {
    lcd_gotoxy(1, 4);
    sprintf(Buffer, "Md:Auto C  Hs:%d",Hs);
    lcd_puts(Buffer);
    init_pwm1();
    float SP;
    float OT;
    if (flagtochangebyreading){
        SP = readedsetpoint;  // Use the read setpoint
        OT = readedoutsidetemp;  // Use the read outside temp
    } else {
        SP = (AI0 * 5.0 / 1023.0) * 100 / 5.0; // Set Point temperature
        OT = (AI1*5.0*100.0)/1023.0/5.0;
    }
    float T = (AI2 * 5.0 / 1023.0) * 100;       // Current temperature
    float coolError = T - SP;

    PIE2bits.TMR3IE = 0; 
    PIE2bits.CCP2IE = 0; 
    PORTCbits.RC2 = 1;
   //CCP1CON = 0x00;
    if (coolError > 0) {
        float percent_value = coolError * 10;
        
        if (percent_value < 25.0) {
            percent_value = 25.0; 
        }
        if (percent_value > 100.0) {
            percent_value = 100.0; 
        }
        
        
        lcd_gotoxy(1, 3);
        sprintf(Buffer, "OT:%2.1f  C:%2.1f%%", OT,percent_value);
        lcd_puts(Buffer);
        lcd_gotoxy(1, 2);
        sprintf(Buffer, "SP: %4.1fC   %d %d", SP, PORTCbits.RC5, PORTCbits.RC2);
        lcd_puts(Buffer);
     
        set_pwm1_percent(percent_value);
        
        PORTCbits.RC5 = 0;
    } 
  
    else if (T <= (SP - Hs)) {
        set_pwm1_percent(0.0);  
        PIE2bits.TMR3IE = 1; 
        PIE2bits.CCP2IE = 1; 

        int raw_value = 512 * 64; 
        CCPR2H = (raw_value >> 8) & 0x00FF;
        CCPR2L = raw_value & 0x00FF;
    }
}

void autoHeat(void) {
    lcd_gotoxy(1, 4);
    sprintf(Buffer, "Md:Auto H  Hs:%d  ",Hs);
    lcd_puts(Buffer);
    init_pwm1();
    float SP;
    float OT;
    if (flagtochangebyreading){
        SP = readedsetpoint;  // Use the read setpoint
        OT = readedoutsidetemp;  // Use the read outside temp
    } else {
        SP = (AI0 * 5.0 / 1023.0) * 100 / 5.0;
        OT = (AI1*5.0*100.0)/1023.0/5.0;
    }
    float T = (AI2 * 5.0 / 1023.0) * 100;

    // Turn off cooler in this mode
    PORTCbits.RC2 = 0;
    CCP1CON = 0x00;
    PORTCbits.RC5 = 1;
     
    // Calculate percent heat based on requirements
     float heatError = SP - T;
    if (heatError > 0) {
        float percent_heat = heatError * 10;  // Scale the heatError to determine heat percentage
        ///////////////////
        
        ///////////////////
        // Implement constraints
        if (percent_heat > 100.0) {
            percent_heat = 100.0;
        }
        if (percent_heat < 25.0) {
            percent_heat = 25.0;  // Minimum heating to maintain temperature
        }
       
        lcd_gotoxy(1, 3);
        sprintf(Buffer, "OT:%2.1f  H:%2.1f%%", OT,percent_heat);
        lcd_puts(Buffer);
        lcd_gotoxy(1, 2);
        sprintf(Buffer, "SP: %4.1fC   %d %d", SP, PORTCbits.RC5, PORTCbits.RC2);
        lcd_puts(Buffer);

        // Enable Timer3 and CCP2 interrupts for PWM
        PIE2bits.TMR3IE = 1;
        PIE2bits.CCP2IE = 1;

        // Calculate and set the compare value for PWM
        int compare_value = (int)(percent_heat * 65535 / 100.0);
        CCPR2H = (compare_value >> 8) & 0x00FF;
        CCPR2L = compare_value & 0x00FF;
    } else {
        // Turn off heater if no heating is required
        PORTCbits.RC5 = 0;
        PIE2bits.TMR3IE = 0;
        PIE2bits.CCP2IE = 0;
    }

    // Turn off heater if temperature exceeds SP + Hs
    if (T > (SP + Hs)) {
        PORTCbits.RC5 = 0;  // Turn off heater
        PIE2bits.TMR3IE = 0;
        PIE2bits.CCP2IE = 0;
    }
}

void autoModeControl(void) {
    lcd_gotoxy(1, 4);
    sprintf(Buffer, "Md:AutoHC  Hs:%d",Hs);  // Changed to Auto HT as specified
    lcd_puts(Buffer);
    float OT;
    if (flagtochangebyreading){
        OT = readedoutsidetemp;
    }else{
    OT = ((AI1*5.0*100.0)/1023.0/5.0);  // Calculate outdoor temperature}
    }
    if (OT > SUMMER_T) {
        autoCool();
    } else if (OT < WINTER_T) {
        autoHeat();
    } else {
        // No heating or cooling needed
        PORTCbits.RC2 = 0;  // Turn off cooler
        PORTCbits.RC5 = 0;  // Turn off heater
        PIE2bits.TMR3IE = 0;
        PIE2bits.CCP2IE = 0;
    }
}
void printingOnScreen(void){
    
    AI0 = read_adc_raw_no_lib(0);
    AI1 = read_adc_raw_no_lib(1);
    AI2 = read_adc_raw_no_lib(2);

    lcd_gotoxy(1, 1);
    sprintf(Buffer, "RT: %4.1fC   H C ",(AI2*100.0*5.0)/1023.0 );
    lcd_puts(Buffer);
   
    
    lcd_gotoxy(1, 2);
   sprintf(Buffer, "SP: %4.1fC   %d %d", (AI0 * 5.0 * 100.0) / (1023.0 * 5.0), PORTCbits.RC5, PORTCbits.RC2);

    lcd_puts(Buffer);

   // lcd_gotoxy(1, 3);
  //  sprintf(Buffer, "HS: %d  HC: %2.1f%%", Hs,(AI1*5.0*100.0)/1023.0/5.0);
   // lcd_puts(Buffer);

        
}
void main(void) {
    setupAllThings();
    // Add initial display test
    lcd_gotoxy(1, 1);
    sprintf(Buffer, "System Starting...");
    lcd_puts(Buffer);
    __delay_ms(1000);  // Give time to see the message
    
    while(1) {
        CLRWDT();
        printingOnScreen();
        
        if (current_mode == OFF) {
            
            OffMode();
        } 
        else if (current_mode == COOL) {
            CoolMode();
        } 
        else if (current_mode == HEAT) {
            heatMode();
        } 
        else if (current_mode == AUTO_COOL_HEAT) {
            autoModeControl();
        }
        RX_isr();
    }
    return;
}