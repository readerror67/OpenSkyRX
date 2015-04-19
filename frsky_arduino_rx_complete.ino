/*
 * Frsky RX 2-way
 *  By  Midelic
 *  on RCGroups
 * an adaptation from Kyrre Aalerud(Kreature)
 * 2012 Frsky rx demo code
 * http://www.rcgroups.com/forums/showthread.php?t=1667453
 * Thanks also to Phracturedblue and his deviation firmware
 **********************************
 */

#include <avr/interrupt.h>
#include <EEPROM.h>
#include "iface_cc2500.h"
//#define DEBUG
//#define DEBUG0
//#define DEBUG1
//#define DEBUG2
//#define DEBUG3
//#define DEBUG4
//#define DEBUG5
#define FAILSAFE
#define SPIBB
//#define SPIHW
#if defined SPIHW
    #include <SPI.h>
#endif

#define chanel_number 8  //set the number of chanels
#define SEEK_CHANSKIP   13
#define MAX_MISSING_PKT 20
#define PPM_FrLen 22500
#define PPM_PulseLen 300
#define default_servo_value 1500
#define onState 0  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 10

#if defined(SPIBB)
    #define MO_pin 5                    //D5
    #define MI_pin 6                    //D6
    #define SCLK_pin 4                  //D4
    #define CS 2                        //D2
    #define GDO_pin 3                   //D3  GDO0 pin
    #define SCK_on PORTD |= 0x10        //D4
    #define SCK_off PORTD &= 0xEF       //D4
    #define MO_on PORTD |= 0x20         //D5 
    #define MO_off PORTD &= 0xDF        //D5
    #define MI_1 (PIND & 0x40) == 0x40  //D6 input
    #define MI_0 (PIND & 0x40) == 0x00  //D6
    #define CS_on PORTD |= 0x04         //D2
    #define CS_off PORTD &= 0xFB        //D2 
    #define GDO_1 (PIND & 0x08) == 0x08 //D3 input
    #define GDO_0 (PIND & 0x08) == 0x00 //D3  
#endif

#define bind_pin A0     //C0 bind plug also servo8

#define Servo1_OUT 7    //Servo1(D7)
#define Servo2_OUT 8    //Servo2(B0)
#define Servo3_OUT 9    //Servo3(B1)
#define Servo4_OUT 10   //Servo4(B2)//PPM pin
#define Servo5_OUT 11   //Servo5(B3)
#define Servo6_OUT 12   //Servo6(B4)
#define Servo7_OUT 13   //Servo7(B5)
#define Servo8_OUT A0   //Servo8(C0)

#define Servo1_OUT_HIGH PORTD |= _BV(7) //Servo1(D7)
#define Servo2_OUT_HIGH PORTB |= _BV(0) //Servo2(B0)
#define Servo3_OUT_HIGH PORTB |= _BV(1) //Servo3(B1)
#define Servo4_OUT_HIGH PORTB |= _BV(2) //Servo4(B2)
#define Servo5_OUT_HIGH PORTB |= _BV(3) //Servo5(B3)
#define Servo6_OUT_HIGH PORTB |= _BV(4) //Servo6(B4)
#define Servo7_OUT_HIGH PORTB |= _BV(5) //Servo7(B5) 
#define Servo8_OUT_HIGH PORTC |= _BV(0) //Servo8(C0)
#define Servo_Ports_LOW PORTD &= 0x7F ; PORTB &= 0xc0; PORTC &=0xFE //all servos low

#define LED_pin A3
#define LED_ON  PORTC |= _BV(3)
#define LED_OFF  PORTC &= ~_BV(3)
#define NOP() __asm__ __volatile__("nop")
        

// Globals:
static uint8_t ccData[27];
static uint8_t ccLen;
static boolean packet = false;
//static uint16_t time;
static uint8_t channr;
static uint8_t missingPackets = 0;
uint8_t calData[60];
uint8_t hopData[60];
uint8_t listLength;
uint8_t txid[2];
static uint8_t counter = 0;
volatile uint16_t Servo_data[10] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
volatile byte scale;
static byte jumper1 = 0;
static byte jumper2 = 0;
volatile int ppm[chanel_number];
static uint16_t total_servo_time = 0;
static byte cur_chan_numb = 0;
boolean debug = false;
int count = 0;
uint16_t c[8];
boolean debug2 = true;
boolean debug3 = false;

void setup()
{

    #if defined(SPIBB)
        pinMode(MO_pin, OUTPUT);    //SI
        pinMode(MI_pin, INPUT);     //SO
        pinMode(SCLK_pin, OUTPUT);  //SCLK
        pinMode(CS, OUTPUT);        //CS output
        pinMode(GDO_pin, INPUT);    //GDO0 pin
        SCK_off;                    //start sck low
        MO_off;                     //low
    #endif
    
    pinMode(LED_pin, OUTPUT);
    CS_on;

    #if defined(SPIHW)
        pinMode(CS, OUTPUT);
        pinMode(GDO_pin, INPUT);
        SPI.setClockDivider(SPI_CLOCK_DIV2);
        SPI.setBitOrder( MSBFIRST);
        SPI.begin();
    #endif

    pinMode(Servo1_OUT, OUTPUT); //Servo1
    pinMode(Servo2_OUT, OUTPUT); //Servo2
    pinMode(Servo3_OUT, OUTPUT); //Servo3
    pinMode(Servo4_OUT, OUTPUT); //Servo4
    //
    pinMode(Servo6_OUT, OUTPUT); //Servo6
    pinMode(Servo7_OUT, OUTPUT); //Servo7
    pinMode(Servo8_OUT, OUTPUT); //Servo8
    //Servo8_OUT_HIGH;//bindpin pullup

    #if defined DEBUG
        Serial.begin(115200);
        int8_t i;
        Serial.print("PartNum ");
        i = cc2500_readReg(CC2500_30_PARTNUM + CC2500_READ_BURST);
        Serial.println(i);
        delay(10);
        Serial.print("Version ");
        i = cc2500_readReg(CC2500_31_VERSION + CC2500_READ_BURST);
        Serial.println(i);
    #endif
    
    #if F_CPU == 16000000
        scale = 2;
    #elif F_CPU == 8000000
        scale = 1;
    #else
        #error // 8 or 16MHz only !
    #endif



    initialize(1);                  //binding
    binding();
    pinMode(Servo8_OUT, OUTPUT);    //Servo8.bind pin is set to output again.
    initialize(0);                  //data
    
    jumper1 = PPM_jumper(); // Check the possible jumper positions for changing the receiver mode.
    
    if (jumper1 == 1) {
        //initiallize default ppm values
        for (int i = 0; i < chanel_number; i++) {
            ppm[i] = default_servo_value;
        }
        pinMode(sigPin, OUTPUT);
        digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
    }
    config_timer();
    channr = 0;
    cc2500_writeReg(CC2500_0A_CHANNR, hopData[channr]);//0A-hop
    cc2500_writeReg(CC2500_23_FSCAL3, 0x89); //23-89
    cc2500_strobe(CC2500_SRX);
}



void loop()
{
    unsigned long time = micros();

    #if defined(FAILSAFE)
        if (missingPackets > 170) {
            //**************************************
            //noInterrupts();//
            //digitalWrite(sigPin, LOW);
            //Servo_Ports_LOW;
            //**********************************************
            missingPackets = 0;
            int i;
            for (i = 0; i < 8; i++) {
                Servo_data[i] = 1000;
                ppm[i] = 1000;
                if (i == 2) {
                    Servo_data[2] = 1000; //THROTLE ON CHN3 here it can be changed Throttle on other channel
                    ppm[2] = 1000;
                }
            }
        }
    #endif

    while (1) {
        if ((micros() - time) > 9000) {
            missingPackets++;
            cc2500_strobe(CC2500_SIDLE);
            if (missingPackets > MAX_MISSING_PKT) {
                nextChannel(SEEK_CHANSKIP);
                LED_OFF;
                counter++;
                if (counter > (MAX_MISSING_PKT << 1))
                    LED_ON;
                if (counter == (MAX_MISSING_PKT << 2)) counter = 0;
                break;
            } else
                nextChannel(1);
            break;
        }
        if (GDO_1) {
            ccLen = cc2500_readReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
            if (ccLen > 20)
                ccLen = 20;//
            if (ccLen) {
                cc2500_readFifo((uint8_t *)ccData, ccLen);
                if (ccData[ccLen - 1] & 0x80) { // Only if correct CRC
                    missingPackets = 0;
                    if (ccData[0] == 0x11) { // Correct length
                        if ((ccData[1] == txid[0]) && (ccData[2] == txid[1])) { // Only if correct txid
                            packet = true;
                            //sei();    ///////////////////////////////////////////////////////////////////////////////////////
                            //int rssi = cc2500_readReg(CC2500_34_RSSI | CC2500_READ_BURST);//check RSSI
                            cc2500_strobe(CC2500_SIDLE);
                            nextChannel(1);
                            LED_ON;
                            break;
                        }
                    }
                }

            }

        }
    }

    if (packet == true) {
        packet = false;
        debug = true;
        //cli();
        c[0] = (uint16_t)(ccData[10] & 0x0F) << 8 | ccData[6];
        c[1] = (uint16_t)(ccData[10] & 0xF0) << 4 | ccData[7];
        c[2] = (uint16_t)(ccData[11] & 0x0F) << 8 | ccData[8];
        c[3] = (uint16_t)(ccData[11] & 0xF0) << 4 | ccData[9];
        c[4] = (uint16_t)(ccData[16] & 0x0F) << 8 | ccData[12];
        c[5] = (uint16_t)(ccData[16] & 0xF0) << 4 | ccData[13];
        c[6] = (uint16_t)(ccData[17] & 0x0F) << 8 | ccData[14];
        c[7] = (uint16_t)(ccData[17] & 0xF0) << 4 | ccData[15];
        //sei();
        for (int i = 0; i < 8; i++) {
            Servo_data[i] = 0.67 * c[i];
            if (Servo_data[i] < 900) { //added new
                Servo_data[i] = 1500; //added new
                Servo_data[2] = 1000;
            }
            ppm[i] = Servo_data[i];
        }
        #if defined(DEBUG5)
                //Serial.println(rssi);
        #endif
        #if defined(DEBUG0)
                for (int i = 0; i < 8; i++) {
                    Serial.print(" ");
                    Serial.print(Servo_data[i]);
                    Serial.print(" ");
                }
                Serial.println(" ");
        #endif
    }

    cc2500_strobe(CC2500_SRX);
    if (debug == true) {
        debug = false;
        #if defined(DEBUG2)
                Serial.println(ccData[3], HEX);
        #endif
    }
}

void initialize(int bind)
{
    cc2500_resetChip();
    cc2500_writeReg(CC2500_02_IOCFG0,   0x01);  // reg 0x02: RX complete interrupt(GDO0)
    cc2500_writeReg(CC2500_17_MCSM1,    0x0C);  // reg 0x17:
    cc2500_writeReg(CC2500_18_MCSM0,    0x18);  // reg 0x18:
    cc2500_writeReg(CC2500_06_PKTLEN,   0x19);  // Leave room for appended status bytes
    cc2500_writeReg(CC2500_08_PKTCTRL0, 0x05);  // reg 0x08:
    cc2500_writeReg(CC2500_3E_PATABLE,  0xFF);  //
    cc2500_writeReg(CC2500_0B_FSCTRL1,  0x08);  // reg 0x0B:
    cc2500_writeReg(CC2500_0C_FSCTRL0,  0x00);  // reg 0x0C
    cc2500_writeReg(CC2500_0D_FREQ2,    0x5C);  // reg 0x0D
    cc2500_writeReg(CC2500_0E_FREQ1,    0x76);  // reg 0x0E
    cc2500_writeReg(CC2500_0F_FREQ0,    0x27);  // reg 0x0F
    cc2500_writeReg(CC2500_10_MDMCFG4,  0xAA);  // reg 0x10
    cc2500_writeReg(CC2500_11_MDMCFG3,  0x39);  // reg 0x11
    cc2500_writeReg(CC2500_12_MDMCFG2,  0x11);  // reg 0x12
    cc2500_writeReg(CC2500_13_MDMCFG1,  0x23);  // reg 0x13
    cc2500_writeReg(CC2500_14_MDMCFG0,  0x7A);  // reg 0x14
    cc2500_writeReg(CC2500_15_DEVIATN,  0x42);  // reg 0x15
    cc2500_writeReg(CC2500_19_FOCCFG,   0x16);  // reg 0x16
    cc2500_writeReg(CC2500_1A_BSCFG,    0x6C);  // reg 0x1A
    cc2500_writeReg(CC2500_1B_AGCCTRL2, 0x03);  // reg 0x1B
    cc2500_writeReg(CC2500_1C_AGCCTRL1, 0x40);  // reg 0x1C
    cc2500_writeReg(CC2500_1D_AGCCTRL0, 0x91);  // reg 0x1D
    cc2500_writeReg(CC2500_21_FREND1,   0x56);  // reg 0x21:
    cc2500_writeReg(CC2500_22_FREND0,   0x10);  // reg 0x22:
    cc2500_writeReg(CC2500_23_FSCAL3,   0xA9);  // reg 0x23:
    cc2500_writeReg(CC2500_24_FSCAL2,   0x05);  // reg 0x24:
    cc2500_writeReg(CC2500_25_FSCAL1,   0x00);  // reg 0x25
    cc2500_writeReg(CC2500_26_FSCAL0,   0x11);  // reg 0x26
    cc2500_writeReg(CC2500_29_FSTEST,   0x59);  // reg 0x29
    cc2500_writeReg(CC2500_2C_TEST2,    0x88);  // reg 0x2C
    cc2500_writeReg(CC2500_2D_TEST1,    0x31);  // reg 0x2D
    cc2500_writeReg(CC2500_2E_TEST0,    0x0B);  // reg 0x2E
    cc2500_writeReg(CC2500_03_FIFOTHR,  0x0F);  // reg 0x03:
    cc2500_writeReg(CC2500_09_ADDR, bind ? 0x03 : txid[0]);
    cc2500_strobe(CC2500_SIDLE);    // Go to idle...
    cc2500_writeReg(CC2500_07_PKTCTRL1, 0x0D);  // reg 0x07 hack: Append status, filter by address, auto-flush on bad crc, PQT=0
    //cc2500_writeReg(CC2500_0C_FSCTRL0, 0);    // Frequency offset...
    cc2500_writeReg(CC2500_0C_FSCTRL0, bind ? 0x00 : count); // Frequency offset hack
    cc2500_writeReg(CC2500_0A_CHANNR, 0x00);
}
// Receives complete bind setup
void getBind(void)
{
    cc2500_strobe(CC2500_SRX);//enter in rx mode
    listLength = 0;
    boolean eol = false;
    //           len|bind |tx id|idx|h0|h1|h2|h3|h4|00|00|00|00|00|00|01
    // Start by getting bind packet 0 and the txid
    //        0  1   2  txid0(3) txid1()4    5  6  7   8  9 10 11 12 13 14 15 16 17
    //ccdata    //11 03 01  d7       2d          00 00 1e 3c 5b 78 00 00 00 00 00 00 01
    //11 03 01  19       3e          00 02 8e 2f bb 5c 00 00 00 00 00 00 01
    while (1) {
        if (GDO_1) {
            ccLen = cc2500_readReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
            if (ccLen) {
                cc2500_readFifo((uint8_t *)ccData, ccLen);
                if (ccData[ccLen - 1] & 0x80) {
                    if (ccData[2] == 0x01) {
                        if (ccData[5] == 0x00) {
                            txid[0] = ccData[3];
                            txid[1] = ccData[4];
                            for (uint8_t n = 0; n < 5; n++) {
                                hopData[ccData[5] + n] = ccData[6 + n];
                            }
                            break;
                        }
                    }
                }
            }
        }
    }

    #if defined(DEBUG)
        Serial.print(txid[0], HEX);
        Serial.println(txid[1], HEX);
    #endif

    for (uint8_t bindIdx = 0x05; bindIdx <= 120; bindIdx += 5) {
        while (1) {
            if (GDO_1) {
                ccLen = cc2500_readReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
                if (ccLen) {
                    cc2500_readFifo((uint8_t *)ccData, ccLen);
                    if (ccData[ccLen - 1] & 0x80) {
                        if (ccData[2] == 0x01) {
                          if(debug3) {
                            Serial.print("ccLen = ");
                            Serial.println(ccLen);
                          }
                            if ((ccData[3] == txid[0]) && (ccData[4] == txid[1])) {
                              if(debug3) 
                              {
                                Serial.print("ccData[5] = ");
                                Serial.println(ccData[5]);
                                Serial.print("bindIdx = ");
                                Serial.println(bindIdx);                      
                              }
                                if (ccData[5] == bindIdx) {
                                    for (uint8_t n = 0; n < 5; n++) {
                                      if(debug3) 
                                      {
                                        Serial.print("ccData[6 + n] = ");
                                        Serial.println(ccData[6 + n]);
                                        Serial.print("ccData[ccLen - 3] = ");
                                        Serial.println(ccData[ccLen - 3]);                      
                                      }
                                        //if (ccData[6 + n] == ccData[ccLen - 3]) {
                                          if (ccData[6 + n] <= 3) {
                                            eol = true;
                                            #if defined(DEBUG)
                                                Serial.print("listLength: ");
                                                Serial.println(listLength);
                                            #endif
                                            listLength = ccData[5] + n;
                                            break;
                                        }
                                        hopData[ccData[5] + n] = ccData[6 + n];
                                    }
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
        #if defined(DEBUG)
                Serial.println(bindIdx / 5);
        #endif
        if (eol) break; // End of list found, stop!
    }

    #if defined(DEBUG)
        listLength = 47;
        Serial.println("jumpIdx list: ");
        for (uint8_t jumpIdx = 0; jumpIdx < (listLength); jumpIdx++) {
            Serial.print(" ");
            Serial.print(hopData[jumpIdx], HEX);
            Serial.print(" ");
        }
        Serial.println(" ");
    #endif

    Store_bind();
    cc2500_strobe(CC2500_SIDLE);    // Back to idle
}

ISR(TIMER1_COMPA_vect)
{
    TCNT1 = 0;
    if (jumper1 == 0) {
        pinMode(Servo5_OUT, OUTPUT);
        Servo_Ports_LOW;
        //code for servo.
        cur_chan_numb++; //next servo
        if (cur_chan_numb < chanel_number) {
            total_servo_time += Servo_data[cur_chan_numb] * scale;
            OCR1A = Servo_data[cur_chan_numb] * scale;
        } else {
            OCR1A = PPM_FrLen * scale - total_servo_time;
            cur_chan_numb = 0xff;
            total_servo_time = 0;
        }

        switch (cur_chan_numb) {
        case 0:
            Servo1_OUT_HIGH;
            break;
        case 1:
            Servo2_OUT_HIGH;
            break;
        case 2:
            Servo3_OUT_HIGH;
            break;
        case 3:
            Servo4_OUT_HIGH;
            break;
        case 4:
            Servo5_OUT_HIGH;
            break;
        case 5:
            Servo6_OUT_HIGH;
            break;
        case 6:
            Servo7_OUT_HIGH;
            break;
        case 7:
            Servo8_OUT_HIGH;
            break;
        }
    } else {
        static boolean state = true;
        pinMode(sigPin, OUTPUT);
        digitalWrite(sigPin, !onState);

        if (state) {
            digitalWrite(sigPin, onState);
            OCR1A = PPM_PulseLen * scale;
            state = false;
        } else {
            static byte cur_chan_numb;
            static unsigned int calc_rest;
            // digitalWrite(sigPin, !onState);//PPM on servo4 pin10
            state = true;

            if (cur_chan_numb >= chanel_number) {
                cur_chan_numb = 0;
                calc_rest = calc_rest + PPM_PulseLen;//
                OCR1A = (PPM_FrLen - calc_rest) * scale;
                calc_rest = 0;
            } else {
                OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * scale;
                calc_rest = calc_rest + ppm[cur_chan_numb];
                cur_chan_numb++;
            }
        }
    }
}

void config_timer()
{
    OCR1A = 50 * scale;
    cli();
    TCCR1A = 0; //
    TCCR1B = 0;
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS11);
    TIMSK1 |= (1 << OCIE1A);
    sei();
}

void nextChannel(uint8_t skip)
{
    channr += skip;//
    if (channr >= listLength) channr -= listLength;
    cc2500_writeReg(CC2500_0A_CHANNR, hopData[channr]);
    cc2500_writeReg(CC2500_23_FSCAL3, 0x89);

}

void binding()
{
    jumper2 = bind_jumper();
    while (1) {
        if (jumper2 == 0) { //bind complete or no bind
            uint8_t i;
            uint8_t adr = 100;
            for (i = 0; i < 2; i++) {
                txid[i] = EEPROM.read(adr + i);
            }
            if (txid[0] == 0xff && txid[1] == 0xff) {
                // No valid txid, forcing bind
                jumper2 = 1;
                continue;
            }
            for (i = 0; i < sizeof(hopData); i++) {
                hopData[i] = EEPROM.read(adr + 10 + i);
            }
            listLength = EEPROM.read(adr + 100);
            count = EEPROM.read(adr + 101);
            break;
        } else {
            LED_ON;
            tunning();
            //count=0xC8;//for test
            cc2500_writeReg(CC2500_0C_FSCTRL0, count);
            int adr = 100;
            EEPROM.write(adr + 101, count);
            getBind();
            while (1) {
                LED_ON;
                delay(500);
                LED_OFF;
                delay(500);
            }
        }
    }
}


void tunning()
{
    cc2500_strobe(CC2500_SRX);//enter in rx mode
    int count1 = 0;
    while (1) {
        count1++;
        if (count >= 250) {
            count = 0;
        }
        if (count1 > 3000) {
            count1 = 0;
            cc2500_writeReg(CC2500_0C_FSCTRL0, count);  // Frequency offset hack
            count = count + 10;
            //cc2500_strobe(CC2500_SRX);//enter in rx mode
        }
        if (GDO_1) {
            ccLen = cc2500_readReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
            if (ccLen) {
                cc2500_readFifo((uint8_t *)ccData, ccLen);
                if (ccData[ccLen - 1] & 0x80) {
                    if (ccData[2] == 0x01) {
                        if (ccData[5] == 0x00) {
                            break;
                        }
                    }
                }
            }
        }
    }
    #if defined(DEBUG1)
        Serial.println(count, HEX);
    #endif
}



void Store_bind()
{
    uint8_t i;
    int adr = 100;
    for (i = 0; i < 2; i++) {
        EEPROM.write(adr + i, txid[i]);
    }
    for (i = 0; i < sizeof(hopData); i++) {
        EEPROM.write(adr + 10 + i, hopData[i]);
    }
    EEPROM.write(adr + 100, listLength);
}

unsigned char PPM_jumper(void)
{
    // PPM Selection (jumper between Ch1 and ch3)
    pinMode(Servo3_OUT, INPUT); //CH3 input
    digitalWrite(Servo3_OUT, HIGH); // pull up
    digitalWrite(Servo1_OUT, HIGH); // CH1 is HIGH
    delayMicroseconds(1);
    if ( digitalRead(Servo3_OUT) == HIGH) {
        digitalWrite(Servo1_OUT, LOW); // CH1 is LOW
        delayMicroseconds(1);
        if (digitalRead(Servo3_OUT) == LOW) { // OK jumper plugged
            pinMode(Servo3_OUT, OUTPUT);
            return  1;
        }
    }
    pinMode(Servo3_OUT, OUTPUT);

    return  0; // servo PWM by default
}

//bind jumper
unsigned char bind_jumper(void)
{
    pinMode(bind_pin, INPUT_PULLUP);//pull up
    if ( digitalRead(bind_pin) == LOW) {
        delayMicroseconds(1);
        return 1;
    }
    return  0;
}
