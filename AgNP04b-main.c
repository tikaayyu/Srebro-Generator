// Silver Nano Particle Generator NP-03
// uses MSP430FR5739(16K FRAM, 1K SRAM) and MOSFETs to switch in resistors to control current
// this version uses a resistor ladder to set the current and a LCD display
//
// P1.0 and P1.1 control switching high side power
// P2.3 and P2.4 control switching to the gnd size resistor banks

//P1.2 (bit 12 NU) LCD reset
//P3.0 bit 11
//P3.1 bit 10
//P3.2 bit 9
//P3.3 bit 8
//P1.3 bit7
//PJ.0 bit6
//PJ.1 bit5
//PJ.2 bit4
//P3.4 bit3
//P3.5 bit2
//P3.6 bit1
//P3.7 bit0

//P2.0,1,2 are switch inputs
//P1.4 ADC input for measuring current across 20 ohm resistor
//P2.7 buzzer output

//calculating PPM. (assumes 100% efficient and 250mL batch)
//  -sum ADC every 1/64 of a minute (to 32 bit value)
//  -multiple by 0.00030678 (example: X20 then divid by 2^16)

#include <msp430.h> 
#include <stdint.h>
#include <stdbool.h>

//-----------[ Function Prototypes ]--------------
void delay_ms(unsigned int xdms);
uint16_t meas(void);
void ledsOff(void);
void setRladder(unsigned int rLadd);     //turn on specified LED #
void ledsSeq(unsigned int dyms);    //LED sequence test
void buzz (unsigned int timems);
void i2c_write(uint8_t addr7, uint8_t *data, uint8_t count);
void numL2_ascii(int16_t vp);
void intLCD(void);
void updateLCD(void);

//*********************************************
#define cM  24                //24MHz clock rate of MSP430

//use 20 ohm sense resistor.  for 10mA = 0.2V(ADC=136.5) 5mA-0.1V, 2mA=0.04V(ADC=27.3)  for 0.5mA=0.01V(ADC=6.827)
// for 0.5mA increments (only allow setting from 2mA(4) to 10mA(20)
//                  0,1,2,3, 4, 5, 6, 7, 8, 9, 10,11,12,13,14, 15, 16, 17, 18, 19, 20, 21
const uint8_t thres[22]= {0,0,0,21,27,34,41,48,55,61,68,75,82,89,96,102,109,116,123,130,137,143};

#pragma PERSISTENT ( thres_mA ) //save result after power down
uint8_t thres_mA=16;            //defaults to 8mA
#pragma PERSISTENT ( minsPol )  //save result after power down
uint8_t minsPol=8;              //# of minutes before switching polarity
#pragma PERSISTENT ( tPPM )   //save result after power down
uint8_t tPPM = 30;            //default target for PPM of AgNP assuming 100% eff
uint8_t mPPM=0;               //calculated PPM

uint16_t adc;                   //averaged ADC result (2 bytes)
uint32_t sumADC=0;              //sum of ADC for estimating the PPM
uint8_t runM=1;                 //setting for minutes of run time
uint16_t cntMain=1;             //count main loops (64 times/min)(used only by watchdog to verify operation)
uint16_t cntMainPrev=0;
uint16_t timePol1;              //based on 64 counts per min 21*64=1344 max count
uint16_t timePol2;
uint8_t flgPol, flgPolPrev;     //polarity flag for use in polarity switching
uint16_t cnt=1;                 //count for polarity changes
uint16_t cntMinsA=0;            //minutes to reach target current(64 increment=1min) (1024mins max)
uint16_t cntMinsB=0;            //minutes of Constant current (64 increment=1min) (1024mins max)
uint16_t rLadder=0xF00;         //12 bit value: starting value
uint8_t flgCur=0;               //flag, counter for number of times current is tuned
uint8_t pC[4];                  //characters for display
uint8_t lcdData[200];           //LCD data: max size 4*20*2=160
uint8_t lcdTxIndex = 0;         //index value into the lcdData array
uint8_t lcdTxSize = 0;          //size of the lcd array being transmitted
uint8_t *lcdDataPtr;
uint8_t mode=1;                 //modes 1=start?, 1=ma?, 3=polM, 4=minLimit, 5=run, 6=done

//#define wdt_start      (WDTPW + WDTTMSEL + WDTCNTCL + WDTSSEL__VLO + WDTIS__32K)  //  *** WATCHDOG TIMER MODE!!!!!  100us*WDTIS__32K= 3.2secs (for test)
#define wdt_start    (WDTPW + WDTTMSEL + WDTCNTCL + WDTSSEL__VLO + WDTIS__512K)     //  *** WATCHDOG TIMER MODE!!!!!  100us*WDTIS__512K= 51.2secs


int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    //set clock for 24MHz
    // FRAM wait states for the FR5739 are inserted automatically
/*    CSCTL0_H = 0xA5;
    CSCTL1 |= DCORSEL + DCOFSEL0 + DCOFSEL1;   // Set max. DCO setting
    CSCTL2 = SELA_3 + SELS_3 + SELM_3;        // set ACLK = MCLK = DCO
    CSCTL3 = DIVA_0 + DIVS_0 + DIVM_0;        // set all dividers to 0
*/
    //configure for 24 MHz external oscillator PJ.4(XIN), PJ.5(XOUT)
    PJSEL0 |=  BIT5 | BIT4;     //set PJ.5 as xout and PJ.4 as xin
    CSCTL0_H = 0xA5;                            //password
    CSCTL1 |= DCORSEL + DCOFSEL0 + DCOFSEL1;   // Set max. DCO setting
    CSCTL2 = SELA_0 + SELS_0 + SELM_0;        // set all clk sources for XT1
    CSCTL3 = DIVA_0 + DIVS_0 + DIVM_0;        // set all dividers to 0
    CSCTL4 |= XT1DRIVE_3 + XTS + XT1BYPASS;   //highest drive, high freq mode, external clk
    //wait for clock to start
    //clear XT1 and DCO fault flag and interrupt
    do{CSCTL5 &= ~XT1OFFG; SFRIFG1 &= ~OFIFG;}
    while (SFRIFG1 & OFIFG);

    WDTCTL = wdt_start; // start the Watchdog

    // PORT 1
    P1DIR = 0b00101111;     //P1.0 -P1.3 and P1.5 output  (P1.2 is LCD reset)
    P1SEL1 |= BIT6 | BIT7;  // Configure I2C pins for LCD: P1.6 = SDA, P1.7 = SCL

    // PORT 2
    P2DIR = 0b11111000;             //outputs (bit2,1,0 inputs for switches
    //P2DIR = 0b11111100;  //P2.2 is temp LCD reset
    P2REN |=     BIT2|BIT1|BIT0;    //P2.0,1,2 pull-up register enable
    P2OUT = 0x00|BIT2|BIT1|BIT0;    //all low expect BIT0,1,2 set for pull up resistor
    P2SEL0 &=  ~(BIT2|BIT1|BIT0);   // Select GPIO function
    P2SEL1 &=  ~(BIT2|BIT1|BIT0);
    P2IES |=    (BIT2|BIT1|BIT0);   // Interrupt edge select: falling-edge detect   1 = High-to-Low, 0 = Low-to-High
    P2IFG &=   ~(BIT2|BIT1|BIT0);   // Clear any pending IFG
    P2IE |=     (BIT2|BIT1|BIT0);   // Enable interrupts for P2.0, P2.1, P2.2

    // PORT 3 (all outputs)
    P3DIR = 0b11111111;     //
    P3OUT = 0x00;           //all low
    //PORT4
    P4DIR = 0b00000011;     //outputs  (only used)
    P4OUT = 0x00;           //all low
    //PORTJ
    PJDIR = 0b00001111;     //outputs (J.0,J.1,J2 used in resistor ladder)
    PJOUT = 0x00;           //all low

    // Configure ADC10
    // By default, REFMSTR=1 => REFCTL is used to configure the internal reference
     while(REFCTL0 & REFGENBUSY);              // If ref generator busy, WAIT
     //REFCTL0 |= REFVSEL_2+REFON;               // Select internal ref = 2.5V. Internal Reference ON
     REFCTL0 |= REFVSEL_0+REFON;               // Select internal ref = 1.5V. Internal Reference ON
     ADC10CTL0 = ADC10SHT_2 + ADC10ON;
     ADC10CTL1 |= ADC10SHP;                    // ADCCLK = MODOSC; sampling timer
     ADC10CTL2 |= ADC10RES;                    // 10-bit conversion results
 //    ADC10IE |= ADC10IE0;                      // Enable ADC conv complete interrupt (routine now uses polling)
     ADC10MCTL0 |= ADC10INCH_4 + ADC10SREF_1;  // A0 ADC input select; Vref=1.5V as set by REFVSEL_0

    // Configure
    PMMCTL0_H = PMMPW_H;                        // Unlock the PMM registers
    PM5CTL0 &= ~LOCKLPM5;                       // Disable the GPIO power-on default high-impedance mode

    //I2C configuration
    UCB0CTLW0 = UCSWRST;                      // Enable SW reset
    UCB0CTLW0 |= UCMODE_3 | UCMST | UCSSEL__SMCLK | UCSYNC; // I2C master mode, SMCLK
    UCB0BRW = 60;                            // fSCL = SMCLK/60 = ~400kHz
    UCB0I2CSA = 0x7A;                       // Slave Address  (0x7A with write bit)
    UCB0IE |= UCTXIE;                        // Enable TX buffer empty interrupt
    UCB0CTLW0 &= ~UCSWRST;                    // Clear SW reset, resume operation
  //  UCB0IE |= UCNACKIE;                     //Enable NACK interrupt

    __enable_interrupt();

    P1OUT &=~BIT1;  P2OUT &=~BIT4;  //turn off lower path
    P1OUT &=~BIT0;  P2OUT &=~BIT3;  //turn off top path
    ledsSeq(200);  //test LEDd
    intLCD();   //initialize LCD

    while (mode<=4) {
         //this loop if so doing the setup before entering run mode (5)
         updateLCD();    //update LCD
         delay_ms(100);
    }
    P2IE &= ~(BIT2|BIT1|BIT0);   // Disable interrupts for P2.0, P2.1, P2.2 (Button presses)

    ledsOff(); //turn off all the LEDs
    buzz(1000);  //buzz the buzzer for power on test

    timePol1=minsPol<<6;   //time for polarity 1 (based on 64 times per minute interval count)
    timePol2=timePol1*2;  //time or other polarity (ie elapsed same time, twice the counter)


    setRladder(rLadder);   //initial setup
    P1OUT &=~BIT1;  P2OUT &=~BIT4;  //turn off lower path
    P1OUT |= BIT0;  P2OUT |= BIT3;  //turn on top path
    flgPol=0;       //forward polarity

    // **************************** MAIN LOOP **********************************
    while(mPPM<=tPPM) //only works up to the limit of minutes
    {
        if (minsPol==21) {cnt=0;} //only reverse polarity if not set to 21 minutes for polarity reversal

        if (cnt<=(timePol1))   {flgPol=0;}  //forward polarity
        else if (cnt<timePol2) {flgPol=1;}  //reverse polarity
        else {cnt=0;}                       //reset counter

        if (flgPol!=flgPolPrev){  //only change settings when required to avoid small glitches in power
            if (flgPol==0){
                // P1.0 and P1.1 control switching high side power
                // P2.3 and P2.4 control switching to the gnd side resistor banks
                P1OUT &=~BIT1;  P2OUT &=~BIT4;  //turn off lower path
                P1OUT |= BIT0;  P2OUT |= BIT3;  //turn on top path
            }
            if (flgPol==1){
                P1OUT &=~BIT0;  P2OUT &=~BIT3;  //turn off top path
                P1OUT |= BIT1;  P2OUT |= BIT4;  //turn on lower path
            }
            //buzz(60);   //chirp to indicate polarity change
            flgPolPrev=flgPol;
        }

        cnt++;  //increment polarity counter (64 times per minute)
        if (flgCur==0) {
            cntMinsA++;   //increment 1/64 minute counter for current to reach target
        } else {
            cntMinsB++;   //increment 1/64 minute counter for constant current
        }
        runM = cntMinsB>>6;  //divid by 64 for minutes

        delay_ms((60000/64)-10);  //1/64 of 1 min, minus 10ms for the display update each loop
        adc =meas();
        sumADC=sumADC+adc;   //sum every 1/64 min reading to allow computing PPM

        if (adc>thres[thres_mA]) {
            flgCur++; // flg to indicate Current reach target, then counts number of times tuned
            while (adc>(thres[thres_mA]-4)){//fast adjustment loop until ~0.3mA below target
                rLadder--;      //increase the resistor value by lowering the resistor ladder count
                if (rLadder<=1) {break;}
                setRladder(rLadder); //set the resistor values
                adc =meas();
            }
        }

        //fix to allow the current to increase due to adding citric acid
        if (adc<(thres[thres_mA]-4)) {
            while (adc<(thres[thres_mA]-4)){//fast adjustment loop until ~0.3mA below target
                rLadder++;      //lower the resistor value by increasing the resistor ladder count
                if (rLadder>=0xF00) {rLadder=0xF00; break;}
                setRladder(rLadder); //set the resistor values
                adc =meas();
            }
        }

        updateLCD();    //update LCD
        cntMain++;      //increment in order to verify program working.

        //check for manual exit request
        if ((P2IN&BIT2)==0b00000000){
            //down button press detected
            buzz(500);
            break;
        }

    }//main while

    //done so turn off the unit power to the electrodes
    P1OUT &=~BIT1;  P2OUT &=~BIT4;  //turn off lower path
    P1OUT &=~BIT0;  P2OUT &=~BIT3;  //turn off top path

    buzz(3000);  //buzz the buzzer for 3000ms
    delay_ms(1000);
    buzz(3000);  //buzz the buzzer for 3000ms

    mode=6;                         //change the mode to done
    updateLCD();                    //update LCD
    P2IE &= ~(BIT0 | BIT1 | BIT2);  // Disable interrupts for P2.0, P2.1, P2.2
    while (1){
        //check for reset
       if ((P2IN&BIT2)==0b00000000){
           //down button press detected
           buzz(500);
           PMMCTL0 |= PMMSWBOR;    //cause a software brown out reset
       }
    }

    return 0;
}

//********************* Interrupt Routines ********************************
// ***************************************************************


// **********************************************************
// *******************   WDT INTERRUPT **********************
// Watchdog Timer interrupt service routine
#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
{
    //verify program working by checking cntMain is incrementing
    if (cntMain==cntMainPrev) {
        //MCU hung, so MCU needs reboot
        PMMCTL0 |= PMMSWBOR;    //cause a software brown out reset
    } else {
        cntMainPrev=cntMain;
    }
}//WD interrupt



// =========================
//   PORT 2 ISR HANDLER
// =========================
// for handling button presses
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
    // Check which pin caused the interrupt
    if (P2IFG & BIT0)
    {
        //mode button pressed, so cycle mode function
        //temp is is up button on test board
        mode++;
        if (mode>4) mode=1;
        P2IFG &= ~BIT0;   // clear flag
    }
    if (P2IFG & BIT1)
    {
        //Up button pressed, so increment right function depending on mode
        if (mode==1) {
            mode=5;  //switch to running mode
        }
        if (mode==2){  //set mA
            thres_mA++;
            if (thres_mA>21) thres_mA=4;
        }
        if (mode==3){  //set polarity mins
            minsPol++;
            if (minsPol>21) minsPol=1;
        }
        if (mode==4){  //set end mins
            tPPM++; //tPPM++;
            if (tPPM>99) tPPM=0;
        }
        P2IFG &= ~BIT1;
    }
    if (P2IFG & BIT2)
    {
        //Down button pressed, so decrement right function depending on mode
        if (mode==1) {
            //do nothing
        }
        if (mode==2){  //set mA
            thres_mA--;
            if (thres_mA<4) thres_mA=21;
        }
        if (mode==3){  //set polarity mins
            minsPol--;
            if (minsPol==0) minsPol=21;
        }
        if (mode==4){  //set end mins
            tPPM--; //tPPM--;
            if (tPPM<1) tPPM=99;
        }
        P2IFG &= ~BIT2;
    }

    delay_ms(200); //debounce
}



// *******************  I2C **********************
#pragma vector=USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
    switch(__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG))
    {
        case USCI_NONE:
            break;
        case USCI_I2C_UCRXIFG0:  // received a byte from Pi
            break;
        case USCI_I2C_UCTXIFG0:        // TX buffer ready
            if (lcdTxIndex < lcdTxSize) {
                UCB0TXBUF = lcdDataPtr[lcdTxIndex++];
            } else {
                UCB0CTLW0 |= UCTXSTP;     // Generate STOP
                UCB0IE &= ~UCTXIE;        // Disable TX interrupt
            }
            break;
        case USCI_I2C_UCSTPIFG:        // STOP detected (not always triggered on RPi)
            UCB0IFG &= ~UCSTPIFG;      // clear STOP flag
            break;
        default:
            break;
    }
}

//**************  multi byte write I2C  ******************
void i2c_write(uint8_t addr7, uint8_t *data, uint8_t count)
{
    while (UCB0CTLW0 & UCTXSTP);     // Wait for previous STOP

    lcdDataPtr = data;
    lcdTxSize  = count;
    lcdTxIndex = 0;

    UCB0I2CSA = addr7;               // Set 7-bit address
    UCB0CTLW0 |= UCTR | UCTXSTT;     // Start + write
    UCB0IE |= UCTXIE;                // Enable TX interrupt
}




//****************** turn LEDs off ******************
void ledsOff(void)
{
    //turn off all the LEDs (12)
     P1OUT &= ~BIT3;
     P3OUT &= 0x00;
     PJOUT &= ~(BIT0 | BIT1 | BIT2);
}

//****************** set the resistor ladder value ******************
//Based on resistor ladder
void setRladder(unsigned int rLadd)
{
    //turn on the correct LED
    if (rLadd&BIT0) {P3OUT |= BIT7;} else {P3OUT &= ~ BIT7;}   //0
    if (rLadd&BIT1) {P3OUT |= BIT6;} else {P3OUT &= ~ BIT6;}   //1
    if (rLadd&BIT2) {P3OUT |= BIT5;} else {P3OUT &= ~ BIT5;}   //2
    if (rLadd&BIT3) {P3OUT |= BIT4;} else {P3OUT &= ~ BIT4;}   //3
    if (rLadd&BIT4) {PJOUT |= BIT2;} else {PJOUT &= ~ BIT2;}   //4
    if (rLadd&BIT5) {PJOUT |= BIT1;} else {PJOUT &= ~ BIT1;}   //5
    if (rLadd&BIT6) {PJOUT |= BIT0;} else {PJOUT &= ~ BIT0;}   //6
    if (rLadd&BIT7) {P1OUT |= BIT3;} else {P1OUT &= ~ BIT3;}   //7
    if (rLadd&BIT8) {P3OUT |= BIT3;} else {P3OUT &= ~ BIT3;}   //8
    if (rLadd&BIT9) {P3OUT |= BIT2;} else {P3OUT &= ~ BIT2;}   //9
    if (rLadd&BITA) {P3OUT |= BIT1;} else {P3OUT &= ~ BIT1;}   //10
    if (rLadd&BITB) {P3OUT |= BIT0;} else {P3OUT &= ~ BIT0;}   //11
}

//****************** LED sequence test ******************
void ledsSeq(unsigned int dyms)
{
    //test LEDs on power up in sequence
    ledsOff();
    P3OUT |= BIT7; delay_ms(dyms);  //0
    P3OUT |= BIT6; delay_ms(dyms);  //1
    P3OUT |= BIT5; delay_ms(dyms);  //2
    P3OUT |= BIT4; delay_ms(dyms);  //2
    PJOUT |= BIT2; delay_ms(dyms);  //4
    PJOUT |= BIT1; delay_ms(dyms);  //5
    PJOUT |= BIT0; delay_ms(dyms);  //6
    P1OUT |= BIT3; delay_ms(dyms);  //7
    P3OUT |= BIT3; delay_ms(dyms);  //8
    P3OUT |= BIT2; delay_ms(dyms);  //9
    P3OUT |= BIT1; delay_ms(dyms);  //10
    P3OUT |= BIT0; delay_ms(dyms);  //11
}

//****************** delay_ms ******************
void delay_ms(unsigned int xdms)
{
    unsigned int xi;
    for (xi = xdms; xi > 0; xi--) {
        __delay_cycles((cM*1000)-10);   //1ms delay minus 10 cycles for loop
    }
}


//****************** delay_ms ******************
void buzz(unsigned int timems)
{
    uint16_t bz;              // for buzzer
    uint16_t time=timems<<2;  //multiple x4, means 1000 would beep for 1 second

    for (bz = time; bz > 0; bz--) {
        P2OUT ^= BIT7;  //toogle line for buzzer
        __delay_cycles(cM*125);
    }
}
//****************** measure ADC ******************
uint16_t meas(void)
{
    uint32_t Vt=0;     //32bit allows for sum of 128 averages
    uint16_t b;        //for next loop
    uint16_t resultADC;//ADC result (2 bytes)

    //Total  2.7ms measured (4clk sample time, 256 averaging, no sleep mode)
    //ADC10CLK ~5MHz(0.2us) : conversion time ~ 2.5us for 12 cycles.
    ADC10CTL0 &= ~ADC10ENC;                 //disable ADC conversion, to allow changing channel in register
    ADC10CTL0 = ADC10SHT_2 | ADC10ON;       // ADCON, S&H=4 ADC clks
    ADC10MCTL0 = ADC10INCH_4 | ADC10SREF_1; // A4 and 1.5V reference as set by REFVSEL_0
    for (b = 128; b > 0; b--) {
        ADC10CTL0 |= ADC10ENC + ADC10SC;    // Sampling and conversion start
        while ((ADC10IFG&ADC10IFG0)==0);    //WAIT for flag set
        resultADC=ADC10MEM0;                //read result also clears flag
        Vt= Vt + resultADC;                 // save previous ADC reading (2bytes)
    }
    Vt=Vt>>7;       //divid by 128 for averaging
    if (Vt>999) Vt=999;   //assign limit
    return Vt;
}


// ---------------------- ASCII characters for LCD display ----------------------------
// Output is in char array pC[]
void numL2_ascii(int16_t vp){
    //max number 999
    int16_t t16;                    //temporary 32 bit holder

    t16=vp;

    // do 100
    pC[3]=0;
    while (t16>=100){
        t16 = t16 - 100;
        if (t16>=0) pC[3]++;
    }
    pC[3]=pC[3]+0x30; //convert to ascii by adding 0x30

    // do 10
    pC[2]=0;
    while (t16>=10){
        t16 = t16 - 10;
        if (t16>=0) pC[2]++;
    }
    pC[2]=pC[2]+0x30; //convert to ascii by adding 0x30
    // do 1
    pC[1] = t16 + 0x30;   //remainder + convert to ascii by adding 0x30
}

// ---------------------- initialize LCD display ----------------------------
void intLCD(void){
    uint8_t x;                      // for loop

    // data sheet says keep low for >=5ms
    for (x = 50; x > 0; x--) {
        __delay_cycles(cM*1000);   //1ms delay
    }
    // reset the LCD display using P1.2
    P1OUT &= ~BIT2;    //force low to reset display
    // data sheet says keep low for >=10ms
    for (x = 10; x > 0; x--) {
        __delay_cycles(cM*1000);   //1ms delay
    }
    P1OUT |= BIT2;      //return to high state
    __delay_cycles(cM*1000);     //delay 1ms (data sheets says >=1ms)

    uint8_t j = 0;
    lcdData[j++]=0x80; lcdData[j++]=0x3A; //control byte ,Function Set: 8 bit data length extension Bet RE=1; REV=0
    lcdData[j++]=0x80; lcdData[j++]=0x09;  //control byte, Extended function set: 4 line Display
    lcdData[j++]=0x80; lcdData[j++]=0x06;  //control byte, Entry Mode Set: bottom view
    lcdData[j++]=0x80; lcdData[j++]=0x1E;  //control byte, Bias Setting: BS1=1
    lcdData[j++]=0x80; lcdData[j++]=0x39;  //control byte, Function Set: 8 bit data length extension Bit RE=0, IS=1
    lcdData[j++]=0x80; lcdData[j++]=0x1B;  //control byte, Internal OSC: BS0=1 -> Bias 1/6
    lcdData[j++]=0x80; lcdData[j++]=0x6E;  //control byte, Follower Control: Divider on and set value
    lcdData[j++]=0x80; lcdData[j++]=0x57;  //control byte, Power Control: Booster on and set contrast (DB1=C5, DB0=C4)
    lcdData[j++]=0x80; lcdData[j++]=0x72;  //control byte, Contrast Set: Set Contrast (DB3-DB0=C3-C0)
    lcdData[j++]=0x80; lcdData[j++]=0x38;  //control byte, Function Set: 8 bet data length extension Bit RE=0; IS=0
    lcdData[j++]=0x80; lcdData[j++]=0x0F;  //control byte, Display on: cursor on, blink on
    lcdData[j++]=0x00; lcdData[j++]=0x01;  //last one  (no more continuation), Display on: cursor on, blink on

    // Send LCD init sequence
    i2c_write(0x3D, lcdData, j);
    delay_ms(10);  //time to finish I2C
}


// ---------------------- update LCD display ----------------------------
void updateLCD(void){
    unsigned char a1,a2,m1,m2,t1,t2,t3,tmp8;     //ascii display values
    switch(thres_mA) {
        case 4: a2='2'; a1='0'; break;  //2.0mA
        case 5: a2='2'; a1='5'; break;  //2.5mA
        case 6: a2='3'; a1='0'; break;  //3.0mA
        case 7: a2='3'; a1='5'; break;  //3.5mA
        case 8: a2='4'; a1='0'; break;  //4.0mA
        case 9: a2='4'; a1='5'; break;  //4.5mA
        case 10: a2='5'; a1='0'; break;  //5.0mA
        case 11: a2='5'; a1='5'; break;  //5.5mA
        case 12: a2='6'; a1='0'; break;  //6.0mA
        case 13: a2='6'; a1='5'; break;  //6.5mA
        case 14: a2='7'; a1='0'; break;  //7.0mA
        case 15: a2='7'; a1='5'; break;  //7.5mA
        case 16: a2='8'; a1='0'; break;  //8.0mA
        case 17: a2='8'; a1='5'; break;  //8.5mA
        case 18: a2='9'; a1='0'; break;  //9.0mA
        case 19: a2='9'; a1='5'; break;  //9.5mA
        case 20: a2='9'; a1='9'; break;  //10.0mA
        default: a2=' '; a1=' '; break;
     }//sw

    numL2_ascii(minsPol); //minutes for polarity switching
    m1=pC[1];
    m2=pC[2];
    numL2_ascii(tPPM);  //stop limit number
    t3=pC[3];
    t2=pC[2];
    t1=pC[1];

    uint8_t i = 0;
    lcdData[i++] = 0x80;
    lcdData[i++] = 0x01;
    lcdData[i++] = 0x80 | 0x40; lcdData[i++] = a2;
    lcdData[i++] = 0x80 | 0x40; lcdData[i++] = '.';
    lcdData[i++] = 0x80 | 0x40; lcdData[i++] = a1;
    lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'm';
    lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'A';
    lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
    lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'P';
    lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'o';
    lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'l';
    lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'M';
    lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ':';
    lcdData[i++] = 0x80 | 0x40; lcdData[i++] = m2;
    lcdData[i++] = 0x80 | 0x40; lcdData[i++] = m1;
    lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
    lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'P';
    lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'P';
    lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'M';
    lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ':';
    lcdData[i++] = 0x80 | 0x40; lcdData[i++] = t2;
    lcdData[i++] = 0x80 | 0x40; lcdData[i++] = t1;

    if (mode==1){
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'S';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 't';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'a';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'r';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 't';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = '?';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'P';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'r';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'e';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 's';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 's';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'u';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'p';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
    }
    if (mode==2){
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'S';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'e';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 't';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'm';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'A';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
    }
    if (mode==3){
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'S';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'e';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 't';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'P';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'o';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'l';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'a';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'r';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'i';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 't';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'y';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'M';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'i';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'n';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 's';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
    }
    if (mode==4){
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'S';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'e';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 't';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'E';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'n';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'd';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'P';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'P';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'M';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
    }
    //running mode
    if (mode==5){
        numL2_ascii(cntMinsA>>6);  //convert mins to reach steady state
        m1=pC[1];
        m2=pC[2];
        numL2_ascii(adc);  //convert ADC reading
        t3=pC[3];
        t2=pC[2];
        t1=pC[1];
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'R';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'a';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'm';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'p';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'M';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'i';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'n';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 's';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ':';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = m2;
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = m1;
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'A';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'D';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'C';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ':';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = t3;
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = t2;
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = t1;

        numL2_ascii(cntMinsB>>6);  //convert minutes reading
        t3=pC[3];
        t2=pC[2];
        t1=pC[1];
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'C';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'C';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'M';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'i';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'n';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 's';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ':';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = t3;
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = t2;
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = t1;
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'P';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'P';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'M';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ':';
        mPPM=(20*sumADC)>>16;   //20/65536=0.0003 (100% eff)
        numL2_ascii(mPPM);  //convert ADC sum to estimated PPM
        t3=pC[3];
        t2=pC[2];
        t1=pC[1];
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = t3;
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = t2;
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = t1;
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';

        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'P';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'r';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'e';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 's';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 's';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'D';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'o';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'w';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'n';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 't';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'o';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'E';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'n';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'd';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';


    }
    if (mode==6){
        //done display mins
        numL2_ascii(cntMinsA>>6);  //convert mins to reach steady state
        m1=pC[1];
        m2=pC[2];
        numL2_ascii(cntMinsB>>6);  //convert minutes reading
        t3=pC[3];
        t2=pC[2];
        t1=pC[1];
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'D';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'o';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'n';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'e';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ':';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = m2;
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = m1;
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ':';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = t3;
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = t2;
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = t1;
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'P';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'P';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'M';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ':';
        tmp8=(20*sumADC)>>16;   //20/65536=0.0003 (100% eff)
        numL2_ascii(tmp8);  //convert ADC sum to estimated PPM
        t3=pC[3];
        t2=pC[2];
        t1=pC[1];
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = t3;
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = t2;
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = t1;

        numL2_ascii(rLadder);  //convert resistor ladder
        t3=pC[3];
        t2=pC[2];
        t1=pC[1];
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'r';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'L';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'a';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'd';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'd';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'e';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'r';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ':';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = t3;
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = t2;
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = t1;
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        numL2_ascii(flgCur);  //number of times the current is tuned
        t3=pC[3];
        t2=pC[2];
        t1=pC[1];
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = '#';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = t3;
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = t2;
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = t1;
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';

        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'D';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'o';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'w';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'n';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'f';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'o';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'r';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'R';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'e';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 's';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 'e';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = 't';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
        lcdData[i++] = 0x80 | 0x40; lcdData[i++] = ' ';
    }

    lcdData[i++] = 0x40;        lcdData[i++] = ' ';  //last one

    i2c_write(0x3D, lcdData, i);
    delay_ms(10);  //time to finish I2C max 200bytes
}
