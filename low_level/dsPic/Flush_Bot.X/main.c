/*
 * File:   main.c
 * Author: Kuy
 *
 * Created on November 11, 2020, 4.14 PM
 */

#include "xc.h"
#include <stdio.h>
#include "configuration.h"

#define ID 0x40
volatile int uartbuf = 0;
volatile long int pose_x = 0;
volatile long int pose_y = 0;

float error[2] = {0.0, 0.0};
float encoder[2] = {0.0, 0.0};
float output[2] = {0.0, 0.0};

const float Kp_x = 1.0;
const float Ki_x = 0.0;
const float Kd_x = 0.0;

const float Kp_y = 1.0;
const float Ki_y = 0.0;
const float Kd_y = 0.0;

float pre_pose[2] = {0.0,0.0};
float CONT = 0.01;
float v[2] = {0.0,0.0};
float sigma_a[2] = {11.0,11.0}; // adjustable
float sigma_w[2] = {0.8,0.8}; // adjustable
//float w_update[2] = {0.0};
//float w_inKalman[2] = {0.0};
//float w_outKalman[2] = {0.0};
float Q, R;
float x1[2] = {0.0, 0.0}; // orientation
float x2[2] = {0.0, 0.0}; // woutKalman (angular_vel)
float p11[2] = {1.0, 1.0}; // adjustable
float p12[2] = {0.0, 0.0};
float p21[2] = {0.0, 0.0};
float p22[2] = {0.9, 0.9}; // adjustable

int package_uart(unsigned char data) {
    int result = 0;
    static unsigned char state = 0;
    static int checksum = 0;
    if ((state == 0)) {
        if (data == 0xBD) {
            state++;
        } else {
            state = 0;
            result = 0xFA;
        }
    } else if (state == 1) {
        if (data == ID) {
            checksum += data;
            state++;
        } else {
            state = 0;
            result = 0xFA;
        }
    } else if (state == 2) {
        checksum += data;
        pose_x = data << 8;
        state++;
    } else if (state == 3) {
        checksum += data;
        pose_x |= data;
        pose_x = pose_x * 154;
        state++;
    } else if (state == 4) {
        checksum += data;
        pose_y = data << 8;
        state++;
    } else if (state == 5) {
        checksum += data;
        pose_y |= data;
        pose_y = pose_y * 154;
        state++;
    } else if (state == 6) {
        checksum = ~checksum;
        checksum = checksum & 0xFF;
        if (checksum == data) {
            result = 0xAC;
        } else {
            result = 0xFA;
        }

        checksum = 0;
        state = 0;
    }
    return result;
}

void send_package(unsigned char data) {
    U1TXREG = data;
    while (U1STAbits.TRMT == 0) {
    }
}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void) {
    IFS0bits.U1TXIF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void) {
    unsigned char data = U1RXREG;
    uartbuf = package_uart(data);
    IFS0bits.U1RXIF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _U1ErrInterrupt(void) {
    U1STAbits.OERR == 1 ? U1STAbits.OERR = 0 : Nop();
    IFS4bits.U1EIF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    //    static long long int output_x = 0;
    //    volatile long long int s = 0;
    //    volatile long long int p = 0;
    encoder[0] = (float) POS1CNT;
    error[0] = (float) pose_x - encoder[0];
    output[0] = 1.0 * error[0];
    encoder[1] = (float) POS2CNT;
    error[1] = (float) pose_y - encoder[1];
    output[1] = 1.0 * error[1];
    const float CONT_pow2 = CONT * CONT;
    const float CONT_pow3 = CONT * CONT * CONT;
    const float CONT_pow4 = CONT * CONT * CONT * CONT;
   
    unsigned char i;
    for (i = 0; i < 2; i++) {
        v[i] = (encoder[i] - pre_pose[i]) / CONT;

        float Q = sigma_a[i] * sigma_a[i];
        float R = sigma_w[i] * sigma_w[i];
        float x1_new = x1[i] + x2[i] * CONT;
        float x2_new = 0 + x2[i];
        float ye = v[i] - x2_new;
        p11[i] = p11[i] + (CONT * p21[i]) + (Q * CONT_pow4) / 4 + (CONT_pow2 * (p12[i] + CONT * p22[i])) / CONT;
        p12[i] = p12[i] + (CONT * p22[i]) + (Q * CONT_pow3) / 2;
        p21[i] = (2 * CONT * p21[i] + Q * CONT_pow2 + 2 * p22[i] * CONT_pow2) / (2 * CONT);
        p22[i] = Q * CONT_pow2 + p22[i];
        x1_new = x1_new + (p12[i] * ye) / (R + p22[i]);
        x2_new = x2_new + (p22[i] * ye) / (R + p22[i]);
        p11[i] = p11[i] - (p12[i] * p21[i]) / (R + p22[i]);
        p12[i] = p12[i] - (p12[i] * p22[i]) / (R + p22[i]);
        p21[i] = -p21[i] * (p22[i] / (R + p22[i]) - 1);
        p22[i] = -p22[i] * (p22[i] / (R + p22[i]) - 1);
        //update variable of kaman filter
        //    pre_position = position;
        x1[i] = x1_new; // position
        x2[i] = x2_new; // velocity

    }
    printf("%.2f\t%.2f\t%.2f\t%.2f\n", encoder[1], x1[1], v[1], x2[1]);

    if (error[0] < 700.0 && error[0] > -700.0) {
        Driver_motor_X(0);
    } else {
        if (output[0] > 10000.0) {
            Driver_motor_X(10000.0);
        } else if (output[0] < -10000.0) {
            Driver_motor_X(-10000.0);
        } else {
            Driver_motor_X(output[0]);
        }
    }


    //    printf("%.2f\t", encoder_x);
    ////    printf("%.2f\t", error_x);
    //    printf("%.2f\t\n", pose_x);


    if (output[1] < 700.0 && output[1] > -700.0) {
        Driver_motor_Y(0);
    } else {
        if (output[1] > 10000.0) {
            Driver_motor_Y(10000.0);
        } else if (output[1] < -10000.0) {
            Driver_motor_Y(-10000.0);
        } else {
            Driver_motor_Y(output[1]);
        }
    }

    pre_pose[0] = encoder[0];
    pre_pose[1] = encoder[1];
    _T1IF = 0;
}

void initPLL() // Set Fcy to 40 MHz
{
    PLLFBD = 150; // M  = 152
    CLKDIVbits.PLLPRE = 5; // N1 = 7
    CLKDIVbits.PLLPOST = 0; // N2 = 2
    OSCTUN = 0; // Tune FRC oscillator, if FRC is used

    // Clock switching to incorporate PLL
    __builtin_write_OSCCONH(0x01); // Initiate Clock Switch to FRCPLL
    // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL(0x01); // Start clock switching

    while (OSCCONbits.COSC != 0b001); // Wait for Clock switch to occur
    while (OSCCONbits.LOCK != 1) {
    }; // Wait for PLL to lock
}

void init_UART() {
    __builtin_write_OSCCONL(OSCCON & 0xBF); //PPS RECONFIG UNLOCK 
    RPINR18bits.U1RXR = 6;
    RPOR2bits.RP5R = 0b00011;
    __builtin_write_OSCCONL(OSCCON | 0x40); //PPS RECONFIG LOCK

    U1MODEbits.STSEL = 0; // 1 Stop bit
    U1MODEbits.PDSEL = 0; // No Parity, 8 data bits
    U1MODEbits.BRGH = 1; // High Speed mode
    U1MODEbits.URXINV = 0; // UxRX idle state is '1'
    U1BRG = 86; // BAUD Rate Setting for 115200
    U1STAbits.UTXISEL0 = 0; // Interrupt after one TX Character is transmitted
    U1STAbits.UTXISEL1 = 0;
    IEC0bits.U1RXIE = 1; // Enable UART RX Interrupt
    IEC0bits.U1TXIE = 1; // Enable UART TX Interrupt
    U1MODEbits.UARTEN = 1; // Enable UART
    U1STAbits.UTXEN = 1; // Enable UART TX
}

void init_QEI() {
    __builtin_write_OSCCONL(OSCCON & 0xBF); //PPS RECONFIG UNLOCK 
    RPINR14bits.QEA1R = 8; //remap RP14 connect to QEI1_A
    RPINR14bits.QEB1R = 9; //remap RP15 connect to QEI1_B
    RPINR16bits.QEA2R = 11; //remap RP6 connect to QEI2_A
    RPINR16bits.QEB2R = 10; //remap RP6 connect to QEI2_B
    __builtin_write_OSCCONL(OSCCON | 0x40); //PPS RECONFIG LOCK 
    /*QEI Mode Select*/
    QEI1CONbits.QEIM = 0b000; // QEI Mode disable
    QEI1CONbits.PCDOUT = 0; // no direction pin out
    QEI1CONbits.QEIM = 0b101; // 2x ,no index

    QEI2CONbits.QEIM = 0b000; // QEI2 Mode disable
    QEI2CONbits.PCDOUT = 0; // no direction pin out
    QEI2CONbits.QEIM = 0b101; // 2x ,no index
    /*digital filter config */
    __builtin_write_OSCCONL(OSCCON & 0xBF); //PPS RECONFIG UNLOCK 
    DFLT1CONbits.QECK = 0b000; // clock divider Fcy/1
    DFLT1CONbits.QEOUT = 1; // enable filter
    DFLT2CONbits.QECK = 0b000; // clock divider Fcy/1
    DFLT2CONbits.QEOUT = 1; // enable filter
    __builtin_write_OSCCONL(OSCCON | 0x40); //PPS RECONFIG LOCK 
}

void init_Timer() {
    T1CONbits.TCKPS = 0b01; //set timer prescaler to 1:8
    PR1 = 50000; //set 100Hz Timer interrupt 
    _T1IE = 1; // enable Timer1 interrupt
    _T1IP = 5; // set priority to 5
}

void dealy_UART() {
    unsigned int i;
    for (i = 0; i < 500; i++) { //347
        Nop();
    }
}

void dealy(unsigned long int time) {
    unsigned long int i;
    for (i = 0; i < time; i++) { //347
        Nop();
    }
}

void init_Motor() {
    T2CONbits.TCKPS = 0b01; //set timer prescaler to 1:64
    PR2 = 10000; //set period to 15,625 tick per cycle 
    OC1CONbits.OCM = 0b000; //Disable Output Compare Module
    OC1CONbits.OCTSEL = 0; //OC1 use timer2 as counter source
    OC1CONbits.OCM = 0b110; //set to pwm without fault pin mode

    __builtin_write_OSCCONL(OSCCON & 0xBF); //PPS RECONFIG UNLOCK 
    _RP15R = 0b10010; //remap RP11 connect to OC1
    __builtin_write_OSCCONL(OSCCON | 0x40); //PPS RECONFIG LOCK
    ///////////////////////////////////////////////////////////
    T3CONbits.TCKPS = 0b01; //set timer prescaler to 1:64
    PR3 = 10000; //set period to 15,625 tick per cycle 
    OC2CONbits.OCM = 0b000; //Disable Output Compare Module
    OC2CONbits.OCTSEL = 1;
    OC2CONbits.OCM = 0b110; //set to pwm without fault pin mode
    __builtin_write_OSCCONL(OSCCON & 0xBF); //PPS RECONFIG UNLOCK 
    _RP14R = 0b10011;
    __builtin_write_OSCCONL(OSCCON | 0x40); //PPS RECONFIG LOCK

    /////////////////////////////////////////////////
}

void init_PinMode() {
    AD1PCFGL = 0xFFFF;
    TRISA = 0xFFF0;
    TRISB = 0x0FD3;
}

void Driver_motor_Y(long int input) {
    OC1RS = abs(input);
    if (input < 0) {
        LATAbits.LATA0 = 1;
        LATAbits.LATA1 = 0;
    } else if (input > 0) {

        LATAbits.LATA0 = 0;
        LATAbits.LATA1 = 1;
    } else {
        LATAbits.LATA0 = 1;
        LATAbits.LATA1 = 1;
    }
}

void Driver_motor_X(long int input) {
    OC2RS = abs(input);
    if (input < 0) {
        LATBbits.LATB2 = 1;
        LATBbits.LATB3 = 0;
    } else if (input > 0) {
        LATBbits.LATB2 = 0;
        LATBbits.LATB3 = 1;
    } else {
        LATBbits.LATB2 = 1;
        LATBbits.LATB3 = 1;
    }
}

void Set_Home() {
    while (_RB4 == 1) {
        Driver_motor_Y(-3000.0);
    }
    Driver_motor_Y(0.0);
    dealy(100000);
    dealy(100000);
    dealy(100000);
    dealy(100000);
    dealy(100000);
    dealy(100000);
    dealy(100000);
    dealy(100000);
    dealy(100000);
    dealy(100000);
    while (_RB7 == 1) {
        Driver_motor_X(-3000.0);
    }
    Driver_motor_X(0.0);
    dealy(100000);
    dealy(100000);
    dealy(100000);
    dealy(100000);
    dealy(100000);
    dealy(100000);
    dealy(100000);
    dealy(100000);
    dealy(100000);
    dealy(100000);


}

int main(void) {
    /*disable global interrupt*/
    __builtin_disable_interrupts();

    initPLL();
    AD1PCFGL = 0xFFFF;
    TRISA = 0xFFF0;
    TRISB = 0x0FD3;
    //    init_INT();
    init_Timer(); // Set timer1_interrupt 100Hz

    T2CONbits.TON = 1; //enable PWM 1
    T3CONbits.TON = 1; //enable PWM 2

    init_Motor();
    __builtin_enable_interrupts();
    init_QEI(); // Set QEI register mode 2x 
    Set_Home();
    __builtin_disable_interrupts();
    init_UART(); // Set UART register
    dealy_UART(); //wait at least 8.68 usec (1/115200) before sending first char 

    /*enable global interrupt*/
    __builtin_enable_interrupts();

    unsigned long int cot;
    dealy(100000);
    dealy(100000);
    dealy(100000);
    dealy(100000);
    dealy(100000);
    dealy(100000);
    dealy(100000);
    dealy(100000);
    dealy(100000);
    dealy(100000);
    POS1CNT = 0;
    POS2CNT = 0;
    T1CONbits.TON = 1; //enable Timer interrupt

    while (1) {
        if (uartbuf != 0) {
            send_package(uartbuf);
            uartbuf = 0;
        }

        if (cot >= 10000) {
            //            int i ;
            //            for (i = 0; i < 2; i++) {printf("%d\n",i);}
            //            printf("%u\t", POS1CNT);
            //            printf("%u\t", POS2CNT);
            //            printf("%.2f\t", error_x);
            //                        printf("%.2f\t",error_y);
            //            printf("%ld\t", pose_x);
            //            printf("\n");
            cot = 0;
        }
        cot++;

    }
    return 0;
}