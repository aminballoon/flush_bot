/*
 * File:   main.c
 * Author: Kuy
 *
 * Created on November 11, 2020, 4.14 PM
 */

#include "xc.h"
#include <stdio.h>
#include <stdbool.h>
#include "configuration.h"
#include <math.h>

#define ID 0x40
volatile int uartbuf = 0;
volatile float Position_X = 0;
volatile float Position_X_Goal = 0;
volatile float Position_Y = 0;
volatile float Position_Y_Goal = 0;
volatile float Velocity_X = 0;
volatile float Velocity_Y = 0;
volatile float Error_posecont[2] = {0.0, 0.0};
volatile float Encoder[2] = {0.0, 0.0};
volatile float Output_posecont[2] = {0.0, 0.0};


float Input_velcont[2] = {0.0, 0.0};
float Error_velcont[2] = {0.0, 0.0};
float Output_velcont[2] = {0.0, 0.0};
float SumError_velcont[2] = {0.0, 0.0};
float PreError_velcont[2] = {0.0, 0.0};

float Pre_position_fixwindow[2] = {0.0, 0.0};
const float CONT = 0.005;
float Trajectory_Time_Now = 0.0;
float Velocity_Input_Kalman[2] = {0.0, 0.0};
float sigma_a[2] = {14.0, 14.0}; // adjustable 14
float sigma_w[2] = {0.8, 0.8}; // adjustable 0.8
//float w_update[2] = {0.0};
//float w_inKalman[2] = {0.0};
//float w_outKalman[2] = {0.0};
float Q, R;
float Position_Kalman[2] = {0.0, 0.0}; // orientation
float Velocity_Kalman[2] = {0.0, 0.0}; // woutKalman (angular_vel)
float p11[2] = {1.0, 1.0}; // adjustable
float p12[2] = {0.0, 0.0};
float p21[2] = {0.0, 0.0};
float p22[2] = {0.9, 0.9}; // adjustable

unsigned char En_T1 = 0;
float Trajectory_Theta = 0;
float Trajectory_Time = 0;
float Trajectory_Magnitude = 0;
float pose_i = 0;
float pose_f = 0;
float delta_x = 0.0;
float delta_y = 0.0;
//float r = 0 ;

float C2 = 0;
float C3 = 0;

float Sx = 0;
float Sy = 0;
float Vx = 0;
float Vy = 0;

bool State_Sethome = 0;
char State_print_Position = 0;
bool State_Control = 0;

int package_uart(unsigned char data) {
    int result = 0;
    static unsigned char state = 0;
    static int checksum = 0;
    static unsigned int pose_x_UART = 0;
    static unsigned int pose_y_UART = 0;
    switch (state) {
        case 0:
            if (data == 0xBD) {
                state = 1;
            } else {
                result = 0xFA;
            }
            break;

        case 1:
            if (data == 0x40) {
                state = 41;
            } else if (data == 0x30) {
                state = 31;
            } else if (data == 0x20) {
                state = 21;
            } else if (data == 0x21) {
                state = 23;
            } else if (data == 0x22) {
                state = 25;
            } else if (data == 0x23) {
                state = 27;
            } else if (data == 0x10) {
                state = 11;
            } else if (data == 0x50) {
                state = 50;
            } 
            else {
                result = 0xFA;
            }
            break;
            
        case 11:
            state = 12;
            break;
            
        case 12:
            State_Sethome = 1;
            state = 0;
            result = 0xAC;
            break;
        
        case 21:
            state = 21;
            break;
            
        case 22:
            State_print_Position = 1;
            state = 0;
            result = 0xAC;
            break;
            
        case 23:
            state = 24;
            break;
            
        case 24:
            State_print_Position = 2;
            state = 0;
            result = 0xAC;
            break;
   
        case 25:
            state = 26;
            break;
            
        case 25:
            State_print_Position = 3;
            state = 26;
            result = 0xAC;
            break;
            
        case 50:
            state = 51;
            break;
            
        case 51: 
            State_Control = 1;
            result = 0xAC;
            break;
            
        case 41:
            checksum += 0x40;
            checksum += data;
            pose_x_UART = data << 8;
            state = 42;
            break;

        case 42:
            checksum += data;
            pose_x_UART |= data;
            pose_x_UART *= 154.0; // 154
            state = 43;
            break;
        case 43:
            checksum += data;
            pose_y_UART = data << 8;
            state = 44;
            break;
        case 44:
            checksum += data;
            pose_y_UART |= data;
            pose_y_UART *= 154.0;
            state = 45;
            break;
        case 45:
            checksum = ~checksum;
            checksum = checksum & 0xFF;
            if (checksum == data) {
                //                Position_X += delta_x;
                //                Position_Y += delta_x;
                delta_x = (float) pose_x_UART - Position_X;
                delta_y = (float) pose_y_UART - Position_Y;
                Trajectory_Theta = atan2(delta_y, delta_x);
                Trajectory_Magnitude = sqrt((delta_y * delta_y) + (delta_x * delta_x));
                
                Trajectory_Time = Trajectory_Magnitude / 8100.0;
                
                C2 = 3.0 / (Trajectory_Time * Trajectory_Time);
                C3 = -2.0 / (Trajectory_Time * Trajectory_Time * Trajectory_Time);

                result = 0xAC;

                
                //                En_T1 = 40;

            } else {
                result = 0xFA;
            }
            //            theta_UART = 0;
            //            t_UART = 0;
            pose_x_UART = 0;
            pose_y_UART = 0;
            checksum = 0;
            state = 0;
            Trajectory_Time_Now = 0;
            break;
            // operator doesn't match any case constant +, -, *, /
        default:
            result = 0xFA;
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

    Position_X_Goal = (float) Position_X + ((C2 * Trajectory_Time_Now * Trajectory_Time_Now)+(C3 * Trajectory_Time_Now * Trajectory_Time_Now * Trajectory_Time_Now)) * delta_x;
    Position_Y_Goal = (float) Position_Y + ((C2 * Trajectory_Time_Now * Trajectory_Time_Now)+(C3 * Trajectory_Time_Now * Trajectory_Time_Now * Trajectory_Time_Now)) * delta_y;
    Velocity_X = ((2.0 * C2 * Trajectory_Time_Now)+(3.0 * C3 * Trajectory_Time_Now * Trajectory_Time_Now)) * delta_x; // * cos(Trajectory_Theta) 
    Velocity_Y = ((2.0 * C2 * Trajectory_Time_Now)+(3.0 * C3 * Trajectory_Time_Now * Trajectory_Time_Now)) * delta_y; //  * sin(Trajectory_Theta) 


//     = Sx;
//     = Sy;
//     = Vx;
//     = Vy;
    //    printf("%.2f\t", Position_X);
    //    printf("%.2f\t", Sx);
    //    printf("%.2f\t", Position_Y);
    //    printf("%.2f\t", Sy);
    //    printf("%.2f\t", Vx);
    //    printf("%.2f\n", Vy);



    Encoder[0] = (float) POS1CNT;
    Encoder[1] = (float) POS2CNT;
    const float CONT_pow2 = CONT * CONT;
    const float CONT_pow3 = CONT * CONT * CONT;
    const float CONT_pow4 = CONT * CONT * CONT * CONT;

    unsigned char i;
    Velocity_Input_Kalman[0] = (Encoder[0] - Pre_position_fixwindow[0]) / CONT;
    Velocity_Input_Kalman[1] = (Encoder[1] - Pre_position_fixwindow[1]) / CONT;

    for (i = 0; i < 2; i++) {

        float Q = sigma_a[i] * sigma_a[i];
        float R = sigma_w[i] * sigma_w[i];
        float Position_Kalman_new = Position_Kalman[i] + Velocity_Kalman[i] * CONT;
        float Velocity_Kalman_new = 0 + Velocity_Kalman[i];
        float ye = Velocity_Input_Kalman[i] - Velocity_Kalman_new;
        p11[i] = p11[i] + (CONT * p21[i]) + (Q * CONT_pow4) / 4 + (CONT_pow2 * (p12[i] + CONT * p22[i])) / CONT;
        p12[i] = p12[i] + (CONT * p22[i]) + (Q * CONT_pow3) / 2;
        p21[i] = (2 * CONT * p21[i] + Q * CONT_pow2 + 2 * p22[i] * CONT_pow2) / (2 * CONT);
        p22[i] = Q * CONT_pow2 + p22[i];
        Position_Kalman_new = Position_Kalman_new + (p12[i] * ye) / (R + p22[i]);
        Velocity_Kalman_new = Velocity_Kalman_new + (p22[i] * ye) / (R + p22[i]);
        p11[i] = p11[i] - (p12[i] * p21[i]) / (R + p22[i]);
        p12[i] = p12[i] - (p12[i] * p22[i]) / (R + p22[i]);
        p21[i] = -p21[i] * (p22[i] / (R + p22[i]) - 1);
        p22[i] = -p22[i] * (p22[i] / (R + p22[i]) - 1);
        //update variable of kalman filter
        Pre_position_fixwindow[i] = Encoder[i];
        //    pre_position = position;
        Position_Kalman[i] = Position_Kalman_new; // position
        Velocity_Kalman[i] = Velocity_Kalman_new; // velocity
    }

    //    printf("%.2f\t",Encoder[0]);

    //SigmaError_velcont[2]
    //PreError_velcont[2]
    const float Kp_px = 0.0;
//    const float Ki_px = 0.0;
//    const float Kd_px = 0.0;

    const float Kp_vx = 4.0; // 4.0
    const float Ki_vx = 0.1; // 0.1
    const float Kd_vx = 2.5; // 2.5

    const float Kp_py = 0.0;
//    const float Ki_py = 0.0;
//    const float Kd_py = 0.0;

    const float Kp_vy = 4.0; // 4.0
    const float Ki_vy = 0.15; //0.15
    const float Kd_vy = 2.5; // 2.5

    // X axis
    //Position Control X
    Encoder[0] = (float) POS1CNT;
    Error_posecont[0] = (float) Position_X_Goal - Encoder[0];
    Output_posecont[0] = Kp_px * Error_posecont[0];
    //Velocity Control X
    Input_velcont[0] = Velocity_X + Output_posecont[0]; // Feed Forward + Output Position Control
    Error_velcont[0] = Input_velcont[0] - Velocity_Kalman[0]; // 
    SumError_velcont[0] += Error_velcont[0];
    Output_velcont[0] = (Kp_vx * Error_velcont[0]) + (Ki_vx * SumError_velcont[0]) + (Kd_vx * (Error_velcont[0] - PreError_velcont[0]));
    // End



    // Y axis
    // Position Control Y
    Encoder[1] = (float) POS2CNT;
    Error_posecont[1] = (float) Position_Y_Goal - Encoder[1];
    Output_posecont[1] = Kp_py * Error_posecont[1];
    // Velocity Control Y
    Input_velcont[1] = Velocity_Y + Output_posecont[1]; // Feed Forward + Output Position Control
    Error_velcont[1] = Input_velcont[1] - Velocity_Kalman[1]; // 
    SumError_velcont[1] += Error_velcont[1];
    Output_velcont[1] = (Kp_vy * Error_velcont[1]) + (Ki_vy * SumError_velcont[1]) + (Kd_vy * (Error_velcont[1] - PreError_velcont[1]));
    //End

    if (Output_velcont[0] > 10000.0) {
        Driver_motor_X(10000);
    } else if (Output_velcont[0] < -10000.0) {
        Driver_motor_X(-10000);
    } else {
        Driver_motor_X((int) Output_velcont[0]);
    }

    if (Output_velcont[1] > 10000.0) {
        Driver_motor_Y(10000);
    } else if (Output_velcont[1] < -10000.0) {
        Driver_motor_Y(-10000);
    } else {
        Driver_motor_Y((int) Output_velcont[1]);
    }

    PreError_velcont[0] = Error_velcont[0];
    PreError_velcont[1] = Error_velcont[1];


    if (Trajectory_Time_Now < Trajectory_Time) {
        Trajectory_Time_Now += CONT;
    } else {
        //        pose_i = Trajectory_Magnitude;
        Trajectory_Time_Now = Trajectory_Time;
        SumError_velcont[0] = 0.0;
        SumError_velcont[1] = 0.0;
        Driver_motor_X(0);
        Driver_motor_Y(0);
        Position_X = Encoder[0];
        Position_Y = Encoder[1];
        T1CONbits.TON = 0;
    }
    
    // print X 
//    printf("%.2f\t", Position_X_Goal);
//    printf("%.2f\t", Encoder[0]);
//    printf("%.2f\t", Velocity_X);
//    printf("%.2f\t", Velocity_Kalman[0]);
    
    // print Y 
//    printf("%.2f\t", Position_Y_Goal);
//    printf("%.2f\t", Encoder[1]);
//    printf("%.2f\t", Velocity_Y);
//    printf("%.2f\t", Velocity_Kalman[1]);

//    printf("\n");



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
    PR1 = 10000; //set 500Hz Timer interrupt 
    _T1IE = 1; // enable Timer1 interrupt
    _T1IP = 3; // set priority to 5
}

void delay_UART() {
    unsigned int i;
    for (i = 0; i < 500; i++) { //347
        Nop();
    }
}

void delay(unsigned long int time) {
    unsigned long int i;
    for (i = 0; i < time; i++) { //347
        Nop();
    }
}

void delay_int(unsigned int time) {
    unsigned int i, j;
    for (i = 0; i < time; i++) {
        for (j = 0; j < 200; j++) {
            Nop();
        }
    }
}

void init_Motor() {
    T2CONbits.TCKPS = 0b01; //set timer prescaler to 1:8
    PR2 = 10000; //set period to 10,000 tick per cycle 500Hz
    OC1CONbits.OCM = 0b000; //Disable Output Compare Module
    OC1CONbits.OCTSEL = 0; //OC1 use timer2 as counter source
    OC1CONbits.OCM = 0b110; //set to pwm without fault pin mode

    __builtin_write_OSCCONL(OSCCON & 0xBF); //PPS RECONFIG UNLOCK 
    _RP15R = 0b10010; //remap RP11 connect to OC1
    __builtin_write_OSCCONL(OSCCON | 0x40); //PPS RECONFIG LOCK
    ///////////////////////////////////////////////////////////
    T3CONbits.TCKPS = 0b01; //set timer prescaler to 1:8
    PR3 = 10000; //set period to 10,000 tick per cycle 500Hz
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

void Driver_motor_Y(int input) {
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
    OC1RS = abs(input);
}

void Driver_motor_X(int input) {
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
    OC1RS = abs(input);
}

void Set_Home() {
    while (_RB4 == 1) {
        Driver_motor_Y(-3000);
    }
    Driver_motor_Y(0);
    delay_int(10000);

    while (_RB7 == 1) {
        Driver_motor_X(-3000);
    }
    Driver_motor_X(0);
    delay_int(10000);



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
    delay_UART(); //wait at least 8.68 usec (1/115200) before sending first char 

    /*enable global interrupt*/
    __builtin_enable_interrupts();

    unsigned long int cot;
    delay_int(10000);

    POS1CNT = 0;
    POS2CNT = 0;
    //enable Timer interrupt
    //    T1CONbits.TON = 1;
    while (1) {
        if (uartbuf != 0) {
            send_package(uartbuf);
            uartbuf = 0;
        }
        if (State_Sethome == 1) {
            State_Sethome = 0;
            Set_Home();
            delay_int(10000);
            POS1CNT = 0;
            POS2CNT = 0;
        }
        if (State_print_Position == 1) {
            State_print_Position = 0;
            printf("%d\t%d\n", Position_X, Position_Y);
        }
        if (State_print_Position == 2) {
            State_print_Position = 0;
            printf("%d\n", Position_X);
        }
        if (State_print_Position == 3) {
            State_print_Position = 0;
            printf("%d\n", Position_Y);
        }
        if (State_Control == 1) {
            State_Control = 0;
            T1CONbits.TON = 1;
        }

        if (cot >= 10000) {
            cot = 0;
        }
        cot++;

    }
    return 0;
}