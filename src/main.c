#include "LPC17xx.h"
#include <stdio.h>
#include "RTL.h"       // RTX RTOS

#define TRIG      (1 << 26)
#define ECHO      (1 << 25)

#define LED_SAFE  (1 << 18)
#define LED_NEAR  (1 << 19)
#define LED_MID   (1 << 20)

#define M1A       (1 << 0)
#define M1B       (1 << 1)
#define M2A       (1 << 2)
#define M2B       (1 << 3)
#define ENA       (1 << 4)
#define ENB       (1 << 5)

#define THRESHOLD_SAFE  10.0
#define THRESHOLD_NEAR  20.0
#define THRESHOLD_MID   30.0

/* -------- Shared Data (RTOS style) -------- */
float distance;
char uart_buf[50];

/* -------- Delay -------- */
void delay_ms(unsigned int ms) {
    unsigned int i, j;
    for(i = 0; i < ms; i++)
        for(j = 0; j < 600; j++);
}

/* -------- UART -------- */
void UART0_Init(void) {
    LPC_SC->PCONP |= (1 << 3);
    LPC_PINCON->PINSEL0 &= ~((3 << 4) | (3 << 6));
    LPC_PINCON->PINSEL0 |= ((1 << 4) | (1 << 6));

    LPC_UART0->LCR = 0x83;
    LPC_UART0->DLL = 162;
    LPC_UART0->DLM = 0;
    LPC_UART0->LCR = 0x03;
}

void UART0_SendString(char *str) {
    while(*str) {
        while(!(LPC_UART0->LSR & (1 << 5)));
        LPC_UART0->THR = *str++;
    }
}

/* -------- Init -------- */
void Ultrasonic_Init(void) {
    LPC_GPIO0->FIODIR |= TRIG;
    LPC_GPIO0->FIODIR &= ~ECHO;
    LPC_SC->PCONP |= (1 << 1);
    LPC_TIM0->PR = 24;
}

void LED_Init(void) {
    LPC_GPIO1->FIODIR |= LED_SAFE | LED_NEAR | LED_MID;
    LPC_GPIO1->FIOCLR = LED_SAFE | LED_NEAR | LED_MID;
}

void Motor_Init(void) {
    LPC_GPIO2->FIODIR |= M1A | M1B | M2A | M2B | ENA | ENB;
    LPC_GPIO2->FIOCLR = ENA | ENB;
}

/* -------- Motor -------- */
void Motor_Forward(void) {
    LPC_GPIO2->FIOSET = M1A | M2A | ENA | ENB;
    LPC_GPIO2->FIOCLR = M1B | M2B;
}

void Motor_Stop(void) {
    LPC_GPIO2->FIOCLR = ENA | ENB;
    LPC_GPIO2->FIOCLR = M1A | M1B | M2A | M2B;
}

/* -------- Ultrasonic -------- */
float Measure_Distance(void) {
    unsigned int time;

    LPC_GPIO0->FIOCLR = TRIG;
    delay_ms(2);
    LPC_GPIO0->FIOSET = TRIG;
    delay_ms(1);
    LPC_GPIO0->FIOCLR = TRIG;

    while(!(LPC_GPIO0->FIOPIN & ECHO));
    LPC_TIM0->TCR = 0x02;
    LPC_TIM0->TCR = 0x01;

    while(LPC_GPIO0->FIOPIN & ECHO);
    LPC_TIM0->TCR = 0x00;

    time = LPC_TIM0->TC;
    return (time * 0.0343f) / 2.0f;
}

/* -------- Task 1: Ultrasonic sensing -------- */
__task void Task_Ultrasonic(void) {
    while(1) {
        distance = Measure_Distance();
        os_dly_wait(5);    // Round Robin slice
    }
}

/* -------- Task 2: Motor + LED control -------- */
__task void Task_Control(void) {
    while(1) {

        LPC_GPIO1->FIOCLR = LED_SAFE | LED_NEAR | LED_MID;

        if(distance <= THRESHOLD_SAFE) {
            Motor_Stop();
            LPC_GPIO1->FIOSET = LED_SAFE;
        }
        else if(distance <= THRESHOLD_NEAR) {
            Motor_Stop();
            LPC_GPIO1->FIOSET = LED_NEAR;
        }
        else if(distance <= THRESHOLD_MID) {
            Motor_Forward();
            LPC_GPIO1->FIOSET = LED_MID;
        }
        else {
            Motor_Forward();
        }

        os_dly_wait(5);
    }
}

/* -------- Task 3: UART logging -------- */
__task void Task_UART(void) {
    while(1) {
        sprintf(uart_buf, "Distance = %.1f cm\r\n", distance);
        UART0_SendString(uart_buf);
        os_dly_wait(10);
    }
}

/* -------- INIT TASK -------- */
__task void Init_Task(void) {
    os_tsk_create(Task_Ultrasonic, 1);
    os_tsk_create(Task_Control, 1);
    os_tsk_create(Task_UART, 1);
    os_tsk_delete_self();
}

/* ================ MAIN ================ */
int main(void) {
    SystemInit();
    UART0_Init();
    Ultrasonic_Init();
    LED_Init();
    Motor_Init();

    UART0_SendString("RTOS Round Robin Obstacle Robot\r\n");

    os_sys_init(Init_Task);    // Start RTOS (Round Robin)
    while(1);
}
