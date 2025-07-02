/*  
    Time of Flight Data Collection for 2DX4 Project
    -----------------------------------------------
    This code collects distance data from the VL53L1X ToF sensor via I2C
		and a stepper motor for rotation.
    It also communicates data over UART and flashes onboard LEDs for status.
    
    Written by Daniel Kim
*/

#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"

// I2C Master Control definitions
#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Data Acknowledge (same as ACK)
#define I2C_MCS_ADRACK          0x00000004  // Address Acknowledge
#define I2C_MCS_STOP            0x00000004  // Generate STOP condition
#define I2C_MCS_START           0x00000002  // Generate START condition
#define I2C_MCS_ERROR           0x00000002  // Error flag
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy flag
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // Maximum I2C receive attempts before giving up

//---------------------------
// I2C Initialization
//---------------------------
void I2C_Init(void){
  // Activate I2C0 and Port B
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           // Enable clock for I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          // Enable clock for Port B
  while((SYSCTL_PRGPIO_R & 0x0002) == 0){};          // Wait for Port B to be ready

  // Configure PB2 and PB3 for I2C functionality
  GPIO_PORTB_AFSEL_R |= 0x0C;                        // Enable alternate function on PB2, PB3 (0b00001100)
  GPIO_PORTB_ODR_R |= 0x08;                          // Enable open-drain on PB3 (SDA)
  GPIO_PORTB_DEN_R |= 0x0C;                          // Enable digital I/O on PB2, PB3
	GPIO_PORTB_AMSEL_R &= ~0x0C;          						 // 7) disable analog functionality on PB2,3

  // Configure port control for I2C (using appropriate alternate function)
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFF00FF) + 0x00002200;
  
  // Enable I2C Master Function and set clock speed (100 kbps with glitch suppression)
  I2C0_MCR_R = I2C_MCR_MFE;                          // Enable I2C master functionality
  I2C0_MTPR_R = 0b0000000000000101000000000111011;   // Configure I2C speed and glitch suppression
}

//---------------------------
// Port G Initialization
// (Used for sensor XSHUT control)
//---------------------------
void PortG_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;           // Enable clock for Port G
  while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R6) == 0){}; // Wait for Port G to be ready
  GPIO_PORTG_DIR_R &= ~0x01;                         // Set PG0 as input (HiZ) initially
  GPIO_PORTG_AFSEL_R &= ~0x01;                       // Disable alternate function on PG0
  GPIO_PORTG_DEN_R |= 0x01;                          // Enable digital I/O on PG0
  GPIO_PORTG_AMSEL_R &= ~0x01;                       // Disable analog function on PG0
  return;
}

//---------------------------
// Port H Initialization
// (Used for stepper motor control outputs)
//---------------------------
void PortH_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;           // Enable clock for Port H
  while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0){}; // Wait for Port H to be ready
  GPIO_PORTH_DIR_R |= 0x0F;                          // Set PH0-PH3 as outputs
  GPIO_PORTH_AFSEL_R &= ~0x0F;                       // Disable alternate functions on PH0-PH3
  GPIO_PORTH_DEN_R |= 0x0F;                          // Enable digital I/O on PH0-PH3
  GPIO_PORTH_AMSEL_R &= ~0x0F;                       // Disable analog functionality on PH0-PH3
  return;
}

//---------------------------
// Port M Initialization
// (Used for status LED input; with pull-up)
//---------------------------
void PortM_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;          // Enable clock for Port M
  while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0){}; // Wait for Port M to be ready
  GPIO_PORTM_DIR_R &= ~0x01;                         // Set PM0 as input
  GPIO_PORTM_AFSEL_R &= ~0x01;                       // Disable alternate function on PM0
  GPIO_PORTM_DEN_R |= 0x01;                          // Enable digital I/O on PM0
  GPIO_PORTM_AMSEL_R &= ~0x01;                       // Disable analog functionality on PM0
  GPIO_PORTM_PUR_R = 0x01;                           // Enable pull-up on PM0
  return;
}

//---------------------------
// Port F Initialization
// (Used for additional status LED)
//---------------------------
void PortF_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;           // Enable clock for Port F
  while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0){}; // Wait for Port F to be ready
  GPIO_PORTF_DIR_R |= 0x10;                          // Set PF4 as output
  GPIO_PORTF_AFSEL_R &= ~0x10;                       // Disable alternate function on PF4
  GPIO_PORTF_DEN_R |= 0x10;                          // Enable digital I/O on PF4
  GPIO_PORTF_AMSEL_R &= ~0x10;                       // Disable analog functionality on PF4
  return;
}

//---------------------------
// Port N Initialization
// (Used for UART or LED status for measurements)
//---------------------------
void PortN_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;          // Enable clock for Port N
  while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0){}; // Wait for Port N to be ready
  GPIO_PORTN_DIR_R |= 0x03;                          // Set PN0-PN1 as outputs
  GPIO_PORTN_AFSEL_R &= ~0x03;                       // Disable alternate functions on PN0-PN1
  GPIO_PORTN_DEN_R |= 0x03;                          // Enable digital I/O on PN0-PN1
  GPIO_PORTN_AMSEL_R &= ~0x03;                       // Disable analog functionality on PN0-PN1
  return;
}

//---------------------------
// Port J Initialization
// (Used for push button input)
//---------------------------
void PortJ_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;           // Enable clock for Port J
  while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0){}; // Wait for Port J to be ready
  GPIO_PORTJ_DIR_R &= ~0x02;                         // Set PJ1 as input
  GPIO_PORTJ_AFSEL_R &= ~0x02;                       // Disable alternate function on PJ1
  GPIO_PORTJ_DEN_R |= 0x02;                          // Enable digital I/O on PJ1
  GPIO_PORTJ_AMSEL_R &= ~0x02;                       // Disable analog functionality on PJ1
  GPIO_PORTJ_PUR_R = 0x02;                           // Enable pull-up on PJ1
  return;
}

//---------------------------
// Sensor XSHUT (Reset) Function
// (Active-low reset using PG0; drive low to shut down sensor)
//---------------------------
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                        // Set PG0 as output
    GPIO_PORTG_DATA_R &= 0b11111110;                 // Drive PG0 low to reset sensor
    FlashAllLEDs();                                  // Flash all LEDs as visual feedback
    SysTick_Wait10ms(10);                            // Wait ~10*10ms for sensor reset
    GPIO_PORTG_DIR_R &= ~0x01;                       // Set PG0 back to input (HiZ) to allow sensor to boot
}

//---------------------------
// Stepper Motor Control Function
// (Controls stepper motor rotation in full-step mode)
//---------------------------
void spin(int isClockwise, int steps){
    uint32_t delay = 50000;                          // Delay between steps (adjust for speed)
    if (isClockwise) {
        for (int i = 0; i < steps; i++){
            GPIO_PORTH_DATA_R = 0b00000011;
            SysTick_Wait(delay);
            GPIO_PORTH_DATA_R = 0b00000110;
            SysTick_Wait(delay);
            GPIO_PORTH_DATA_R = 0b00001100;
            SysTick_Wait(delay);
            GPIO_PORTH_DATA_R = 0b00001001;
            SysTick_Wait(delay);
        }
    } else {
        for (int i = 0; i < steps; i++){
            GPIO_PORTH_DATA_R = 0b00001001;
            SysTick_Wait(delay);
            GPIO_PORTH_DATA_R = 0b00001100;
            SysTick_Wait(delay);
            GPIO_PORTH_DATA_R = 0b00000110;
            SysTick_Wait(delay);
            GPIO_PORTH_DATA_R = 0b00000011;
            SysTick_Wait(delay);
        }
    }
}

//*********************************************************************************************************
//                                MAIN FUNCTION
//*********************************************************************************************************

uint16_t dev = 0x29;       // I2C address of the VL53L1X ToF sensor
int status = 0;

int main(void) {
  uint8_t sensorState = 0;
  uint8_t myByteArray[10] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  uint16_t wordData;
  uint16_t Distance;
  uint8_t RangeStatus;
  uint8_t dataReady;
  // (Other variables like SignalRate, AmbientRate, SpadNum, RdByte declared but not used)
  uint8_t i = 0;  // Used for displacement (z-axis)
  
  // Initialization of system modules and ports
  PLL_Init();            // Configure system clock (bus speed)
  SysTick_Init();        // Initialize SysTick for delays
  onboardLEDs_Init();    // Initialize onboard LEDs
  I2C_Init();            // Initialize I2C for sensor communication
  UART_Init();           // Initialize UART for data transmission
  PortG_Init();          // Initialize Port G for sensor XSHUT
  PortH_Init();          // Initialize Port H for stepper motor control
  PortN_Init();          // Initialize Port N for LED status (measurement/UART)
  PortF_Init();          // Initialize Port F for additional status LED (troubleshooting)
  PortM_Init();          // Initialize Port M for push-button input
  PortJ_Init();          // Initialize Port J for additional push-button input

  // Check sensor identity
  status = VL53L1X_GetSensorId(dev, &wordData);

  // Wait for the sensor to boot
  while(sensorState == 0) {
      status = VL53L1X_BootState(dev, &sensorState);
      SysTick_Wait10ms(10);
  }
  // Clear sensor interrupt to prepare for first measurement
  status = VL53L1X_ClearInterrupt(dev);
  
  // Initialize the sensor with default settings
  status = VL53L1X_SensorInit(dev);
  Status_Check("SensorInit", status);

  // Start ranging: enables sensor measurements
  status = VL53L1X_StartRanging(dev);

  // Main loop: if the measurement start button (on PM0, read via Port M) is pressed...
  while (1) {
      if (!(GPIO_PORTM_DATA_R & 0b01)) {  // If PM0 is low (button pressed)
          SysTick_Wait10ms(50);           // Debounce/delay
          GPIO_PORTF_DATA_R |= 0x10;       // Turn on additional status LED (PF4)
          for (int i = 0; i < 32; i++) {   // For one full 360° scan (assumed 32 measurements)
              // Wait until the sensor indicates data is ready
              while (dataReady == 0) {
                  status = VL53L1X_CheckForDataReady(dev, &dataReady);
                  VL53L1_WaitMs(dev, 5);
              }
              dataReady = 0;
              
              // Get the measured distance from the sensor
              status = VL53L1X_GetDistance(dev, &Distance);
              GPIO_PORTN_DATA_R |= 0x02;       // Flash measurement status LED (PN1)
              SysTick_Wait10ms(1);
              GPIO_PORTN_DATA_R &= ~0x02;
              SysTick_Wait10ms(3);
              // Clear the sensor interrupt to allow the next measurement
              status = VL53L1X_ClearInterrupt(dev);
              
              // Send the distance via UART
              sprintf(printf_buffer, "%u\r\n", Distance);
              GPIO_PORTN_DATA_R |= 0x01;       // Flash UART status LED (PN0)
              SysTick_Wait10ms(1);
              GPIO_PORTN_DATA_R &= ~0x01;
              UART_printf(printf_buffer);
              
              // Rotate the sensor a small step (16 microsteps for this example)
              spin(1, 16);
          }
          // After a full 360° scan, return sensor to home position by spinning in reverse
          spin(0, 512);
          GPIO_PORTF_DATA_R &= ~0x10;          // Turn off additional status LED (PF4)
          UART_printf("i\r\n");                // Send a marker (character "i") to indicate slice completion
      }
      
      // Check for a separate stop condition (e.g., push button on PJ1)
      if (!(((GPIO_PORTJ_DATA_R & 0b10) >> 1) & 0b01)) { 
          UART_printf("q\r\n");                // Send a marker (character "q") to indicate quit
          SysTick_Wait10ms(100);
      }
  }
  
  VL53L1X_StopRanging(dev);  // Stop sensor ranging (never reached in current loop)
}
