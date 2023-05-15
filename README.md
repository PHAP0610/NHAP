# STM32F103

## Hardware BOARD V0.1

### Module 7 segment with two ic 74595
1. Pin configuration.

<!-- cách ra 1 dòng để git hiểu là xuống hàng -->
<!-- :--- là căn trái :---: là căn giữa ---: là căn phải trong cột -->
| STM32 | 74595  | 
| :---: | :---: | 
| PA4 | SER |
| PA5 | OE |
| PA6 | RCLK |
| PA7 | SRCLK |

2. Code 

high 8-bit to select a 7-segment digit 
low 8-bit is code-7-segment 
code7segment is used in this board sequence:
<!--  có nhiều fotmat khác như c, cpp, js, html,...... -->
```c
/* this code only use for this version board */
const char code7seg[] = {0xD7, 0x14, 0xCD, 0x5D, 0x1E, 0x5B, 0xDB, 0x15, 0xDF, 0x5F};
```
### Module rotary encoder 
1. Pin configuration 

| STM32 | Rotary Encoder | description | 
| :---: | :---: | :--- |
| PA8 | channel A | pull up |
| PA9 | channel B | pull up |
| PA10 | buttuon | pull up |

2. Code

PA8 - PA9 is encoder mode of timer1 
```c
/* Enable TIM1 encoder mode */
/* USER CODE BEGIN 2 */
HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);
/* USER CODE END 2 */

/* Get encoder value */
encoderValue = __HAL_TIM_GetCounter(&htim1);
```

### AC voltage control
1. Pin configuration 

| STM32 | Module | description |
| :---: | :---: | :--- |
| PB0 | AC Detec| pull up |
| PB1 | Triac |  |

2. code 

PB0 should be an EXT pin 
``` c
/* proccess when interrupt */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == ACDET_Pin){
		SETFLAG(FLAG_ACDET_TRIGGER, flag);
        // control angle to set triac base on time which is calculated by PID
		__HAL_TIM_SET_COUNTER(&htim4, (uint16_t)(triac.giaTriDieuKhien * 1000));
//		__HAL_TIM_SET_COUNTER(&htim4, 9000);
	}
}
```