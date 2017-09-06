/**
 *FLIP.CLOCK  FW3 PCB REV B.
 * 
 */
#ifndef X198842382129_H
#define X198842382129_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"


// OTHER GPIO
#define AMB_LT_INT		2
#define MIN_INT		4
#define SEC_INT_CALIB_OUT	29
#define INT4	14
#define INT3	15
#define INT2	16
#define INT1	17
#define INT4	14


#define NFC1	9
#define NFC2	10
#define LEDZ	16
#define BUZZ	17
#define RS_485_DIR	19


// LEDs definitions for PCA10040
#define LEDS_NUMBER    3

#define LED_0          25	//BLUE
#define LED_1          26	//RED
#define LED_2          27	//GREEN

  
  
#define LED_BLUE          LED_0	//BLUE
#define LED_RED          LED_1	//RED
#define LED_GREEN          LED_2	//GREEN
  
  
  
#define LEDS_ACTIVE_STATE 0

#define LEDS_INV_MASK  LEDS_MASK

#define LEDS_LIST {LED_0, LED_1, LED_2}

#define BSP_LED_0      LED_0
#define BSP_LED_1      LED_1
#define BSP_LED_2      LED_2

//#define BUTTONS_NUMBER 2

#define BUTTON_LEFT       28 //button label: "CLEAR"
#define BUTTON_RIGHT       11 //button label: "TIMER PLUS"
//#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

//#define BUTTONS_ACTIVE_STATE 0

//#define BUTTONS_LIST { BUTTON_0, BUTTON_1 }


//#define BSP_BUTTON_0   BUTTON_0
//#define BSP_BUTTON_1   BUTTON_1

#define RX_PIN_NUMBER  13
#define TX_PIN_NUMBER  18
#define CTS_PIN_NUMBER 14
#define RTS_PIN_NUMBER 15
#define HWFC           false

#define SCL_PIN             0    // SCL signal pin
#define SDA_PIN             1    // SDA signal pin
#define ARDUINO_SCL_PIN             0    // SCL signal pin
#define ARDUINO_SDA_PIN             1    // SDA signal pin

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_SYNTH,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#ifdef __cplusplus
}
#endif

#endif // X198842382129_H
