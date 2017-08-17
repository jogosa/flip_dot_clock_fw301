/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */
#include "nrf_delay.h"

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "nordic_common.h"
#include "nrf.h"

#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "nrf_ble_gatt.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define CONN_CFG_TAG                    1                                           /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */

#define BOARD_CUSTOM    1


#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */


#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0   ///was 180                                         /**< The advertising timeout (in units of seconds). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static nrf_ble_gatt_t                   m_gatt;                                     /**< GATT module instance. */
static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */
static uint16_t                         m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;  /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */




#define CURRENT_VERSION_TOKEN           'Y'                             //change everytime there is a need to restore eeprom settings to defaults
#define DEVICE_NAME                     "FLIP.CLOCK BLUE"               /**< Name of device. Will be included in the advertising data. */     //DEPENDENT ON DIFFERENT HEX FILES FOR DIFFERENT COLORS
#define CURRENT_SKU                     'B'                                                                                                   //DEPENDENT ON DIFFERENT HEX FILES FOR DIFFERENT COLORS

/*
"FLIP.CLOCK BLUE"
  CURRENT_SKU 'B'

"FLIP.CLOCK CLEAR"
  CURRENT_SKU 'C'

"FLIP.CLOCK GREEN"
  CURRENT_SKU 'G'

"FLIP.CLOCK YELLOW"
  CURRENT_SKU 'Y'

"FLIP.CLOCK BLUERED"
  CURRENT_SKU 'R'
*/


#define EMPTY   0
#define NaN 666666666

//ee_settings ARRAY ADDRESSES  //USE ONLY 2Kb EEPROM!!!!!!!!!

#define EEPROM 0x50

#define ee_sys_defaultstoken	0
#define ee_t_format             1
#define ee_t_sep                2
#define reserved1              3
#define ee_msg_speed            4
#define ee_msg_len              5
#define reserved2              6
#define ee_sys_calibrationtoken 7
#define reserved4               8
#define ee_darknightmode_lowlimit         9
#define ee_darknightmode_highlimit        10
#define ee_darknightmode    	11
#define ee_sys_sku				12
#define reserved3    	        13
#define ee_cal                  14
#define ee_ddl_end_yr           15
#define ee_ddl_end_mon          16
#define ee_ddl_end_day          17
#define ee_ddl_end_hr           18
#define ee_ddl_end_min          19
#define ee_ddl_end_sec          20
#define ee_ddl_start_yr         21
#define ee_ddl_start_mon        22
#define ee_ddl_start_day        23
#define ee_ddl_start_hr         24
#define ee_ddl_start_min        25
#define ee_ddl_start_sec		26
#define ee_timenightmode				27
#define ee_timenightmode_on_hrs				28
#define ee_timenightmode_on_min				29
#define ee_timenightmode_off_hrs				30
#define ee_timenightmode_off_min				31
#define ee_show1                32
#define ee_show2                33
#define ee_show3                34
#define ee_show4                35
#define ee_show5                36
#define ee_show6                37
#define ee_show1_time           38
#define ee_show2_time           39
#define ee_show3_time           40
#define ee_show4_time           41
#define ee_show5_time           42
#define ee_show6_time           43
#define ee_message_text         44


#define NOTHING    				'0'
#define DEADLINEDOTS    			'1'
#define TIMEDOTS    			'2'
#define TIMECLOCK    			'3'
#define IMAGEDOTS    			'4'
#define MESSAGE    			'5'
#define WEATHER    				'6'
#define HOURS_ONLY 			'7'
#define MINUTES_ONLY 				'8'
#define PAUSE                                   'A'

#define HRS_24    				'1'
#define HRS_12    				'0'

#define STARTED 1
#define ENDED 2

//MUSICAL NOTES
#define C6         31
#define Db6         30
#define D6         28
#define Eb6         26
#define E6         25
#define F6         23
#define Gb6         22
#define G6         21
#define Ab6         20
#define A6         19
#define Bb6         18
#define B6         17
#define C7         16
#define Db7         15
#define D7         14
#define Eb7         13
#define E7         12
#define F7         12
#define Gb7         11
#define G7         10
#define Ab7         10
#define A7         9
#define Bb7         9
#define B7         8

// PCF85063TP
#define PCF85063TP 				0x51
#define RTC_CTL1 				0x00
#define RTC_CTL2 				0x01
#define RTC_OFFSET 				0x02
#define RTC_RAM_BYTE 			0x03
#define RTC_SECONDS 			0x04
#define RTC_MINUTES 			0x05
#define RTC_HOURS 				0x06
#define RTC_DAYS 				0x07
#define RTC_WEEKDAYS 			0x08
#define RTC_MONTHS 				0x09
#define RTC_YEARS 				0x0A
#define RTC_SEC_ALARM           0x0B
#define RTC_MIN_ALARM           0x0C



#define mask_60_u 				0x0F
#define mask_60_t 				0x70
#define mask_12hr_u 			0x0F
#define mask_12hr_t 			0x10
#define mask_24hr_u 			0x0F
#define mask_24hr_t 			0x30
#define mask_ampm 				0x20
#define mask_day_u 				0x0F
#define mask_day_t 				0x30
#define mask_mon_u 				0x0F
#define mask_mon_t 				0x10
#define mask_yr_u 				0x0F
#define mask_yr_t 				0xF0
#define mask_clock_integrity                    0x80

//rtc time array addresses
#define rtctime_sec   			0
#define rtctime_min   			1
#define rtctime_hour   			2
#define rtctime_day   			3
#define rtctime_weekday   		4
#define rtctime_mon   			5
#define rtctime_year   			6

//OPT3001
#define OPT3001 				0x44
#define AMBLT_RESULT 			0x00
#define AMBLT_CONF 				0x01
#define AMBLT_LOW_LIMIT 		0x02
#define AMBLT_HIGH_LIMIT 		0x03
#define AMBLT_ID 				0x7E
#define AMBLT_DEVICE_ID 		0x7F
//
#define symbol_xpause 			50


#include <time.h>
#include "app_error.h"
#include "nrf_drv_twi.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"


/* TWI instance ID. */
#define TWI_INSTANCE_ID     0



/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);



/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
    case NRF_DRV_TWI_EVT_DONE:
        if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
        {
            ///      data_handler(m_sample);///display temp etc
        }
        m_xfer_done = true;
        break;
    default:
        break;
    }
}


/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lm75b_config =
    {
        .scl                = ARDUINO_SCL_PIN,
        .sda                = ARDUINO_SDA_PIN,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
        .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_lm75b_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}


#define DISPLAY_HEAD 0x80
#define DISPLAY_INSTANT 0x87
#define DISPLAY_DELAYED 0x88
#define DISPLAY_REFRESH 0x82
#define DISPLAY_END 0x8F
#define DISPLAY_ADDR 0

// bro system
#define BRO0    0
#define BRO1    1
#define BRO2    2

uint8_t display_buffer[21] =
{
    0xe6, 0x8d, 0x70, 0xea, 0x19, 0x6b, 0xdf,//bro1
    0x41, 0x5e, 0x2d, 0x4c, 0xc6, 0xb8, 0x90,//bro2
    0xee, 0xb4, 0x72, 0xa5, 0xbb, 0x21, 0x0d//bro3
};

bool display_busy=false,countdown_timer_running=false,countdown_timer_setup_mode=false,draw_mode=false, time_correct=false, dark_mode_check=false;
uint8_t countm=0, current_bro=BRO0, brocount=1,ddl_status=0;
uint16_t refresh_speed_scroll,refresh_speed_time,refresh_speed_time_sep,stepj=0, countdown_timer=0,times_ups=0;
uint32_t number_handler_status=0;

uint8_t ee_settings[144]="";
uint8_t stngs_t_sep[4]= {':',0x27,'.',';'};
uint8_t stngs_d_sep[4]= {'/','-',',',' '};
uint8_t disp_tx_buffer[11]= {DISPLAY_HEAD,DISPLAY_INSTANT,DISPLAY_ADDR,EMPTY,EMPTY,EMPTY,EMPTY,EMPTY,EMPTY,EMPTY,DISPLAY_END};

const uint8_t symbol_len_map[69]= {5,3,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,1,5,5,5,5,6,4,4,3,5,2,3,1,5,1,2,4,5,4,5,6,3,3,5,5,5,4,1,4,5,1,5,7};
const uint16_t k_multiplier[6]= {0,0,1,10,100,1000};
const uint16_t scroll_speed_table[11]= {1,450,260,190,160,130,90,60,50,40,30};
const uint16_t time_sep_speed_table[11]= {1,720,600,500,400,360,260,200,130,100,80};
const uint16_t time_speed_table[11]= {1,1200,850,680,580,500,400,280,200,170,150};
const uint16_t frequency_table[11]= {1,15900,14000,12000,10000,8000,7000,6000,5000,2000,1000};
const uint16_t amb_lt_table[11]= {0,6,12,24,48,96,192,384,768,1536,3072};
const uint16_t ppm_to_calib_table[277]= {0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,3,4,4,4,4,5,5,5,5,6,6,6,6,6,7,7,7,7,8,8,8,8,9,9,9,9,9,10,10,10,10,11,11,11,11,12,12,12,12,12,13,13,13,13,14,14,14,14,15,15,15,15,15,16,16,16,16,17,17,17,17,18,18,18,18,18,19,19,19,19,20,20,20,20,21,21,21,21,21,22,22,22,22,23,23,23,23,24,24,24,24,24,25,25,25,25,26,26,26,26,26,27,27,27,27,28,28,28,28,29,29,29,29,29,30,30,30,30,31,31,31,31,32,32,32,32,32,33,33,33,33,34,34,34,34,35,35,35,35,35,36,36,36,36,37,37,37,37,38,38,38,38,38,39,39,39,39,40,40,40,40,41,41,41,41,41,42,42,42,42,43,43,43,43,44,44,44,44,44,45,45,45,45,46,46,46,46,47,47,47,47,47,48,48,48,48,49,49,49,49,50,50,50,50,50,51,51,51,51,52,52,52,52,53,53,53,53,53,54,54,54,54,55,55,55,55,56,56,56,56,56,57,57,57,57,58,58,58,58,59,59,59,59,59,60,60,60,60,61,61,61,61,62,62,62,62,62,63,63,63,63,64};
const uint16_t calib_to_ppm_table[23]= {0,4,9,13,17,22,26,30,35,39,43,48,52,56,61,65,69,74,78,82,87,91,95};

const uint8_t disp_buffer_remap1[7]= {0x40,0x20,0x10,0x08,0x04,0x02,0x01};
const uint8_t dotness_map[8]= {0x00,0x40,0x60,0x70,0x78,0x7C,0x7E,0x7F};
const uint8_t bro_disp_buf_addr[4]= {0,7,14,21};

const uint8_t symbolmap[483]= //69bytes/symbols per line
{
    0x0e, 0x06, 0x0e, 0x0e, 0x12, 0x1f, 0x0f, 0x1f, 0x0e, 0x0e, 0x0e, 0x1e, 0x0e, 0x1e, 0x1f, 0x1f,
    0x0e, 0x11, 0x1f, 0x1f, 0x11, 0x10, 0x11, 0x11, 0x0e, 0x1e, 0x0e, 0x1e, 0x0f, 0x1f, 0x11, 0x11,
    0x11, 0x11, 0x11, 0x1f, 0x01, 0x1b, 0x0a, 0x04, 0x0d, 0x18, 0x07, 0x0e, 0x02, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x0e, 0x07, 0x07, 0x04, 0x00, 0x00, 0x03, 0x01,
    0x0c, 0x10, 0x01, 0x00, 0x7f, 0x11, 0x02, 0x11, 0x11, 0x12, 0x10, 0x10, 0x01, 0x11, 0x11, 0x11,
    0x11, 0x11, 0x11, 0x10, 0x10, 0x11, 0x11, 0x04, 0x02, 0x11, 0x10, 0x1b, 0x19, 0x11, 0x11, 0x11,
    0x11, 0x10, 0x04, 0x11, 0x11, 0x11, 0x11, 0x11, 0x01, 0x01, 0x09, 0x0a, 0x0f, 0x15, 0x24, 0x08,
    0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x11, 0x04, 0x01,
    0x0a, 0x00, 0x00, 0x04, 0x01, 0x02, 0x10, 0x01, 0x00, 0x5d, 0x13, 0x02, 0x01, 0x01, 0x12, 0x10,
    0x10, 0x01, 0x11, 0x11, 0x11, 0x11, 0x10, 0x11, 0x10, 0x10, 0x10, 0x11, 0x04, 0x02, 0x12, 0x10,
    0x15, 0x15, 0x11, 0x11, 0x11, 0x11, 0x10, 0x04, 0x11, 0x11, 0x11, 0x0a, 0x11, 0x02, 0x01, 0x09,
    0x1f, 0x14, 0x1e, 0x18, 0x08, 0x01, 0x02, 0x04, 0x00, 0x00, 0x00, 0x02, 0x01, 0x01, 0x03, 0x00,
    0x0c, 0x01, 0x07, 0x04, 0x01, 0x11, 0x00, 0x00, 0x04, 0x01, 0x02, 0x08, 0x00, 0x00, 0x63, 0x15,
    0x02, 0x0e, 0x06, 0x1f, 0x1e, 0x1e, 0x06, 0x0e, 0x0f, 0x11, 0x1e, 0x10, 0x11, 0x1c, 0x1c, 0x17,
    0x1f, 0x04, 0x02, 0x1c, 0x10, 0x15, 0x13, 0x11, 0x1e, 0x11, 0x11, 0x0e, 0x04, 0x11, 0x11, 0x15,
    0x04, 0x0a, 0x04, 0x01, 0x12, 0x0a, 0x0e, 0x04, 0x29, 0x08, 0x01, 0x05, 0x04, 0x00, 0x00, 0x00,
    0x04, 0x00, 0x00, 0x04, 0x1f, 0x02, 0x06, 0x09, 0x04, 0x01, 0x00, 0x00, 0x09, 0x08, 0x01, 0x01,
    0x04, 0x00, 0x00, 0x45, 0x19, 0x02, 0x10, 0x01, 0x02, 0x01, 0x11, 0x08, 0x11, 0x01, 0x1f, 0x11,
    0x10, 0x11, 0x10, 0x10, 0x11, 0x11, 0x04, 0x12, 0x12, 0x10, 0x11, 0x11, 0x11, 0x10, 0x11, 0x1e,
    0x01, 0x04, 0x11, 0x11, 0x15, 0x0a, 0x04, 0x08, 0x01, 0x00, 0x1f, 0x05, 0x0b, 0x26, 0x08, 0x01,
    0x00, 0x1f, 0x03, 0x1f, 0x00, 0x08, 0x00, 0x00, 0x08, 0x00, 0x01, 0x04, 0x07, 0x04, 0x01, 0x00,
    0x00, 0x16, 0x04, 0x01, 0x02, 0x02, 0x00, 0x00, 0x49, 0x11, 0x02, 0x10, 0x11, 0x02, 0x01, 0x11,
    0x08, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x10, 0x10, 0x11, 0x11, 0x04, 0x12, 0x11, 0x10, 0x11,
    0x11, 0x11, 0x10, 0x15, 0x11, 0x01, 0x04, 0x11, 0x0a, 0x1b, 0x11, 0x04, 0x10, 0x00, 0x00, 0x0a,
    0x1e, 0x15, 0x24, 0x08, 0x01, 0x00, 0x04, 0x01, 0x00, 0x00, 0x10, 0x01, 0x01, 0x04, 0x1f, 0x02,
    0x00, 0x11, 0x04, 0x01, 0x00, 0x00, 0x00, 0x04, 0x01, 0x02, 0x01, 0x00, 0x00, 0x49, 0x0e, 0x07,
    0x1f, 0x0e, 0x02, 0x1e, 0x0e, 0x08, 0x0e, 0x0e, 0x11, 0x1e, 0x0e, 0x1e, 0x1f, 0x10, 0x0e, 0x11,
    0x1f, 0x0c, 0x11, 0x1f, 0x11, 0x11, 0x0e, 0x10, 0x0e, 0x11, 0x1e, 0x04, 0x0e, 0x04, 0x11, 0x11,
    0x04, 0x1f, 0x01, 0x00, 0x0a, 0x04, 0x17, 0x1b, 0x07, 0x0e, 0x00, 0x04, 0x02, 0x00, 0x01, 0x10,
    0x00, 0x03, 0x03, 0x00, 0x0c, 0x04, 0x1e, 0x07, 0x07, 0x00, 0x1f, 0x00, 0x03, 0x01, 0x0c, 0x01,
    0x00, 0x00, 0x7f
};

const uint8_t symbolmap_half_font[70]= //10bytes/symbols per line
{
    0x07, 0x06, 0x07, 0x07, 0x05, 0x07, 0x07, 0x07, 0x07, 0x07, 0x05, 0x02, 0x01, 0x01, 0x05, 0x04,
    0x04, 0x01, 0x05, 0x05, 0x05, 0x02, 0x01, 0x01, 0x05, 0x04, 0x04, 0x01, 0x05, 0x05, 0x05, 0x02,
    0x02, 0x02, 0x07, 0x07, 0x07, 0x01, 0x02, 0x07, 0x05, 0x02, 0x04, 0x01, 0x01, 0x01, 0x05, 0x02,
    0x05, 0x01, 0x05, 0x02, 0x04, 0x01, 0x01, 0x01, 0x05, 0x02, 0x05, 0x01, 0x07, 0x07, 0x07, 0x07,
    0x01, 0x07, 0x07, 0x02, 0x07, 0x07
};

uint8_t rtctime[7] ="";
uint8_t rtctimehex[7] ="";

const uint8_t usb_img[7] = {0x7F, 0x7F, 0x7F, 0x14, 0x7F, 0x7F, 0x7F};
const uint8_t test1_img[7] = {0x55, 0x2A, 0x55, 0x2A, 0x55, 0x2A, 0x55};
const uint8_t bad_clock_img[7] = {0x3e, 0x49, 0x49, 0x4d, 0x41, 0x41, 0x3e};
const uint8_t am_img[7] = {0x07, 0x05, 0x05, 0x07, 0x05, 0x05, 0x05};
const uint8_t pm_img[7] = {0x07, 0x05, 0x05, 0x07, 0x04, 0x04, 0x04};
const uint8_t three_dots_img[7] = {0x00, 0x00, 0x00, 0x2a, 0x00, 0x00, 0x00};
const uint8_t plus_img[7] = {0x00, 0x08, 0x08, 0x3e, 0x08, 0x08, 0x00};
const uint8_t sleep_img[7] = {0x00, 0x00, 0x77, 0x00, 0x08, 0x00, 0x00};

int32_t divRoundClosest(const int32_t n, const int32_t d)
{
    return ((n < 0) ^ (d < 0)) ? ((n - d/2)/d) : ((n + d/2)/d);
}

void init_gpio(void)
{

    nrf_gpio_cfg(
        RS_485_DIR,
        NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_NOSENSE);

    nrf_gpio_cfg(
        AMB_LT_INT,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        NRF_GPIO_PIN_PULLUP,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_SENSE_LOW);

    nrf_gpio_cfg(
        MIN_INT,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        NRF_GPIO_PIN_PULLUP,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_SENSE_LOW);

    nrf_gpio_cfg(
        SEC_INT_CALIB_OUT,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        NRF_GPIO_PIN_PULLUP,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_SENSE_LOW);


    nrf_gpio_pin_write(RS_485_DIR,1);//enable tx on the RS-485

}

void i2c_tx(uint8_t addr, uint8_t const * p_data, uint8_t length, bool no_stop)
{
    uint32_t                err_code;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, addr, p_data, length, no_stop);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}

void i2c_rx(uint8_t addr, uint8_t * p_data, uint8_t length)
{
    uint32_t                err_code;
    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi, addr, p_data, length);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}



static void calibration_enter(void)
{

    //RTC CALIB ENTRY CODE STARTS HERE

    const uint8_t RTC_ENTER_CALIB[2]= {RTC_CTL2,0x23}; // SECONDS OUTPUT NOW OUTPUTS 4096HZ
    i2c_tx(PCF85063TP,RTC_ENTER_CALIB,sizeof(RTC_ENTER_CALIB), false);

}

static void uart_send(uint8_t * p_data, uint16_t length)
{
    uint32_t err_code;

    for (uint32_t i = 0; i < length; i++)
    {
        do
        {
            err_code = app_uart_put(p_data[i]);
            if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. \r\n", err_code);
                APP_ERROR_CHECK(err_code);
            }
        }
        while (err_code == NRF_ERROR_BUSY);
    }
    if (p_data[length-1] == '\r')
    {
        while (app_uart_put('\n') == NRF_ERROR_BUSY);
    }

    // nrf_gpio_pin_write(RS_485_DIR,0);//disable tx on the RS-485

}

static void disp_refresh(void)
{
    unsigned char i;

    disp_tx_buffer[3]=0;
    disp_tx_buffer[4]=0;
    disp_tx_buffer[5]=0;
    disp_tx_buffer[6]=0;
    disp_tx_buffer[7]=0;
    disp_tx_buffer[8]=0;
    disp_tx_buffer[9]=0;

    for (i=0; i<7; i++)
    {

        //mirror image and copy
        disp_tx_buffer[i+3] |= (display_buffer[i+bro_disp_buf_addr[current_bro]] & 0x01)<<6;
        disp_tx_buffer[i+3] |= (display_buffer[i+bro_disp_buf_addr[current_bro]] & 0x02)<<4;
        disp_tx_buffer[i+3] |= (display_buffer[i+bro_disp_buf_addr[current_bro]] & 0x04)<<2;
        disp_tx_buffer[i+3] |= (display_buffer[i+bro_disp_buf_addr[current_bro]] & 0x08);
        disp_tx_buffer[i+3] |= (display_buffer[i+bro_disp_buf_addr[current_bro]] & 0x10)>>2;
        disp_tx_buffer[i+3] |= (display_buffer[i+bro_disp_buf_addr[current_bro]] & 0x20)>>4;
        disp_tx_buffer[i+3] |= (display_buffer[i+bro_disp_buf_addr[current_bro]] & 0x40)>>6;
    }
    uart_send(disp_tx_buffer,sizeof(disp_tx_buffer));
}

static void disp_clear_buffer(uint8_t bro)
{
    uint8_t i;
    for (i=0; i<7; i++)
    {
        display_buffer[i+bro_disp_buf_addr[bro]]=0x00;
    }
};


static void OPT3001_init(void)
{
    uint8_t init_opt1[3]= {AMBLT_LOW_LIMIT,EMPTY,EMPTY};
    uint8_t init_opt2[3]= {AMBLT_HIGH_LIMIT,EMPTY,EMPTY};
    uint8_t init_opt3[3]= {AMBLT_CONF,0xC4,0x01};

    init_opt1[1]=(amb_lt_table[ee_settings[ee_darknightmode_lowlimit]]&0xFF00)>>8;
    init_opt1[2]=amb_lt_table[ee_settings[ee_darknightmode_lowlimit]]&0x00FF;
    init_opt2[1]=(amb_lt_table[ee_settings[ee_darknightmode_highlimit]]&0xFF00)>>8;
    init_opt2[2]=amb_lt_table[ee_settings[ee_darknightmode_highlimit]]&0x00FF;
    i2c_tx(OPT3001,init_opt1,sizeof(init_opt1), false);
    i2c_tx(OPT3001,init_opt2,sizeof(init_opt2), false);
    i2c_tx(OPT3001,init_opt3,sizeof(init_opt3), false);

    
}


static bool OPT3001_check_ID(void)
{
  
    uint8_t read_opt1[1] = {AMBLT_DEVICE_ID};
    uint8_t read_opt2[2] = {EMPTY,EMPTY};
    i2c_tx(OPT3001,read_opt1,sizeof(read_opt1),true);
    i2c_rx(OPT3001,read_opt2,sizeof(read_opt2));
    
    if ((read_opt2[0]!=0x30)||(read_opt2[1]!=0x01))
    {
      ble_nus_string_send(&m_nus, "OPT_READ_ERROR\n",15);
      nrf_delay_ms(200);
      return true;
    }
    else
    {
      return false;
    }
}

static void calibration_exit(void)
{

    //RTC CALIB Exit CODE STARTS HERE
    const uint8_t RTC_EXIT_CALIB[2]= {RTC_CTL2,0x26}; // minute and second outputs enabled
    i2c_tx(PCF85063TP,RTC_EXIT_CALIB,sizeof(RTC_EXIT_CALIB), false);

}

static bool PCF85063_check_ID(void)
{
      uint8_t read_rtc0[1] = {RTC_SEC_ALARM};
  
      i2c_tx(PCF85063TP,read_rtc0,sizeof(read_rtc0),true);
      i2c_rx(PCF85063TP,read_rtc0,sizeof(read_rtc0));
      
    if (!(read_rtc0[0]&0x80))
    {
      nrf_gpio_pin_write(LED_RED,!true);
      ble_nus_string_send(&m_nus, "RTC_READ_ERROR\n",15);
      nrf_delay_ms(200);
      return true;
    }
    else
    {
      return false;
    }
}

static void PCF85063_init(void)
{
  

  
  
    uint8_t init_rtc3[2]= {RTC_OFFSET,EMPTY};
    init_rtc3[1]=ee_settings[ee_cal];

    uint8_t init_rtc1[2]= {RTC_CTL1,0x01};
    uint8_t init_rtc2[2]= {RTC_CTL2,0x26};

    if(ee_settings[ee_t_format]==HRS_12)
    {
        init_rtc1[1] |= 0x02;
    }

    i2c_tx(PCF85063TP,init_rtc3,sizeof(init_rtc3), false);
    i2c_tx(PCF85063TP,init_rtc2,sizeof(init_rtc2), false);
    i2c_tx(PCF85063TP,init_rtc1,sizeof(init_rtc1), false);

    

    
    if(ee_settings[ee_sys_calibrationtoken]!='1')
    {
        calibration_enter();
        ble_nus_string_send(&m_nus, "CALIB_REQUEST\n",14);
    }
}



//max input size 256 bytes
static void ee_i2c_mass_write(uint8_t addr, uint8_t *datax, uint8_t size)
{
    uint8_t eepage[18]="";
    uint8_t i,g,pagescount;

    pagescount=size/16;

    for (g = 0; g < pagescount; g++)
    {
        eepage[0]=(g*16);//memory address

        for (i = 0; i < 16; i++)
        {
            eepage[i+1]=datax[i+(g*16)];
        }
        i2c_tx(addr, eepage, 17, false);
        nrf_delay_ms(6);
    }
}



static void disp_invert_buffer(uint8_t bro)
{
    uint8_t i;
    for (i=0; i<7; i++)
    {
        display_buffer[i+bro_disp_buf_addr[bro]] = ~display_buffer[i+bro_disp_buf_addr[bro]];
    }
}

static void display_img(const uint8_t *image, uint8_t invert, uint8_t bro)
{
    uint8_t i;
    display_busy=1;

    //copy from one buffer to the other
    for (i=0; i<7; i++)
    {
        display_buffer[i+bro_disp_buf_addr[bro]]=image[i];
    }

    if(invert)
    {
        disp_invert_buffer(bro);
    }
    disp_refresh();
}

static void store_ee_settings(void)
{
    ee_i2c_mass_write(EEPROM,ee_settings,sizeof(ee_settings));
}
static void read_ee_settings(void)
{
    uint8_t read_ee[1] = {0};

    i2c_tx(EEPROM,read_ee,sizeof(read_ee),true);
    i2c_rx(EEPROM,ee_settings,sizeof(ee_settings));
}
static void store_ee_settings_partial(uint8_t memaddr, uint8_t val)
{
    uint8_t storedata[2]="";
    storedata[0]=memaddr;
    storedata[1]=val;

    i2c_tx(EEPROM,storedata,sizeof(storedata),false);
    nrf_delay_ms(6);
    read_ee_settings();
}
static void write_default_ee_settings(void)
{

    ee_settings[ee_sys_defaultstoken]=EMPTY;
    ee_settings[ee_t_format]=HRS_24;
    ee_settings[ee_t_sep]='0';
    ee_settings[ee_msg_speed]=5;
    ee_settings[ee_msg_len]=4;
    ee_settings[ee_sys_calibrationtoken]=0xFF;//reset calibration validness
    ee_settings[ee_darknightmode_lowlimit]=1;
    ee_settings[ee_darknightmode_highlimit]=2;
    ee_settings[ee_darknightmode]='1';
    ee_settings[ee_cal]=0;
    //  ee_settings[ee_ddl_yr]='';
    //  ee_settings[ee_ddl_mon]='';
    //  ee_settings[ee_ddl_day]='';
    //  ee_settings[ee_ddl_hr]='';
    //  ee_settings[ee_ddl_min]='';
    //  ee_settings[ee_ddl_sec]='';
    ee_settings[ee_timenightmode]='0';
    ee_settings[ee_timenightmode_on_hrs]=0;
    ee_settings[ee_timenightmode_on_min]=0;
    ee_settings[ee_timenightmode_off_hrs]=8;
    ee_settings[ee_timenightmode_off_min]=0;
    ee_settings[ee_sys_sku]=CURRENT_SKU;
    ee_settings[ee_show1]=TIMEDOTS;
    ee_settings[ee_show2]=NOTHING;
    ee_settings[ee_show3]=NOTHING;
    ee_settings[ee_show4]=NOTHING;
    ee_settings[ee_show5]=NOTHING;
    ee_settings[ee_show6]=NOTHING;
    ee_settings[ee_show1_time]=10;//in hundreds of miliseconds
    ee_settings[ee_show2_time]=NOTHING;//in hundreds of miliseconds
    ee_settings[ee_show3_time]=NOTHING;//in hundreds of miliseconds
    ee_settings[ee_show4_time]=NOTHING;//in hundreds of miliseconds
    ee_settings[ee_show5_time]=NOTHING;//in hundreds of miliseconds
    ee_settings[ee_show6_time]=NOTHING;//in hundreds of miliseconds
    ee_settings[ee_message_text]='H';
    ee_settings[ee_message_text+1]='E';
    ee_settings[ee_message_text+2]='Y';
    ee_settings[ee_message_text+3]='!';
    
    store_ee_settings();
    
    //store token separately, after all data were written
    store_ee_settings_partial(ee_sys_defaultstoken,CURRENT_VERSION_TOKEN);

    
}


static bool check_ee_token(void)
{
    uint8_t read_ee[1] = {ee_sys_defaultstoken};
    i2c_tx(EEPROM,read_ee,sizeof(read_ee),true);
    i2c_rx(EEPROM,read_ee,sizeof(read_ee));
    if (read_ee[0]!=CURRENT_VERSION_TOKEN)
    {
      ble_nus_string_send(&m_nus, "EE_READ_ERROR\n",14);
      nrf_delay_ms(200);
      return true;
    }
    else
    {
      return false;
    }
}



/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

void command_reply_nope(void)
{
    ble_nus_string_send(&m_nus,"NOPE\n",5);
}
void command_reply_nan(void)
{
    ble_nus_string_send(&m_nus,"NAN\n",5);
}
void command_reply_range(void)
{
    ble_nus_string_send(&m_nus,"RANGE\n",6);
}
void command_reply_ok(void)
{
    ble_nus_string_send(&m_nus,"OK\n",3);
}
void command_reply_questionmark(void)
{
    ble_nus_string_send(&m_nus,"?\n",2);
}
void command_reply_sign(void)
{
    ble_nus_string_send(&m_nus,"NOSIGN\n",7);
}

uint8_t parameter_number_handler(uint8_t* string_data, uint8_t maxlimit,uint8_t minlimit,uint8_t position)
{
    uint16_t tempval=0;

    if((string_data[position]>='0')&&(string_data[position]<='9'))
    {
        tempval = tempval+((string_data[position]-'0')*10);
        if((string_data[position+1]>='0')&&(string_data[position+1]<='9'))
        {
            tempval = tempval+(string_data[position+1]-'0');
            number_handler_status=0;
        }
        else
        {
            number_handler_status=NaN;
        }
    }
    else
    {
        number_handler_status=NaN;
    }
    if(number_handler_status==NaN)
    {
        command_reply_nan();
    }
    else
    {
        if((tempval<=maxlimit)&&(tempval>=minlimit))
        {
            number_handler_status=0;
            return tempval;
        }
        else
        {
            number_handler_status=NaN;
            command_reply_range();
        }
    }
    return 0;
}



static void disp_fill_buffer(uint8_t bro)
{
    uint8_t i;
    for (i=0; i<7; i++)
    {
        display_buffer[i+bro_disp_buf_addr[bro]]=0x7F;
    }
};

static void display_pixel(uint8_t x, uint8_t y, bool color,uint8_t bro)
{
    if(draw_mode)
    {
        if(color)
        {
            display_buffer[y+bro_disp_buf_addr[bro]] |= disp_buffer_remap1[x];
        }
        else
        {
            display_buffer[y+bro_disp_buf_addr[bro]] &= ~disp_buffer_remap1[x];
        }
        disp_refresh();
    }
}

int8_t symbol_adress_translate(uint8_t characterx)
{
    switch(characterx)
    {
    case '0':
        return 0;
    case '1':
        return 1;
    case '2':
        return 2;
    case '3':
        return 3;
    case '4':
        return 4;
    case '5':
        return 5;
    case '6':
        return 6;
    case '7':
        return 7;
    case '8':
        return 8;
    case '9':
        return 9;
    case 'A':
        return 10;
    case 'B':
        return 11;
    case 'C':
        return 12;
    case 'D':
        return 13;
    case 'E':
        return 14;
    case 'F':
        return 15;
    case 'G':
        return 16;
    case 'H':
        return 17;
    case 'I':
        return 18;
    case 'J':
        return 19;
    case 'K':
        return 20;
    case 'L':
        return 21;
    case 'M':
        return 22;
    case 'N':
        return 23;
    case 'O':
        return 24;
    case 'P':
        return 25;
    case 'Q':
        return 26;
    case 'R':
        return 27;
    case 'S':
        return 28;
    case 'T':
        return 29;
    case 'U':
        return 30;
    case 'V':
        return 31;
    case 'W':
        return 32;
    case 'X':
        return 33;
    case 'Y':
        return 34;
    case 'Z':
        return 35;
    case '!':
        return 36;
    case '"':
        return 37;
    case '#':
        return 38;
    case '$':
        return 39;
    case '%':
        return 40;
    case '&':
        return 41;
    case '(':
        return 42;
    case ')':
        return 43;
    case '*':
        return 44;
    case '+':
        return 45;
    case ',':
        return 46;
    case '-':
        return 47;
    case '.':
        return 48;
    case '/':
        return 49;
    case ':':
        return 50;
    case ';':
        return 51;
    case '<':
        return 52;
    case '=':
        return 53;
    case '>':
        return 54;
    case '?':
        return 55;
    case '@':
        return 56;
    case '[':
        return 57;
    case ']':
        return 58;
    case '^':
        return 59;
    case '_':
        return 60;
    case '~':
        return 61;
    case '{':
        return 62;
    case '|':
        return 63;
    case '}':
        return 64;
    case 0x5C:
        return 65;
    case 0x27:
        return 66;
    case ' ':
        return 67;
    default:
        return 68;
    }
}

static void PCF85063_gettime(void)
{

    
    uint8_t read_time[1]= {RTC_SECONDS};

    i2c_tx(PCF85063TP,read_time,sizeof(read_time),true);
    i2c_rx(PCF85063TP,rtctime,sizeof(rtctime));

    
    if(rtctime[rtctime_sec]&mask_clock_integrity)
    {
      time_correct=false;
      ble_nus_string_send(&m_nus, "TIME_REQUEST\n",13);
    }
    else
    {
      time_correct=true;      
    }
    
    
    
    ////////////ADDITIONAL CONVERTION TO HEX BELOW
    rtctimehex[rtctime_sec]  = (((rtctime[rtctime_sec]&mask_60_t)>>4)*10)+(rtctime[rtctime_sec]&mask_60_u);
    rtctimehex[rtctime_min]  = (((rtctime[rtctime_min]&mask_60_t)>>4)*10)+(rtctime[rtctime_min]&mask_60_u);

    uint8_t hours;

    if(ee_settings[ee_t_format]==HRS_12)
    {
        hours  = (((rtctime[rtctime_hour]&mask_12hr_t)>>4)*10)+rtctime[rtctime_hour]&mask_12hr_u;

        //if time is in 12hr format convert time to 24 hour format
        if((rtctime[rtctime_hour]&mask_ampm)&&(hours==12))
        {
            //is 12pm
            rtctimehex[rtctime_hour] = 12;
        }
        else if(rtctime[rtctime_hour]&mask_ampm)
        {
            //is pm
            rtctimehex[rtctime_hour]= hours+12;
        }
        else if(hours==12)
        {
            //is 12am
            rtctimehex[rtctime_hour] = 0;
        }
        else
        {
            rtctimehex[rtctime_hour]  = hours;
        }
    }
    else
    {
        rtctimehex[rtctime_hour]  = (((rtctime[rtctime_hour]&mask_24hr_t)>>4)*10)+(rtctime[rtctime_hour]&mask_24hr_u);
    }

    rtctimehex[rtctime_day] = (((rtctime[rtctime_day]&mask_day_t)>>4)*10)+rtctime[rtctime_day]&mask_day_u;
    rtctimehex[rtctime_weekday] = rtctime[rtctime_weekday] ;
    rtctimehex[rtctime_mon] = (((rtctime[rtctime_mon]&mask_mon_t)>>4)*10)+rtctime[rtctime_mon]&mask_mon_u;
    rtctimehex[rtctime_year] =  (((rtctime[rtctime_year]&mask_yr_t)>>4)*10)+rtctime[rtctime_year]&mask_yr_u;

}

//@FZ1HJM
static void PCF85063_settime(void)
{
    uint8_t i;

    
    rtctime[rtctime_sec]&= ~mask_clock_integrity;
 
    
    
    
    uint8_t set_time[8]= {RTC_SECONDS,EMPTY,EMPTY,EMPTY,EMPTY,EMPTY,EMPTY,EMPTY};
    //uint8_t set_time[8]={RTC_SECONDS,0x50,0x59,0x21,0x04,0,0x08,0};

    for(i=0; i<7; i++)
    {
        set_time[i+1]=rtctime[i];
    }
    i2c_tx(PCF85063TP,set_time,sizeof(set_time),false);
}

static void convert_time_to_24hr(void)
{
    display_img(test1_img,0,BRO0);
    uint8_t hours  = (((rtctime[rtctime_hour]&mask_12hr_t)>>4)*10)+rtctime[rtctime_hour]&mask_12hr_u;

    if((rtctime[rtctime_hour]&mask_ampm)&&(hours==12))
    {
        //is 12pm
        rtctime[rtctime_hour] = 1<<4 | 2;
    }
    else if(rtctime[rtctime_hour]&mask_ampm)
    {
        //is pm
        hours=hours+12;
        rtctime[rtctime_hour] = (hours/10)<<4 | hours%10;
    }
    else if(hours==12)
    {
        //is 12am
        rtctime[rtctime_hour] = 0;
    }
    else
    {
        //is am
        rtctime[rtctime_hour] = (hours/10)<<4 | hours%10;
    }
}

static void convert_time_to_12hr(void)
{
    display_img(test1_img,1,BRO0);
    uint8_t hours = (((rtctime[rtctime_hour]&mask_24hr_t)>>4)*10)+(rtctime[rtctime_hour]&mask_24hr_u);

    if(hours==12)
    {
        //is 12
        rtctime[rtctime_hour] = mask_ampm | (hours/10)<<4 | hours%10;
    }
    else if(hours==0)
    {
        //is 0
        rtctime[rtctime_hour] = 1<<4 | 2;
    }
    else if(hours>12)
    {
        //is >12
        hours=hours-12;
        rtctime[rtctime_hour] = mask_ampm | (hours/10)<<4 | hours%10;
    }
}

static void PCF85063_setCALIB(void)
{
    uint8_t set_calib[2]= {RTC_OFFSET,EMPTY};
    set_calib[1]=ee_settings[ee_cal];
    i2c_tx(PCF85063TP,set_calib,sizeof(set_calib),false);

    store_ee_settings_partial(ee_sys_calibrationtoken,'1');
    calibration_exit();
}

static void display_symbol(uint8_t symbol, uint8_t invert,uint8_t bro)
{
    uint8_t i,symbolshift=0;
    uint8_t h=symbol_adress_translate(symbol);

    display_busy=1;
    disp_clear_buffer(bro);

    switch(symbol_len_map[h])//shift symbol for center justification
    {
    case 1:
        symbolshift=3;
        break;
    case 2:
        symbolshift=2;
        break;
    case 3:
        symbolshift=2;
        break;
    case 4:
        symbolshift=1;
        break;
    case 5:
        symbolshift=1;
        break;
    case 6:
        symbolshift=0;
        break;
    case 7:
        symbolshift=0;
        break;
    }

    //copy from one buffer to the other
    for (i=0; i<7; i++)
    {
        display_buffer[i] |= (symbolmap[(h+(i*69))])<<(symbolshift);
    }

    if(invert)
    {
        disp_invert_buffer(bro);
    }
    disp_refresh();
}


static void DEADLINEDOTS_start(void)
{
    ee_settings[ee_ddl_start_hr] = rtctimehex[rtctime_hour];
    ee_settings[ee_ddl_start_min] = rtctimehex[rtctime_min];
    ee_settings[ee_ddl_start_sec] = rtctimehex[rtctime_sec];
    ee_settings[ee_ddl_start_mon] = rtctimehex[rtctime_mon];
    ee_settings[ee_ddl_start_day] = rtctimehex[rtctime_day];
    ee_settings[ee_ddl_start_yr] = rtctimehex[rtctime_year];

    ddl_status = STARTED;
}

static void display_ddl_ended_anim(uint16_t howlong_ms)
{
  
/*  change images in these, then uncomment ///f/
    display_img(countdown_timer_img, 0, BRO0);
    display_img(countdown_timer_img, 1, BRO1);
    display_img(countdown_timer_img, 0, BRO2);
    nrf_delay_ms(howlong_ms/4);

    display_img(countdown_timer_img, 1, BRO0);
    display_img(countdown_timer_img, 0, BRO1);
    display_img(countdown_timer_img, 1, BRO2);
    nrf_delay_ms(howlong_ms/4);

    display_img(countdown_timer_img, 0, BRO0);
    display_img(countdown_timer_img, 1, BRO1);
    display_img(countdown_timer_img, 0, BRO2);
    nrf_delay_ms(howlong_ms/4);

    display_img(countdown_timer_img, 1, BRO0);
    display_img(countdown_timer_img, 0, BRO1);
    display_img(countdown_timer_img, 1, BRO2);
    
    */
    
    nrf_delay_ms(howlong_ms/4);
}

static void check_darknightmode(void)
{
  if(bsp_board_led_state_get(0) || bsp_board_led_state_get(1) || bsp_board_led_state_get(2))
  {
    
  }
  else
  {
    dark_mode_check = nrf_gpio_pin_read(AMB_LT_INT);
  }
}

void command_responder(uint8_t * bt_received_string_data)
{
    if(bt_received_string_data[2]==',')
    {
        switch(bt_received_string_data[0])
        {
        case 'p':
            if(bt_received_string_data[1]=='q')//@SLW6ON
            {
                if(bt_received_string_data[3]=='0')
                {
                    //ping
                    ble_nus_string_send(&m_nus,"CAKE1\n",6);
                }
                else if(bt_received_string_data[3]=='1')
                {

                    disp_clear_buffer(BRO0);
                    disp_clear_buffer(BRO1);
                    disp_clear_buffer(BRO2);
                    disp_refresh();

                    ////D/dfu mode?
                }
                else if(bt_received_string_data[3]=='2')
                {
                    //reset
                    disp_clear_buffer(BRO0);
                    disp_clear_buffer(BRO1);
                    disp_clear_buffer(BRO2);
                    disp_refresh();
                    sd_nvic_SystemReset();
                }
                else if(bt_received_string_data[3]=='3')
                {
                    disp_clear_buffer(BRO0);
                    disp_clear_buffer(BRO1);
                    disp_clear_buffer(BRO2);
                    disp_refresh();
                    ///write all default values to eeprom
                    write_default_ee_settings();
                    command_reply_ok();
                }
                else if(bt_received_string_data[3]=='4')
                {
                    DEADLINEDOTS_start();
                    command_reply_ok();
                }
                else if(bt_received_string_data[3]=='5')
                {
                    calibration_enter();
                    command_reply_ok();
                }
                else if(bt_received_string_data[3]=='6')
                {
                    calibration_exit();
                    command_reply_ok();
                }
                else
                {
                    command_reply_nope();
                }
            }
            else
            {
                command_reply_questionmark();
            }
            break;
        case 's':
            switch(bt_received_string_data[1])
            {
            case 'a'://@V060L4
                PCF85063_gettime();

                parameter_number_handler(bt_received_string_data,59,0,3);
                if(number_handler_status !=NaN)
                {
                    rtctime[rtctime_sec] =  (bt_received_string_data[3]-'0')<<4 | bt_received_string_data[4]-'0' ;

                    parameter_number_handler(bt_received_string_data,59,0,6);
                    if(number_handler_status !=NaN)
                    {
                        rtctime[rtctime_min] =  (bt_received_string_data[6]-'0')<<4 | bt_received_string_data[7]-'0' ;

                        parameter_number_handler(bt_received_string_data,23,0,9);
                        if(number_handler_status !=NaN)
                        {
                            rtctime[rtctime_hour] = (bt_received_string_data[9]-'0')<<4 | bt_received_string_data[10]-'0' ;

                            parameter_number_handler(bt_received_string_data,31,1,12);
                            if(number_handler_status !=NaN)
                            {
                                rtctime[rtctime_day] = (bt_received_string_data[12]-'0')<<4 | bt_received_string_data[13]-'0' ;

                                parameter_number_handler(bt_received_string_data,12,1,15);
                                if(number_handler_status !=NaN)
                                {
                                    rtctime[rtctime_mon] =  (bt_received_string_data[15]-'0')<<4 | bt_received_string_data[16]-'0' ;
                                    parameter_number_handler(bt_received_string_data,99,0,18);
                                    if(number_handler_status !=NaN)
                                    {
                                        rtctime[rtctime_year] =  (bt_received_string_data[18]-'0')<<4 | bt_received_string_data[19]-'0' ;

                                        if(ee_settings[ee_t_format]==HRS_12)
                                        {
                                            convert_time_to_12hr();
                                        }
                                        PCF85063_settime();
                                        command_reply_ok();
                                    }
                                }
                            }
                        }
                    }
                }

                break;
            case 'c'://@NKX3DI
                PCF85063_gettime();

                parameter_number_handler(bt_received_string_data,59,0,3);
                if(number_handler_status !=NaN)
                {
                    ee_settings[ee_ddl_end_sec] =  parameter_number_handler(bt_received_string_data,59,0,3); ;

                    parameter_number_handler(bt_received_string_data,59,0,6);
                    if(number_handler_status !=NaN)
                    {
                        ee_settings[ee_ddl_end_min] =  parameter_number_handler(bt_received_string_data,59,0,6);;

                        parameter_number_handler(bt_received_string_data,23,0,9);
                        if(number_handler_status !=NaN)
                        {
                            ee_settings[ee_ddl_end_hr] = parameter_number_handler(bt_received_string_data,23,0,9);

                            parameter_number_handler(bt_received_string_data,31,1,12);
                            if(number_handler_status !=NaN)
                            {
                                ee_settings[ee_ddl_end_day] = parameter_number_handler(bt_received_string_data,31,1,12);

                                parameter_number_handler(bt_received_string_data,12,1,15);
                                if(number_handler_status !=NaN)
                                {
                                    ee_settings[ee_ddl_end_mon] =  parameter_number_handler(bt_received_string_data,12,1,15);
                                    parameter_number_handler(bt_received_string_data,99,0,18);
                                    if(number_handler_status !=NaN)
                                    {
                                        ee_settings[ee_ddl_end_yr] =  parameter_number_handler(bt_received_string_data,99,0,18);

                                        int32_t timeleft_secs;
                                        struct tm start_date;
                                        struct tm end_date;
                                        time_t start_time, end_time;

                                        start_date.tm_hour = rtctimehex[rtctime_hour];
                                        start_date.tm_min = rtctimehex[rtctime_min];
                                        start_date.tm_sec = rtctimehex[rtctime_sec];
                                        start_date.tm_mon = rtctimehex[rtctime_mon];
                                        start_date.tm_mday = rtctimehex[rtctime_day];
                                        start_date.tm_year = 100+rtctimehex[rtctime_year];
                                        end_date.tm_hour = ee_settings[ee_ddl_end_hr];
                                        end_date.tm_min = ee_settings[ee_ddl_end_min];
                                        end_date.tm_sec = ee_settings[ee_ddl_end_sec];
                                        end_date.tm_mon = ee_settings[ee_ddl_end_mon];
                                        end_date.tm_mday = ee_settings[ee_ddl_end_day];
                                        end_date.tm_year = 100+ee_settings[ee_ddl_end_yr];
                                        start_time = mktime(&start_date);
                                        end_time = mktime(&end_date);
                                        timeleft_secs = (int32_t)difftime(end_time, start_time);

                                        if(timeleft_secs<=0)
                                        {
                                            ble_nus_string_send(&m_nus,"DDL NEG ETA\n",12);
                                        }
                                        else
                                        {
                                            store_ee_settings();
                                            command_reply_ok();
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                break;
            case 'd'://@GTVOQS
                if(bt_received_string_data[3]=='0' || bt_received_string_data[3]=='1')
                {
                    store_ee_settings_partial(ee_timenightmode,bt_received_string_data[3]);
                    command_reply_ok();
                }
                else
                {
                    command_reply_nope();
                }
                break;
            case 'e'://@GTU86U
                parameter_number_handler(bt_received_string_data,59,0,3);
                if(number_handler_status !=NaN)
                {
                    store_ee_settings_partial(ee_timenightmode_on_min,parameter_number_handler(bt_received_string_data,59,0,3));
                    command_reply_ok();

                    parameter_number_handler(bt_received_string_data,23,0,6);
                    if(number_handler_status !=NaN)
                    {
                        store_ee_settings_partial(ee_timenightmode_on_hrs,parameter_number_handler(bt_received_string_data,23,0,6));
                        command_reply_ok();
                    }
                }
                break;
            case 'f'://@FLC0MG
                parameter_number_handler(bt_received_string_data,59,0,3);
                if(number_handler_status !=NaN)
                {
                    store_ee_settings_partial(ee_timenightmode_off_min,parameter_number_handler(bt_received_string_data,59,0,3));
                    command_reply_ok();

                    parameter_number_handler(bt_received_string_data,23,0,6);
                    if(number_handler_status !=NaN)
                    {
                        store_ee_settings_partial(ee_timenightmode_off_hrs,parameter_number_handler(bt_received_string_data,23,0,6));
                        command_reply_ok();
                    }
                }
                break;
            case 'g'://@GXTP7I
                if(bt_received_string_data[3]=='0' || bt_received_string_data[3]=='1')
                {
                    store_ee_settings_partial(ee_darknightmode,bt_received_string_data[3]);
                    command_reply_ok();
                }
                else
                {
                    command_reply_nope();
                }
                break;
            case 'h'://@UCNXSH
                parameter_number_handler(bt_received_string_data,10,0,3);
                if(number_handler_status !=NaN)
                {
                    store_ee_settings_partial(ee_darknightmode_lowlimit,parameter_number_handler(bt_received_string_data,10,0,3));
                    OPT3001_init();
                    command_reply_ok();
                }
                break;
            case 'i'://@HO88XM
                parameter_number_handler(bt_received_string_data,10,0,3);
                if(number_handler_status !=NaN)
                {
                    store_ee_settings_partial(ee_darknightmode_highlimit,parameter_number_handler(bt_received_string_data,10,0,3));
                    OPT3001_init();
                    command_reply_ok();
                }
                break;
            case 'j'://@ZKQ9B3
                if(bt_received_string_data[3]==HRS_12 || bt_received_string_data[3]==HRS_24)
                {
                    if(bt_received_string_data[3]==HRS_24 && ee_settings[ee_t_format]==HRS_12)
                    {
                        store_ee_settings_partial(ee_t_format,bt_received_string_data[3]);
                        PCF85063_init();
                        PCF85063_gettime();
                        convert_time_to_24hr();
                        PCF85063_settime();
                    }
                    else if(bt_received_string_data[3]==HRS_12 && ee_settings[ee_t_format]==HRS_24)
                    {
                        store_ee_settings_partial(ee_t_format,bt_received_string_data[3]);
                        PCF85063_init();
                        PCF85063_gettime();
                        convert_time_to_12hr();
                        PCF85063_settime();
                    }
                    command_reply_ok();
                }
                else
                {
                    command_reply_nope();
                }
                break;
            case 'k'://@NQSFDQ

                break;
            case 'l'://@6V886A
                parameter_number_handler(bt_received_string_data,10,1,3);
                if(number_handler_status !=NaN)
                {
                    store_ee_settings_partial(ee_msg_speed,parameter_number_handler(bt_received_string_data,10,1,3));
                    command_reply_ok();
                }
                break;
            case 'm'://@UU0FEH
                switch(bt_received_string_data[3])
                {
                case '+':
                    ee_settings[ee_cal] = 0;
                    ee_settings[ee_cal] &= ~0x40;
                    ee_settings[ee_cal]|= ppm_to_calib_table[parameter_number_handler(bt_received_string_data,99,0,4)];

                    store_ee_settings_partial(ee_cal,ee_settings[ee_cal]);

                    PCF85063_setCALIB();
                    command_reply_ok();
                    break;
                case '-':
                    ee_settings[ee_cal] = 0;
                    ee_settings[ee_cal] |= 0x40;
                    ee_settings[ee_cal]|= ppm_to_calib_table[parameter_number_handler(bt_received_string_data,99,0,4)];

                    store_ee_settings_partial(ee_cal,ee_settings[ee_cal]);

                    PCF85063_setCALIB();
                    command_reply_ok();
                    break;
                default:
                    command_reply_sign();
                    break;
                }
                break;
            default:
                command_reply_questionmark();
                break;
            }
            break;
        case 'w':
            switch(bt_received_string_data[1])
            {
            case 'a'://@A43OJD
                if(bt_received_string_data[3]==NOTHING || bt_received_string_data[3]==DEADLINEDOTS || bt_received_string_data[3]==TIMEDOTS || bt_received_string_data[3]==TIMECLOCK || bt_received_string_data[3]==IMAGEDOTS || bt_received_string_data[3]==WEATHER || bt_received_string_data[3]==MESSAGE || bt_received_string_data[3]==PAUSE  || bt_received_string_data[3]==MINUTES_ONLY  || bt_received_string_data[3]==HOURS_ONLY)
                {
                    store_ee_settings_partial(ee_show1,bt_received_string_data[3]);
                    parameter_number_handler(bt_received_string_data,99,1,5);
                    if(number_handler_status !=NaN)
                    {
                        store_ee_settings_partial(ee_show1_time,parameter_number_handler(bt_received_string_data,99,1,5));
                        command_reply_ok();
                    }
                }
                else
                {
                    command_reply_nope();
                }
                break;
            case 'b':
                if(bt_received_string_data[3]==NOTHING || bt_received_string_data[3]==DEADLINEDOTS || bt_received_string_data[3]==TIMEDOTS || bt_received_string_data[3]==TIMECLOCK || bt_received_string_data[3]==IMAGEDOTS || bt_received_string_data[3]==WEATHER || bt_received_string_data[3]==MESSAGE || bt_received_string_data[3]==PAUSE)
                {
                    store_ee_settings_partial(ee_show2,bt_received_string_data[3]);
                    parameter_number_handler(bt_received_string_data,99,1,5);
                    if(number_handler_status !=NaN)
                    {
                        store_ee_settings_partial(ee_show2_time,parameter_number_handler(bt_received_string_data,99,1,5));
                        command_reply_ok();
                    }
                }
                else
                {
                    command_reply_nope();
                }
                break;
            case 'c':
                if(bt_received_string_data[3]==NOTHING || bt_received_string_data[3]==DEADLINEDOTS || bt_received_string_data[3]==TIMEDOTS || bt_received_string_data[3]==TIMECLOCK || bt_received_string_data[3]==IMAGEDOTS || bt_received_string_data[3]==WEATHER || bt_received_string_data[3]==MESSAGE || bt_received_string_data[3]==PAUSE)
                {
                    store_ee_settings_partial(ee_show3,bt_received_string_data[3]);
                    parameter_number_handler(bt_received_string_data,99,1,5);
                    if(number_handler_status !=NaN)
                    {
                        store_ee_settings_partial(ee_show3_time,parameter_number_handler(bt_received_string_data,99,1,5));
                        command_reply_ok();
                    }
                }
                else
                {
                    command_reply_nope();
                }
                break;
            case 'd':
                if(bt_received_string_data[3]==NOTHING || bt_received_string_data[3]==DEADLINEDOTS || bt_received_string_data[3]==TIMEDOTS || bt_received_string_data[3]==TIMECLOCK || bt_received_string_data[3]==IMAGEDOTS || bt_received_string_data[3]==WEATHER || bt_received_string_data[3]==MESSAGE || bt_received_string_data[3]==PAUSE)
                {
                    store_ee_settings_partial(ee_show4,bt_received_string_data[3]);
                    parameter_number_handler(bt_received_string_data,99,1,5);
                    if(number_handler_status !=NaN)
                    {
                        store_ee_settings_partial(ee_show4_time,parameter_number_handler(bt_received_string_data,99,1,5));
                        command_reply_ok();
                    }
                }
                else
                {
                    command_reply_nope();
                }
                break;
            case 'e':
                if(bt_received_string_data[3]==NOTHING || bt_received_string_data[3]==DEADLINEDOTS || bt_received_string_data[3]==TIMEDOTS || bt_received_string_data[3]==TIMECLOCK || bt_received_string_data[3]==IMAGEDOTS || bt_received_string_data[3]==WEATHER || bt_received_string_data[3]==MESSAGE || bt_received_string_data[3]==PAUSE)
                {
                    store_ee_settings_partial(ee_show5,bt_received_string_data[3]);
                    parameter_number_handler(bt_received_string_data,99,1,5);
                    if(number_handler_status !=NaN)
                    {
                        store_ee_settings_partial(ee_show5_time,parameter_number_handler(bt_received_string_data,99,1,5));
                        command_reply_ok();
                    }
                }
                else
                {
                    command_reply_nope();
                }
                break;
            case 'f':
                if(bt_received_string_data[3]==NOTHING || bt_received_string_data[3]==DEADLINEDOTS || bt_received_string_data[3]==TIMEDOTS || bt_received_string_data[3]==TIMECLOCK || bt_received_string_data[3]==IMAGEDOTS || bt_received_string_data[3]==WEATHER || bt_received_string_data[3]==MESSAGE || bt_received_string_data[3]==PAUSE)
                {
                    store_ee_settings_partial(ee_show6,bt_received_string_data[3]);
                    parameter_number_handler(bt_received_string_data,99,1,5);
                    if(number_handler_status !=NaN)
                    {
                        store_ee_settings_partial(ee_show6_time,parameter_number_handler(bt_received_string_data,99,1,5));
                        command_reply_ok();
                    }
                }
                else
                {
                    command_reply_nope();
                }
                break;
            default:
                command_reply_questionmark();
                break;
            }
            break;
        case 'c':
            switch(bt_received_string_data[1])
            {
            case 'a'://@FSR3HA
                display_symbol(bt_received_string_data[3],0,BRO0);
                break;
            case 'b'://@U3KIEK
                if(bt_received_string_data[3]=='0')
                {
                    disp_fill_buffer(BRO0);
                    disp_fill_buffer(BRO1);
                    disp_fill_buffer(BRO2);

                    disp_refresh();
                    nrf_delay_ms(1000);
                    command_reply_ok();
                }
                else if(bt_received_string_data[3]=='1')
                {
                    // blank
                    disp_clear_buffer(BRO0);
                    disp_clear_buffer(BRO1);
                    disp_clear_buffer(BRO2);
                    disp_refresh();
                    nrf_delay_ms(1000);
                    command_reply_ok();
                }
                else if(bt_received_string_data[3]=='2')
                {
                    // all
                    display_symbol('0',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('1',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('2',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('3',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('4',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('5',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('6',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('7',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('8',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('9',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('A',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('B',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('C',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('D',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('E',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('F',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('G',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('H',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('I',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('J',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('K',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('L',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('M',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('N',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('O',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('P',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('Q',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('R',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('S',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('T',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('U',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('V',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('W',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('X',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('Y',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('Z',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('!',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('"',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('#',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('$',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('%',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('&',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('(',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol(')',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('*',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('+',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol(',',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('-',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('.',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('/',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol(':',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol(';',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('<',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('=',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('>',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('?',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('@',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('[',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol(']',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('^',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('_',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('~',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('{',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('|',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol('}',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol(0x5C,0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol(0x27,0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol(' ',0,BRO0);
                    nrf_delay_ms(400);
                    display_symbol(0xFF,0,BRO0);
                    nrf_delay_ms(400);
                }
                else
                {
                    command_reply_nope();
                }
                break;
            case 'c'://@ONMESS
                if(bt_received_string_data[3]=='0')
                {
                    draw_mode=false;
                    command_reply_ok();
                    // exit
                }
                else if(bt_received_string_data[3]=='1')
                {
                    draw_mode=true;
                    disp_clear_buffer(BRO0);
                    disp_clear_buffer(BRO1);
                    disp_clear_buffer(BRO2);

                    command_reply_ok();
                    // enter
                }
                else
                {
                    command_reply_nope();
                }
                break;
            case 'd'://@D1FVG4
                parameter_number_handler(bt_received_string_data,20,0,3);
                if(number_handler_status !=NaN)
                {
                    display_pixel(parameter_number_handler(bt_received_string_data,20,0,3),parameter_number_handler(bt_received_string_data,6,0,6),1,parameter_number_handler(bt_received_string_data,2,0,9));
                    command_reply_ok();
                }
                break;
            case 'e'://@D71FLG
                parameter_number_handler(bt_received_string_data,20,0,3);
                if(number_handler_status !=NaN)
                {
                    display_pixel(parameter_number_handler(bt_received_string_data,20,0,3),parameter_number_handler(bt_received_string_data,6,0,6),0,parameter_number_handler(bt_received_string_data,2,0,9));
                    command_reply_ok();
                }
                break;
            default:
                command_reply_questionmark();
                break;
            }
            break;
        }
    }
    else if(bt_received_string_data[2]=='?')///  //READBACK COMMANDS START HERE
    {
        switch(bt_received_string_data[0])
        {
        case 'p':
            if(bt_received_string_data[1]=='q')//@Q970G8
            {
              
                PCF85063_gettime();
                check_darknightmode();
                     
                uint8_t reply_string[29]="pq,*,*,*,*,*,*,*,*,*,*,*,*,*\n";
                reply_string[3]=CURRENT_SKU;
                reply_string[5]=CURRENT_VERSION_TOKEN;//VERSION
                reply_string[7]=current_bro+'0';//BRONUMBER
                reply_string[9]=brocount+'0';//BROTOTAL                                            
                reply_string[11]=dark_mode_check+'0';// should read true when dark
                reply_string[13]=bsp_board_led_state_get(0)+'0';//BLUE LED   should read true    
                reply_string[15]=bsp_board_led_state_get(1)+'0';//GREEN LED                   
                reply_string[17]=bsp_board_led_state_get(2)+'0';//RED LED        
                reply_string[19]=nrf_gpio_pin_read(SEC_INT_CALIB_OUT)+'0';//clock pin    
                reply_string[21]=check_ee_token()+'0';// should read false
                reply_string[23]=PCF85063_check_ID()+'0';// should read false
                reply_string[25]=OPT3001_check_ID()+'0';// should read false              
                reply_string[27]=time_correct+'0';// should read true if time correct
                

                ble_nus_string_send(&m_nus,reply_string,sizeof(reply_string));

                if(ee_settings[ee_sys_calibrationtoken]!='1')
                {
                    ble_nus_string_send(&m_nus, "CALIB_REQUEST\n",14);
                }

            }
            else
            {
                command_reply_questionmark();
            }
            break;
        case 's':

            switch(bt_received_string_data[1])
            {
            case 'a'://@8U11CV
            {
                PCF85063_gettime();

                uint8_t reply_string1[21]="sa,**,**,**,**,**,**\n";

                reply_string1[3]=((rtctimehex[rtctime_sec]%100)/10)+'0';
                reply_string1[4]=(rtctimehex[rtctime_sec]%10)+'0';

                reply_string1[6]=((rtctimehex[rtctime_min]%100)/10)+'0';
                reply_string1[7]=(rtctimehex[rtctime_min]%10)+'0';

                reply_string1[9]=((rtctimehex[rtctime_hour]%100)/10)+'0';
                reply_string1[10]=(rtctimehex[rtctime_hour]%10)+'0';

                reply_string1[12]=((rtctimehex[rtctime_day]%100)/10)+'0';
                reply_string1[13]=(rtctimehex[rtctime_day]%10)+'0';

                reply_string1[15]=((rtctimehex[rtctime_mon]%100)/10)+'0';
                reply_string1[16]=(rtctimehex[rtctime_mon]%10)+'0';

                reply_string1[18]=((rtctimehex[rtctime_year]%100)/10)+'0';
                reply_string1[19]=(rtctimehex[rtctime_year]%10)+'0';

                ble_nus_string_send(&m_nus,reply_string1,sizeof(reply_string1));

                break;
            }
            case 'c'://@KTSTP6
            {
                PCF85063_gettime();

                uint8_t reply_string2[21]="sc,**,**,**,**,**,**\n";

                reply_string2[3]=((ee_settings[ee_ddl_end_sec]%100)/10)+'0';
                reply_string2[4]=(ee_settings[ee_ddl_end_sec]%10)+'0';

                reply_string2[6]=((ee_settings[ee_ddl_end_min]%100)/10)+'0';
                reply_string2[7]=(ee_settings[ee_ddl_end_min]%10)+'0';

                reply_string2[9]=((ee_settings[ee_ddl_end_hr]%100)/10)+'0';
                reply_string2[10]=(ee_settings[ee_ddl_end_hr]%10)+'0';

                reply_string2[12]=((ee_settings[ee_ddl_end_day]%100)/10)+'0';
                reply_string2[13]=(ee_settings[ee_ddl_end_day]%10)+'0';

                reply_string2[15]=((ee_settings[ee_ddl_end_mon]%100)/10)+'0';
                reply_string2[16]=(ee_settings[ee_ddl_end_mon]%10)+'0';

                reply_string2[18]=((ee_settings[ee_ddl_end_yr]%100)/10)+'0';
                reply_string2[19]=(ee_settings[ee_ddl_end_yr]%10)+'0';

                ble_nus_string_send(&m_nus,reply_string2,sizeof(reply_string2));
                break;
            }
            case 'd'://@VJ2B2X
            {
                uint8_t reply_string3[5]="sd,*\n";
                reply_string3[3]=ee_settings[ee_timenightmode];
                ble_nus_string_send(&m_nus,reply_string3,sizeof(reply_string3));
                break;
            }

            case 'e'://@XU58CJ
            {
                uint8_t reply_string4[9]="se,**,**\n";

                reply_string4[3]=((ee_settings[ee_timenightmode_on_min]%100)/10)+'0';
                reply_string4[4]=(ee_settings[ee_timenightmode_on_min]%10)+'0';

                reply_string4[6]=((ee_settings[ee_timenightmode_on_hrs]%100)/10)+'0';
                reply_string4[7]=(ee_settings[ee_timenightmode_on_hrs]%10)+'0';

                ble_nus_string_send(&m_nus,reply_string4,sizeof(reply_string4));
                break;
            }
            case 'f'://@AL3N9P
            {
                uint8_t reply_string5[9]="sf,**,**\n";

                reply_string5[3]=((ee_settings[ee_timenightmode_off_min]%100)/10)+'0';
                reply_string5[4]=(ee_settings[ee_timenightmode_off_min]%10)+'0';

                reply_string5[6]=((ee_settings[ee_timenightmode_off_hrs]%100)/10)+'0';
                reply_string5[7]=(ee_settings[ee_timenightmode_off_hrs]%10)+'0';

                ble_nus_string_send(&m_nus,reply_string5,sizeof(reply_string5));
                break;
            }

            case 'g'://@Y8LU7Y
            {
                uint8_t reply_string6[5]="sg,*\n";
                reply_string6[3]=ee_settings[ee_darknightmode];
                ble_nus_string_send(&m_nus,reply_string6,sizeof(reply_string6));
                break;
            }
            case 'h'://@D8HTIB
            {
                uint8_t reply_string7[6]="sh,**\n";
                reply_string7[3]=((ee_settings[ee_darknightmode_lowlimit]%100)/10)+'0';
                reply_string7[4]=(ee_settings[ee_darknightmode_lowlimit]%10)+'0';
                ble_nus_string_send(&m_nus,reply_string7,sizeof(reply_string7));
                break;
            }
            case 'i'://@LA255U
            {
                uint8_t reply_string8[6]="si,**\n";
                reply_string8[3]=((ee_settings[ee_darknightmode_highlimit]%100)/10)+'0';
                reply_string8[4]=(ee_settings[ee_darknightmode_highlimit]%10)+'0';
                ble_nus_string_send(&m_nus,reply_string8,sizeof(reply_string8));
                break;
            }
            case 'j'://@WRYH0H
            {
                uint8_t reply_string9[5]="sj,*\n";
                reply_string9[3]=ee_settings[ee_t_format];
                ble_nus_string_send(&m_nus,reply_string9,sizeof(reply_string9));
                break;
            }
            case 'k'://@H4QO0M

                break;
            case 'l'://@BRDAH7
            {
                uint8_t reply_string10[6]="sl,**\n";
                reply_string10[3]=((ee_settings[ee_msg_speed]%100)/10)+'0';
                reply_string10[4]=(ee_settings[ee_msg_speed]%10)+'0';
                ble_nus_string_send(&m_nus,reply_string10,sizeof(reply_string10));
                break;
            }
            break;
            case 'm'://@2T67HB
            {
            uint8_t reply_string11[7]="sm,***\n";

                uint8_t read_calib[1]= {RTC_OFFSET};

                i2c_tx(PCF85063TP,read_calib,sizeof(read_calib),true);
                i2c_rx(PCF85063TP,read_calib,sizeof(read_calib));

                
                if(read_calib[0]&0x40)
                {
                    reply_string11[3]='-';
                }
                else
                {
                    reply_string11[3]='+';
                }
                reply_string11[4]=((calib_to_ppm_table[read_calib[0]&0x3F]%100)/10)+'0';
                reply_string11[5]=(calib_to_ppm_table[read_calib[0]&0x3F]%10)+'0';
                ble_nus_string_send(&m_nus,reply_string11,sizeof(reply_string11));
                break;
            }
            break;
            default:
                command_reply_questionmark();
                break;
            }
            break;
        case 'w':
            switch(bt_received_string_data[1])
            {
            case 'a'://@ETG35Z
            {
                uint8_t reply_string12[8]="wa,*,**\n";
                reply_string12[3]=ee_settings[ee_show1];

                reply_string12[5]=((ee_settings[ee_show1_time]%100)/10)+'0';
                reply_string12[6]=(ee_settings[ee_show1_time]%10)+'0';

                ble_nus_string_send(&m_nus,reply_string12,sizeof(reply_string12));
                break;
            }
            case 'b'://@UWXAUJ
            {
                uint8_t reply_string13[8]="wb,*,**\n";
                reply_string13[3]=ee_settings[ee_show2];

                reply_string13[5]=((ee_settings[ee_show2_time]%100)/10)+'0';
                reply_string13[6]=(ee_settings[ee_show2_time]%10)+'0';

                ble_nus_string_send(&m_nus,reply_string13,sizeof(reply_string13));
                break;
            }
            break;
            case 'c'://@UAU47I
            {
                uint8_t reply_string14[8]="wc,*,**\n";
                reply_string14[3]=ee_settings[ee_show3];

                reply_string14[5]=((ee_settings[ee_show3_time]%100)/10)+'0';
                reply_string14[6]=(ee_settings[ee_show3_time]%10)+'0';

                ble_nus_string_send(&m_nus,reply_string14,sizeof(reply_string14));
                break;
            }
            break;
            case 'd'://@E728X5
            {
                uint8_t reply_string15[8]="wd,*,**\n";
                reply_string15[3]=ee_settings[ee_show4];

                reply_string15[5]=((ee_settings[ee_show4_time]%100)/10)+'0';
                reply_string15[6]=(ee_settings[ee_show4_time]%10)+'0';

                ble_nus_string_send(&m_nus,reply_string15,sizeof(reply_string15));
                break;
            }
            break;
            case 'e'://@3XKC47
            {
                uint8_t reply_string16[8]="we,*,**\n";
                reply_string16[3]=ee_settings[ee_show5];

                reply_string16[5]=((ee_settings[ee_show5_time]%100)/10)+'0';
                reply_string16[6]=(ee_settings[ee_show5_time]%10)+'0';

                ble_nus_string_send(&m_nus,reply_string16,sizeof(reply_string16));
                break;
            }
            break;
            case 'f'://@0W7A12
            {
                uint8_t reply_string17[8]="wf,*,**\n";
                reply_string17[3]=ee_settings[ee_show6];

                reply_string17[5]=((ee_settings[ee_show6_time]%100)/10)+'0';
                reply_string17[6]=(ee_settings[ee_show6_time]%10)+'0';

                ble_nus_string_send(&m_nus,reply_string17,sizeof(reply_string17));
                break;
            }
            break;
            default:
                command_reply_questionmark();
                break;
            }
            break;
        case 'c':
            switch(bt_received_string_data[1])
            {
            case 'c'://@1249TQ
            {
                uint8_t reply_string18[5]="cc,*\n";

                if(draw_mode)
                {
                    reply_string18[3]='1';
                }
                else
                {
                    reply_string18[3]='0';
                }
                ble_nus_string_send(&m_nus,reply_string18,sizeof(reply_string18));
                break;
            }
            default:
                command_reply_questionmark();
                break;
            }
            break;
        }
    }
    else
    {
        command_reply_questionmark();
    }
}



/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    // uint32_t err_code;

    NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.\r\n");
    NRF_LOG_HEXDUMP_DEBUG(p_data, length);

    command_responder(p_data);
    /*
    for (uint32_t i = 0; i < length; i++)
    {
        do
        {
            err_code = app_uart_put(p_data[i]);
            if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. \r\n", err_code);
                APP_ERROR_CHECK(err_code);
            }
        } while (err_code == NRF_ERROR_BUSY);
    }
    if (p_data[length-1] == '\r')
    {
        while (app_uart_put('\n') == NRF_ERROR_BUSY);
    }
    */
}
/**@snippet [Handling the data received over BLE] */



/*
//done
static void disp_shift_buffer(void)
{
    uint8_t i;
    for (i=0; i<sizeof(display_buffer); i++) {
        display_buffer[i] = display_buffer[i]<<1;
    }
}

*/







/*
static void draw_image(void)
{



}
*/



/*
static void display_double_digits(uint8_t num, uint8_t invert, uint8_t bro)
{
    uint8_t i;

             disp_clear_buffer(bro);

            for (i=0; i<7; i++) {
                display_buffer[i+bro_disp_buf_addr[bro]] |= (symbolmap_half_font[((num%10)+(i*10))])<<0;
            }

            if (num>9)
            {
                for (i=0; i<7; i++) {
                    display_buffer[i+bro_disp_buf_addr[bro]] |= (symbolmap_half_font[(((num%100)/10)+(i*10))])<<4;
                }
            }

            if(invert)
            {
                disp_invert_buffer(bro);
            }
            disp_refresh();
}
*/


static void display_double_digits_bcd(uint8_t numt, uint8_t numu, uint8_t invert, uint8_t bro)
{
    uint8_t i;

    disp_clear_buffer(bro);

    for (i=0; i<7; i++)
    {
        display_buffer[i+bro_disp_buf_addr[bro]] |= (symbolmap_half_font[(numu+(i*10))])<<0;
    }

    if (numt)
    {
        for (i=0; i<7; i++)
        {
            display_buffer[i+bro_disp_buf_addr[bro]] |= (symbolmap_half_font[(numt+(i*10))])<<4;
        }
    }

    if(invert)
    {
        disp_invert_buffer(bro);
    }
    disp_refresh();


}

//to-do: remake for bro system
void display_dots(unsigned char numx, unsigned char invert)
{
    disp_clear_buffer(BRO0);
    disp_clear_buffer(BRO1);
    disp_clear_buffer(BRO2);


    if(brocount==1)
    {
        if (numx >=49)
        {
            display_buffer[0]=dotness_map[7];
            display_buffer[1]=dotness_map[7];
            display_buffer[2]=dotness_map[7];
            display_buffer[3]=dotness_map[7];
            display_buffer[4]=dotness_map[7];
            display_buffer[5]=dotness_map[7];
            display_buffer[6]=dotness_map[7];
        }
        else if (numx >42)
        {
            display_buffer[0]=dotness_map[7];
            display_buffer[1]=dotness_map[7];
            display_buffer[2]=dotness_map[7];
            display_buffer[3]=dotness_map[7];
            display_buffer[4]=dotness_map[7];
            display_buffer[5]=dotness_map[7];
            display_buffer[6]=dotness_map[numx-42];
        }
        else if (numx >35)
        {
            display_buffer[0]=dotness_map[7];
            display_buffer[1]=dotness_map[7];
            display_buffer[2]=dotness_map[7];
            display_buffer[3]=dotness_map[7];
            display_buffer[4]=dotness_map[7];
            display_buffer[5]=dotness_map[numx-35];
        }
        else if (numx >28)
        {
            display_buffer[0]=dotness_map[7];
            display_buffer[1]=dotness_map[7];
            display_buffer[2]=dotness_map[7];
            display_buffer[3]=dotness_map[7];
            display_buffer[4]=dotness_map[numx-28];
        }
        else if (numx >21)
        {
            display_buffer[0]=dotness_map[7];
            display_buffer[1]=dotness_map[7];
            display_buffer[2]=dotness_map[7];
            display_buffer[3]=dotness_map[numx-21];
        }
        else if (numx >14)
        {
            display_buffer[0]=dotness_map[7];
            display_buffer[1]=dotness_map[7];
            display_buffer[2]=dotness_map[numx-14];
        }
        else if (numx >7)
        {
            display_buffer[0]=dotness_map[7];
            display_buffer[1]=dotness_map[numx-7];
        }
        else
        {
            display_buffer[0]=dotness_map[numx];
        }

    }

    if(invert)
    {
        disp_invert_buffer(BRO0);
        disp_invert_buffer(BRO1);
        disp_invert_buffer(BRO2);
    }
    disp_refresh();

}


//@DCX5OH
//to-do: remake for bro system
static void show_message(uint8_t howlong)
{



    /*
    char i=0,h=0,symbolen=0,symbolshift=0,string_pos=0;
        display_busy=1;
        disp_clear_buffer(BRO0);
        disp_clear_buffer(BRO1);
        disp_clear_buffer(BRO2);

        for (string_pos=0; string_pos<ee_settings[ee_msg_len]; string_pos++)
        {
            h = symbol_adress_translate(ee_settings[ee_message_text+string_pos]);
            symbolen = symbol_len_map[h];
            for (symbolshift=symbolen; symbolshift>0; symbolshift--)
            {
                for (i=0; i<7; i++)
                {
                    display_buffer[i] |= ((symbolmap[(h+(i*69))])>>(symbolshift-1)) & 0x01;//copy from symbolmap -> shift to the right -> mask to single rightest line-> merge to disp buff
                }
                disp_refresh();
                disp_shift_buffer();
                delay_ms(refresh_speed_scroll);
            }
            disp_shift_buffer();//empty line placing
            disp_refresh();
            disp_shift_buffer();
            delay_ms(refresh_speed_scroll);
        }

        disp_clear_buffer(BRO0);
        disp_clear_buffer(BRO1);
        disp_clear_buffer(BRO2);

        display_symbol(' ',refresh_speed_time_sep,0);
        display_busy=0;
    */
}

uint32_t display_dots_timeratio(uint32_t timeleft_mins,uint32_t timetotal_mins)
{
    //input check
    if((timeleft_mins>timetotal_mins)||(timeleft_mins>87652390))
    {
        return NaN;
    }



    display_dots(divRoundClosest((timeleft_mins*49*brocount),timetotal_mins), 0);///

    return 0;
}



//@N5UHA7
static void show_DEADLINEDOTS(uint16_t howlong_ms)
{
    struct tm start_date;
    struct tm end_date;
    time_t start_time, end_time;
    int32_t timetotal_secs, timeleft_secs;

    PCF85063_gettime();
    if(time_correct)
    {
        switch(ddl_status)
        {
        case STARTED:

            start_date.tm_hour = ee_settings[ee_ddl_start_hr];
            start_date.tm_min = ee_settings[ee_ddl_start_min];
            end_date.tm_sec = ee_settings[ee_ddl_start_sec];
            start_date.tm_mon = ee_settings[ee_ddl_start_mon];
            start_date.tm_mday = ee_settings[ee_ddl_start_day];
            start_date.tm_year = 100+ee_settings[ee_ddl_start_yr];
            end_date.tm_hour = ee_settings[ee_ddl_end_hr];
            end_date.tm_min = ee_settings[ee_ddl_end_min];
            end_date.tm_sec = ee_settings[ee_ddl_end_sec];
            end_date.tm_mon = ee_settings[ee_ddl_end_mon];
            end_date.tm_mday = ee_settings[ee_ddl_end_day];
            end_date.tm_year = 100+ee_settings[ee_ddl_end_yr];
            start_time = mktime(&start_date);
            end_time = mktime(&end_date);
            timetotal_secs = (int32_t)difftime(end_time, start_time);
            if(timetotal_secs<=0)
            {
                ble_nus_string_send(&m_nus,"DDL NEG ETA\n",12);
                break;
            }

            start_date.tm_hour = rtctimehex[rtctime_hour];
            start_date.tm_min = rtctimehex[rtctime_min];
            start_date.tm_sec = rtctimehex[rtctime_sec];
            start_date.tm_mon = rtctimehex[rtctime_mon];
            start_date.tm_mday = rtctimehex[rtctime_day];
            start_date.tm_year = 100+rtctimehex[rtctime_year];
            end_date.tm_hour = ee_settings[ee_ddl_end_hr];
            end_date.tm_min = ee_settings[ee_ddl_end_min];
            end_date.tm_sec = ee_settings[ee_ddl_end_sec];
            end_date.tm_mon = ee_settings[ee_ddl_end_mon];
            end_date.tm_mday = ee_settings[ee_ddl_end_day];
            end_date.tm_year = 100+ee_settings[ee_ddl_end_yr];
            start_time = mktime(&start_date);
            end_time = mktime(&end_date);
            timeleft_secs = (int32_t)difftime(end_time, start_time);

            if(timeleft_secs<=0)
            {
                ble_nus_string_send(&m_nus,"DDL ENDED\n",10);
                ddl_status=ENDED;
                display_ddl_ended_anim(howlong_ms);
            }
            else
            {
                display_dots_timeratio(timeleft_secs,timetotal_secs);
                nrf_delay_ms(howlong_ms);
            }


        case ENDED:

            display_ddl_ended_anim(howlong_ms);
            break;
        default:
            display_symbol('?', 0,BRO0);
            ble_nus_string_send(&m_nus,"DDL NOT STARTED\n",16);
            nrf_delay_ms(howlong_ms);
            break;
        }
    }
    else
    {
      ///time incorrect?
      display_img(bad_clock_img, false, BRO0);
      nrf_delay_ms(howlong_ms);
    }
    
}


static void show_timedots(uint16_t howlong_ms)
{
    PCF85063_gettime();
    if(time_correct)
    {
          display_dots_timeratio((rtctimehex[rtctime_min]+(rtctimehex[rtctime_hour]*60)),1439);
    }
    else
    {
        ///time incorrect?
        display_img(bad_clock_img, false, BRO0);
        
    }

    nrf_delay_ms(howlong_ms);
}




/*
//done
static uint32_t OPT3001_getAMBLT(void)
{

	uint8_t read_amblt[1]={AMBLT_RESULT};
	uint8_t ambltresult[2]={EMPTY,EMPTY};

	i2c_tx(OPT3001,read_amblt,sizeof(read_amblt),true);
	i2c_rx(OPT3001,ambltresult,sizeof(ambltresult));


	return = (2^((ambltresult[0]&0xF0)>>4))*((ambltresult[0]&0x0F)<<8 | ambltresult[1]);


}
*/

/*
//NOT USED COZ GPIO CHECKING METHOD IS WAY SIMPLER
static bool OPT3001_check_if_nightmode(void)
{
	uint8_t read_amblt[1]={AMBLT_CONF};
	uint8_t ambltresult[2]={EMPTY,EMPTY};

    i2c_tx(OPT3001,read_amblt,sizeof(read_amblt),true);
	i2c_rx(OPT3001,ambltresult,sizeof(ambltresult));

    if(ee_settings[ee_darknightmode]=='1')
    {
            if (ambltresult[1]&0x20)
	        {
	          return true;
	        }
	        else if(ambltresult[1]&0x40)
	        {
	          return false;
	        }
	        else
	        {
	          return false;
	        }
    }
    else
    {
      	return false;
    }
}
*/



static void show_timeclock(uint16_t howlong_ms)
{

    uint8_t i;

    PCF85063_gettime();
    if(time_correct)
    {
      if(brocount==3)
        {
            if(ee_settings[ee_t_format]==HRS_24)
            {
                for (i=0; i<10; i++)
                {
                    display_double_digits_bcd((rtctime[rtctime_hour]&mask_24hr_t)>>4,rtctime[rtctime_hour]&mask_24hr_u,0,BRO0);
                    display_double_digits_bcd((rtctime[rtctime_min]&mask_60_t)>>4, rtctime[rtctime_min]&mask_60_u,0,BRO1);
                    display_double_digits_bcd((rtctime[rtctime_sec]&mask_60_t)>>4, rtctime[rtctime_sec]&mask_60_u,0,BRO2);
                    nrf_delay_ms(howlong_ms/10);
                }
            }
            else//12 hour format
            {
                for (i=0; i<5; i++)
                {
                    display_double_digits_bcd((rtctime[rtctime_hour]&mask_12hr_t)>>4,rtctime[rtctime_hour]&mask_12hr_u,0,BRO0);
                    display_double_digits_bcd((rtctime[rtctime_min]&mask_60_t)>>4, rtctime[rtctime_min]&mask_60_u,0,BRO1);
                    if(rtctime[rtctime_hour]&mask_ampm)///check if the register is correct here
                    {
                        display_img(pm_img,0,BRO2);
                    }
                    else
                    {
                        display_img(am_img,0,BRO2);
                    }
                    nrf_delay_ms(howlong_ms/5);
                }
            }
        }
        else if(brocount==2)
        {
            if(ee_settings[ee_t_format]==HRS_24)
            {
                for (i=0; i<5; i++)
                {
                    PCF85063_gettime();
                    display_double_digits_bcd((rtctime[rtctime_hour]&mask_24hr_t)>>4,rtctime[rtctime_hour]&mask_24hr_u,0,BRO0);
                    display_double_digits_bcd((rtctime[rtctime_min]&mask_60_t)>>4, rtctime[rtctime_min]&mask_60_u,0,BRO1);
                    nrf_delay_ms(howlong_ms/5);
                }
            }
            else
            {
                for (i=0; i<5; i++)
                {
                    PCF85063_gettime();
                    display_double_digits_bcd((rtctime[rtctime_hour]&mask_12hr_t)>>4,rtctime[rtctime_hour]&mask_12hr_u,0,BRO0);
                    display_double_digits_bcd((rtctime[rtctime_min]&mask_60_t)>>4, rtctime[rtctime_min]&mask_60_u,0,BRO1);
                    nrf_delay_ms(howlong_ms/5);
                }
            }
        }
        else if(brocount==1)
        {
            if(ee_settings[ee_t_format]==HRS_24)
            {
                PCF85063_gettime();
                display_double_digits_bcd((rtctime[rtctime_hour]&mask_24hr_t)>>4,rtctime[rtctime_hour]&mask_24hr_u,0,BRO0);
                nrf_delay_ms((howlong_ms*2)/5);
                display_symbol(stngs_t_sep[ee_settings[ee_t_sep]-'0'],0,BRO0);
                nrf_delay_ms((howlong_ms)/5);
                display_double_digits_bcd((rtctime[rtctime_min]&mask_60_t)>>4, rtctime[rtctime_min]&mask_60_u,0,BRO0);
                nrf_delay_ms((howlong_ms*2)/5);
            }
            else
            {
                display_double_digits_bcd((rtctime[rtctime_hour]&mask_12hr_t)>>4,rtctime[rtctime_hour]&mask_12hr_u,0,BRO0);
                nrf_delay_ms((howlong_ms/3));
                display_symbol(stngs_t_sep[ee_settings[ee_t_sep]-'0'],0,BRO0);
                nrf_delay_ms((howlong_ms/6));
                display_double_digits_bcd((rtctime[rtctime_min]&mask_60_t)>>4, rtctime[rtctime_min]&mask_60_u,0,BRO0);
                nrf_delay_ms((howlong_ms/3));
                if(rtctime[rtctime_hour]&mask_ampm)///check if the register is correct here
                {
                    display_img(pm_img,0,BRO0);
                }
                else
                {
                    display_img(am_img,0,BRO0);
                }
                nrf_delay_ms((howlong_ms/6));
            }

        }


        disp_clear_buffer(BRO0);
        disp_clear_buffer(BRO1);
        disp_clear_buffer(BRO2);

    //   display_symbol(' ',time_sep_speed_table[ee_settings[ee_t_speed]],0);
    }
    else
    {
            ///time incorrect?
      display_img(bad_clock_img, false, BRO0);
      nrf_delay_ms(howlong_ms);
    }

}
//


static void show_minutes_only(uint16_t howlong_ms)
{

    PCF85063_gettime();
    display_double_digits_bcd((rtctime[rtctime_min]&mask_60_t)>>4, rtctime[rtctime_min]&mask_60_u,0,BRO0);
    nrf_delay_ms(howlong_ms);

    disp_clear_buffer(BRO0);
    disp_clear_buffer(BRO1);
    disp_clear_buffer(BRO2);

//   display_symbol(' ',time_sep_speed_table[ee_settings[ee_t_speed]],0);
    display_busy=0;

}

static void show_hours_only(uint16_t howlong_ms)
{
    PCF85063_gettime();
    if(ee_settings[ee_t_format]==HRS_24)
    {
        display_double_digits_bcd((rtctime[rtctime_hour]&mask_24hr_t)>>4,rtctime[rtctime_hour]&mask_24hr_u,0,BRO0);
    }
    else
    {
        display_double_digits_bcd((rtctime[rtctime_hour]&mask_12hr_t)>>4,rtctime[rtctime_hour]&mask_12hr_u,0,BRO0);
    }

    nrf_delay_ms(howlong_ms);

    disp_clear_buffer(BRO0);
    disp_clear_buffer(BRO1);
    disp_clear_buffer(BRO2);

}
//@QD08WZ
static void show_imagedots(uint16_t howlong_ms)
{




}

//@QM2VU5
static void show_weather(uint16_t howlong_ms)
{




}
//done



static bool check_timesleepmode(void)
{
    uint16_t timecurrent,timeon,timeoff;


    PCF85063_gettime();



    timecurrent = (rtctimehex[rtctime_hour]*60)+rtctimehex[rtctime_min];
    timeon = (ee_settings[ee_timenightmode_on_hrs]*60)+ee_settings[ee_timenightmode_on_min];
    timeoff = (ee_settings[ee_timenightmode_off_hrs]*60)+ee_settings[ee_timenightmode_off_min];

    if(timeon==timeoff)//weird error situation here
    {
        ble_nus_string_send(&m_nus,"TIMESLEEP EQUAL TIMES\n",22);
        return false;
    }
    else if(timeon>timeoff)//SLEEP_ON is in the PM , SLEEP_OFF is in the AM
    {
        if((timecurrent>=timeon)||(timecurrent<timeoff))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else//SLEEP_ON is in the AM , SLEEP_OFF is in the PM
    {
        if((timecurrent>=timeon)&&(timecurrent<timeoff))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

}





static void display_thing(uint8_t what,uint16_t howlong_ms)
{
  check_darknightmode();
  
    if(draw_mode)
    {
      
    }
    else if((!(dark_mode_check&&(ee_settings[ee_darknightmode]-'0'))) && (!((ee_settings[ee_timenightmode]-'0') && check_timesleepmode())))
    {
        switch(what)
        {
        case DEADLINEDOTS:
            show_DEADLINEDOTS(howlong_ms);
            break;
        case TIMEDOTS:
            show_timedots(howlong_ms);
            break;
        case TIMECLOCK:
            show_timeclock(howlong_ms);
            break;
        case IMAGEDOTS:
            show_imagedots(howlong_ms);
            break;
        case WEATHER:
            show_weather(howlong_ms);
            break;
        case MESSAGE:
            show_message(howlong_ms);
            break;
        case MINUTES_ONLY:
            show_minutes_only(howlong_ms);
            break;
        case HOURS_ONLY:
            show_hours_only(howlong_ms);
            break;
        case NOTHING:
            break;

        default:
                                    // blank
                    disp_clear_buffer(BRO0);
                    disp_clear_buffer(BRO1);
                    disp_clear_buffer(BRO2);
                    disp_refresh();
            nrf_delay_ms(howlong_ms);
            break;
        }

    }
    else
    {
      
            display_img(sleep_img, false, BRO0);
            display_img(sleep_img, false, BRO1);
            display_img(sleep_img, false, BRO2);
            nrf_delay_ms(howlong_ms);
    }
}

void show_all(void)
{
    display_thing(ee_settings[ee_show1],ee_settings[ee_show1_time]*100);
    display_thing(ee_settings[ee_show2],ee_settings[ee_show2_time]*100);
    display_thing(ee_settings[ee_show3],ee_settings[ee_show3_time]*100);
    display_thing(ee_settings[ee_show4],ee_settings[ee_show4_time]*100);
    display_thing(ee_settings[ee_show5],ee_settings[ee_show5_time]*100);
    display_thing(ee_settings[ee_show6],ee_settings[ee_show6_time]*100);
}
////////////////////////////////////////display_functions end

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
    case BLE_ADV_EVT_FAST:
        err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
        APP_ERROR_CHECK(err_code);
        break;
    case BLE_ADV_EVT_IDLE:
        sleep_mode_enter();
        break;
    default:
        break;
    }
}


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
        APP_ERROR_CHECK(err_code);
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

        NRF_LOG_INFO("Connected\r\n");
        break; // BLE_GAP_EVT_CONNECTED

    case BLE_GAP_EVT_DISCONNECTED:
        err_code = bsp_indication_set(BSP_INDICATE_IDLE);
        APP_ERROR_CHECK(err_code);
        m_conn_handle = BLE_CONN_HANDLE_INVALID;
        NRF_LOG_INFO("Disconnected\r\n");
        break; // BLE_GAP_EVT_DISCONNECTED

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        // Pairing not supported
        err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
        APP_ERROR_CHECK(err_code);
        break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

    case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
    {
        ble_gap_data_length_params_t dl_params;

        // Clearing the struct will effectivly set members to @ref BLE_GAP_DATA_LENGTH_AUTO
        memset(&dl_params, 0, sizeof(ble_gap_data_length_params_t));
        err_code = sd_ble_gap_data_length_update(p_ble_evt->evt.gap_evt.conn_handle, &dl_params, NULL);
        APP_ERROR_CHECK(err_code);
    }
    break;

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        // No system attributes have been stored.
        err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
        APP_ERROR_CHECK(err_code);
        break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

    case BLE_GATTC_EVT_TIMEOUT:
        // Disconnect on GATT Client timeout event.
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break; // BLE_GATTC_EVT_TIMEOUT

    case BLE_GATTS_EVT_TIMEOUT:
        // Disconnect on GATT Server timeout event.
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break; // BLE_GATTS_EVT_TIMEOUT

    case BLE_EVT_USER_MEM_REQUEST:
        err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
        APP_ERROR_CHECK(err_code);
        break; // BLE_EVT_USER_MEM_REQUEST

    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
    {
        ble_gatts_evt_rw_authorize_request_t  req;
        ble_gatts_rw_authorize_reply_params_t auth_reply;

        req = p_ble_evt->evt.gatts_evt.params.authorize_request;

        if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
        {
            if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
            {
                if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                {
                    auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                }
                else
                {
                    auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                }
                auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                           &auth_reply);
                APP_ERROR_CHECK(err_code);
            }
        }
    }
    break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

    default:
        // No implementation needed.
        break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    nrf_ble_gatt_on_ble_evt(&m_gatt, p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);

}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = softdevice_app_ram_start_get(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Overwrite some of the default configurations for the BLE stack.
    ble_cfg_t ble_cfg;

    // Configure the maximum number of connections.
    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.gap_cfg.role_count_cfg.periph_role_count  = BLE_GAP_ROLE_COUNT_PERIPH_DEFAULT;
    ble_cfg.gap_cfg.role_count_cfg.central_role_count = 0;
    ble_cfg.gap_cfg.role_count_cfg.central_sec_count  = 0;
    err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Configure the maximum ATT MTU.
    memset(&ble_cfg, 0x00, sizeof(ble_cfg));
    ble_cfg.conn_cfg.conn_cfg_tag                 = CONN_CFG_TAG;
    ble_cfg.conn_cfg.params.gatt_conn_cfg.att_mtu = NRF_BLE_GATT_MAX_MTU_SIZE;
    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATT, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Configure the maximum event length.
    memset(&ble_cfg, 0x00, sizeof(ble_cfg));
    ble_cfg.conn_cfg.conn_cfg_tag                     = CONN_CFG_TAG;
    ble_cfg.conn_cfg.params.gap_conn_cfg.event_length = 320;
    ble_cfg.conn_cfg.params.gap_conn_cfg.conn_count   = BLE_GAP_CONN_COUNT_DEFAULT;
    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GAP, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = softdevice_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, const nrf_ble_gatt_evt_t * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)\r\n", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x\r\n", p_gatt->att_mtu_desired_central, p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, 64);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
    case BSP_EVENT_SLEEP:
        sleep_mode_enter();
        break;

    case BSP_EVENT_DISCONNECT:
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
        break;

    case BSP_EVENT_WHITELIST_OFF:
        if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
        {
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
        }
        break;

    default:
        break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{



    /*
      static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
      static uint8_t index = 0;
      uint32_t       err_code;

      switch (p_event->evt_type)
      {
          case APP_UART_DATA_READY:
              UNUSED_VARIABLE(app_uart_get(&data_array[index]));
              index++;

              if ((data_array[index - 1] == '\n') || (index >= (m_ble_nus_max_data_len)))
              {
                  NRF_LOG_DEBUG("Ready to send data over BLE NUS\r\n");
                  NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                  do
                  {
                      err_code = ble_nus_string_send(&m_nus, data_array, index);
                      if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_BUSY) )
                      {
                          APP_ERROR_CHECK(err_code);
                      }
                  } while (err_code == NRF_ERROR_BUSY);

                  index = 0;
              }
              break;

          case APP_UART_COMMUNICATION_ERROR:
              APP_ERROR_HANDLER(p_event->data.error_communication);
              break;

          case APP_UART_FIFO_ERROR:
              APP_ERROR_HANDLER(p_event->data.error_code);
              break;

          default:
              break;
      }*/
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud57600
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_advdata_t          scanrsp;
    ble_adv_modes_config_t options;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;///// infinite time advertisinng
////    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;/////d/

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

////d//button actions(non ble)//see usage example
//    err_code = bsp_event_to_button_action_assign(BUTTON_0, BSP_BUTTON_ACTION_PUSH, EVENT_COUNTDOWNTIMER_ADD_ONE);
//    APP_ERROR_CHECK(err_code);


//    err_code = bsp_event_to_button_action_assign(BUTTON_0, BSP_BUTTON_ACTION_LONG_PUSH, EVENT_COUNTDOWNTIMER_ADD_TEN);
//    APP_ERROR_CHECK(err_code);


//    err_code = bsp_event_to_button_action_assign(BUTTON_1, BSP_BUTTON_ACTION_PUSH,  EVENT_COUNTDOWNTIMER_CLEAR);
//    APP_ERROR_CHECK(err_code);

////d//


    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}



/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}



static void ee_check(void)
{
    if(check_ee_token())
    {
        nrf_gpio_pin_write(LED_RED,!true);
        write_default_ee_settings();
        ble_nus_string_send(&m_nus, "EE_DEFAULTED\n",13);
        nrf_delay_ms(400);
        nrf_gpio_pin_write(LED_RED,!false);
    }
}

static void paranoid_checks(void)
{
  
  if(OPT3001_check_ID()||PCF85063_check_ID()||check_ee_token())
  {
    nrf_gpio_pin_write(LED_RED,!true);
  }
  else
  {
    nrf_gpio_pin_write(LED_RED,!false);
  }

}

/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    bool     erase_bonds;

    // Initialize.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    init_gpio();
    twi_init();
    uart_init();
    log_init();
    buttons_leds_init(&erase_bonds);
    
    draw_mode=true;
     nrf_gpio_pin_write(LED_RED,!true);   
    disp_clear_buffer(BRO0);
    display_pixel(0,2,true,BRO0);      
    display_pixel(1,2,true,BRO0);  
    display_pixel(2,2,true,BRO0);  
    read_ee_settings();
    display_pixel(0,2,false,BRO0); 
    PCF85063_init();
    display_pixel(1,2,false,BRO0); 
    OPT3001_init();
    display_pixel(2,2,false,BRO0);
    
    nrf_gpio_pin_write(LED_RED,!false);
    draw_mode=false;  
    ee_check();
    
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

//    printf("\r\nUART Start!\r\n");
//   NRF_LOG_INFO("UART Start!\r\n");
    err_code = ble_advertising_start(BLE_ADV_MODE_DIRECTED_SLOW);//was BLE_ADV_MODE_FAST
    APP_ERROR_CHECK(err_code);


    // Enter main loop.
    for (;;)
    {
      
        paranoid_checks();
        show_all();
        nrf_delay_ms(100);
        power_manage();

        
    }
}


/**
 * @}
 */
