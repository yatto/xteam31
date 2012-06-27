/*
 * rk29_m911.h
 *
 * Overview:  
 *
 * Copyright (c) 2011, YiFang Digital
 *
 * Version:  1.0
 * Created:  02/22/2011 03:36:04 PM
 * Author:  zqqu <zqqu@yifangdigital.com>
 * Company:  YiFang Digital
 * History:
 *
 * 
 */
/***************************************************
 *
 *				    LCD  
 *
 **************************************************/
#define LCD_TXD_PIN          INVALID_GPIO
#define LCD_CLK_PIN          INVALID_GPIO
#define LCD_CS_PIN           INVALID_GPIO

#define FB_ID                       0
#define FB_DISPLAY_ON_PIN           RK29_PIN6_PD1
#define FB_LCD_STANDBY_PIN          RK29_PIN6_PD0
#define FB_LCD_CABC_EN_PIN          INVALID_GPIO
#define FB_MCU_FMK_PIN              INVALID_GPIO

#define FB_DISPLAY_ON_VALUE         GPIO_HIGH
#define FB_LCD_STANDBY_VALUE        GPIO_HIGH

/* Base */
#define OUT_TYPE		SCREEN_RGB
#define OUT_FACE		OUT_P888
#define OUT_CLK			 40000000
#define LCDC_ACLK       150000000     //29 lcdc axi DMA 撞楕

/* Timing */
#define H_PW			1
#define H_BP			46
#define H_VD			800
#define H_FP			210

#define V_PW			3
#define V_BP			23
#define V_VD			600
#define V_FP			12

/* Other */
#define DCLK_POL		0
#define SWAP_RB			0

/***************************************************
 *
 *				     BACKLIGHG
 *
 **************************************************/
#define PWM_ID            0
#define PWM_MUX_NAME      GPIO1B5_PWM0_NAME
#define PWM_MUX_MODE      GPIO1L_PWM0
#define PWM_MUX_MODE_GPIO GPIO1L_GPIO1B5
#define PWM_EFFECT_VALUE  0
#define PWM_GPIO		RK29_PIN1_PB5

/**the value of MIN_BACKLIGHT_SCALE must be between 0~10*/
#define MIN_BACKLIGHT_SCALE	12



//#define LCD_DISP_ON_PIN

#ifdef  LCD_DISP_ON_PIN
#define BL_EN_MUX_NAME    GPIOF34_UART3_SEL_NAME
#define BL_EN_MUX_MODE    IOMUXB_GPIO1_B34

#define BL_EN_PIN         GPIO0L_GPIO0A5
#define BL_EN_VALUE       GPIO_HIGH
#endif

/***************************************************
 *
 *                      BATTERY 
 *
 **************************************************/
#define DC_DET_EFFECTIVE		1
#define CHG_OK_EFFECTIVE		1
#define GPIO_DC_DET			RK29_PIN4_PA1
#define GPIO_CHG_OK			RK29_PIN4_PA3
#define ADC_ADD_VALUE		1
#define ADC_CLI_VALUE		50
#define CHARGE_FULL_GATE 		4150

//This parameter is for new battery driver//
#define	TIMER_MS_COUNTS		            50	//timers length(ms)

#define	SLOPE_SECOND_COUNTS	            15	//time interval(s) for computing voltage slope
#define	DISCHARGE_MIN_SECOND	        60	//minimum time interval for discharging 1% battery
#define	CHARGE_MIN_SECOND	            90	//minimum time interval for charging 1% battery
#define	CHARGE_MID_SECOND	            160	//time interval for charging 1% battery when battery capacity over 80%
#define	CHARGE_MAX_SECOND	            220	//max time interval for charging 1% battery
#define CHARGE_FULL_DELAY_TIMES         10  //delay time when charging FULL
#define USBCHARGE_IDENTIFY_TIMES        5   //time for identifying USB and Charge
#define STABLE_SECOND					8  //check ok µçÆ½»á»Î¶¯¡£¡£
#define SHUTDOWN_SECOND					20
#define SPEEDLOSE_SECOND                120 //play game rapid down

#define	NUM_VOLTAGE_SAMPLE	            ((SLOPE_SECOND_COUNTS * 1000) / TIMER_MS_COUNTS)	//samling numbers
#define	NUM_DISCHARGE_MIN_SAMPLE	    ((DISCHARGE_MIN_SECOND * 1000) / TIMER_MS_COUNTS)	
#define	NUM_CHARGE_MIN_SAMPLE	        ((CHARGE_MIN_SECOND * 1000) / TIMER_MS_COUNTS)	    
#define	NUM_CHARGE_MID_SAMPLE	        ((CHARGE_MID_SECOND * 1000) / TIMER_MS_COUNTS)	    
#define	NUM_CHARGE_MAX_SAMPLE	        ((CHARGE_MAX_SECOND * 1000) / TIMER_MS_COUNTS)	    
#define NUM_CHARGE_FULL_DELAY_TIMES     ((CHARGE_FULL_DELAY_TIMES * 1000) / TIMER_MS_COUNTS)	
#define NUM_USBCHARGE_IDENTIFY_TIMES    ((USBCHARGE_IDENTIFY_TIMES * 1000) / TIMER_MS_COUNTS)	
#define NUM_STABLE_SAMPLE				((STABLE_SECOND * 1000) / TIMER_MS_COUNTS)
#define NUM_SHUTD0WN_SAMPLE             ((SHUTDOWN_SECOND * 1000) / TIMER_MS_COUNTS)
#define NUM_SPEEDLOSE_SAMPLE  			((SPEEDLOSE_SECOND * 1000) / TIMER_MS_COUNTS)

#define BAT_2V5_VALUE	        2500
#define BATT_MAX_VOL_VALUE	    4190	//voltage of FULL battery
#define	BATT_ZERO_VOL_VALUE     3500	//voltage when poweroff
#define BATT_NOMAL_VOL_VALUE    3800
#define SHUTDOWNVOLTAGE			3400
//define  divider resistors for ADC sampling, units as K
#define BAT_PULL_UP_R           549
#define BAT_PULL_DOWN_R         200


