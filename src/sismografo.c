/**
 * @file Primera versión
 * @author Kevin Campos Castro
 * @author Josué Salmerón Córdoba
 * @brief En este primer bloque de código se prueba el giroscopio.
 * @version 0.1
 * @date 2024-02-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <stdio.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include "clock.h"
#include "console.h"
#include "sdram.h"
#include "lcd-spi.h" // Llamado de los colores.
#include "gfx.h"

// Para el GYRO se utilizó como base tanto el ejemplo de
// la libreria stm32/f3/stm32-discovery/spi y de stm32/f4/stm.../spi
#define GYR_RNW			(1 << 7)
#define GYR_MNS			(1 << 6)
#define GYR_WHO_AM_I		0x0F
#define GYR_OUT_TEMP		0x26
#define GYR_STATUS_REG		0x27
#define GYR_CTRL_REG1		0x20
#define GYR_CTRL_REG1_PD	(1 << 3)
#define GYR_CTRL_REG1_XEN	(1 << 1)
#define GYR_CTRL_REG1_YEN	(1 << 0)
#define GYR_CTRL_REG1_ZEN	(1 << 2)
#define GYR_CTRL_REG1_BW_SHIFT	4
#define GYR_CTRL_REG4		0x23
#define GYR_CTRL_REG4_FS_SHIFT	4

// Se definen para todos los ejes X, Y, Z
#define GYR_OUT_X_L		0x28
#define GYR_OUT_X_H		0x29
#define GYR_OUT_Y_L		0x2A
#define GYR_OUT_Y_H		0x2B
#define GYR_OUT_Z_L		0x2C
#define GYR_OUT_Z_H		0x2D

// Parámetros para la sensibilidad de la pantalla
#define L3GD20_SENSITIVITY_250DPS  (0.00875F)      
// Struct del giroscopio
typedef struct GYRO {
  int16_t X;
  int16_t Y;
  int16_t Z;
} GYRO;

// Declaración de funciones
static void spi_setup(void);
void input_setup(void);
static void adc_setup(void);

GYRO mostrar_XYZ(void);

static void spi_setup(void){

    // Periféricos del reloj
    rcc_periph_clock_enable(RCC_SPI5);
	/* For spi signal pins */
	rcc_periph_clock_enable(RCC_GPIOC);
	/* For spi mode select on the l3gd20 */
	rcc_periph_clock_enable(RCC_GPIOF);

    // GPIO
    /* Setup GPIOE3 pin for spi mode l3gd20 select. */
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);
	/* Start with spi communication disabled */
	gpio_set(GPIOC, GPIO1);

	/* Setup GPIO pins for AF5 for SPI1 signals. */
	gpio_mode_setup(GPIOF, GPIO_MODE_AF, GPIO_PUPD_NONE,
			GPIO7 | GPIO8 | GPIO9);
	gpio_set_af(GPIOF, GPIO_AF5, GPIO7 | GPIO8 | GPIO9);

	//Inicilizar spi
	spi_set_master_mode(SPI5);
	spi_set_baudrate_prescaler(SPI5, SPI_CR1_BR_FPCLK_DIV_64);
	spi_set_clock_polarity_0(SPI5);
	spi_set_clock_phase_0(SPI5);
	spi_set_full_duplex_mode(SPI5);
	spi_set_unidirectional_mode(SPI5); /* bidirectional but in 3-wire */
	spi_enable_software_slave_management(SPI5);
	spi_send_msb_first(SPI5);
	spi_set_nss_high(SPI5);

	//spi_enable_ss_output(SPI5);
	SPI_I2SCFGR(SPI5) &= ~SPI_I2SCFGR_I2SMOD;
	spi_enable(SPI5);

    gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_CTRL_REG1); 
	spi_read(SPI5);
	spi_send(SPI5, GYR_CTRL_REG1_PD | GYR_CTRL_REG1_XEN |
			GYR_CTRL_REG1_YEN | GYR_CTRL_REG1_ZEN |
			(3 << GYR_CTRL_REG1_BW_SHIFT));
	spi_read(SPI5);
	gpio_set(GPIOC, GPIO1); 

    gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_CTRL_REG4);
	spi_read(SPI5);
	spi_send(SPI5, (1 << GYR_CTRL_REG4_FS_SHIFT));
	spi_read(SPI5);
	gpio_set(GPIOC, GPIO1);
	
}


void input_setup(void){

	rcc_periph_clock_enable(RCC_ADC1);
	rcc_periph_clock_enable(RCC_USART1);

    /* Enable GPIOA clock. */
	rcc_periph_clock_enable(RCC_GPIOA);

    /* Enable GPIOG clock. */
	rcc_periph_clock_enable(RCC_GPIOG);

	/* Set GPIO0 (in GPIO port A) to 'input open-drain'. */
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);

    /* Set GPIO13 (in GPIO port G) to 'output push-pull'. LED PG13 */
	gpio_mode_setup(GPIOG, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);

	/* Set GPIO14 (in GPIO port G) to 'output push-pull'. LED EMERGENCIA */
	gpio_mode_setup(GPIOG, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14);
}

// Utilizando como base el ejemplo 
// libopencm3-examples/examples/stm32/f4/stm32f429i-discovery/adc-dac-printf/adc-dac-printf.c
static void adc_setup(void){

    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
	adc_power_off(ADC1);
	adc_disable_scan_mode(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_3CYC);
	adc_power_on(ADC1);
}
// Funcion que lee las coordenadas xyz del GYRO
GYRO mostrar_XYZ(void){
	GYRO SHOW;
	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_WHO_AM_I | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	spi_read(SPI5);
	gpio_set(GPIOC, GPIO1);

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_STATUS_REG | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	spi_read(SPI5);
	gpio_set(GPIOC, GPIO1);

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_OUT_TEMP | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	spi_read(SPI5);
	gpio_set(GPIOC, GPIO1);

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_OUT_X_L | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	SHOW.X = spi_read(SPI5);
	gpio_set(GPIOC, GPIO1);

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_OUT_X_H | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	SHOW.X |=spi_read(SPI5) << 8;
	gpio_set(GPIOC, GPIO1);

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_OUT_Y_L | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	SHOW.Y =spi_read(SPI5);
	gpio_set(GPIOC, GPIO1);

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_OUT_Y_H | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	SHOW.Y|=spi_read(SPI5) << 8;
	gpio_set(GPIOC, GPIO1);

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_OUT_Z_L | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	SHOW.Z=spi_read(SPI5);
	gpio_set(GPIOC, GPIO1);

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_OUT_Z_H | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	SHOW.Z|=spi_read(SPI5) << 8;
	gpio_set(GPIOC, GPIO1);

	SHOW.X = SHOW.X*L3GD20_SENSITIVITY_250DPS;
	SHOW.Y = SHOW.Y*L3GD20_SENSITIVITY_250DPS;
	SHOW.Z = SHOW.Z*L3GD20_SENSITIVITY_250DPS;
	return SHOW;
}


int main(void){

    console_setup(115200);

    clock_setup();
    input_setup();
    spi_setup();
    adc_setup();
    sdram_init();
    lcd_spi_init();

	gfx_init(lcd_draw_pixel, 240, 320);

    GYRO SHOW;
    char AXIS_X[20];
	char AXIS_Y[20];
	char AXIS_Z[20];

    while (1){

		// Se pasan las variables a strings utilizando las variable inicializadas
		sprintf(AXIS_X, "%d", SHOW.X);
		sprintf(AXIS_Y, "%d", SHOW.Y);
		sprintf(AXIS_Z, "%d", SHOW.Z);

        // Mostrando información en pantalla
		gfx_fillScreen(LCD_BLACK);
		gfx_setTextColor(LCD_MAGENTA, LCD_BLACK); // Letra color azul
		gfx_setTextSize(2);			
		gfx_setCursor(30, 30);
		gfx_puts(" IE-0624");	
	
		// Info de los ejes
		gfx_setTextColor(LCD_WHITE, LCD_BLACK);
		gfx_setCursor(60, 90);
		gfx_setTextSize(2);
		gfx_puts("Eje X: ");
		gfx_setTextColor(LCD_RED, LCD_BLACK);
		gfx_puts(AXIS_X);
		
		gfx_setTextColor(LCD_WHITE, LCD_BLACK);
		gfx_setCursor(60, 130);
		gfx_puts("Eje Y: ");
		gfx_setTextColor(LCD_RED, LCD_BLACK);
		gfx_puts(AXIS_Y);

		gfx_setTextColor(LCD_WHITE, LCD_BLACK);
		gfx_setCursor(60, 170);
		gfx_puts("Eje Z: ");
		gfx_setTextColor(LCD_RED, LCD_BLACK);
		gfx_puts(AXIS_Z);

		lcd_show_frame();
		
		//Se leen los datos
		SHOW = mostrar_XYZ();
		gpio_set(GPIOC, GPIO1);

		int i;
		for (i = 0; i < 80000; i++)    /* Waiting. */
			__asm__("nop");	
	}
	return 0;
}