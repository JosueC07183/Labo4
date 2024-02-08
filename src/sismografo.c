/**
 * @file sismografo.c
 * @author Kevin Campos Castro
 * @author Josué Salmerón Córdoba
 * @brief En este trabajo se realiza un sismografo que funciona por medio de un giroscopio usando los 3 ejes X, Y, Z, también se usa una batería y se hace una
 * transmisión serial de los datos por medio de una plataforma de IoT; Thingsboard. Los 3 componentes principales de la placa se resumen de la siguiente manera:
 * 1. Al mover la placa las coordenas de los 3 ejes pasan de su estado normal, es decir, 0 a diferentes valores según el movimiento que se emplee. El nivel de la batería se mostrará conforme ésta 
 * vaya descargándose, esto se mostrará más claro con la ayuda de un LED que se enciende y se apaga cuando se encuentra por debajo de 7V o cuando es superior a 7V respectivamente. El botón de la 
 * transmisión serial posee el mismo comportamiento, el LED PG13 se enciende cuando hay transmisión y se mantiene en bajo cuando no la hay.
 * @version 0.1
 * @date 2024-02-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */

/*
* Para la mayoria del codigo implementado se tomaron ya codigos hechos dentro de los
* ejemplos porporcionado por libopencm3
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
#define GYR_RNW			(1 << 7) /* Escribe cuando es 0*/
#define GYR_MNS			(1 << 6) /* Multiples lecturas cuando es 1 */
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

// Sensibilidad de la pantalla
#define L3GD20_SENSITIVITY_250DPS  (0.00875F)      

// Struct del giroscopio
typedef struct GYRO {
  int16_t X;
  int16_t Y;
  int16_t Z;
} GYRO;

// Indicador de cuando se da la transmision
int transmision = 0;
// Variable para guardar mensaje a imprimir en consola
char mensaje [40], comma[] = ",";

// Declaración de funciones
static void spi_setup(void);
GYRO mostrar_XYZ(void);
/**
 * @brief En esta función se configuran los perifericos del reloj. Los pines del GPIO, se inicializa el SPI para poder ver los movimientos en los 3 ejes. 
 * Luego, se colocan otros bloques de código para poder observar los elementos en la pantalla LCD, también se toman en cuenta la tasa de los baudios a trabajar.
 * Se configuran los pines de salida para los LEDs de alarma, y el modo analógico para poder conectar la batería a la placa. En resumen, es función
 * se compone de bloques de código de los ejemplos brindados en la librería libopencm3.
 * 
 */
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

	//Inicializar spi
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
	// Código tomado de: ./libopencm3-examples/examples/stm32/f3/stm32f3-discovery/spi
	// Esto está dentro del main
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
	//*********************************************************************//

	// Código tomado de: ./libopencm3-examples/examples/stm32/f3/stm32f3-discovery/spi
	// Función usart_setup()
	/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
	rcc_periph_clock_enable(RCC_ADC1);
	rcc_periph_clock_enable(RCC_USART1);
	/* Setup GPIO pin GPIO_USART1_TX/GPIO9 on GPIO port A for transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO9);
	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	// Finally enable the USART
	usart_enable(USART1);
//*********************************************************************//
 /* Enable GPIOA clock. */
	rcc_periph_clock_enable(RCC_GPIOA);

    /* Enable GPIOG clock. */
	rcc_periph_clock_enable(RCC_GPIOG);

	/* Set GPIO0 (in GPIO port A) to 'input open-drain'. */
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);

    /* Set GPIO14 (in GPIO port G) LED PG14 */
	gpio_mode_setup(GPIOG, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14);

	/* Set GPIO13 (in GPIO port G). LED ALARMA */
	gpio_mode_setup(GPIOG, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);
//*********************************************************************//
	// Código tomado de: libopencm3-examples/examples/stm32/f4/stm32f429i-discovery/adc-dac-printf
	// Función adc_setup()
 	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
	adc_power_off(ADC1);
	adc_disable_scan_mode(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_3CYC);
	adc_power_on(ADC1);
}

// Función read_adc_naiive(), esta función hace una conversión analogico-digital en un canal determinado
// y devuelve dicho valor. Esto puede servir para la batería.
static uint16_t read_adc_naiive(uint8_t channel){
	uint8_t channel_array[16];
	channel_array[0] = channel;
	adc_set_regular_sequence(ADC1, 1, channel_array);
	adc_start_conversion_regular(ADC1);
	while (!adc_eoc(ADC1));
	uint16_t reg16 = adc_read_regular(ADC1);
	return reg16;
}

/**
 * @brief Se hace una lectura del giroscopio que se mostrarán los 3 ejes en la pantalla LCD, donde su 
 * valor por default serán todas las coordenadas en 0, que al mover la placa éstos valores cambian.
 * Otro detalle importante es con respecto a la sensibilidad en cada eje, se configuró la misma para que no
 * exista ningún desfase y tenga sentido. Dentro de esta misma función se hace un condicional que sirve de alarma
 * para el LED que indica cuando la batería es baja o se mantiene en el umbral adecuado. El otro condicional es para
 * saber cuando se establecenla comunicación serial.
 * 
 * @return GYRO 
 */
GYRO mostrar_XYZ(void){
	GYRO SHOW;
	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_WHO_AM_I | 0x80);
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
	float bateria_carga;
    clock_setup();
    spi_setup();
    sdram_init();
    lcd_spi_init();

	gfx_init(lcd_draw_pixel, 240, 320);

    GYRO SHOW;
    char AXIS_X[20];
	char AXIS_Y[20];
	char AXIS_Z[20];
	char battery_v[20];
    
    while (1){
		int comunicacion = 0; // Bandera para ver si hay que realizar comunicacion o no
		//Se leen los datos
		SHOW = mostrar_XYZ();
		// Se lee voltaje en el pin PA0
		bateria_carga = (float)(read_adc_naiive(1)*9)/195;
		
		// Se pasan las variables a strings utilizando las variable inicializadas
		sprintf(AXIS_X, "%d", SHOW.X);
		sprintf(AXIS_Y, "%d", SHOW.Y);
		sprintf(AXIS_Z, "%d", SHOW.Z);
		sprintf(battery_v, "%.2f", bateria_carga);

		// Montar todos los datos en un string para el msj
		char msj[200] = "";
		sprintf(msj, "%s,%s,%s,%s\n", AXIS_X, AXIS_Y, AXIS_Z, battery_v); // Esto para separar valores con comas
		
		// Enviar los datos
		for (int j = 0; msj[j] != '\0'; j++)
		{
			usart_send_blocking(USART1, msj[j]); // Enviar datos
		}

        // Info pantalla
		gfx_fillScreen(LCD_BLACK);
		gfx_setTextColor(LCD_MAGENTA, LCD_BLACK);
		gfx_setTextSize(2);			
		gfx_setCursor(50, 30);
		gfx_puts(" IE-0624");	

		// Info ejes, color de fondo negro
		gfx_setTextColor(LCD_WHITE, LCD_BLACK);
		gfx_setCursor(60, 90);
		gfx_setTextSize(1.8);
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
		// Info bateria
		gfx_setTextColor(LCD_WHITE, LCD_BLACK); // Letra color blanco
		gfx_setCursor(60, 220);
		gfx_setTextSize(1.8);
		gfx_puts("Bateria: ");
		gfx_setCursor(170, 220);
		gfx_puts(battery_v);
		gfx_puts(" V");
		lcd_show_frame();
		gpio_set(GPIOC, GPIO1);
		
		// LED alarma de batería, GIPO13
		if (bateria_carga <=7.0)
		{
			gpio_toggle(GPIOG, GPIO14); // Parpadeo porque bateria esta baja
		}
		else{
			gpio_clear(GPIOG, GPIO14);
		}

		// Determinar si se da comunicacion o no
		comunicacion = gpio_get(GPIOA, GPIO0); // Ver si se apreta boton
		if (comunicacion)
		{
			gpio_toggle(GPIOG, GPIO13); // Encender led porque hay comunicacion
		}
		else
		{
			gpio_clear(GPIOG, GPIO13); // Apagar led, no hay comunicacion
		}

		// Codigo para indicar la comunicacion, pero el script de python genera errores cuando no se
		// estan mandando datos
		// // Determinar si se da comunicacion o no
		// comunicacion = gpio_get(GPIOA, GPIO0); // Ver si se apreta boton
		// if (comunicacion)
		// {
		// 	gpio_toggle(GPIOG, GPIO13); // Encender led porque hay comunicacion

		// 	// Montar todos los datos en un string para el msj
		// 	char msj[200] = "";
		// 	sprintf(msj, "%s,%s,%s,%s\n", AXIS_X, AXIS_Y, AXIS_Z, battery_v); // Esto para separar valores con comas
			
		// 	// Enviar los datos
		// 	for (int j = 0; msj[j] != '\0'; j++) {
		// 		usart_send_blocking(USART1, msj[j]); // Enviar datos
		// 	}
		// }
		// else 
		// {
		// 	char msj[200] = "";
		// 	sprintf(AXIS_X, "%s", "OFF");
		// 	sprintf(AXIS_Y, "%s", "OFF");
		// 	sprintf(AXIS_Z, "%s", "OFF");
		// 	sprintf(battery_v, "%s", "OFF");
		// 	sprintf(msj, "%s,%s,%s,%s\n", AXIS_X, AXIS_Y, AXIS_Z, battery_v); // Esto para separar valores con comas
		// 	gpio_clear(GPIOG, GPIO13); // Apagar led
		// }

		int i;
		for (i = 0; i < 1000000; i++)    /* Waiting. */
			__asm__("nop");	
	}
	return 0;
}