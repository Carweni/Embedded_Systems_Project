#ifndef CONFIG_H
#define CONFIG_H

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/sys_heap.h>

#define UART_DEVICE_NODE    DT_CHOSEN(zephyr_console) // Nó escolhido para comunicação serial na Device Tree
#define LED0_NODE           DT_ALIAS(led0) // Define nó do LED q será utilizado pelo código

#define LED_BLINK_INTERVAL_MS   500  // Tempo entre cada piscada do LED

// Definição do ADC na Device Tree:
#define ADC_NODE DT_NODELABEL(adc1)
#define ADC_RESOLUTION 12
#define ADC_GAIN ADC_GAIN_1
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME_DEFAULT
#define ADC_CHANNEL_ID 1

#define TEXT_BUFFER_WIDTH  160   
#define TEXT_BUFFER_HEIGHT  64   
#define TEXT_BUFFER_SIZE    (TEXT_BUFFER_WIDTH * TEXT_BUFFER_HEIGHT)

#define APP_HEAP_SIZE 16384  // Tamanho do heap em bytes
static char app_heap_mem[APP_HEAP_SIZE];
static struct sys_heap app_heap;

extern const struct device *uart_dev; // Declara um ponteiro para o dispositivo UART a ser definido em outros arquivos (n vai precisar redefinir)
extern const struct gpio_dt_spec led0; // Declara uma estrutura para configurar um pino GPIO em outro arquivo.
extern const struct device *display_dev; // Declara um ponteiro pro display.

// Protótipos de funções:
void led_control(int command);
void display_init(void);
void display_update_status(const char *status);

#endif /* CONFIG_H */