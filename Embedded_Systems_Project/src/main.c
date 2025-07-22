#include "config.h"

// Definição das threads e suas pilhas:
K_THREAD_STACK_DEFINE(blink_thread_stack, 512);   // Thread para piscar o LED
K_THREAD_STACK_DEFINE(uart_thread_stack, 512);    // Thread para UART
K_THREAD_STACK_DEFINE(adc_thread_stack, 512);     // Thread para ADC
K_THREAD_STACK_DEFINE(display_thread_stack, 512); // Thread para display
static struct k_thread blink_thread_data;
static struct k_thread uart_thread_data;
static struct k_thread adc_thread_data;  
static struct k_thread display_thread_data;
static k_tid_t blink_tid;
static k_tid_t uart_tid;
static k_tid_t adc_tid; 
static k_tid_t display_tid;

// Definição de prioridades:
#define BLINK_PRIORITY 6
#define UART_PRIORITY 5
#define ADC_PRIORITY 3
#define DISPLAY_PRIORITY 8

// Semáforos para sincronização entre threads:
K_SEM_DEFINE(adc_update_sem, 0, 1);       // Semáforo para atualizar ADC
K_SEM_DEFINE(display_update_sem, 0, 1);   // Semáforo para atualizar display
K_SEM_DEFINE(blink_control_sem, 0, 1);    // Semáforo para controlar piscar

// Timer para ADC:
K_TIMER_DEFINE(adc_timer, NULL, NULL);

// Verifica disponibilidade do ADC na Device Tree:
#if !DT_NODE_HAS_STATUS(ADC_NODE, okay)
#error "ADC devicetree node is disabled"
#endif

static int16_t adc_sample_buffer[1];   // Buffer para guardar resultado da conversão ADC.

static uint16_t text_display_buffer[TEXT_BUFFER_SIZE]; // Buffer estático para o display

// Configuração do ADC:
static const struct adc_channel_cfg channel_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = ADC_CHANNEL_ID,
    .differential = 0,
#ifdef CONFIG_ADC_CONFIGURABLE_INPUTS
    .input_positive = ADC_CHANNEL_ID,
#endif
};

// Configuração da leitura do ADC:
static const struct adc_sequence adc_seq = {
    .channels = BIT(ADC_CHANNEL_ID),
    .buffer = adc_sample_buffer,
    .buffer_size = sizeof(adc_sample_buffer),
    .resolution = ADC_RESOLUTION,
};

// Variáveis globais para compartilhar dados do ADC entre threads:
static int32_t current_voltage_mv = 0;
static uint8_t current_percentage = 0;
static bool adc_data_ready = false;

// Mutex para proteger acesso aos dados do ADC:
K_MUTEX_DEFINE(adc_data_mutex);

// Fila de mensagens para comunicação UART:
#define MSG_SIZE 16
K_MSGQ_DEFINE(msgq, MSG_SIZE, 10, 4); // 10 mensagens de até 16 caracteres          

const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);  // Obtem o dispositivo da UART - converte nó da Device Tree em ponteiro para o dispositivo.

const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios); // Device tree diz que o LED está no pino 0

const struct device *display_dev; // O display vai ser inicializado em display_init()

// Estados do LED (inicializam falsos, pois o LED está desligado):
static bool led_on = false;
static bool led_blinking = false;

static struct display_capabilities capabilities; // Struct com informações sobre as capacidades do display (largura,altura e formato de pixels)

// Fonte bitmap 8x8 para os caracteres a serem impressos no display:
static const uint8_t font_8x8[][8] = {
    {0x3C, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C}, // 0: O  
    {0x66, 0x76, 0x7E, 0x7E, 0x6E, 0x66, 0x66, 0x66}, // 1: N 
    {0x7E, 0x60, 0x60, 0x7C, 0x60, 0x60, 0x60, 0x60}, // 2: F
    {0x7C, 0x66, 0x66, 0x7C, 0x66, 0x66, 0x66, 0x7C}, // 3: B
    {0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x7E}, // 4: L
    {0x3C, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C}, // 5: I
    {0x66, 0x6C, 0x78, 0x70, 0x78, 0x6C, 0x66, 0x66}, // 6: K
    {0x3C, 0x66, 0x60, 0x60, 0x6E, 0x66, 0x66, 0x3C}, // 7: G
    {0x7E, 0x60, 0x60, 0x7C, 0x60, 0x60, 0x60, 0x7E}, // 8: E
    {0x7C, 0x66, 0x66, 0x7C, 0x78, 0x6C, 0x66, 0x66}, // 9: R
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // 10: Espaço
    {0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00}, // 11: :
    {0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00}, // 12: -
    {0x7E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18}, // 13: T
    {0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00}, // 14: .
    {0x63, 0x66, 0x0C, 0x18, 0x30, 0x60, 0xC6, 0xC6}, // 15: %
    {0x7C, 0x66, 0x66, 0x7C, 0x60, 0x60, 0x60, 0x60}, // 16: P
    {0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x18}, // 17: V
    {0x66, 0x66, 0x66, 0x7E, 0x7E, 0x5A, 0x42, 0x42}, // 18: M
    {0x7C, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x7C}, // 19: D
    {0x18, 0x24, 0x42, 0x42, 0x7E, 0x42, 0x42, 0x42}, // 20: A 
    {0x3C, 0x66, 0x60, 0x60, 0x60, 0x60, 0x66, 0x3C}, // 21: C
    {0x3C, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C}, // 22: 0
    {0x18, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x7E}, // 23: 1
    {0x3C, 0x66, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x7E}, // 24: 2
    {0x3C, 0x66, 0x06, 0x1C, 0x06, 0x66, 0x66, 0x3C}, // 25: 3
    {0x0C, 0x1C, 0x3C, 0x6C, 0x6C, 0x7E, 0x0C, 0x0C}, // 26: 4
    {0x7E, 0x60, 0x60, 0x7C, 0x06, 0x06, 0x66, 0x3C}, // 27: 5
    {0x3C, 0x66, 0x60, 0x7C, 0x66, 0x66, 0x66, 0x3C}, // 28: 6
    {0x7E, 0x06, 0x06, 0x0C, 0x18, 0x30, 0x30, 0x30}, // 29: 7
    {0x3C, 0x66, 0x66, 0x3C, 0x66, 0x66, 0x66, 0x3C}, // 30: 8
    {0x3C, 0x66, 0x66, 0x3E, 0x06, 0x06, 0x66, 0x3C}  // 31: 9
};

// Mapeamento dos caracteres:
static int get_char_index(char c) {
    switch(c) {
        case 'O': return 0;
        case 'N': return 1;
        case 'F': return 2;
        case 'B': return 3;
        case 'L': return 4;
        case 'I': return 5;
        case 'K': return 6;
        case 'G': return 7;
        case 'E': return 8;
        case 'R': return 9;
        case ' ': return 10;
        case ':': return 11;
        case '-': return 12;
        case 'T': return 13;
        case '.': return 14;
        case '%': return 15;
        case 'P': return 16;
        case 'V': return 17;  
        case 'M': return 18;
        case 'D': return 19;  
        case 'A': return 20;  
        case 'C': return 21;  
        case '0': return 22;
        case '1': return 23;
        case '2': return 24;
        case '3': return 25;
        case '4': return 26;
        case '5': return 27;
        case '6': return 28;
        case '7': return 29;
        case '8': return 30;
        case '9': return 31;
        default: return 10; // Retorna espaço para caracteres não encontrados
    }
}

// Renderiza caracteres no buffer de display:
void draw_char(uint16_t *buffer, int buf_width, int x, int y, char c, uint16_t color) {
    int16_t char_idx = get_char_index(c);
    
    for (uint8_t row = 0; row < 8; row++) {
        uint8_t char_row = font_8x8[char_idx][row];
        for (uint8_t col = 0; col < 8; col++) {
            if (char_row & (0x80 >> col)) {
                uint16_t pixel_x = x + col;
                uint16_t pixel_y = y + row;
                if (pixel_x < buf_width && pixel_y < 64) {
                    buffer[pixel_y * buf_width + pixel_x] = color;
                }
            }
        }
    }
}

// Escreve strings no buffer do display:
void draw_text(uint16_t *buffer, int buf_width, int x, int y, const char *text, uint16_t color) {
    uint16_t char_x = x;
    
    while (*text) {
        draw_char(buffer, buf_width, char_x, y, *text, color);
        char_x += 9;
        if (char_x + 8 > buf_width) {
            y += 9;
            char_x = x;
        }
        text++;
    }
}

// Inicializa e configura o display:
void display_init(void)
{
    int ret = 0;
    display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    
    if (!device_is_ready(display_dev)) {
        printk("Display is not ready.\n");
        return;
    }
    
    display_get_capabilities(display_dev, &capabilities);
    
    ret = display_blanking_off(display_dev);
    if (ret != 0) {
        printk("Failed to disable display blanking\n");
    }
}

// Atualiza o display com status do LED e dados do ADC:
void display_update_status(const char *status)
{
    if (!display_dev || !device_is_ready(display_dev)) {
        return;
    }
    
    uint16_t *text_buffer = text_display_buffer;
    memset(text_buffer, 0, sizeof(text_display_buffer));

    // Cores para diferentes status:
    uint16_t led_color;
    if (strcmp(status, "OFF") == 0) {
        led_color = 0xF800; // Vermelho
    } else if (strcmp(status, "ON") == 0) {
        led_color = 0x07E0; // Verde
    } else if (strncmp(status, "BLINKING", 8) == 0) {
        led_color = 0xFFE0; // Amarelo
    } else {
        led_color = 0xFFFF; // Branco
    }
    
    // Desenha status do LED:
    draw_text(text_buffer, TEXT_BUFFER_WIDTH, 5, 5, "LED: ", 0xFFFF);
    draw_text(text_buffer, TEXT_BUFFER_WIDTH, 50, 5, status, led_color);
    
    // Desenha dados do ADC:
    if (k_mutex_lock(&adc_data_mutex, K_MSEC(10)) == 0) {
        if (adc_data_ready) {
            char adc_text[32];
            
            draw_text(text_buffer, TEXT_BUFFER_WIDTH, 5, 20, "VOLTAGE:", 0xFFFF);
            snprintf(adc_text, sizeof(adc_text), "%d", current_voltage_mv);
            draw_text(text_buffer, TEXT_BUFFER_WIDTH, 90, 20, adc_text, 0x07FF);
            
            draw_text(text_buffer, TEXT_BUFFER_WIDTH, 5, 35, "PERCENT:", 0xFFFF);
            snprintf(adc_text, sizeof(adc_text), "%d %%", current_percentage);
            draw_text(text_buffer, TEXT_BUFFER_WIDTH, 90, 35, adc_text, 0x07FF);
        } else {
            draw_text(text_buffer, TEXT_BUFFER_WIDTH, 5, 20, "ADC: READING...", 0xFFFF);
        }
        k_mutex_unlock(&adc_data_mutex);
    }
    
    struct display_buffer_descriptor desc = {
        .buf_size = TEXT_BUFFER_WIDTH * TEXT_BUFFER_HEIGHT * 2,
        .width = TEXT_BUFFER_WIDTH,
        .height = TEXT_BUFFER_HEIGHT,
        .pitch = TEXT_BUFFER_WIDTH
    };
    
    display_write(display_dev, 10, 10, &desc, text_buffer);
}

// Função para obter dados do ADC:
void get_adc_data(int32_t *voltage_mv, uint8_t *percentage)
{
    if (k_mutex_lock(&adc_data_mutex, K_MSEC(100)) == 0) {
        if (adc_data_ready) {
            *voltage_mv = current_voltage_mv;
            *percentage = current_percentage;
        } else {
            *voltage_mv = 0;
            *percentage = 0;
        }
        k_mutex_unlock(&adc_data_mutex);
    }
}

// Callback do timer ADC para sinalizar leitura:
void adc_timer_callback(struct k_timer *timer)
{
    k_sem_give(&adc_update_sem);
}

// Thread para leitura do ADC:
static void adc_thread(void *a, void *b, void *c)
{
    const struct device *adc_dev;
    int32_t ret;
    int32_t voltage_mv;
    
    adc_dev = DEVICE_DT_GET(ADC_NODE);
    if (!device_is_ready(adc_dev)) {
        printk("ADC device not ready\n");
        return;
    }
    
    ret = adc_channel_setup(adc_dev, &channel_cfg);
    if (ret < 0) {
        printk("Error setting up ADC channel: %d\n", ret);
        return;
    }
    
    // Inicia o timer para leitura periódica do ADC:
    k_timer_init(&adc_timer, adc_timer_callback, NULL);
    k_timer_start(&adc_timer, K_MSEC(500), K_MSEC(500));
    
    while (1) {
        // Aguarda semáforo:
        k_sem_take(&adc_update_sem, K_FOREVER);
        
        // Realiza leitura do ADC:
        ret = adc_read(adc_dev, &adc_seq);
        if (ret < 0) {
            printk("ADC read error: %d\n", ret);
            continue;
        }
        
        // Converte valor bruto para tensão em mV:
        voltage_mv = adc_sample_buffer[0];
        ret = adc_raw_to_millivolts(adc_ref_internal(adc_dev), ADC_GAIN, ADC_RESOLUTION, &voltage_mv);
        
        if (ret < 0) {
            printk("Error converting to mV: %d\n", ret);
            continue;
        }
        
        // Calcula porcentagem da tensão em relação aos 3,3V:
        uint8_t percentage = (voltage_mv * 100) / 3300;
        if (percentage > 100) percentage = 100;
        if (percentage < 0) percentage = 0;
        
        // Atualiza variáveis globais:
        if (k_mutex_lock(&adc_data_mutex, K_MSEC(100)) == 0) {
            current_voltage_mv = voltage_mv;
            current_percentage = percentage;
            adc_data_ready = true;
            k_mutex_unlock(&adc_data_mutex);
        }
        
        // Sinaliza que display deve ser atualizado:
        k_sem_give(&display_update_sem);
    }
}

// Thread dedicada para atualização do display:
static void display_thread(void *a, void *b, void *c)
{
    while (1) {
        // Aguarda sinal para atualizar display:
        k_sem_take(&display_update_sem, K_FOREVER);
        
        // Atualiza display com status atual:
        if (led_blinking) {
            display_update_status("BLINKING");
        } else if (led_on) {
            display_update_status("ON");
        } else {
            display_update_status("OFF");
        }
    }
}

// Função de controle do LED:
void led_control(int command)
{
    const char *status_text;
    
    switch(command) {
        case 0: // Desliga o LED
            led_blinking = false;
            led_on = false;
            gpio_pin_set_dt(&led0, 0);
            status_text = "OFF";
            printk("LED OFF\n");
            break;
        case 1: // Liga o LED
            led_blinking = false;
            led_on = true;
            gpio_pin_set_dt(&led0, 1);
            status_text = "ON";
            printk("LED ON\n");
            break;
        case 2: // Pisca o LED
            led_blinking = true;
            led_on = false;
            status_text = "BLINKING";
            printk("LED BLINKING\n");
            k_sem_give(&blink_control_sem); // Sinaliza para a thread de piscar começar.
            break;
        default:
            led_blinking = false;
            led_on = false;
            gpio_pin_set_dt(&led0, 0);
            status_text = "ERROR";
            printk("Invalid command! Use: 0=OFF, 1=ON, 2=BLINK\n");
            break;
    }
    
    // Sinaliza atualização do display:
    k_sem_give(&display_update_sem);
}

// Funções de informação do sistema:
static void show_thread_info(void)
{
    printk("\n=== THREAD INFORMATION ===\n");
    printk("%-20s %-10s\n", "Thread Name", "Priority");
    printk("------------------------------------------------------------\n");
    
    printk("%-20s %-10d\n", "main", 0);
    printk("%-20s %-10d\n", "blink_thread", BLINK_PRIORITY);
    printk("%-20s %-10d\n", "uart_thread", UART_PRIORITY);
    printk("%-20s %-10d\n", "adc_thread", ADC_PRIORITY);
    printk("%-20s %-10d\n", "display_thread", DISPLAY_PRIORITY);
    
    printk("\nSemaphores:\n");
    printk("- adc_update_sem: Controls ADC reading\n");
    printk("- display_update_sem: Controls display updates\n");
    printk("- blink_control_sem: Controls LED blinking\n");
    printk("=========================\n\n");
}

// Função para mostrar utilização da heap:
static void show_heap_info(void)
{
    struct sys_memory_stats stats;
    sys_heap_runtime_stats_get(&app_heap, &stats);

    printk("\n=== HEAP INFORMATION ===\n");
    printk("Heap Size:       %u bytes\n", APP_HEAP_SIZE);
    printk("Allocated:       %zu bytes\n", APP_HEAP_SIZE - stats.free_bytes);
    printk("Free:            %zu bytes\n", stats.free_bytes);
    printk("========================\n\n");
}

// Função para mostrar algumas informações de runtime do programa:
static void show_runtime_info(void)
{
    printk("\n=== RUNTIME INFORMATION ===\n");
    printk("System Uptime: %lld ms\n", k_uptime_get());
    printk("System Tick Rate: %d Hz\n", CONFIG_SYS_CLOCK_TICKS_PER_SEC);
    
    printk("\nThread Status:\n");
    printk("- blink_thread: %s\n", led_blinking ? "Active (blinking)" : "Inactive");
    printk("- uart_thread:  Active (waiting for commands)\n");
    printk("- adc_thread:   Active (timer-driven)\n");
    printk("- display_thread: Active (event-driven)\n");
    printk("===========================\n\n");
}

// Função para mostrar configurações de real time do programa:
static void show_realtime_info(void)
{
    printk("\n=== REAL-TIME INFORMATION ===\n");
    printk("Thread Priorities (lower number = higher priority):\n");
    printk("- main thread:     0\n");
    printk("- blink_thread:    %d\n", BLINK_PRIORITY);
    printk("- uart_thread:     %d\n", UART_PRIORITY);
    printk("- adc_thread:      %d\n", ADC_PRIORITY);
    printk("- display_thread:  %d\n", DISPLAY_PRIORITY);
    
    printk("\nSynchronization Mechanisms:\n");
    printk("- Semaphores: Event-driven execution\n");
    printk("- Timer: Periodic ADC reading\n");
    printk("- Mutex: Thread-safe data access\n");
    printk("- Message Queue: UART communication\n");
    printk("=============================\n\n");
}

// Mostra menu de comandos:
static void show_help(void)
{
    printk("\n=== COMMANDS ===\n");
    printk("LED Control:\n");
    printk("  0 - Turn LED OFF\n");
    printk("  1 - Turn LED ON\n");
    printk("  2 - Start LED BLINKING\n");
    printk("\nSystem Information:\n");
    printk("  info     - Show thread information\n");
    printk("  heap     - Show heap information\n");
    printk("  runtime  - Show runtime information\n");
    printk("  realtime - Show real-time information\n");
    printk("  status   - Show current system status\n");
    printk("  help     - Show this help menu\n");
    printk("==========================\n\n");
}

// Mostra status atuais do sistema:
static void show_current_status(void)
{
    int32_t voltage_mv;
    uint8_t percentage;
    get_adc_data(&voltage_mv, &percentage);
    
    printk("\n=== CURRENT STATUS ===\n");
    printk("LED State: %s\n", led_blinking ? "BLINKING" : (led_on ? "ON" : "OFF"));
    printk("ADC Voltage: %d mV\n", voltage_mv);
    printk("ADC Percentage: %d%%\n", percentage);
    printk("System Uptime: %lld ms\n", k_uptime_get());
    printk("Data Ready: %s\n", adc_data_ready ? "YES" : "NO");
    printk("======================\n\n");
}

// Callback da UART:
static void uart_cb(const struct device *dev, void *user_data)
{
    static char rx_buf[MSG_SIZE];
    static int rx_buf_pos = 0;
    uint8_t c;
    
    if (!uart_irq_update(uart_dev)) {
        return;
    }
    
    if (!uart_irq_rx_ready(uart_dev)) {
        return;
    }
    
    while (uart_fifo_read(uart_dev, &c, 1) == 1) {
        if ((c == '\r' || c == '\n') && rx_buf_pos > 0) {
            rx_buf[rx_buf_pos] = '\0';
            
            printk("Received command: %s\n", rx_buf);
            
            if (strlen(rx_buf) == 1 && rx_buf[0] >= '0' && rx_buf[0] <= '2') {
                int led_command = rx_buf[0] - '0';
                led_control(led_command);
            }
            else if (strcmp(rx_buf, "info") == 0) {
                show_thread_info();
            } else if (strcmp(rx_buf, "heap") == 0) {
                show_heap_info();
            } else if (strcmp(rx_buf, "runtime") == 0) {
                show_runtime_info();
            } else if (strcmp(rx_buf, "realtime") == 0) {
                show_realtime_info();
            } else if (strcmp(rx_buf, "help") == 0) {
                show_help();
            } else if (strcmp(rx_buf, "status") == 0) {
                show_current_status();
            } else {
                printk("Unknown command: %s\n", rx_buf);
                printk("Type 'help' for available commands.\n");
            }
            
            rx_buf_pos = 0;
        } else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
            rx_buf[rx_buf_pos++] = c;
        }
    }
}

// Thread para processar mensagens da UART:
static void uart_thread(void *a, void *b, void *c)
{
    char rx_data[MSG_SIZE];
    
    while (1) {
        k_msgq_get(&msgq, &rx_data, K_FOREVER);
        //printk("Processing queued command: %s\n", rx_data);
    }
}

// Thread que pisca LED:
static void blink_thread(void *a, void *b, void *c)
{
    static bool blink_state = false;
    
    while (1) {
        if (led_blinking) {
            blink_state = !blink_state;
            gpio_pin_set_dt(&led0, blink_state ? 1 : 0);
            k_sem_take(&blink_control_sem, K_MSEC(LED_BLINK_INTERVAL_MS));  // Bloqueia até timeout ou comando
        } else {
            gpio_pin_set_dt(&led0, led_on ? 1 : 0);
            // Aguarda semáforo para iniciar a piscar:
            k_sem_take(&blink_control_sem, K_FOREVER);
        }
    }
}

int main(void)
{
    int ret;
    
    printk("\n=== LED Control with ADC Test ===\n");
    printk("LED Commands:\n");
    printk("  0 - Turn LED OFF\n");
    printk("  1 - Turn LED ON\n");
    printk("  2 - Start LED BLINKING\n");
    printk("System Commands:\n");
    printk("  info, heap, runtime, realtime, help, status\n");
    printk("Enter command: ");
    
    // Inicializa o heap de aplicação antes de utilizá-lo:
    sys_heap_init(&app_heap, app_heap_mem, APP_HEAP_SIZE);
    
    // Inicializa o display:
    display_init();
    
    // Verifica se os periféricos estão prontos:
    if (!device_is_ready(led0.port)) {
        printk("ERROR: LED device not ready\n");
        return -1;
    }
    
    if (!device_is_ready(uart_dev)) {
        printk("ERROR: UART device not ready\n");
        return -1;
    }
    
    ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("ERROR: Cannot configure LED (%d)\n", ret);
        return ret;
    }
    
    // Configura callbacks da UART
    uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);
    uart_irq_rx_enable(uart_dev);
    
    // Cria thread para piscar LED
    blink_tid = k_thread_create(&blink_thread_data, blink_thread_stack, 
                               K_THREAD_STACK_SIZEOF(blink_thread_stack),
                               blink_thread, NULL, NULL, NULL, BLINK_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(blink_tid, "blink_thread");
    
    // Cria thread para processar mensagens UART
    uart_tid = k_thread_create(&uart_thread_data, uart_thread_stack, 
                              K_THREAD_STACK_SIZEOF(uart_thread_stack),
                              uart_thread, NULL, NULL, NULL, UART_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(uart_tid, "uart_thread");
    
    // Cria thread para leitura do ADC
    adc_tid = k_thread_create(&adc_thread_data, adc_thread_stack, 
                             K_THREAD_STACK_SIZEOF(adc_thread_stack),
                             adc_thread, NULL, NULL, NULL, ADC_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(adc_tid, "adc_thread");
    
    // Cria thread para atualização do display
    display_tid = k_thread_create(&display_thread_data, display_thread_stack, 
                                 K_THREAD_STACK_SIZEOF(display_thread_stack),
                                 display_thread, NULL, NULL, NULL, DISPLAY_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(display_tid, "display_thread");
    
    // Atualiza o display inicial:
    display_update_status("OFF");
    
    // Sinaliza primeira atualização do display:
    k_sem_give(&display_update_sem);
    
    while (1) {
        k_msleep(1000); 
    }
    
    return 0;
}