#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "inc/font.h"
#include "inc/ssd1306.h"
#include "pico/bootrom.h"

//-------------------------------------------------------------------------

// Botão A conectado à GPIO 5.
const uint BTN_A_GPIO = 5;
volatile bool btn_a_pressed = false;

const uint BTN_B_GPIO = 6;

//  Botão do Joystick conectado à GPIO 22
const uint8_t JOY_BTN_GPIO = 22;

// Joystick conectado aos GPIOs 26 e 27.
const uint8_t JOY_X_GPIO = 26;
const uint8_t JOY_Y_GPIO = 27;

// Configurações LED RGB
const uint LED_GREEN_GPIO = 11;
static bool led_green_state = false;

const uint LED_BLUE_GPIO = 12;
uint16_t blue_slice;

const uint LED_RED_GPIO = 13;
uint16_t red_slice;

// Configurações do PWM
const uint LED_PWM_WRAP = 4096; // Extremos da variação de valores do eixo Y e X (0 e 4095)

// Display SSD1306 conectado via I2C (GPIO 14 e GPIO15).
static i2c_inst_t* const I2C_PORT = i2c1; // Porta I2C
static const uint I2C_SDA  = 14; // Pinos I2C SDA
static const uint I2C_SCL  = 15; // Pinos I2C SCL
static const uint I2C_SPEED = 400 * 1000;   // Operando à 400 kHz
static const uint OLED_ADDR = 0x3C; // Endereço do display OLED
ssd1306_t display;

uint display_border = 1;

volatile absolute_time_t last_interrupt_time = {0};

//-------------------------------------------------------------------------

void gpio_irq_handler_cb(uint gpio, uint32_t events)
{
    absolute_time_t now = to_us_since_boot(get_absolute_time());
    

    if (absolute_time_diff_us(last_interrupt_time, now) < 500000)
    {
        return;
    }

    last_interrupt_time = now;

    if (gpio == BTN_A_GPIO)
    {
        // Alterna o estado do btn_a_pressed
        btn_a_pressed = !btn_a_pressed;

        pwm_set_enabled(red_slice, !btn_a_pressed);
        pwm_set_enabled(blue_slice, !btn_a_pressed);
    } else if (BTN_B_GPIO)
    {
        reset_usb_boot(0, 0);
    }
    else if (gpio == JOY_BTN_GPIO)
    {
        led_green_state = !led_green_state;
        gpio_put(LED_GREEN_GPIO, led_green_state);

        display_border = (display_border + 1) % 4; // Alterna entre 4 estilos de borda
        ssd1306_draw_border(&display, display_border);
        ssd1306_show(&display);
    }
}


void initialize_gpio_pins()
{
    // Inicializa os pinos GPIO
    gpio_init(BTN_A_GPIO);
    gpio_init(BTN_B_GPIO);
    gpio_init(JOY_BTN_GPIO);

    gpio_init(LED_RED_GPIO);
    gpio_init(LED_GREEN_GPIO);
    gpio_init(LED_BLUE_GPIO);

    // Configura os pinos GPIO como entrada ou saída
    gpio_set_dir(BTN_A_GPIO, GPIO_IN);
    gpio_set_dir(BTN_B_GPIO, GPIO_IN);
    gpio_set_dir(JOY_BTN_GPIO, GPIO_IN);
    gpio_set_dir(LED_RED_GPIO, GPIO_OUT);
    gpio_set_dir(LED_GREEN_GPIO, GPIO_OUT);
    gpio_set_dir(LED_BLUE_GPIO, GPIO_OUT);

    gpio_pull_up(BTN_A_GPIO);
    gpio_pull_up(JOY_BTN_GPIO);
    gpio_pull_up(BTN_B_GPIO);

    gpio_put(LED_RED_GPIO, 0);
    gpio_put(LED_GREEN_GPIO, 0);
    gpio_put(LED_BLUE_GPIO, 0);

    gpio_set_irq_enabled_with_callback(BTN_A_GPIO, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler_cb);
    gpio_set_irq_enabled_with_callback(BTN_B_GPIO, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler_cb);
    gpio_set_irq_enabled_with_callback(JOY_BTN_GPIO, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler_cb);

    adc_init();
    adc_gpio_init(JOY_X_GPIO);
    adc_gpio_init(JOY_Y_GPIO);
}


// Inicializar PWM
void initialize_pwm()
{
    // Configuração do PWM para o LED RED (GPIO13)
    gpio_set_function(LED_RED_GPIO, GPIO_FUNC_PWM);
    red_slice = pwm_gpio_to_slice_num(LED_RED_GPIO);

    // Valor máximo do PWM
    pwm_set_wrap(red_slice, LED_PWM_WRAP);
    pwm_set_enabled(red_slice, true);

    // Configuração do PWM para o LED BLUE (GPIO12)
    gpio_set_function(LED_BLUE_GPIO, GPIO_FUNC_PWM);
    blue_slice = pwm_gpio_to_slice_num(LED_BLUE_GPIO);

    pwm_set_wrap(blue_slice, LED_PWM_WRAP);
    pwm_set_enabled(blue_slice, true);
}

void init_i2c()
{
    i2c_init(I2C_PORT, I2C_SPEED);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

void init_display(ssd1306_t *ssd)
{
    ssd1306_init(ssd, WIDTH, HEIGHT, false, OLED_ADDR, I2C_PORT);
    ssd1306_config(ssd);
    ssd1306_send_data(ssd);
    // Limpa o display
    ssd1306_fill(ssd, false);
    ssd1306_send_data(ssd);
}

int main()
{
    stdio_init_all();
    initialize_gpio_pins();

    // Inicializa I2C e display
    init_i2c();
    init_display(&display);

    // Inicializa PWM
    initialize_pwm();

    pwm_set_gpio_level(LED_RED_GPIO, 0);
    pwm_set_gpio_level(LED_BLUE_GPIO, 0);


    //  Configuração do PWM para o LED BLUE (GPIO12)
    int led_brightness = 0;
    int led_fade_factor = 1; // Fator de incremento/decremento do brilho

    uint adc_x, adc_y;
    uint16_t joy_x, joy_y;

    while (true)
    {

        // Leitura do Joystick por canal 
        // Canal 0: X
        // Canal 1: Y
        adc_select_input(0);   
        adc_x = adc_read();
        adc_select_input(1);
        adc_y = adc_read();

        // Mapeamento dos valores do Joystick para o intervalo de 0 a 4095
        joy_x = adc_x * LED_PWM_WRAP / 4095;
        joy_y = adc_y * LED_PWM_WRAP / 4095;

        // Ajusta o brilho do LED Azul conforme o valor do eixo Y
        if (joy_y > 2048) {
            pwm_set_gpio_level(LED_BLUE_GPIO, (joy_y - 2048) * 2);
        } else {
            pwm_set_gpio_level(LED_BLUE_GPIO, (2048 - joy_y) * 2);
        }

        // Ajusta o brilho do LED Vermelho conforme o valor do eixo X
        if (joy_x > 2048) {
            pwm_set_gpio_level(LED_RED_GPIO, (joy_x - 2048) * 2);
        } else {
            pwm_set_gpio_level(LED_RED_GPIO, (2048 - joy_x) * 2);
        }

        // Calcula a posição do quadrado com base nos valores do joystick
        int square_x = (joy_x * (WIDTH - 8)) / 4095;
        int square_y = (joy_y * (HEIGHT - 8)) / 4095;

        // Limpa o display
        ssd1306_fill(&display, false);

        // Desenha o quadrado na nova posição
        ssd1306_draw_square(&display, square_x, square_y, 8, true);
        
        // Atualiza o display
        ssd1306_show(&display);

        sleep_ms(10);
    }

    return 0;
}
