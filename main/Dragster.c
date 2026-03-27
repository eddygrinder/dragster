// ===============================
// Includes principais
// ===============================
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tuning.h"
#include "ble.h"
#include "strip_leds.h"
#include "motors_control.h"
#include "qtr_sensors.h"
#include "encoders.h"
// #include "braking.h"

// ===============================
// Definições de hardware
// ===============================

#define BTN_CAL 48 ///< GPIO do botão de calibração

// ----- Botões -----y
bool calib_done = false; // Flag calibração

volatile uint32_t encoder_total_ticks = 0;
uint32_t IGNORE_LINE_TICKS = 5; // ticks para ignorar a linha de partida (ajustar conforme necessário)

// ===============================
// Estruturas de dados
// ===============================

#define TAG "DRAGSTER"

typedef struct
{
    float s1, s2, s3, s4; ///< Valores normalizados dos sensores
} SensorValues;

uint32_t TICKS_REDUCE_SPEED = 15; // ticks correspondentes a 2 m
uint32_t TICKS_BRAKE_SPEED = 140; // ticks correspondentes à travagem
uint32_t BREAK_TICKS = 150;       // ticks correspondentes À DISTÂNCIA DE TRAVAGEM
uint32_t KP_BRAKE = 2;            // ganho de travagem - ajustável conforme testes
uint32_t MAX_BRAKE_PWM = 400;     // valor máximo de PWM para travagem segura sem queimar drivers - ajustável conforme testes
uint32_t STOP_THRESHOLD = 5;      // ticks por ciclo abaixo do qual consideramos que o robô parou - ajustável conforme testes
bool run_active = false; 

// ===============================
// Protótipos de funções
// ===============================

/// @brief Task principal de controlo do robô
void controlTask(void *pvParameters);
void runfollowerTask(void *pvParameters);
void encoderTestTask(void *pvParameters);
TaskHandle_t controlTaskHandle;
TaskHandle_t lineFollowerTaskHandle;
TaskHandle_t encoderTaskHandle;

/// @brief Funções auxiliares
bool calib_button_pressed(void);
// void LED_START(void);

/// @brief Estados possíveis do robô durante a prova
///
/// O robô passa por estes estados em sequência:
/// 1. Espera pelo botão de calibração
/// 2. Executa a calibração dos sensores
/// 3. Espera pelo sinal de arranque (LED)
/// 4. Executa o seguimento da linha
typedef enum
{
    WAIT_CALIB,  ///< Estado inicial: espera que o botão de calibração seja pressionado
    CALIBRATING, ///< Estado de calibração: calibra sensores QTR-8C, acontece apenas uma vez
    WAIT_START,  ///< Estado de espera pelo LED de arranque
    RUN,         ///< Estado de execução: segue a linha utilizando os valores calibrados
} robot_state_t;

typedef enum
{
    SUBSTATE_RUN,              // velocidade normal
    SUBSTATE_REDUCE,           // redução de velocidade
    SUBSTATE_BRAKE,            // travagem ativa
    SUBSTATE_REDUCE_LOOP,      // loop de redução até atingir o ponto de travagem
    SUBSTATE_START_LINE_IGNORE // subestado inicial para ignorar a linha de partida
} dragster_substate_t;

/**
 * @brief Inicializa hardware, ADC, PWM e cria a task de controlo
 */
void app_main(void)
{

    // Inicializar RGB interno
    rgb_init();
    // Inicializar fita de LEDs externa
    strip_init();
    // Inicializar Bluetooth
    ble_init();

    // Carregar os valores de KP e BASE_SPEED da NVS para a estrutura global de tuning
    nvs_flash_init();
    tuning_load();
    tuning_print_saved(); // opcional: imprime os valores carregados para confirmação
    read_final_ticks();   // lê os ticks finais da última prova para referência
    encoder_init();       // Configura o GPIO do encoder e a ISR para contagem de ticks

    // -------------------------------
    // Configuração dos GPIOs de direção
    // -------------------------------

    esp_rom_gpio_pad_select_gpio(BTN_CAL);
    gpio_set_direction(BTN_CAL, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BTN_CAL, GPIO_PULLUP_ONLY);

    motors_init(); // Configura os GPIOs dos motores e o PWM

    qtr_init(); // Configura ADC para os sensores QTR-8C

    // -------------------------------
    // Criar a task principal de controlo
    // -------------------------------
    xTaskCreatePinnedToCore(
        controlTask,        ///< Função da task
        "Control Task",     ///< Nome da task
        4096,               ///< Stack size
        NULL,               ///< Parâmetros
        1,                  ///< Prioridade
        &controlTaskHandle, // guarda handle
        0                   ///< Core 0
    );

    // Criação da task isolada para RUN
    xTaskCreatePinnedToCore(
        runfollowerTask, // função que faz o loop RUN
        "LineFollowerTask",
        4096,
        NULL,
        tskIDLE_PRIORITY + 5,    // prioridade alta
        &lineFollowerTaskHandle, // guarda handle
        1                        // Core 1
    );

    xTaskCreatePinnedToCore(
        encoderTestTask,
        "EncoderTest",
        4096,
        NULL,
        tskIDLE_PRIORITY + 1,
        &encoderTaskHandle, // ← guarda handle,
        0);
}

void encoderTestTask(void *pvParameters)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // espera início da prova
        run_active = true;

        while (run_active)
        {
            encoder_total_ticks = encoder_get_ticks();
            vTaskDelay(pdMS_TO_TICKS(10)); // 5ms é suficiente
        }
    }
}

void runfollowerTask(void *pvParameters)
{
    while (1)
    {
        // Espera até receber sinal de iniciar a prova
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // bloqueia até ser notificado pelo ControlTask

        // Inicializa subestado
        dragster_substate_t state = SUBSTATE_START_LINE_IGNORE;

        while (run_active)
        {
            switch (state)
            {
            case SUBSTATE_START_LINE_IGNORE:
                strip_set_green();    // opcional: mantém LEDs vermelhos acesos para indicar que parou
                motors_set(200, 200); // velocidade baixa para sair da linha de partida
                if (encoder_total_ticks >= IGNORE_LINE_TICKS)
                {
                    state = SUBSTATE_RUN;
                    vTaskDelay(pdMS_TO_TICKS(5)); // permite que o FreeRTOS rode outras tasks
                }
                break;

            case SUBSTATE_RUN:
                strip_set_blue();    // opcional: mantém LEDs vermelhos acesos para indicar que parou
                run_line_follower(); // PID + motores
                if (encoder_total_ticks >= TICKS_REDUCE_SPEED)
                    state = SUBSTATE_REDUCE;
                break;

            case SUBSTATE_REDUCE:
                strip_set_yellow(); // opcional: mantém LEDs vermelhos acesos para indicar que parou
                tuning.BASE_SPEED = 180;
                tuning.KP = 38;
                run_line_follower(); // PID + motores reduzidos
                if (encoder_total_ticks >= TICKS_BRAKE_SPEED)
                {
                    state = SUBSTATE_REDUCE_LOOP; // entra no loop de redução até travar
                }
                break;

            case SUBSTATE_REDUCE_LOOP:
                strip_set_red();     // opcional: mantém LEDs vermelhos acesos para indicar que parou
                run_line_follower(); // PID + motores reduzidos
                if (encoder_total_ticks >= BREAK_TICKS)
                {
                    state = SUBSTATE_BRAKE; // começa travagem agressiva
                }
                break;

            case SUBSTATE_BRAKE:
                const int brake_delay = 5;            // atraso adicional para medir a velocidade
                int prev_ticks = encoder_total_ticks; // captura ticks atuais para comparação
                bool stopped = false;

                while (!stopped)
                {
                    vTaskDelay(pdMS_TO_TICKS(brake_delay));  // espera um pouco para medir efeito
                    int current_ticks = encoder_total_ticks; // lê ticks atuais
                    int dticks = current_ticks - prev_ticks;
                    prev_ticks = current_ticks;

                    int pwm = dticks * KP_BRAKE; // cálculo de PWM baseado na velocidade atual
                    if (pwm > MAX_BRAKE_PWM)
                        pwm = MAX_BRAKE_PWM; // limita PWM para evitar danos

                    motors_brake_set(pwm); // aplica travagem proporcional

                    if (dticks <= STOP_THRESHOLD)
                    {
                        stopped = true;
                        strip_set_color(); // opcional: mantém LEDs vermelhos acesos para indicar que parou
                        motors_coast();    // desliga travagem para evitar sobreaquecimento
                    }
                }
                run_active = false; // ← dentro do case
                break;              // ← break do case
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // loop rápido sem travar CPU
    }
    xTaskNotifyGive(controlTaskHandle); // Notifica ControlTask que a prova terminou
}

// ================================
// TAREFA QUE LÊ OS 4 SENSORES
// ================================
void controlTask(void *pvParameters)
{
    uint32_t duration_ms = CALIBRATION_TIME_MS; ///< Duração da calibração em milissegundos
    robot_state_t state = WAIT_CALIB;
    // Variável global ou estática
    static bool ble_active = false;

    /// @brief Espera até que o botão de calibração seja pressionado
    /*
    Liga robô
        ↓
    WAIT_CALIB   → botão pressionado
        ↓
    CALIBRATING  → termina
        ↓
    WAIT_START   → LED OFF
        ↓
    WAIT_START   → LED ON  ← AQUI
        ↓
    RUN          → segue linha até ao fim
    */

    // Assegura motores parados no início
    motors_coast();

    while (1)
    {
        switch (state)
        {
        case WAIT_CALIB:
            motors_coast(); // garante que motores estão em coast enquanto espera
            if (calib_done)
            {
                // Se já calibrado, pula direto para start
                state = WAIT_START;
            }
            else if (!calib_button_pressed())
            {
                state = CALIBRATING;
            }
            break;

        case CALIBRATING:
            strip_set_color();
            motors_coast();
            qtr_calibrate(duration_ms); // corre UMA VEZ
            calib_done = true;          // Marca calibração feita
            state = WAIT_START;
            break;

        case WAIT_START:
            motors_coast();
            if (LED_START() > 2000) // Verifica se o valor do LED indica que está aceso
            {
                if (ble_active)
                {
                    ble_stop_advertising(); // ← função pública do ble.h
                    ble_active = false;
                    ESP_LOGI(TAG, "BLE disabled before RUN");
                }
                state = RUN;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            break;

        case RUN:
            encoder_reset_ticks();
            // Notifica a task do line follower para iniciar
            xTaskNotifyGive(lineFollowerTaskHandle);
            xTaskNotifyGive(encoderTaskHandle); // dispara encoder ao mesmo tempo
            tuning_print_saved();               // ← lê e imprime da NVS no fim de cada prova

            // Liga o BLE se ainda não estiver ativo
            if (!ble_active)
            {
                ble_restart_advertising(); // inicia advertising para KP
                ble_active = true;
            }
            state = WAIT_START; // volta para esperar próximo teste
            break;
        }
    }
}

/**
 * @brief Verifica se o botão de calibração foi pressionado
 *
 * @return true se pressionado, false caso contrário
 */
bool calib_button_pressed(void)
{
    static int last_btn_level = -1;      // guarda último estado do botão
    int level = gpio_get_level(BTN_CAL); // lê GPIO atual

    if (level != last_btn_level)
    { // só loga se houver alteração
        ESP_LOGI(TAG, "BTN_CAL = %d", level);
        last_btn_level = level;
    }

    // **cede CPU para não disparar watchdog**
    vTaskDelay(pdMS_TO_TICKS(10)); // ou taskYIELD()

    return (level == 1); // devolve true se botão pressionado
}

/**
 * @brief Controlo do LED Verde RGB - Built-In
 *
 */
/*

VER ISTO PARA UNIFIRMIZAR O CÓDIGO DOS MOTORES QUE PODE ESTAR REPETIDO

static void motor_set(int pwm, int gpio_a, int gpio_b, ledc_channel_t channel)
{
    if (pwm == 0)
    {
        gpio_set_level(gpio_a, 0);
        gpio_set_level(gpio_b, 0);
        ledc_set_duty(MOTOR_PWM_MODE, channel, 0);
        ledc_update_duty(MOTOR_PWM_MODE, channel);
        return;
    }
    if (pwm > MAX_DUTY_CYCLE) pwm = MAX_DUTY_CYCLE;
    if (pwm < -MAX_DUTY_CYCLE) pwm = -MAX_DUTY_CYCLE;

    if (pwm > 0) {
        gpio_set_level(gpio_a, 1);
        gpio_set_level(gpio_b, 0);
    } else {
        gpio_set_level(gpio_a, 0);
        gpio_set_level(gpio_b, 1);
        pwm = -pwm;
    }
    ledc_set_duty(MOTOR_PWM_MODE, channel, pwm);
    ledc_update_duty(MOTOR_PWM_MODE, channel);
}

void motor_left_set(int pwm)  { motor_set(pwm, AIN1, AIN2, MOTOR_PWM_CHANNEL_ESQ); }
void motor_right_set(int pwm) { motor_set(pwm, BIN1, BIN2, MOTOR_PWM_CHANNEL_DTA); }

LED - GPIO


*/