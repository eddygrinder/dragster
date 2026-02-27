// ===============================
// Includes principais
// ===============================
#include <stdio.h>
#include "tuning.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "driver/ledc.h"    // PWM control
#include "hal/ledc_types.h" // Tipos LEDC
#include "esp_rom_gpio.h"   // esp_rom_gpio_pad_select_gpio()

#include "nvs_flash.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_gap.h"

#include "strip_leds.h"
#include "motors_control.h"
#include "tuning.h"
#include "qtr_sensors.h"

#include "driver/rmt_tx.h"

#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// ===============================
// Definições de hardware
// ===============================

#define BTN_CAL 48 ///< GPIO do botão de calibração
#define BTN_RUN 47 ///< GPIO do botão de arranque

// ----- Botões -----
bool calib_done = false; // Flag calibração

volatile bool led_command = false;
volatile uint8_t led_value_new = 0;

// Dente azul - Tag para logs
static const char *TAG = "NIMBLE_SCAN";
static uint8_t own_addr_type;

// ------------------------------
// PID / control parameters
// ------------------------------
volatile float KP_new = 150.0f;   // valor atualizado via BLE
volatile bool KP_command = false; // flag para task principal

// ===============================
// Estruturas de dados
// ===============================

typedef struct
{
    float s1, s2, s3, s4; ///< Valores normalizados dos sensores
} SensorValues;

volatile SensorValues sensors; ///< Valores atuais dos sensores

// ===============================
// Protótipos de funções
// ===============================

/// @brief Task principal de controlo do robô
void controlTask(void *pvParameters);
void runfollowerTask(void *pvParameters);
TaskHandle_t controlTaskHandle;
TaskHandle_t lineFollowerTaskHandle;

/// @brief Funções auxiliares
bool calib_button_pressed(void);
bool start_led_detected(void);
bool test_button_pressed(void);

/// @brief Dente azul
static void ble_init(void);
static void host_task(void *param);
static void ble_on_sync(void);
int gatt_svr_init(void);
static int led_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static void start_scan(void);
static int gap_event_cb(struct ble_gap_event *event, void *arg);
static void addr_to_str(const ble_addr_t *addr, char *str, size_t size);
void ble_restart_advertising(void);

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
    POST_RUN     ///< Estado pós-prova: permite ajustes de KP via BLE antes do próximo teste
} robot_state_t;

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
        tskIDLE_PRIORITY + 5, // prioridade alta
        &lineFollowerTaskHandle, // guarda handle
        1 // Core 1
    );
}

void runfollowerTask(void *pvParameters)
{
    while (1)
    {
        // Espera até receber sinal de iniciar a prova
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // bloqueia até ser notificado pelo ControlTask

        // LOOP CRÍTICO DA PROVA
        while (!run_line_follower()) // retorna true quando a prova termina
        {
            vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS)); // loop rápido, ex: 1–2 ms
        }

        // Prova terminou, avisa ControlTask
        xTaskNotifyGive(controlTaskHandle); // pode usar handle da task de controlo
    }
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
    motors_stop();

    while (1)
    {
        switch (state)
        {
        case WAIT_CALIB:
            motors_stop();
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
            motors_stop();
            qtr_calibrate(duration_ms); // corre UMA VEZ
            calib_done = true;          // Marca calibração feita
            state = WAIT_START;
            break;

        case WAIT_START:
            motors_stop();
            if (start_led_detected())
            {
                vTaskDelay(pdMS_TO_TICKS(2000)); // Pequena espera antes de arrancar
                state = RUN;
            }
            break;

        case RUN:
            // Notifica a task do line follower para iniciar
            xTaskNotifyGive(lineFollowerTaskHandle);

            // Espera que a prova termine
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            
           // state = POST_RUN;
            break;

        case POST_RUN:
            // Liga o BLE se ainda não estiver ativo
            if (!ble_active)
            {
                ble_restart_advertising(); // inicia advertising para KP
                ble_active = true;
            }

            // Atualiza KP via BLE se comando recebido
            if (KP_command)
            {
                KP = KP_new; // atualizar KP para o próximo teste
                ESP_LOGI(TAG, "KP received: %.2f", KP_new);
                KP_command = false;
            }

            // Aguarda botão extra para iniciar próximo teste
            if (test_button_pressed())
            {
                // Pausa o BLE apenas uma vez
                if (ble_active)
                {
                    ble_gap_adv_stop();
                    ble_gap_disc_cancel();
                    ble_active = false;
                }
                state = RUN;
            }
        }
    }
}

bool test_button_pressed(void)
{
    static int last_btn_level = -1;      // guarda último estado do botão
    int level = gpio_get_level(BTN_RUN); // lê GPIO atual

    if (level != last_btn_level)
    { // só loga se houver alteração
        ESP_LOGI(TAG, "BTN_RUN = %d", level);
        last_btn_level = level;
    }

    // **cede CPU para não disparar watchdog**
    vTaskDelay(pdMS_TO_TICKS(10)); // ou taskYIELD()

    return (level == 1); // devolve true se botão pressionado
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

bool start_led_detected(void)
{
    return true; // ou true, conforme precisares nos testes
}

/**
 * @brief Controlo do LED Verde RGB - Built-In
 *
 */

/* Converte endereço BLE em string XX:XX:XX:XX:XX:XX */
static void addr_to_str(const ble_addr_t *addr, char *str, size_t size)
{
    snprintf(str, size,
             "%02X:%02X:%02X:%02X:%02X:%02X",
             addr->val[5], addr->val[4], addr->val[3],
             addr->val[2], addr->val[1], addr->val[0]);
}

/* Callback de anúncio recebido */
static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    if (event->type == BLE_GAP_EVENT_DISC)
    {
        const struct ble_gap_disc_desc *d = &event->disc;

        char addr_str[18];
        addr_to_str(&d->addr, addr_str, sizeof(addr_str));

        /* Tenta extrair nome do advertising */
        struct ble_hs_adv_fields fields;
        int rc = ble_hs_adv_parse_fields(&fields, d->data, d->length_data);
        char name[64] = "Unknown";

        if (rc == 0)
        {
            if (fields.name != NULL && fields.name_len > 0)
            {
                int len = fields.name_len < (int)sizeof(name) - 1 ? fields.name_len : (int)sizeof(name) - 1;
                memcpy(name, fields.name, len);
                name[len] = '\0';
            }
        }

        ESP_LOGI(TAG, "Device: %s | MAC: %s | RSSI: %d dBm",
                 name, addr_str, d->rssi);
    }

    return 0;
}

/* Inicia scan contínuo */
static void start_scan(void)
{
    struct ble_gap_disc_params disc_params;

    memset(&disc_params, 0, sizeof(disc_params));

    disc_params.itvl = 0x50;       // intervalo de scan
    disc_params.window = 0x30;     // janela de scan
    disc_params.passive = 0;       // 0 = active scan
    disc_params.limited = 0;       // general discovery
    disc_params.filter_policy = 0; // BLE_HCI_SCAN_FILT_NO_WL

    /* Algumas versões não têm filter_dup/filter_dups – se não existir, ignora.
    disc_params.filter_dup = 0; */

    int rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER,
                          &disc_params, gap_event_cb, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error starting discovery; rc=%d", rc);
    }
    else
    {
        ESP_LOGI(TAG, "Scanning for BLE devices...");
    }
}

/* Callback chamado quando o cliente escreve na characteristic */
static int led_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR)
    {
        if (ctxt->om->om_len >= 2) // precisamos de 2 bytes
        {
            uint16_t raw = (ctxt->om->om_data[0] << 8) | ctxt->om->om_data[1]; // big-endian
            KP_new = (float)raw;                                               // valor literal
            KP_command = true;                                                 // sinaliza task principal
            ESP_LOGI(TAG, "KP write received: %.2f", KP_new);
        }
    }

    return 0; // sucesso
}

/* Tabela GATT com um serviço simples e characteristic LED */
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        /*** Serviço personalizado ***/
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID128_DECLARE(0x04, 0xa4, 0xc3, 0x5f, 0xef, 0xba, 0x6f, 0xae, 0xa7, 0x43, 0xff, 0x43, 0x92, 0x9e, 0x68, 0x07),

        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = BLE_UUID128_DECLARE(0x4f, 0x46, 0x90, 0x8f, 0x77, 0x46, 0x42, 0x9f, 0xa7, 0x5c, 0xcc, 0x71, 0x2e, 0x14, 0x59, 0xc9),

                .access_cb = led_chr_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE, // write sem resposta chega
            },
            {0} // terminador
        },
    },
    {0} // terminador de serviços
};

int gatt_svr_init(void)
{
    int rc;

    // Inicializa serviços padrão
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Conta e adiciona os teus serviços
    rc = ble_gatts_count_cfg(gatt_svcs);
    ESP_LOGI(TAG, "ble_gatts_count_cfg() rc=%d", rc);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Erro count_cfg: %d", rc);
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svcs);
    ESP_LOGI(TAG, "ble_gatts_add_svcs() rc=%d", rc);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Erro add_svcs: %d", rc);
        return rc;
    }

    ESP_LOGI(TAG, "GATT services registados OK!");
    return 0;
}

/* Callback NimBLE quando o host está pronto */
static void ble_on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Address infer failed; rc=%d", rc);
        return;
    }
    // NOVA PARTE: registar serviços GATT AQUI (após sync)
    rc = gatt_svr_init();
    if (rc != 0)
    {
        ESP_LOGE(TAG, "gatt_svr_init() falhou: %d", rc);
        return;
    }
    // Nome do dispositivo
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    const char *name = "DRAGSTER";
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    // Parâmetros de advertising
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ESP_LOGI(TAG, "Starting advertising...");
    ble_gap_adv_start(
        own_addr_type,
        NULL,
        BLE_HS_FOREVER,
        &adv_params,
        NULL,
        NULL);
}

void ble_restart_advertising(void)
{
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, NULL, NULL);
}

/* Task principal NimBLE (corre o host) */
static void host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run(); // Nunca retorna
    nimble_port_freertos_deinit();
}

/* Inicialização NimBLE + FreeRTOS */
static void ble_init(void)
{
    // NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // A partir do IDF 5.x, o controller + HCI é feito dentro do nimble_port_init()
    nimble_port_init(); // se isto falhar, retorna assert/log
    // Serviços padrão GAP/GATT
    // ble_svc_gap_init();
    // ble_svc_gatt_init();

    // Registar os teus serviços
    int rc = ble_gatts_count_cfg(gatt_svcs);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(gatt_svcs);
    assert(rc == 0);
    // Configurar callbacks NimBLE host
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.reset_cb = NULL;

    // Criar task FreeRTOS para o host NimBLE
    nimble_port_freertos_init(host_task);
}

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


*/