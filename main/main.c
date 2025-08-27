#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "sdkconfig.h"
#include "driver/i2c.h"
#include "nvs_flash.h"

#include "BH1750.h"

// ERROR HANDLE
// return 1 je error
// return 0 je ok
// 

uint8_t ble_addr_type;
void ble_app_advertise(void);

#define I2C_MASTER_NUM        I2C_NUM_0 // koristi prvu I2C sabirnicu kao uređaj za komunikaciju
#define I2C_MASTER_SDA_IO     11 // GPIO pin broj 11 za SDA (data linija)
#define I2C_MASTER_SCL_IO     22 // GPIO pin broj 22 za SCL (clock linija)
#define I2C_MASTER_FREQ_HZ    400000 // frekvencija sabirnice 400kHz fast mode
#define BH1750_ADDR           0x23 // ADDR nije spojen na VCC ili GND pa default adresa 0x23


static float light_intensity = 0.0f;


void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO, // definira sda pin
        .scl_io_num = I2C_MASTER_SCL_IO, // definira scl pin
        .sda_pullup_en = GPIO_PULLUP_ENABLE, // omogućuje pull-up otpornike na sda
        .scl_pullup_en = GPIO_PULLUP_ENABLE, // omogućuje pull-up otpornike na scl
        .master.clk_speed = I2C_MASTER_FREQ_HZ, // postavlja brzinu sabirnice
    };
    i2c_param_config(I2C_MASTER_NUM, &conf); // primjenjuje konfiguraciju na I2C sabirnicu
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0); //instalira I2C driver
}

static int device_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // ctxt pokazivač na ble stack
    // cxt->om pokazivač na os_mbuf strukturu koja 
    //sadrži podatke koje je korisnik poslao
    char data[ctxt->om->om_len + 1];
    // memcpy kopira podatke iz om_data u data
    memcpy(data, ctxt->om->om_data, ctxt->om->om_len);
    // dodaj null terminator na kraj stringa
    data[ctxt->om->om_len] = '\0';


    if (strcmp(data, "Pokreni") == 0)
    {
        bh1750_power_on(I2C_MASTER_NUM, BH1750_ADDR);
    }
    else if (strcmp(data, "Ugasi") == 0)
    {
        bh1750_power_off(I2C_MASTER_NUM, BH1750_ADDR);
    }
    else if (strcmp(data, "const-visok") == 0)
    {
        bh1750_set_mode(I2C_MASTER_NUM, BH1750_ADDR, CONT_HIGH_RES_MODE);
    }
    else if (strcmp(data, "const-nizak") == 0)
    {
        bh1750_set_mode(I2C_MASTER_NUM, BH1750_ADDR, CONT_LOW_RES_MODE);
    }
    else if(strcmp(data, "jednom-visok")==0)
    {
        bh1750_set_mode(I2C_MASTER_NUM, BH1750_ADDR, ONE_TIME_HIGH_RES_MODE);
    }
    else if(strcmp(data, "jednom-nizak")==0)
    {
        bh1750_set_mode(I2C_MASTER_NUM, BH1750_ADDR, ONE_TIME_LOW_RES_MODE);
    }
    else
    {
        printf("error");
    }

    return 0;
}

static int device_read(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    char buf[32];
    // sprema u buffer tekst "Lux: light_intensity(value)"
    // vraća broj znakova spremljenih u buf
    int len = snprintf(buf, sizeof(buf), "Lux: %.2f", light_intensity);
    if (len > 0) {
        // kopiraj buf u mbuf (om pokazivač na mbuf strukturu)
        return os_mbuf_append(ctxt->om, buf, len) == 0 ? 0 : 1;
    }
    return 1;
}

// gatt_svcs definira servise i karakteristike koje uređaj nudi
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY, // primarni servis
     .uuid = BLE_UUID16_DECLARE(0x180),  // 0x180 General Access servis
     .characteristics = (struct ble_gatt_chr_def[]){ // funkcije koje servis nudi
         {.uuid = BLE_UUID16_DECLARE(0xFEF4), // na adresi 0xFEF4 korisnik može samo čitati podatke
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = device_read},
         {.uuid = BLE_UUID16_DECLARE(0xDEAD), // na adresi 0xDEAD korisnik može pisati podatke
          .flags = BLE_GATT_CHR_F_WRITE,
          .access_cb = device_write},
         {0}}}, // kraj karakteristika
    {0}};

// callback funkcija za GAP događaje
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    // kada se dogodi connect
    // pogledaj status konekcije
    // ako je status 0, konekcija je uspjela
    // ako nije, ponovo pokreće oglašavanje
    case BLE_GAP_EVENT_CONNECT:
        printf("BLE GAP EVENT CONNECTED");
        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }
        break;
    // kada se dogodi disconnect
    // ponovo pokreće oglašavanje
    case BLE_GAP_EVENT_DISCONNECT:
        printf("BLE GAP EVENT DISCONNECTED");
        ble_app_advertise();
        break;
    // kada oglašavanje završi
    //ponovo pokreće oglašavanje
    case BLE_GAP_EVENT_ADV_COMPLETE:
        printf("BLE GAP EVENT");
        ble_app_advertise();
        break;
    default:
        break;
    }
    return 0;
}

void ble_app_advertise(void)
{
    // fields sadrži konfiguraciju oglašavanja
    struct ble_hs_adv_fields fields;
    // definira pokazivač na ime uređaja
    const char *device_name;
    // postavlja sve vrijednosti u fields na 0
    memset(&fields, 0, sizeof(fields));
    // dohvaća ime uređaja definirano u GAP servisu
    device_name = ble_svc_gap_device_name();
    // postavlja ime uređaja u fields
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);


    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // dozvoli povezivanje svim uređajima
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // discovery mode general (reklamira sebe svima)
    // ble_add_type = adresa
    // NULL = nema whitelist
    // BLE_HS_FOREVER = oglašava se zauvijek
    // &adv_params = parametri oglašavanja
    // ble_gap_event = callback funkcija za događaje
    // NULL = nema dodatnih argumenata za callback
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL); 
}
// Automatski generira BLE adresu i pokreće oglašavanje
void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type); // generira adresu automatski
    ble_app_advertise(); // poziv funkcije za oglašavanje
}

// task koji pokreće BLE stack
void host_task(void *param)
{
    nimble_port_run();
}

void app_main()
{
    i2c_master_init();
    bh1750_power_on(I2C_MASTER_NUM, BH1750_ADDR);
    bh1750_set_mode(I2C_MASTER_NUM, BH1750_ADDR, CONT_HIGH_RES_MODE);

    nimble_port_init();

    ble_svc_gap_device_name_set("server-za-projekt");
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);

    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(host_task);

    while (1)
    {
        bh1750_read_light(I2C_MASTER_NUM, BH1750_ADDR, &light_intensity);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}