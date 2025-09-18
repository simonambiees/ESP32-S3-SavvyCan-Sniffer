// slcan_esp32s3.c — ESP32-S3 SLCAN (Lawicel) bridge for SavvyCAN
// IDF: v5.5. Native USB-Serial/JTAG (no extra UART dongle).
// Connect a CAN transceiver to the TWAI pins below.

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "driver/usb_serial_jtag.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_check.h"              // for ESP_RETURN_ON_ERROR

#define TAG "SLCAN"

// ---- TWAI (CAN) pins (adjust to your board) ----
#define TWAI_TX_GPIO 4
#define TWAI_RX_GPIO 5

// ---- Globals ----
static bool bus_open = false;
static bool listen_only = false;
static bool ts_enabled = false;          // timestamps Z1/Z0
static twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();  // default

// ---------- Small helpers ----------
static inline char hex_digit(int v){ return (v<10)?('0'+v):('A'+v-10); }
static inline int from_hex(char c){
    if(c>='0'&&c<='9') return c-'0';
    if(c>='A'&&c<='F') return c-'A'+10;
    if(c>='a'&&c<='f') return c-'a'+10;
    return -1;
}

// ---------- USB CDC I/O ----------
static size_t usb_write_buf(const void *buf, size_t len){
    return usb_serial_jtag_write_bytes((const uint8_t*)buf, len, pdMS_TO_TICKS(1000));
}
static void usb_write_str(const char *s){ (void)usb_write_buf(s, strlen(s)); }
static void usb_write_cr(void){ const char r='\r'; (void)usb_write_buf(&r,1); }
static void usb_write_bell(void){ const char b=0x07; (void)usb_write_buf(&b,1); }

// Blocking get one char (1 s polling, yields to scheduler)
static int usb_read_char(uint8_t *out){
    while (1) {
        size_t n = usb_serial_jtag_read_bytes(out, 1, pdMS_TO_TICKS(1000));
        if (n == 1) return 1;          // got a byte
        vTaskDelay(pdMS_TO_TICKS(1));  // avoid tight spin if no host/data
    }
}

// ---------- Bitrate handling (Sx) ----------
// LAWICEL Sx map: S0=10k, S1=20k, S2=50k, S3=100k, S4=125k, S5=250k, S6=500k, S7=800k, S8=1M
static bool set_bitrate_from_code(int code){
    switch(code){
        case 0: timing = (twai_timing_config_t)TWAI_TIMING_CONFIG_10KBITS();  return true;
        case 1: timing = (twai_timing_config_t)TWAI_TIMING_CONFIG_20KBITS();  return true;
        case 2: timing = (twai_timing_config_t)TWAI_TIMING_CONFIG_50KBITS();  return true;
        case 3: timing = (twai_timing_config_t)TWAI_TIMING_CONFIG_100KBITS(); return true;
        case 4: timing = (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS(); return true;
        case 5: timing = (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS(); return true;
        case 6: timing = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS(); return true;
        case 7: timing = (twai_timing_config_t)TWAI_TIMING_CONFIG_800KBITS(); return true;
        case 8: timing = (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();   return true;
        default: return false;
    }
}

// ---------- CAN control ----------
static esp_err_t can_open(void){
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        TWAI_TX_GPIO, TWAI_RX_GPIO,
        listen_only ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL
    );
    g.rx_queue_len = 64;
    g.tx_queue_len = 32;
    g.alerts_enabled = TWAI_ALERT_NONE;

    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_RETURN_ON_ERROR(twai_driver_install(&g, &timing, &f), TAG, "twai_driver_install failed");
    ESP_RETURN_ON_ERROR(twai_start(), TAG, "twai_start failed");
    bus_open = true;
    return ESP_OK;
}

static void can_close(void){
    if(bus_open){
        twai_stop();
        twai_driver_uninstall();
        bus_open = false;
    }
}

// ---------- Frame encode (to host) ----------
static void send_frame_slcan(const twai_message_t *m){
    // Format (timestamps disabled):
    //  tIII DL D0 D1 ... CR    (11-bit)
    //  TIIIIIIII DL D0 ... CR  (29-bit)
    // With timestamps enabled (Z1): append 4 hex of 1ms timer (wrap 16-bit)
    char buf[80];
    int p = 0;
    const bool ext = m->extd;
    buf[p++] = ext ? 'T' : 't';

    if(ext){
        for(int sh=28; sh>=0; sh-=4) buf[p++] = hex_digit((m->identifier >> sh) & 0xF);
    }else{
        for(int sh=8; sh>=0; sh-=4)  buf[p++] = hex_digit((m->identifier >> sh) & 0xF);
    }

    int dlc = m->data_length_code & 0xF;
    buf[p++] = hex_digit(dlc);

    for(int i=0;i<dlc;i++){
        buf[p++] = hex_digit((m->data[i] >> 4) & 0xF);
        buf[p++] = hex_digit( m->data[i]       & 0xF);
    }

    if(ts_enabled){
        uint16_t ts = (uint16_t)((esp_timer_get_time() / 1000ULL) & 0xFFFF);
        buf[p++] = hex_digit((ts >> 12) & 0xF);
        buf[p++] = hex_digit((ts >>  8) & 0xF);
        buf[p++] = hex_digit((ts >>  4) & 0xF);
        buf[p++] = hex_digit((ts      ) & 0xF);
    }

    buf[p++] = '\r';
    (void)usb_write_buf(buf, p);
}

// ---------- RX Task (CAN -> Host) ----------
static void can_rx_task(void *arg){
    while(1){
        if(!bus_open){ vTaskDelay(pdMS_TO_TICKS(50)); continue; }
        twai_message_t m;
        if(twai_receive(&m, pdMS_TO_TICKS(50)) == ESP_OK){
            if(!m.rtr) send_frame_slcan(&m); // ignore RTR for simplicity
        }
    }
}

// ---------- Parse & send (Host -> CAN) ----------
static bool parse_and_send_frame(const char *cmd){
    if(!bus_open) return false;
    bool ext = (cmd[0] == 'T');
    const char *p = cmd + 1;

    uint32_t id = 0;
    int n_hex = ext ? 8 : 3;
    for(int i=0;i<n_hex;i++){
        int d = from_hex(*p++);
        if(d < 0) return false;
        id = (id << 4) | (uint32_t)d;
    }
    int dlc = from_hex(*p++);
    if(dlc < 0 || dlc > 8) return false;

    twai_message_t m = {0};
    m.identifier = id;
    m.extd = ext;
    m.data_length_code = dlc;
    for(int i=0;i<dlc;i++){
        int hi = from_hex(*p++);
        int lo = from_hex(*p++);
        if(hi < 0 || lo < 0) return false;
        m.data[i] = (uint8_t)((hi << 4) | lo);
    }
    return (twai_transmit(&m, pdMS_TO_TICKS(50)) == ESP_OK);
}

// ---------- Command handler ----------
static void handle_cmd(char *cmd){
    switch(cmd[0]){
        case 'V': usb_write_str("V1013"); usb_write_cr(); break;   // firmware version
        case 'v': usb_write_str("vESP32S3"); usb_write_cr(); break;// hardware version
        case 'N': usb_write_str("NESPS3"); usb_write_cr(); break;  // serial num / name
        case 'Z': // timestamps: Z0 off, Z1 on
            if(cmd[1]=='0'){ ts_enabled=false; usb_write_cr(); }
            else if(cmd[1]=='1'){ ts_enabled=true; usb_write_cr(); }
            else usb_write_bell();
            break;
        case 'S': { // set bitrate (must be before opening)
            if(bus_open){ usb_write_bell(); break; }
            int code = cmd[1]-'0';
            if(code>=0 && code<=8 && set_bitrate_from_code(code)) usb_write_cr();
            else usb_write_bell();
            break;
        }
        case 'O': // open (active)
            listen_only = false;
            if(!bus_open && can_open()==ESP_OK) usb_write_cr(); else usb_write_bell();
            break;
        case 'L': // open (listen-only)
            listen_only = true;
            if(!bus_open && can_open()==ESP_OK) usb_write_cr(); else usb_write_bell();
            break;
        case 'C': // close
            can_close(); usb_write_cr(); break;

        case 't': case 'T':
            if(parse_and_send_frame(cmd)) usb_write_cr(); else usb_write_bell();
            break;

        default:
            usb_write_bell(); // unknown / unsupported
            break;
    }
}

// ---------- USB RX Task (Host -> Commands) ----------
static void usb_rx_task(void *arg){
    char buf[256]; int len = 0;
    while(1){
        uint8_t c;
        if(usb_read_char(&c) == 1){
            if(c == '\r'){        // command terminator
                buf[len] = 0;
                if(len > 0) handle_cmd(buf);
                len = 0;
            }else{
                if(len < (int)sizeof(buf)-1) buf[len++] = (char)c;
            }
        }
    }
}

// ---------- Main ----------
void app_main(void){
    ESP_LOGI(TAG, "SLCAN bridge starting (USB CDC + TWAI)");

    // *** IMPORTANT: Install USB-Serial-JTAG driver BEFORE using read/write ***
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = 512,
        .rx_buffer_size = 512,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));

    // Now it's safe to start tasks that call usb_serial_jtag_* APIs
    xTaskCreatePinnedToCore(usb_rx_task, "usb_rx", 4096, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(can_rx_task, "can_rx", 4096, NULL, 9,  NULL, 1);
}
