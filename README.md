# Liquid Dispensing Radiofarmaka I-131 ESP32

Program kendali untuk liquid dispensing I-131 menggunakan ESP32 DevKit V1.

### Pengaturan Pertama

Salin file `include/env.h.example` ke `include/env.h` dan isi definisi konstanta yang masih kosong.

```h
#define WIFI_SSID "ssid wifi anda"
#define WIFI_PASSWORD "password wifi anda"

#define MQTT_SERVER "server mqtt yang digunakan"
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID "liquid_dispensing_esp32"
#define MQTT_USERNAME "username server mqtt"
#define MQTT_PASSWORD "password server mqtt"
```