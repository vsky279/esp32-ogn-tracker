#define DEFAULT_AcftType        1          // [0..15] default aircraft-type: glider
#define DEFAULT_GeoidSepar     40          // [m]
#define DEFAULT_CONbaud    115200
#define DEFAULT_PPSdelay      100
#define DEFAULT_FreqPlan        0
#define DEFAULT_DispPage        3          // Fab501 Page to Display After Boot or Reset	
#define WIFI_ADDRESS_IP1	192	 // 192.168.1.1 for IP Address
#define WIFI_ADDRESS_IP2	168
#define WIFI_ADDRESS_IP3	1
#define WIFI_ADDRESS_IP4	1
#define WIFI_ADDRESS_GW1	0	// 0.0.0.0 for Gateway
#define WIFI_ADDRESS_GW2	0
#define WIFI_ADDRESS_GW3	0
#define WIFI_ADDRESS_GW4	0
#define WIFI_ADDRESS_MK1	255	// 255.255.255.0 for Mask
#define WIFI_ADDRESS_MK2	255
#define WIFI_ADDRESS_MK3	255
#define WIFI_ADDRESS_MK4	0

// #define WITH_HELTEC                        // HELTEC module: PCB LED on GPI025
#define WITH_HELTEC_V2                     // HELTEC module v2
// #define WITH_TTGO                          // TTGO module: PCB LED on GPIO2, GPIO25 free to use as DAC2 output
// #define WITH_TBEAM                          // T-Beam module
// #define WITH_TBEAM_V10                      // T-Beam module
// #define WITH_M5_JACEK                         // JACEK M5 ESP32 OGN-Tracker
// #define WITH_FollowMe                         // by Avionix

// #define WITH_ILI9341                        // 320x240 M5stack
// #define WITH_ST7789                         // IPS 240x240 ST7789
// #define WITH_TFT_LCD                       // TFT LCD
// #define WITH_OLED                          // OLED display on the I2C: some TTGO modules are without OLED display
// #define WITH_OLED2                         // 2nd OLED display, I2C address next higher
#define WITH_U8G2_OLED                     // I2C OLED through the U8g2 library
// #define WITH_U8G2_SH1106                   // correct controller for the bigger OLED
// #define WITH_U8G2_FLIP                     // flip the OLED screen (rotate by 180deg)

#define WITH_RFM95                         // RF chip selection:  both HELTEC and TTGO use sx1276 which is same as RFM95
//#define WITH_SX1262                         // SX1262 Support

// #define WITH_SLEEP                         // with software sleep mode controlled by the long-press on the button

// #define WITH_AXP                           // with AXP192 power controller (T-BEAM V1.0)
// #define WITH_BQ                            // with BQ24295  power controller (FollowMe)

// #define WITH_LED_RX
// #define WITH_LED_TX

// #define WITH_GPS_ENABLE                    // use GPS_ENABLE control line to turn the GPS ON/OFF
#define WITH_BTSERIAL_GPS                  // connect to GPS using bluetooth
#define WITH_GPS_PPS                       // use the PPS signal from GPS for precise time-sync.
// #define WITH_GPS_CONFIG                    // attempt to configure higher GPS baud rate and airborne mode

// #define WITH_GPS_UBX                       // GPS understands UBX
// #define WITH_GPS_MTK                       // GPS understands MTK
// #define WITH_GPS_SRF
// #define WITH_MAVLINK

#define WITH_GPS_UBX_PASS                  // to pass directly UBX packets to/from GPS
#define WITH_GPS_NMEA_PASS                  // to pass directly NMEA to/from GPS

// #define WITH_BMP180                        // BMP180 pressure sensor
// #define WITH_BMP280                        // BMP280 pressure sensor
// #define WITH_BME280                        // BMP280 with humidity (but still works with BMP280)
// #define WITH_MS5607                        // MS5607 pressure sensor
// #define WITH_MS5611                        // MS5611 pressure sensor

// #define WITH_BMX055                        // BMX055 magnetic and IMU sensor

#define WITH_LORAWAN                       // LoRaWAN connectivity
#define WITH_FANET                         // FANET transmission and reception
#define WITH_PAW			   // Add PAW transmission

#define WITH_PFLAA                         // PFLAU and PFLAA for compatibility with XCsoar and LK8000
// #define WITH_POGNT
// #define WITH_GDL90
// #define WITH_PGAV5
#define WITH_LOOKOUT

#define WITH_SKYDEMON			//Adapt NMEA Output for SKYDEMON

#define WITH_CONFIG                        // interpret the console input: $POGNS to change parameters

// #define WITH_BEEPER                        // with digital buzzer
// #define WITH_SOUND                         // with analog sound produced by DAC on pin 25

// #define WITH_KNOB
// #define WITH_VARIO

// #define WITH_SD                            // use the SD card in SPI mode and FAT file system
#define WITH_SPIFFS                        // use SPIFFS file system in Flash
// #define WITH_SPIFFS_FAT
// #define WITH_LOG                           // log own positions and other received to SPIFFS
// #define WITH_SDLOG                         // log own position and other data to uSD card

//#define WITH_STRATUX
// #define WITH_BT_SPP                        // Bluetooth serial port for smartphone/tablet link
// #define WITH_WIFI                          // attempt to connect to the wifi router for uploading the log files
#define WITH_AP                            // Open Access Point MOde
#define WITH_HTTP                           // Open Web Interface

// #define WITH_ENCRYPT                       // Encrypt (optionally) the position

