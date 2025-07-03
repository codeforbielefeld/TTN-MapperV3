/*
-----------------------------------------------------------------------------------------------------------
 TTNMapper node with GPS running on an Heltec "WiFi Lora32 v2": https://heltec.org/project/wifi-lora-32/

 Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman: https://github.com/matthijskooijman/arduino-lmic/blob/master/examples/ttn-abp/ttn-abp.ino
 Copyright (c) 2018 sbiermann: https://github.com/sbiermann/Lora-TTNMapper-ESP32
 Copyright (c) 2019 sistemasorp: https://github.com/sistemasorp/Heltec-Wifi-Lora-32-TTN-Mapper
 Copyright (c) 2020 Stefan Onderka: https://www.onderka.com/computer-und-netzwerk/ttnmapper-node-mit-heltec-sx1276-lora-esp32-v2
 Case: https://www.thingiverse.com/thing:4145143

 This code expects a serial connected GPS module like an U-Blox NEO-6m, you may change baud rate and pins. 
 Default is GPIO 2 Rx and GPIO 17 Tx (Closest to "end" of board), speed 9600 Baud. Use 3V3 or Vext for GPS power
 
 Set your gateway GPS coordinates to show distance and direction when mapping.

 The activation method of the TTN device has to be ABP. 

 Libraries needed: U8x8 (From U8g2), TinyGPS++, LMICPP-Arduino by ngraziano, SPI, Wifi
-----------------------------------------------------------------------------------------------------------
*/

#include <Arduino.h>
#include <boardconfig.h>
#include <certificationprotocol.h>


/* Hardware serial */
#include <HardwareSerial.h>
/* GPS */
#include <TinyGPS++.h>

/* Hardware abstraction layer */
#include <hal/hal.h>
#include <hal/hal_io.h>
#include <keyhandler.h>
/* SPI */
#include <SPI.h>
/* OLED */
#include <U8x8lib.h>
/* Wireless */
#include "WiFi.h"
#include <lmic.h>
#include "settings.h"

/* GPS Rx pin */
#define GPS_RX 6   // 12 (V2)
/* GPS Tx pin */  
#define GPS_TX 7   // 13 (V2)
/* GPS baud rate */
#define GPS_BAUD 9600
/* send with confirmation */
#define send_confirmed 0

void show_gps_status();
void do_send();

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
constexpr OsDeltaTime TX_INTERVAL = OsDeltaTime::from_sec(30);

/* Upper TinyGPS++ HDOP value limit to send Pakets with */
int HDOP_MAX = 300; // Set to 200-500 (HDOP 2.00 - 5.00)

/* Time to wait in seconds if HDOP is above limit */
const unsigned TX_WAIT_INTERVAL = 5;

/* Define serial for GPS */
HardwareSerial GPSSerial(1);

/* Init GPS */
TinyGPSPlus gps;

/* Define OLED (SCL, SDA, RST - See pinout) */

U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(SCL_OLED, SDA_OLED, RST_OLED);

OsJobType<Lmic> sendjob;

// Pin mapping Heltec LoRa V2
constexpr lmic_pinmap lmic_pinsV2 = {
    .nss = 18,
    .prepare_antenna_tx = nullptr,
    .rst = 14,
    .dio = {35, 34}, // BUSY, DIO1
};

// Pin mapping Heltec LoRa V3
constexpr lmic_pinmap lmic_pinsV3 = {
    .nss = 8,
    .prepare_antenna_tx = nullptr,
    .rst = 12,
    .dio = {13, 14}, // BUSY, DIO1
};

// Heltec LoRa V2
// Radio class for SX1276
// RadioSx1276 radio{lmic_pinsV2, ImageCalibrationBand::band_863_870};

// Heltec LoRa V3
// Radio class for SX1262
RadioSx1262 radio{lmic_pinsV3, ImageCalibrationBand::band_863_870};

// Create an LMIC object with the right band
LmicEu868 LMIC{radio};

OsTime nextSend;

/* Buffer with data to send to TTNMapper */
uint8_t txBuffer[9];

// buffer to save current lmic state (size may be reduce)
const size_t SAVE_BUFFER_SIZE = 200;
RTC_DATA_ATTR uint8_t saveState[SAVE_BUFFER_SIZE];

/* Buffer for sending to serial line*/
char sbuf[16];

void show_gps_status() {
  /* For sprintf to OLED display */
  char s[16]; 
  String toLog;
  
  uint32_t LatitudeBinary, LongitudeBinary;
  uint16_t altitudeGps;
  uint8_t hdopGps;
  
  while (GPSSerial.available())
    gps.encode(GPSSerial.read());

  /* Get GPS data */
  double latitude = gps.location.lat();
  double longitude = gps.location.lng();
  double altitude = gps.altitude.meters();
  uint32_t hdop = gps.hdop.value();

  /* For OLED display */
  float hdopprint = (float)hdop / 100.0;
  uint32_t sats = gps.satellites.value();
  float speed = gps.speed.kmph();

  /* Serial output */
  Serial.println();

  /* Distance to GW in Meters */
  double fromhome = gps.distanceBetween(gps.location.lat(), gps.location.lng(), HOME_LAT, HOME_LNG);

  /* Course to GW */
  double direction_home = gps.courseTo(gps.location.lat(), gps.location.lng(), HOME_LAT, HOME_LNG);

  Serial.print("Latitude:             ");
  sprintf(s, "%f", latitude);
  Serial.println(s);
  //Serial.println(latitude);
  Serial.print("Longitude:            ");
  sprintf (s, "%f", longitude);
  Serial.println(s);
  //Serial.println(longitude);
  Serial.print("Altitude (m):         ");
  Serial.println(altitude);
  Serial.print("Speed (km/h):         ");
  Serial.println(speed);
  Serial.print("HDOP:                 ");
  Serial.println(hdopprint);
  Serial.print("Satellite count:      ");
  Serial.println(sats);
  if ( fromhome < 1000) {
    // Less than 1000 m
    Serial.print("Distance to GW (m):   ");
    Serial.println(fromhome);
  } else {
    // More than 1 km
    Serial.print("Distance to GW (km):  ");
    Serial.println(fromhome/1000);
  }
  Serial.print("Direction to GW:      ");
  Serial.print((String)gps.cardinal(direction_home) + " (" + (String)direction_home + "°)");

  /*
  Serial.print("Characters processed: ");
  Serial.println(characters);
  Serial.print("Sentences with fix:   ");
  Serial.println(fixes);
  Serial.print("Checksum passed:      ");
  Serial.println(passedchecksums);
  Serial.print("Checksum failed:      ");
  Serial.println(failedchecksums);
  */

  LatitudeBinary = ((latitude + 90) / 180.0) * 16777215;
  LongitudeBinary = ((longitude + 180) / 360.0) * 16777215;

  /* OLED output */
  // Latitude
  u8x8.clearLine(1);
  u8x8.setCursor(0, 1);
  u8x8.print("Lati ");
  u8x8.setCursor(5, 1);
  sprintf(s, "%f", latitude);
  u8x8.print(s);

  // Longitude
  u8x8.clearLine(2);
  u8x8.setCursor(0, 2);
  u8x8.print("Long ");
  u8x8.setCursor(5, 2);
  sprintf(s, "%f", longitude);
  u8x8.print(s);

  // Altitude
  u8x8.clearLine(3);
  u8x8.setCursor(0, 3);
  u8x8.print("Alti ");
  u8x8.setCursor(5, 3);
  sprintf(s, "%f", altitude);
  u8x8.print(s);

  // Number of fixes
  /*
  u8x8.clearLine(4);
  u8x8.setCursor(0, 4);
  u8x8.print("Fixs ");
  u8x8.setCursor(5, 4);
  u8x8.print(fixes);
  */

  // Distance from home GW
  u8x8.clearLine(4);
  u8x8.setCursor(0, 4);
  u8x8.print("Dist ");
  u8x8.setCursor(5, 4);
  if ( fromhome < 1000) {
    // Less than 1000 m
    sprintf(s, "%.0f m", fromhome);
    // More than 1 km
  } else {
    sprintf(s, "%.3f km", fromhome/1000);
  }
  u8x8.print(s);

  // HDOP
  u8x8.clearLine(5);
  u8x8.setCursor(0, 5);
  u8x8.print("HDOP ");
  u8x8.setCursor(5, 5);
  u8x8.print(hdopprint);

  // Number of satellites
  u8x8.clearLine(6);
  u8x8.setCursor(0, 6);
  u8x8.print("Sats ");
  u8x8.setCursor(5, 6);
  u8x8.print(sats);

  txBuffer[0] = ( LatitudeBinary >> 16 ) & 0xFF;
  txBuffer[1] = ( LatitudeBinary >> 8 ) & 0xFF;
  txBuffer[2] = LatitudeBinary & 0xFF;

  txBuffer[3] = ( LongitudeBinary >> 16 ) & 0xFF;
  txBuffer[4] = ( LongitudeBinary >> 8 ) & 0xFF;
  txBuffer[5] = LongitudeBinary & 0xFF;

  altitudeGps = altitude;
  txBuffer[6] = ( altitudeGps >> 8 ) & 0xFF;
  txBuffer[7] = altitudeGps & 0xFF;

  hdopGps = hdop/10;
  txBuffer[8] = hdopGps & 0xFF;

  toLog = "";
  for(size_t i = 0; i<sizeof(txBuffer); i++) {
    char buffer[3];
    sprintf(buffer, "%02x", txBuffer[i]);
    toLog = toLog + String(buffer);
  }

  Serial.println();

  /* Print Tx buffer on serial console */
  // Serial.println(toLog);
}

void show_lora_status (int16_t rssi, int8_t snr) {
  int8_t snrDecimalFraction = 0; // ToDo

  Serial.print(F("Received "));
  Serial.print(LMIC.getDataLen());
  Serial.println(F(" bytes payload"));
  Serial.print("RSSI: ");
  Serial.print(rssi);
  Serial.print(" dBm, SNR: ");
  Serial.print(snr);
  Serial.print(".");
  Serial.print(snrDecimalFraction);
  Serial.println(" dB");
  /*
  u8x8.clearLine(6);
  u8x8.drawString(0, 6, "RX ");
  u8x8.setCursor(3, 6);
  u8x8.printf("%i bytes", LMIC.dataLen);
  */
  u8x8.clearLine(6);
  u8x8.setCursor(0, 6);
  u8x8.printf("RSSI: %4d dBm", rssi);
  u8x8.clearLine(7);
  u8x8.setCursor(0, 7);
  u8x8.printf("SNR: %d.%d dB", snr, snrDecimalFraction);
}

void onEvent(EventType ev) {
  /* Clear OLED line #8 */
  u8x8.clearLine(7);
  switch (ev) {
    case EventType::JOINING:
      Serial.println(F("> EV_JOINING"));
      u8x8.drawString(0, 7, "EV_JOINING      ");
      break;
    case EventType::JOINED:
      Serial.println(F("> EV_JOINED"));
      u8x8.drawString(0, 7, "EV_JOINED       ");
      /* Disable link check validation (automatically enabled during join, but not supported by TTN at this time). */
      LMIC.setLinkCheckMode(0);
      break;
    case EventType::TXCOMPLETE:
      Serial.println(F("> EV_TXCOMPLETE (including wait for RX window)"));
      u8x8.drawString(0, 7, "EV_TXCOMPLETE   ");
      digitalWrite(BUILTIN_LED, LOW);
      if (LMIC.getTxRxFlags().test(TxRxStatus::ACK)) {
        /* Received ACK */
        Serial.println(F("Received ack"));
        u8x8.drawString(0, 7, "ACK RECEIVED    ");
        int16_t rssi = radio.get_last_packet_rssi();
        int8_t snr = radio.get_last_packet_snr_x4() / 4;

        show_lora_status(rssi, snr);
      } else {
        /* no ACK */
        Serial.println(F("No ack received"));
        u8x8.drawString(0, 7, "NO ACK RECEIVED ");
      }
/*
      if (LMIC.getDataLen() != 0) {        
        // Received data
      }
  */    
      // we have transmit
      // save before going to deep sleep.
      /*
      {
      auto store = StoringBuffer{saveState};
      LMIC.saveState(store);
      saveState[SAVE_BUFFER_SIZE - 1] = 51;
      Serial.print(F("State save len = "));
      Serial.println(store.length());
      ESP.deepSleep(TX_INTERVAL.to_us());
      }
      */
      break;
    case EventType::RESET:
      Serial.println(F("> EV_RESET"));
      u8x8.drawString(0, 7, "EV_RESET        ");
      break;
    case EventType::LINK_DEAD:
      Serial.println(F("> EV_LINK_DEAD"));
      u8x8.drawString(0, 7, "EV_LINK_DEAD    ");
      break;
    case EventType::LINK_ALIVE:
      Serial.println(F("> EV_LINK_ALIVE"));
      u8x8.drawString(0, 7, "EV_LINK_ALIVE   ");
      break;
    default:
      Serial.print(F("Unknown event "));
      Serial.println("");
      u8x8.drawString(0, 7, "UNKNOWN EVT     ");
      break;
  }
}

void do_send() {
  /* Check for running Tx/Rx job */
  if (LMIC.getOpMode().test(OpState::TXRXPEND)) {
    /* Tx pending */
    Serial.println(F("> OP_TXRXPEND, not sending"));
    u8x8.drawString(0, 7, "OP_TXRXPEND, not sent");
  } else {
    /* No pending Tx, Go! */
    show_gps_status();

    if ( ((int)gps.hdop.value() < HDOP_MAX) && ((int)gps.hdop.value() != 0) ) {
      /* GPS OK, prepare upstream data transmission at the next possible time */
      //build_packet();
      LMIC.setTxData2(1, txBuffer, sizeof(txBuffer), send_confirmed);
      Serial.println();
      Serial.println(F("> PACKET QUEUED"));
      u8x8.drawString(0, 7, "PACKET_QUEUED   ");
      digitalWrite(BUILTIN_LED, HIGH);
      nextSend = os_getTime() + TX_INTERVAL;
    } else { 
      /* GPS not ready */
      //Serial.println();
      Serial.println(F("> GPS NOT READY, not sending"));
      //Serial.println(gps.hdop.value());
      u8x8.drawString(0, 7, "NO_GPS_WAIT     ");
      /* Schedule next transmission in TX_WAIT_INTERVAL seconds */
      // os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_WAIT_INTERVAL), do_send);
      delay(10000);
    }
  } 
  /* Next TX is scheduled after TX_COMPLETE event. */
}

void setup() {
  /* Start serial */
  Serial.begin(115200);

  /* Turn off WiFi and Bluetooth */
  WiFi.mode(WIFI_OFF);
  btStop();

  /* Turn on Vext pin fpr GPS power */
  Serial.println("Turning on Vext pins for GPS ...");
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);

  Serial.println("Waiting 2 seconds for GPS to power on ...");
  delay(2000);

  /* Initialize GPS */
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  /* Initialize OLED */
  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);

  /* Initialize SPI */
  SPI.begin();    // sck, miso, mosi, ss

  /* Initialize LMIC */
  os_init();
  LMIC.init();   // V3

  /* Reset the MAC state. Session and pending data transfers will be discarded. */
  LMIC.reset();

  AesKey appkey;
  std::copy(APP_KEY, APP_KEY + 16, appkey.begin());
  AesKey netkey;
  std::copy(NET_KEY, NET_KEY + 16, netkey.begin());
  LMIC.setSession(TTN_NET_ID, DEV_ADRESS, netkey, appkey);

// SF9 rx2
  LMIC.setRx2Parameter(869525000, SF9);

  // Channel 0,1,2 : default channel for EU868
  LMIC.setupChannel(0, 868100000, dr_range_map(SF12, SF7));
  LMIC.setupChannel(1, 868300000, dr_range_map(SF12, SF7));
  LMIC.setupChannel(2, 868500000, dr_range_map(SF12, SF7));
  LMIC.setupChannel(3, 867100000, dr_range_map(SF12, SF7));
  LMIC.setupChannel(4, 867300000, dr_range_map(SF12, SF7));
  LMIC.setupChannel(5, 867500000, dr_range_map(SF12, SF7));
  LMIC.setupChannel(6, 867700000, dr_range_map(SF12, SF7));
  LMIC.setupChannel(7, 867900000, dr_range_map(SF12, SF7));
  LMIC.setupChannel(8, 868800000, dr_range_map(FSK, FSK));

  // Tx Datarate for EU868  0 => SF12 ... 5 => SF7
  LMIC.setDrTx(SF7);

  LMIC.setEventCallBack(onEvent);
 
  // set clock error to allow good connection.
  LMIC.setClockError(MAX_CLOCK_ERROR * 1 / 100);
 
  /* Disable link check validation */
  LMIC.setLinkCheckMode(0);

  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);

  // Restore saved state
  if (saveState[SAVE_BUFFER_SIZE - 1] == 51) {
    auto retrieve = RetrieveBuffer{saveState};
    LMIC.loadState(retrieve);
    saveState[SAVE_BUFFER_SIZE - 1] = 0;
  }

  // Start job to send data
  nextSend = os_getTime();
}

void loop() {
   OsDeltaTime freeTimeBeforeNextCall = LMIC.run();

  if (freeTimeBeforeNextCall > OsDeltaTime::from_ms(10)) {
    // we have more than 10 ms to do some work.
    // the test must be adapted from the time spend in other task
    if (nextSend < os_getTime()) {
      if (LMIC.getOpMode().test(OpState::TXRXPEND)) {
        Serial.println("OpState::TXRXPEND, not sending");
      } else {
        do_send();
      }
    } else {
      OsDeltaTime freeTimeBeforeSend = nextSend - os_getTime();
      OsDeltaTime to_wait =
          std::min(freeTimeBeforeNextCall, freeTimeBeforeSend);
      delay(to_wait.to_ms() / 2);
    }
  }
}
