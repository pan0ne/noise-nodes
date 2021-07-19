/*   Sketch: ESP32's read i2c MEMS Sensor from
 *   its I2C Bus, send the data over wifi directly into InfluxDB
 *   
 *   BOARD: DOIT ESP32 DEVKIT V1
 *   USED BOARD MANAGER URL: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
 *   ADD LIBRARIES: Unzip github repos (see links below) to /Arduino/Libraries/ 
 *   ADD TABS: credentials.h and sos-iir-filter.h (Drag&Drop files from folder to Arduino IDE)
 *   CONFIG: change credentials.h with wifi, influxdb, node ares and names
 *   
 *   PIN CONNECTION ESP32 to INMP441 Mic Sensor
 *   3.3V - VDD
 *   GND - GND
 *   PIN 19 - SD
 *   PIN 18 - WS
 *   PIN23 - SCK
 *   
 *   Author: Pan0ne 
 *   Repository: https://github.com/pan0ne/noise-nodes 
 *   Thanks to: https://github.com/shantanoo-desai and https://github.com/ikostoski/esp32-i2s-slm
*/
#include <Arduino.h>
#include <Wire.h>                     // https://github.com/espressif/arduino-esp32
#include <SPI.h>                      // https://github.com/espressif/arduino-esp32
#include <driver/i2s.h>               // https://github.com/espressif/esp-idf
#include "sos-iir-filter.h"           // https://github.com/ikostoski/esp32-i2s-slm
#include <WiFi.h>                     // https://github.com/espressif/arduino-esp32
#include <PubSubClient.h>             // https://github.com/knolleary/pubsubclient
#include "credentials.h"              // drag and drop credentials.h as new tab
#include <InfluxDbClient.h>

WiFiClient espClient; 

// InfluxDB client instance
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_DB_NAME);

// Data point
Point sensor(SENSOR_ID);

// Calculate reference amplitude value at compile time
constexpr double MIC_REF_AMPL = pow(10, double(MIC_SENSITIVITY)/20) * ((1<<(MIC_BITS-1))-1);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("NoiseNode - Decibel Meter Mesh System");
  Serial.println("Device ID: " DEVICE);
  Serial.println("Node ID: "  NODE_ID);
  Serial.println("Geohash: "  GEOHASH);

  Serial.println("Found Sensor, Setting up");
  delay(1000);
  Serial.println("Connecting to WLAN Access-Point");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
 }
 Serial.println("main:setup: WiFi Connected");
 Serial.println("main:setup: IP Address: ");
 Serial.println(WiFi.localIP());

 // Set InfluxDB 1 authentication params
 client.setConnectionParamsV1(INFLUXDB_URL, INFLUXDB_DB_NAME, INFLUXDB_USER, INFLUXDB_PASSWORD);

 // Add constant tags - only once
 sensor.addTag("device", DEVICE);
 sensor.addTag("geohash", GEOHASH);

  // Check server connection
  if (client.validateConnection()) {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }

 // Create FreeRTOS queue
  samples_queue = xQueueCreate(8, sizeof(sum_queue_t));
  
  // Create the I2S reader FreeRTOS task
  // NOTE: Current version of ESP-IDF will pin the task 
  //       automatically to the first core it happens to run on
  //       (due to using the hardware FPU instructions).
  //       For manual control see: xTaskCreatePinnedToCore
  xTaskCreate(mic_i2s_reader_task, "Mic I2S Reader", I2S_TASK_STACK, NULL, I2S_TASK_PRI, NULL);

  sum_queue_t q;
  uint32_t  Leq_samples = 0;
  double    Leq_sum_sqr = 0;
  double    Leq_dB = 0;

  // Read sum of samaples, calculated by 'i2s_reader_task'
  while (xQueueReceive(samples_queue, &q, portMAX_DELAY)) {

    // Calculate dB values relative to MIC_REF_AMPL and adjust for microphone reference
    double short_RMS = sqrt(double(q.sum_sqr_SPL) / SAMPLES_SHORT);
    double short_SPL_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(short_RMS / MIC_REF_AMPL);

    // In case of acoustic overload or below noise floor measurement, report infinty Leq value
    if (short_SPL_dB > MIC_OVERLOAD_DB) {
      Leq_sum_sqr = INFINITY;
    } else if (isnan(short_SPL_dB) || (short_SPL_dB < MIC_NOISE_DB)) {
      Leq_sum_sqr = -INFINITY;
    }

    // Accumulate Leq sum
    Leq_sum_sqr += q.sum_sqr_weighted;
    Leq_samples += SAMPLES_SHORT;

    // When we gather enough samples, calculate new Leq value
    if (Leq_samples >= SAMPLE_RATE * LEQ_PERIOD) {
      double Leq_RMS = sqrt(Leq_sum_sqr / Leq_samples);
      Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(Leq_RMS / MIC_REF_AMPL);
      Leq_sum_sqr = 0;
      Leq_samples = 0;
      
      // Serial.printf("Decible: %.1f\n", Leq_dB);
      sendToInfluxDB(Leq_dB);

      // Debug only
     // Serial.printf("%u processing ticks\n", q.proc_ticks);
    }
  }
  
}

void sendToInfluxDB(float valueInDecibels)
{
  double Leq_dB = valueInDecibels;
 // Store measured value into point
      sensor.clearFields();
      sensor.addField("location", NODE_ID);
      sensor.addField("dB", Leq_dB);
      //sensor.addField("uptime", millis());
    
      // Print what are we exactly writing or coment
      // Serial.print("Writing: ");
      // Serial.println(client.pointToLineProtocol(sensor));
      // If no Wifi signal, try to reconnect it
      
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Wifi connection lost");
      }
      // Write point
      if (!client.writePoint(sensor)) {
        Serial.print("InfluxDB write failed: ");
        Serial.println(client.getLastErrorMessage());
      }
  delay(2000);
}

void loop() {

}
