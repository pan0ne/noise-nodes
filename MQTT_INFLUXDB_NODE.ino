/*   Arduino Sketch to Program an ESP32 to read i2c MEMS Sensor from
 *   its I2C Bus, convert the data into InfluxDB's Line Protocol Strings
 *   and Publish it to an MQTT broker running on an IoTStack (Pi4)
 *   Author: Pan0ne https://github.com/pan0ne/noise-nodes 
 *   Thanks to <https://github.com/shantanoo-desai> and https://github.com/ikostoski/esp32-i2s-slm
*/
#include <Arduino.h>
#include <Wire.h>                     // https://github.com/espressif/arduino-esp32
#include <SPI.h>                      // https://github.com/espressif/arduino-esp32
#include <driver/i2s.h>               // https://github.com/espressif/esp-idf
#include "sos-iir-filter.h"           // https://github.com/ikostoski/esp32-i2s-slm
#include <WiFi.h>                     // https://github.com/espressif/arduino-esp32
#include <PubSubClient.h>             // https://github.com/knolleary/pubsubclient
#include "credentials.h"              // Drag and drop credentials.h as new tab

WiFiClient espClient; 
PubSubClient client(espClient);

// Calculate reference amplitude value at compile time
constexpr double MIC_REF_AMPL = pow(10, double(MIC_SENSITIVITY)/20) * ((1<<(MIC_BITS-1))-1);

/*Reconnect to MQTT Broker*/
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("MQTT:reconnect: Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(client_id)) {
      Serial.println("MQTT:reconnect: Connected");
    } else {
      Serial.print("MQTT:reconnect: Failed, rc=");
      Serial.print(client.state());
      Serial.println("MQTT:reconnect: Trying Again in 2 seconds");
      // Wait 2 seconds before retrying
      delay(2000);
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("LSM MQTT Demo");
  /*
  if (!lsm.begin()) {
    Serial.println("Sensor Not Initialized");
    while(1);
  }
  */
  Serial.println("Found Sensor, Setting up");
  delay(1000);
  Serial.println("Connecting to WLAN Access-Point");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
 }
 Serial.println("main:setup: WiFi Connected");
 Serial.println("main:setup: IP Address: ");
 Serial.println(WiFi.localIP());
 Serial.println("main:setup: Setting up MQTT Configuration");
 client.setServer(mqtt_broker_address, mqtt_port);

 // Create FreeRTOS queue
  samples_queue = xQueueCreate(8, sizeof(sum_queue_t));
  
  // Create the I2S reader FreeRTOS task
  // NOTE: Current version of ESP-IDF will pin the task 
  //       automatically to the first core it happens to run on
  //       (due to using the hardware FPU instructions).
  //       For manual control see: xTaskCreatePinnedToCore
  xTaskCreate(mic_i2s_reader_task, "Mic I2S Reader", I2S_TASK_STACK, NULL, I2S_TASK_PRI, NULL);

  sum_queue_t q;
  uint32_t Leq_samples = 0;
  double Leq_sum_sqr = 0;
  double Leq_dB = 0;

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
      
      // Serial output, customize (or remove) as needed
      Serial.printf("Decible: %.1f\n", Leq_dB);
      sendToInfluxDB(Leq_dB);
      delay(1000);

      // Debug only
     // Serial.printf("%u processing ticks\n", q.proc_ticks);
    }
  }
}

void sendToInfluxDB(float valueInDecibels)
{
  if (!client.connected()){
    reconnect();
  }
  client.loop();
   // read temperature
  dtostrf(valueInDecibels, 3, 1, valTemperature);
  String payload;
  payload = "sensor2 db=";
  payload += valTemperature;
  // publish temperature
  client.publish("db", (char*) payload.c_str());
  Serial.println("Send Data");
  delay(1000);
}

void loop() {

}
