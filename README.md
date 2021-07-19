# noise-nodes project 
detect and visulize noisy spots (streets, parks)

### Nodes = ESP32 and INMS411 Mic Sensor -> send data to InfluxDB Server and visualized by Grafana incl. MAP Plugin
Measurements in decibels (dB) are sent to influxdb over wifi. Goal should be a LoRa mesh network.

### Pin Diagramm
![Fritzing](https://github.com/pan0ne/noise-nodes/blob/main/fritzing.png)

### Server 
Based on IOTStack Docker Script and contains Grafana, Telegraf, Mosquito and InfluxDB.
Check: https://sensorsiot.github.io/IOTstack/

### Output Example in Grafana
![Grafana](https://github.com/pan0ne/noise-nodes/blob/main/NoiseNode_GrafanaOutputExample.png)
