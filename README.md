# noise-nodes
Project to detect and visulize noisy spots (streets, parks)

### Nodes = ESP32 and IMNS411 Mic Sensor -> send data to InfluxDB Server and visulized by Grafana incl. MAP Plugin
Measurements are sent to the influxdb database over wifi. But the goal should be a LoRa mesh network

### Server 
Based on IOTStack Docker Script and contains Grafana, Telegraf, Mosquito and InfluxDB.
Check: https://sensorsiot.github.io/IOTstack/

### Output Example in Grafana
![Grafana](https://github.com/pan0ne/noise-nodes/blob/main/NoiseNode_GrafanaOutputExample.png)

### Pin Diagramm
![Fritzing](https://github.com/pan0ne/noise-nodes/blob/main/fritzing.png)
