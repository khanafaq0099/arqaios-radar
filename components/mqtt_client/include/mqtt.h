#ifndef MQTT_H_
#define MQTT_H_
 
#define MQTT_TOPIC "/topic/radar/data"
void Init_Mqtt();
void Publish(char *Data, char *topic);


#endif /* MQTT_H_ */