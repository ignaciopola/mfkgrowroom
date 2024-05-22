#include <ESP8266WiFi.h>
#include <PubSubClient.h>


const char* ssid = "";
const char* password = "";
const char* mqtt_server = "";
const int mqtt_port = 1883;
const char* mqtt_user = "";
const char* mqtt_password = "";

WiFiClient espClient;
PubSubClient client(espClient);

void jsonizer(String mensaje, String emisor, String tipo) 
{
  String json="{\"tx\": \""+emisor+"\", \"ty\": \""+tipo+"\", \"msj\": \"" + mensaje + "\"}";
  Serial.println(json);
  delay(100);
}

void conexion_wifi() 
{
  String json_conexion;
  String conexion;
  delay(100);
  
  conexion = "Conectando a ";
  conexion += ssid;
  jsonizer(conexion, "d1","wifi");
  IPAddress ip(192, 168, 0, 200);
  IPAddress gateway(192, 168, 0, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.config(ip, gateway, subnet);
  WiFi.begin(ssid, password);
  if (WiFi.status() == WL_CONNECTED) 
  { jsonizer("Conectado","d1","WIFI"); 
  } else
  {
    switch (WiFi.status()) {
        case WL_NO_SSID_AVAIL:
            jsonizer("Red no disponible","d1","WIFI");
            break;
        case WL_CONNECT_FAILED:
            jsonizer("Falla conexión","d1","WIFI");
            break;
        case WL_CONNECTION_LOST:
            jsonizer("Conexión perdida","d1","WIFI");
            break;
        case WL_DISCONNECTED:
            jsonizer("Desconectado","d1","WIFI");
            break;
        default:
            conexion=WiFi.status();
            jsonizer(conexion,"d1","WIFI");
            break;
    }
  } 
  
}
void conexion_mqtt() 
{
  String conexion_mqtt;
  if (client.connected())
  {
    jsonizer("Conectado","d1","MQTT");
  } else 
  {
    while (!client.connected()) {
      if (client.connect("esp8266-client", mqtt_user, mqtt_password)) {
        jsonizer("Conectado","d1","MQTT");
        client.subscribe("arduino");
      } else 
      {
        conexion_mqtt="Falla MQTT, rc="+client.state();
        jsonizer(conexion_mqtt, "d1","MQTT");
        delay(5000);
      }
    }
  }
}

void recepcion_mqtt(char* topic, byte* payload, unsigned int length) {
  String recepcion;
  for (int i = 0; i < length; i++) {
    recepcion=recepcion+(char)payload[i];
  }
  Serial.println(recepcion);
}



void setup() {
  Serial.begin(115200);
  conexion_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(recepcion_mqtt);
}

void loop() {
  if (!client.connected()) {
    conexion_mqtt();
  } 
  
  if (WiFi.status() != WL_CONNECTED) {
    conexion_wifi();
  }
  
  client.loop(); //Función esencial. Verifica si existen nuevos msj --> llama a la recepción.

  if (Serial.available() > 0) 
  {
    // Lee los datos recibidos del Arduino Mega a través de la comunicación serial
    String mensaje = Serial.readStringUntil('\n');
    
    // Publica el mensaje en el servidor MQTT
    client.publish("arduino", mensaje.c_str());
  }
}
