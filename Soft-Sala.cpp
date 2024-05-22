#include <Arduino.h>

#include <EEPROM.h>
#include "RTClib.h"
#include "U8glib.h"
#include <Pushbutton.h>
#include <avr/wdt.h>
#include <ArduinoJson.h>

// INFRARROJO
#define DISABLE_CODE_FOR_RECEIVER
#define EXCLUDE_EXOTIC_PROTOCOLS
#define IR_SEND_PIN 11
#define NO_LED_FEEDBACK_CODE
#define NO_DECODER

#include "Pines.h" //Configuración y definición de pines
#include <IRremote.hpp>

// DECLARO FUNCIONES EN CPP
void InterrupcionBoton1();
void InterrupcionBoton2();
void InterrupcionBoton3();
void RevisoConectividad(JsonDocument &msj);
void EstadoFotoperiodo(DateTime date);
void LeoDispositivos();
void EstadoFotoperiodo();
void GestorCo2(DateTime date);
void Criticos(DateTime date);
void EstadoHumidificador();
void printDate(DateTime date);
void EnvioSensores();
void AnalizoAccion(JsonDocument &msj);
void Mqtt_Config_Dispositivos(JsonDocument &msj);
void Mqtt_Fotoperiodo(JsonDocument &msj);
void Mqtt_Humedad(JsonDocument &msj);
void Mqtt_Temperatura(JsonDocument &msj);
void Mqtt_Riego(JsonDocument &msj);
void Mqtt_Aire(JsonDocument &msj);
void Mqtt_Hora_Update(JsonDocument &msj);
void Guardovalores(String evento, byte menu_int);
void Agregar_Buffer(String msj_serial);
void Procesar_Buffer_Serial();
void Aire_26();
void Aire_Display();
void Aire_Deshu();
void Aire_Off();
void Gestor_Aire(int orden);
void Gestor_Riego();
void SensoresTyH();

int estado_aire = 0;

// BUFFER SERIAL
long ultimo_serial = 0;
int index_buffer = 0;
String buffer_serial[15];

// Sensor CO2

#include "MHZ19.h"
MHZ19 myMHZ19; // Constructor for library

// Co2
int CO2;
int Co2Max;
int Co2Min;
#define Co2Emergencia 2500
String Co2Desactivado;

// Para destildar el sensor
long Co2Muerto0 = 0;
long TiempoCo2Muerto;
bool EstadoSerial3 = true;

// Riego
int pulso_riego;
DateTime ultimo_riego;
long tiempo_transcurrido_riego;
bool estado_riego = false;
const int riego_minimo = 20;
const int riego_maximo = 1500;

// Humedad Deseada
int HMin;
int HMax;

// Temperatura Deseada
int TMin;
int TMax;

#define PreNoche 10 // Minutos
long UltimoEnvio = 0;

// CONECTIVIDAD
String estado_wifi;
String estado_mqtt;

// COMUNICACIÓN CON D1 SERIAL2
String msj_recibido;

// Sensores de temperatura y humedad
#include <DHT.h>
#define DHTPIN1 30
#define DHTPIN2 31
#define DHTPIN3 32
#define DHTTYPE DHT22
DHT dht1(DHTPIN1, DHTTYPE);
DHT dht2(DHTPIN2, DHTTYPE);
DHT dht3(DHTPIN3, DHTTYPE);

// Variables de temp y humedad
byte t01;
byte h01;
byte t02;
byte h02;
byte t03;
byte h03;
byte Tmhz;
DateTime UltimaLecturaSensores;
DateTime InicioPrograma;
DateTime UltimoReinicio;

// Puntos Criticos
byte tmaxhoy;
byte tmaxhoyhora;
byte tminhoyhora;
byte tmaxhoymin;
byte tminhoymin;
byte tminhoy;

byte hminhoyhora;
byte hmaxhoyhora;
byte hminhoymin;
byte hmaxhoymin;
byte hminhoy;
byte hmaxhoy;

// Pins relays
// #define pinfotoperiodo 40 --> 0 en el array
// #define pinextractor 41  --> 1 en el array
// #define pinco2 42        --> 2 en el array
// #define pinhumidificador 43> 3 en el array
// #define pinriego 44      --> 4 en el array
// #define intractor 44      --> 4 en el array
byte Pines[6] = {40, 41, 42, 43, 44, 45};

// Relays
byte RPines[6] = {};
String ERPines[6];
bool BoolPines[6];

// Fotoperiodo
byte fotoperiodo1a;
byte fotoperiodo1b;

// Reloj
RTC_DS3231 rtc;

// CONSTRUCTOR PANTALLA OLED
U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE);

// Creo los botones
// Pushbutton boton2(19); //Cambia de numero
// Pushbutton boton3(20); //Entra a edicion

// Botones con interrupciones.
const int timeThreshold = 300;
long timeCounter = 0;
long timeCounter2 = 0;
long timeCounter3 = 0;
byte menu = 1;

// Variables de edición
byte edicion2etapa = 0;
bool edicion2 = false;

byte edicion4etapa = 0;
bool edicion4 = false;

byte edicion3etapa = 0;
bool edicion3 = false;

byte edicion5etapa = 0;
bool edicion5 = false;

byte edicion7etapa = 0;
bool edicion7 = false;

byte edicion8etapa = 0;
bool edicion8 = false;

byte Navego3;

// Variables FOR
byte i = 0;
byte x = 0;
byte y = 0;
byte r = 0;

void setup()
{
    wdt_disable(); // No te olvides

    Serial.begin(115200);
    Serial2.begin(115200); // Serial Hardware para el D1-MINI WIFI
    Serial3.begin(9600);   // Serial Hardware para el sensor de CO2 MHZ19

    Serial.println("Iniciando Arduino");

    attachInterrupt(digitalPinToInterrupt(18), InterrupcionBoton1, FALLING);
    attachInterrupt(digitalPinToInterrupt(19), InterrupcionBoton2, FALLING);
    attachInterrupt(digitalPinToInterrupt(2), InterrupcionBoton3, FALLING);
    myMHZ19.begin(Serial3);        // *Serial(Stream) refence must be passed to library begin().
    myMHZ19.autoCalibration(true); // Turn auto calibration ON (OFF autoCalibration(false))
    myMHZ19.setFilter(true, true);

    // Sensores humedad y temp
    dht1.begin();
    dht2.begin();
    dht3.begin();

    // Declaro pins relays
    for (i = 0; i < 6; i++)
    {
        pinMode(Pines[i], OUTPUT);
        RPines[i] = EEPROM.read(Pines[i]);
        digitalWrite(Pines[i], HIGH);
    }

    u8g.setColorIndex(1); // Instructs the display to draw with a pixel on.
    if (!rtc.begin())
    {
        // Serial.println("Couldn't find RTC");
        while (1)
            ;
    }

    // Leo los valores de los timers guardados en la memoria
    fotoperiodo1a = EEPROM.read(0);
    fotoperiodo1b = EEPROM.read(1);

    // Leo los valores max y min del co2;
    Co2Max = EEPROM.read(5);
    Co2Min = EEPROM.read(6);
    if (Co2Max < 400)
    {
        Co2Max = 1000;
    }
    if (Co2Min < 400)
    {
        Co2Min = 400;
    }

    // Leo los valroes max y min de Humedad
    HMax = EEPROM.read(7);
    HMin = EEPROM.read(8);
    if (HMax < 1 or HMax > 90)
    {
        HMax = 70;
    } // Parece que descarto lecturas aberrantes y lo pongo en valores estandar para que luego sean sobreescritos por lecturas reales.
    if (HMin < 1 or HMin > 90)
    {
        HMin = 40;
    }

    pulso_riego = EEPROM.read(9);
    if (pulso_riego == 0)
    {
        pulso_riego = riego_minimo;
    }

    Guardovalores("inicio", 0);

    u8g.setFont(u8g_font_6x13r);

    // Configuración del pin 22 de reinicio sensores
    InicioPrograma = rtc.now();
    UltimoReinicio = InicioPrograma;
    pinMode(24, OUTPUT);
    digitalWrite(24, HIGH);

    wdt_enable(WDTO_8S);
}

void loop()
{
    Procesar_Buffer_Serial();

    if (Serial2.available())
    {
        msj_recibido = Serial2.readStringUntil('\n');

        StaticJsonDocument<512> doc; // Ajusta este tamaño según el tamaño máximo esperado de tus mensajes JSON
        DeserializationError error = deserializeJson(doc, msj_recibido);
        if (!error)
        {
            AnalizoAccion(doc);
        }
        else
        {
            // Si hubo error en la deserialización, opcionalmente puedes manejarlo aquí
            Serial.print("Error deserializando el mensaje JSON: ");
            Serial.println(error.c_str());
        }
    }

    // Establezco Hora
    DateTime now = rtc.now();

    SensoresTyH(); // Recojo los valores de temp y humedad
    EnvioSensores();
    Gestor_Riego();
    EstadoFotoperiodo(now); // Compruebo valor del Timer
    GestorCo2(now);
    Criticos(now);
    EstadoHumidificador();
    LeoDispositivos(); // Leo los dispositivos
    // Grafico(now);

    // ############## Inicio "picture loop" #######
    u8g.firstPage();
    do
    {
        printDate(now);
    } while (u8g.nextPage());
    // ############## Fin "picture loop" ##########

    wdt_reset();
}

void printDate(DateTime date)
{
    // MENU 0 --> PANTALLA APAGADA (inactividad de 1 minuto)
    long TiempoSinTecla;
    TiempoSinTecla = (millis() - timeCounter) / 1000;

    // DESCOMENTAR PARA VOLVER A APAGAR LA PANTALLA POR INACTIVIDAD.
    // if (TiempoSinTecla>60) { menu=0; }

    // MENU 1 --> Críticos: máximos y mínimos de Tº y H%

    if (menu == 1)
    {

        u8g.setPrintPos(0, 10);
        u8g.print(date.day());
        u8g.drawStr(13, 10, "/");
        u8g.setPrintPos(19, 10);
        u8g.print(date.month());
        u8g.drawStr(40, 10, "Sonda");

        u8g.setPrintPos(90, 10);
        u8g.print(date.hour());
        u8g.drawStr(100, 10, ":");
        u8g.setPrintPos(105, 10);
        u8g.print(date.minute());

        u8g.drawLine(0, 10, 128, 10); // linea separacion

        u8g.drawStr(0, 25, "Tmin:");
        u8g.setPrintPos(30, 25);
        u8g.print(tminhoy);
        u8g.drawStr(43, 25, "C (");
        u8g.setPrintPos(59, 25);
        u8g.print(tminhoyhora);
        u8g.drawStr(71, 25, ":");
        u8g.setPrintPos(77, 25);
        u8g.print(tminhoymin);
        u8g.drawStr(90, 25, "hs)");

        u8g.drawStr(0, 35, "Tmax:");
        u8g.setPrintPos(30, 35);
        u8g.print(tmaxhoy);
        u8g.drawStr(43, 35, "C (");
        u8g.setPrintPos(59, 35);
        u8g.print(tmaxhoyhora);
        u8g.drawStr(71, 35, ":");
        u8g.setPrintPos(77, 35);
        u8g.print(tmaxhoymin);
        u8g.drawStr(90, 35, "hs)");

        u8g.drawStr(0, 45, "Hmin:");
        u8g.setPrintPos(30, 45);
        u8g.print(hminhoy);
        u8g.drawStr(43, 45, "% (");
        u8g.setPrintPos(59, 45);
        u8g.print(hminhoyhora);
        u8g.drawStr(71, 45, ":");
        u8g.setPrintPos(77, 45);
        u8g.print(hminhoymin);
        u8g.drawStr(90, 45, "hs)");

        u8g.drawStr(0, 55, "Hmax:");
        u8g.setPrintPos(30, 55);
        u8g.print(hmaxhoy);
        u8g.drawStr(43, 55, "% (");
        u8g.setPrintPos(59, 55);
        u8g.print(hmaxhoyhora);
        u8g.drawStr(71, 55, ":");
        u8g.setPrintPos(77, 55);
        u8g.print(hmaxhoymin);
        u8g.drawStr(90, 55, "hs)");

    } // Fin Menu 1.

    // MENU 2 Temperatura y Humedad
    if (menu == 2)
    {

        String hx1 = String(h01) + "%-" + String(t01) + "C-" + String(Tmhz) + "C";
        String hx2 = String(h02) + "%-" + String(t02) + "C";
        String hx3 = String(h03) + "%-" + String(t03) + "C";

        u8g.drawStr(30, 9, "Valores Tº y H%");
        u8g.drawLine(0, 10, 128, 10); // linea separacion
        u8g.drawStr(0, 23, "S1: ");
        u8g.drawStr(0, 33, "S2: ");
        u8g.drawStr(0, 43, "S3: ");
        u8g.setPrintPos(40, 23);
        u8g.print(hx1);
        u8g.setPrintPos(40, 33);
        u8g.print(hx2);
        u8g.setPrintPos(40, 43);
        u8g.print(hx3);

        if (RPines[3] == 2)
        {
            u8g.drawStr(0, 38, "Maximo: ");
            u8g.setPrintPos(70, 38);
            u8g.print(HMax);
            u8g.drawStr(85, 38, "%");
            u8g.drawStr(0, 48, "Minimo: "); // AGREGAR % AL FINAL
            u8g.setPrintPos(70, 48);
            u8g.print(HMin);
            u8g.drawStr(85, 48, "%");

            if (BoolPines[3] == true)
            {
                u8g.drawStr(40, 63, "Deshu On");
            }
            else
            {
                u8g.drawStr(40, 63, "Deshu Off");
            }

            if (edicion2 == true)
            {
                if (edicion2etapa == 1)
                {
                    u8g.setCursorFont(u8g_font_cursor);
                    u8g.setCursorStyle(142);
                    u8g.enableCursor(); // Cursor de edicion
                    u8g.setCursorPos(100, 35);
                    u8g.drawLine(70, 39, 100, 39); // linea bajo el 1º nº
                }
                if (edicion2etapa == 2)
                {
                    u8g.setCursorFont(u8g_font_cursor);
                    u8g.setCursorStyle(142);
                    u8g.enableCursor(); // Cursor de edicion
                    u8g.setCursorPos(100, 40);
                    u8g.drawLine(70, 49, 100, 49); // linea bajo el 2º nº
                }
            }
        }
    }

    // MENU 3 DISPOSITIVOS
    if (menu == 3)
    {

        u8g.drawStr(0, 10, "Fotoperiodo");
        u8g.drawStr(0, 20, "Extractor: ");
        u8g.drawStr(0, 30, "Co2: ");
        u8g.drawStr(0, 40, "Humidificador: ");
        u8g.drawStr(0, 50, "Riego: ");
        u8g.drawStr(0, 60, "Intractor: ");

        for (i = 0; i < 6; i++)
        {
            x = 10 + (i * 10);
            u8g.setPrintPos(90, x);
            u8g.print(ERPines[i]);
        }

        // Edicion Menu 3 = Dispositivos
        if (edicion3 == true)
        {
            Navego3 = (edicion3etapa - 1) * 10; // ubicacion y de las lineas
            Navego3 = Navego3 + 10;
            u8g.drawLine(90, Navego3, 120, Navego3); // linea de edicion
        } // Fin edición 3

    } // Fin menu 3

    // MENU 4 FOTOPERIODO
    if (menu == 4)
    {
        // Dibujo el menu

        u8g.drawStr(30, 10, "Fotoperiodo");
        u8g.drawLine(0, 12, 127, 12); // linea separacion

        // T1

        u8g.drawStr(0, 25, "Hora ");
        u8g.setPrintPos(40, 25);

        u8g.print(date.hour());
        u8g.drawStr(53, 25, ":");
        u8g.setPrintPos(60, 25);
        u8g.print(date.minute());

        u8g.drawStr(0, 40, "Timer ");
        u8g.setPrintPos(40, 40);
        u8g.print(fotoperiodo1a);
        u8g.drawStr(55, 40, "a");
        u8g.setPrintPos(65, 40);
        u8g.print(fotoperiodo1b);
        u8g.drawStr(78, 40, "hs");

        if (BoolPines[0] == true)
        {
            u8g.drawStr(33, 53, "Dia");
        }
        else
        {
            u8g.drawStr(33, 53, "Noche");
        }

        // Inicio la edición del fotoperiodo
        if (edicion4etapa == 1)
        {
            u8g.setCursorFont(u8g_font_cursor);
            u8g.setCursorStyle(142);
            u8g.enableCursor(); // Cursor de edicion
            u8g.setCursorPos(100, 35);
            u8g.drawLine(40, 41, 50, 41); // linea bajo el 1º nº
        }
        if (edicion4etapa == 2)
        {
            u8g.drawLine(65, 41, 75, 41); // linea bajo el 2º nº
        }

    } // FIN MENU 4

    // MENU 5 - CO2
    if (menu == 5)
    {
        u8g.drawStr(20, 10, "Control de CO2");
        u8g.drawLine(0, 12, 127, 12); // linea separacion

        if (CO2 != 0)
        {
            u8g.drawStr(0, 25, "Nivel CO2: ");
            u8g.setPrintPos(70, 25);
            u8g.print(CO2);
        }
        else
        {
            if (millis() < 120000)
            {
                u8g.drawStr(0, 25, "Calentando Sensor... ");
            }
            else
            {
                u8g.drawStr(0, 25, "Tildado: ");
                u8g.setPrintPos(80, 25);
                //          u8g.print(CO2);
                //          Serial3.end();
                //          delay(1000);
                //          Serial3.begin(9600);
                //          Serial.println("Se trato de reiniciar el sensor");
            }
        }

        u8g.drawStr(0, 35, "Maximo: ");
        u8g.setPrintPos(70, 35);
        u8g.print(Co2Max);
        u8g.drawStr(0, 45, "Minimo: ");
        u8g.setPrintPos(70, 45);
        u8g.print(Co2Min);

        if (Co2Desactivado != "")
        {
            u8g.setPrintPos(0, 55);
            u8g.print(Co2Desactivado);
        }

        if (edicion5 == true)
        {
            if (edicion5etapa == 1)
            {
                u8g.setCursorFont(u8g_font_cursor);
                u8g.setCursorStyle(142);
                u8g.enableCursor(); // Cursor de edicion
                u8g.setCursorPos(100, 30);
                u8g.drawLine(70, 36, 100, 36); // linea bajo el 1º nº
            }
            if (edicion5etapa == 2)
            {
                u8g.setCursorFont(u8g_font_cursor);
                u8g.setCursorStyle(142);
                u8g.enableCursor(); // Cursor de edicion
                u8g.setCursorPos(100, 40);
                u8g.drawLine(70, 46, 100, 46); // linea bajo el 2º nº
            }
        }

    } // Fin Menu 5 - Gestor CO2

    // Menu 6: INFO CONECTIVIDAD
    if (menu == 6)
    {
        u8g.drawStr(20, 10, "Conectividad");
        u8g.drawLine(0, 12, 127, 12); // linea separacion
        u8g.drawStr(0, 25, "Wi-Fi:");
        u8g.setPrintPos(30, 25);
        u8g.print(estado_wifi);

        u8g.drawStr(0, 40, "MQTT:");
        u8g.setPrintPos(30, 40);
        u8g.print(estado_mqtt);
    }

    if (menu == 7)

    {
        u8g.drawStr(20, 10, "Riego");
        u8g.drawLine(0, 12, 127, 12); // linea separacion
        u8g.drawStr(0, 25, "Pulso:");
        u8g.setPrintPos(50, 25);
        u8g.print(pulso_riego);

        u8g.drawStr(0, 40, "Estado Riego: ");
        u8g.setPrintPos(30, 40);
        if (estado_riego == true)
        {
            u8g.drawStr(90, 40, "ON");
        }
        else
        {
            u8g.drawStr(90, 40, "OFF");
        }

        if (estado_riego == true)
        {
            u8g.drawStr(0, 55, "Timer: ");
            u8g.setPrintPos(40, 55);
            u8g.print(tiempo_transcurrido_riego);
            u8g.drawStr(60, 55, "/");
            u8g.setPrintPos(67, 55);
            u8g.print(pulso_riego);
        }

        if (edicion7 == true)
        {
            if (edicion7etapa == 1)
            {
                u8g.setCursorFont(u8g_font_cursor);
                u8g.setCursorStyle(142);
                u8g.enableCursor(); // Cursor de edicion
                u8g.setCursorPos(100, 19);
                u8g.drawLine(50, 25, 70, 25); // linea bajo el pulso riego
            }
            if (edicion7etapa == 2)
            {
                u8g.setCursorFont(u8g_font_cursor);
                u8g.setCursorStyle(142);
                u8g.enableCursor(); // Cursor de edicion
                u8g.setCursorPos(110, 35);
                u8g.drawLine(86, 43, 111, 43); // linea el ON - OFF
            }
        }
    }

    if (menu == 8)
    {
        u8g.drawStr(10, 10, "Aire Acondicionado");
        u8g.drawLine(0, 12, 127, 12); // linea separacion

        u8g.drawStr(0, 25, "Prender a 26Cº");
        u8g.drawStr(0, 35, "Prender Deshu");
        u8g.drawStr(0, 45, "Apagar");

        if (estado_aire == 0)
        {
            u8g.drawStr(60, 45, "<");
        }
        if (estado_aire == 1)
        {
            u8g.drawStr(90, 25, "<");
        }
        if (estado_aire == 2)
        {
            u8g.drawStr(90, 35, "<");
        }

        if (edicion8etapa == 1)
        {
            u8g.setCursorFont(u8g_font_cursor);
            u8g.setCursorStyle(142);
            u8g.enableCursor(); // Cursor de edicion
            u8g.setCursorPos(100, 19);
            u8g.drawLine(0, 25, 95, 25); // linea bajo prender a 26
        }
        if (edicion8etapa == 2)
        {
            u8g.setCursorFont(u8g_font_cursor);
            u8g.setCursorStyle(142);
            u8g.enableCursor(); // Cursor de edicion
            u8g.setCursorPos(100, 29);
            u8g.drawLine(0, 35, 95, 35); // linea bajo prendender en deshu
        }
        if (edicion8etapa == 3)
        {
            u8g.setCursorFont(u8g_font_cursor);
            u8g.setCursorStyle(142);
            u8g.enableCursor(); // Cursor de edicion
            u8g.setCursorPos(100, 39);
            u8g.drawLine(0, 45, 40, 45); // linea bajo Apagar
        }
    }
} // Fin loop grafico

void Guardovalores(String evento, byte menu_int)
{

    edicion2 = false;
    edicion3 = false;
    edicion4 = false;
    edicion5 = false;
    edicion7 = false;
    edicion8 = false;
    edicion2etapa = 0;
    edicion3etapa = 0;
    edicion4etapa = 0;
    edicion5etapa = 0;
    edicion7etapa = 0;
    edicion8etapa = 0;
    u8g.disableCursor();
    EEPROM.write(0, fotoperiodo1a);
    EEPROM.write(1, fotoperiodo1b);
    EEPROM.put(5, Co2Max);
    EEPROM.put(6, Co2Min);
    EEPROM.write(7, HMax);
    EEPROM.write(8, HMin);
    EEPROM.write(9, pulso_riego);
    EEPROM.write(10, estado_aire);

    for (i = 0; i < 6; i++)
    {
        EEPROM.write(Pines[i], RPines[i]);
    }

    String json;
    if ((evento == "cfg_disp") || (menu_int == 3) || (evento == "inicio"))
    {
        // Construir el JSON DE CONFIGURACIÓN DE DISPOSITIVOS (OFF, ON, AUTO)
        String json_disp = "\"" + String(RPines[0]) + "," + String(RPines[1]) + "," + String(RPines[2]) + "," + String(RPines[3]) + "," + String(RPines[4]) + "," + String(RPines[5]) + "\"";
        json = "{\"tx\":\"mega\",\"ty\":\"upd\",\"disp\":" + json_disp + "}";
        Agregar_Buffer(json);

        // Construir el JSON DE ESTADO DE DISPOSITIVOS (OFF, ON)
        json_disp = "\"" + String(BoolPines[0]) + "," + String(BoolPines[1]) + "," + String(BoolPines[2]) + "," + String(BoolPines[3]) + "," + String(BoolPines[4]) + "," + String(BoolPines[5]) + "\"";
        json = "{\"tx\":\"mega\",\"ty\":\"relays\",\"rys\":" + json_disp + "}";
        Agregar_Buffer(json);
    }

    if ((evento == "foto") || (menu_int == 4) || (evento == "inicio"))
    {
        // Construyo el JSON de fotoperiodo
        json = "{\"tx\":\"mega\", \"ty\":\"foto\", \"f1\":\"" + String(fotoperiodo1a) + "\", \"f2\":\"" + String(fotoperiodo1b) + "\"}";
        Agregar_Buffer(json);
    }
    if ((evento == "hum") || (menu_int == 2) || (evento == "inicio"))
    {
        // Construyo el JSON de humedad deseada
        json = "{\"tx\":\"mega\", \"ty\":\"hum\", \"h1\":\"" + String(HMin) + "\", \"h2\":\"" + String(HMax) + "\"}";
        Agregar_Buffer(json);
    }
    if ((evento == "temp") || (menu_int == 6) || (evento == "inicio"))
    {
        // Construyo el JSON de temperatura
        json = "{\"tx\":\"mega\", \"ty\":\"temp\", \"t1\":\"" + String(TMin) + "\", \"t2\":\"" + String(TMax) + "\"}";
        Agregar_Buffer(json);
    }

    if ((evento == "riego") || (menu_int == 7) || (evento == "inicio"))
    {
        // Construyo el JSON de Riego.  Envío duración del pulso y hora de inicio
        if (estado_riego)
        {
            json = "{\"tx\":\"mega\", \"ty\":\"riego_on\", \"p\":\"" + String(pulso_riego) + "\", \"hs\":\"" + String(ultimo_riego.hour()) + ":" + String(ultimo_riego.minute()) + ":" + String(ultimo_riego.second()) + "\"}";
        }
        else
        {
            json = "{\"tx\":\"mega\", \"ty\":\"pulso_riego\", \"p\":\"" + String(pulso_riego) + "\"}";
        }
        Agregar_Buffer(json);
    }
    if ((evento == "aire") || (menu_int == 8) || (evento == "inicio"))
    {
        // Construyo el JSON del aire, mando sólo la orden.
        json = "{\"tx\":\"mega\", \"ty\":\"aire\", \"orden\":\"" + String(estado_aire) + "\"}";
        Agregar_Buffer(json);
    }
}

void EstadoFotoperiodo(DateTime date)
{
    float hora = date.hour() + date.minute() / 60.0;
    // Serial.println(hora);

    // Compruebo timer1
    int diferencia = fotoperiodo1b - fotoperiodo1a;

    if (diferencia > 0)
    {
        if (hora > fotoperiodo1a and hora < fotoperiodo1b)
        {
            if (BoolPines[0] == false)
            {
                Guardovalores("cfg_disp", 0);
            }
            BoolPines[0] = true;
        }
        else
        {
            if (BoolPines[0] == true)
            {
                Guardovalores("cfg_disp", 0);
            }
            BoolPines[0] = false;
        }
    }
    else
    {
        if ((hora > fotoperiodo1a and hora < 24) or (hora >= 0 and hora < fotoperiodo1b))
        {
            if (BoolPines[0] == false)
            {
                Guardovalores("cfg_disp", 0);
            }
            BoolPines[0] = true;
        }
        else
        {
            if (BoolPines[0] == true)
            {
                Guardovalores("cfg_disp", 0);
            }
            BoolPines[0] = false;
        }
    }
}

void EstadoHumidificador()
{

    if (h01 > HMax)
    {
        BoolPines[3] = true;
    }

    if (h01 < HMin)
    {
        BoolPines[3] = false;
    }
}

void GestorCo2(DateTime date)
{

} // Fin GestorCo2

void LeoDispositivos()
{
    // Leo la configuración de los dispositivos y actúo en los relays en consecuencia.

    for (i = 0; i < 6; i++)
    {
        if (RPines[i] == 1)
        {
            ERPines[i] = "On";
            digitalWrite(Pines[i], LOW);
            BoolPines[i] = 1;
        }
        if (RPines[i] == 0)
        {
            ERPines[i] = "Off";
            digitalWrite(Pines[i], HIGH);
            BoolPines[i] = 0;
        }
        if (RPines[i] == 2)
        {
            ERPines[i] = "Auto";

            if (BoolPines[i] == true)
            {
                digitalWrite(Pines[i], LOW);
                BoolPines[i] = 1;
            }
            else
            {
                digitalWrite(Pines[i], HIGH);
                BoolPines[i] = 0;
            }
        }
    }
}

void Criticos(DateTime date)
{
    // SONDA 1
    if (date.hour() == 0 and date.minute() == 0)
    {
        hmaxhoy = h01;
        tmaxhoy = t01;
        tminhoy = t01;
        hminhoy = h01;
    }
    if (tmaxhoy == 0 or tmaxhoy < t01)
    {
        tmaxhoy = t01;
        tmaxhoyhora = date.hour();
        tmaxhoymin = date.minute();
    }

    if (tminhoy == 0 or t01 < tminhoy)
    {
        tminhoy = t01;
        tminhoyhora = date.hour();
        tminhoymin = date.minute();
    }

    if (hmaxhoy == 0 or hmaxhoy < h01)
    {
        hmaxhoy = h01;
        hmaxhoyhora = date.hour();
        hmaxhoymin = date.minute();
    }
    if (hminhoy == 0 or h01 < hminhoy)
    {
        hminhoy = h01;
        hminhoyhora = date.hour();
        hminhoymin = date.minute();
    }
}

void InterrupcionBoton1() // Navegación y salida de ediciones
{
    if (millis() > timeCounter + timeThreshold)
    {
        timeCounter = millis();

        if (edicion2 == false and edicion3 == false and edicion4 == false and edicion5 == false and edicion7 == false and edicion8 == false)
        {

            if (menu == 8)
            {
                menu = 0;
            }
            else
            {
                menu++;
            }
        }
        else
        {
            Guardovalores("fin", menu);
        }
    }
}

void InterrupcionBoton2() // Cambio de valores
{
    if (millis() > timeCounter + timeThreshold)
    {
        timeCounter = millis();

        // EDICION HUMEDAD
        if (edicion2 == true)
        {
            if (edicion2etapa == 1)
            {
                if (HMax < 90)
                {
                    HMax = HMax + 5;
                }
                else
                {
                    HMax = HMin;
                }
            }
            if (edicion2etapa == 2)
            {
                if (HMin < HMax)
                {
                    HMin = HMin + 5;
                }
                else
                {
                    HMin = 40;
                }
            }

            if (edicion2etapa == 3)
            {
                Guardovalores("hum", 0);
            }
        }

        // EDICION DISPOSITIVOS
        if (edicion3 == true) // Edicion dispositivos
        {
            i = edicion3etapa - 1;

            if (RPines[i] == 2)
            {
                RPines[i] = 0;
            }
            else
            {
                RPines[i]++;
            }

        } // Fin editando numero

        // EDICIÓN FOTOPERIODO
        if (edicion4etapa == 1)
        {
            if (fotoperiodo1a < 23)
            {
                fotoperiodo1a++;
            }
            else
            {
                fotoperiodo1a = 0;
            }
        }
        if (edicion4etapa == 2)
        {
            if (fotoperiodo1b < 23)
            {
                fotoperiodo1b++;
            }
            else
            {
                fotoperiodo1b = 0;
            }
        }
        if (edicion4etapa == 3)
        {
            Guardovalores("foto", 0);
        }

        // EDICIÓN CO2
        if (edicion5 == true)
        {
            if (edicion5etapa == 1)
            {
                if (Co2Max < Co2Emergencia)
                {
                    Co2Max = Co2Max + 20;
                }
                else
                {
                    Co2Max = Co2Min;
                }
            }
            if (edicion5etapa == 2)
            {
                if (Co2Min < Co2Max)
                {
                    Co2Min = Co2Min + 20;
                }
                else
                {
                    Co2Min = 400;
                }
            }

            if (edicion5etapa == 3)
            {
                Guardovalores("co2", 0);
            }
        }
        if (edicion7 == true)
        {
            if (edicion7etapa == 1)
            {
                if (pulso_riego < riego_maximo)
                {
                    pulso_riego = pulso_riego + 5;
                }
                else
                {
                    pulso_riego = riego_minimo;
                }
            }
            if (edicion7etapa == 2)
            {
                if (estado_riego)
                {
                    estado_riego = false;
                }
                else
                {
                    estado_riego = true;
                }
            }
            if (edicion7etapa == 3)
            {
            }
        }
        if (edicion8 == true)
        {
            if (edicion8etapa == 1)
            {
                Gestor_Aire(1);
                Guardovalores("aire", 0);
            }
            if (edicion8etapa == 2)
            {
                Gestor_Aire(2);
                Guardovalores("aire", 0);
            }
            if (edicion8etapa == 3)
            {
                Gestor_Aire(0);
                Guardovalores("aire", 0);
            }
        }
    }
}

void InterrupcionBoton3() // Entrar en edicion
{
    if (millis() > timeCounter + timeThreshold)
    {
        timeCounter = millis();

        // EDICION MENU2 HUMEDAD
        if (menu == 2)
        {
            if (edicion2 == false)
            {
                edicion2 = true;
                edicion2etapa = 1;
            }
            else
            {
                if (edicion2etapa < 2)
                {
                    edicion2etapa++;
                }
                else
                {
                    Guardovalores("hum", 0);
                }
            }
        }

        // EDICION MENU3 DISPOSITIVOS
        if (menu == 3)
        {
            if (edicion3 == true)
            {
                if (edicion3etapa < 6)
                {
                    edicion3etapa++;
                }
                else
                {
                    Guardovalores("cfg_disp", 0);
                }
            }
            else
            {
                edicion3 = true;
                edicion3etapa = 1;
            }
        }
        // EDICION MENU 4=FOTOPERIODO
        if (menu == 4)
        {
            if (edicion4 == false)
            {
                edicion4 = true;
                edicion4etapa = 1;
            }
            else
            {
                if (edicion4etapa < 2)
                {
                    edicion4etapa++;
                }
                else
                {
                    Guardovalores("foto", 0);
                }
            }
        }

        // EDICION MENU 5=CO2
        if (menu == 5)
        {
            if (edicion5 == false)
            {
                edicion5 = true;
                edicion5etapa = 1;
            }
            else
            {
                if (edicion5etapa < 2)
                {
                    edicion5etapa++;
                }
                else
                {
                    Guardovalores("co2", 0);
                }
            }
        }

        if (menu == 6)
        {
        }

        if (menu == 7) // EDICION RIEGO
        {
            if (edicion7 == false)
            {
                edicion7 = true;
                edicion7etapa = 1;
            }
            else
            {
                if (edicion7etapa < 2)
                {
                    edicion7etapa++;
                }
                else
                {
                    Guardovalores("riego", 0);
                    Guardovalores("cfg_disp", 0);
                }
            }
        }
        if (menu == 8) // EDICION AIRE ACONDICIONADO
        {
            if (edicion8 == false)
            {
                edicion8 = true;
                edicion8etapa = 1;
            }
            else
            {
                if (edicion8etapa < 3)
                {
                    edicion8etapa++;
                }
                else
                {
                    Guardovalores("aire", 0);
                }
            }
        }
    }
}

// FUNCIONES DE CONECTIVIDAD
void EnvioSensores()
{
    // Obtener la fecha y hora actual del RTC
    DateTime now = rtc.now();

    if (now.unixtime() - UltimoEnvio >= 15)
    {

        // Construir el JSON

        String json = "{\"tx\":\"mega\",\"ty\":\"s\",\"hs\":\"" + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()) + "\",\"co2\":" + String(CO2) + ",\"h1\":" + String(h01) + ",\"t1\":" + String(t01) + ",\"h2\":" + String(h02) + ",\"t2\":" + String(t02) + ",\"h3\":" + String(h03) + ",\"t3\":" + String(t03) + ",\"t4\":" + String(Tmhz) + "}";

        Agregar_Buffer(json);
        UltimoEnvio = now.unixtime();
    }
}

void AnalizoAccion(JsonDocument &msj)
{
    // const char *emisor = msj["tx"];
    const char *tipo = msj["ty"];

    if ((strcmp(tipo, "WIFI") == 0) || (strcmp(tipo, "MQTT") == 0))
    {
        RevisoConectividad(msj);
    }
    if (strcmp(tipo, "cfg_disp") == 0) // EL MENSAJE RECIBIDO ES CONFIGURACIÓN DE DISPOSITIVOS EN OFF, ON, AUTO
    {
        Mqtt_Config_Dispositivos(msj);
    }
    if (strcmp(tipo, "foto") == 0) // EL MENSAJE RECIBIDO ES CONFIGURACIÓN DE FOTOPERIODO
    {
        Mqtt_Fotoperiodo(msj);
    }
    if (strcmp(tipo, "hum") == 0) // EL MENSAJE RECIBIDO ES CONFIGURACIÓN DE HUMEDAD DESEADA
    {
        Mqtt_Humedad(msj);
    }
    if (strcmp(tipo, "temp") == 0) // EL MENSAJE RECIBIDO ES CONFIGURACIÓN DE TEMPERATURA
    {
        Mqtt_Temperatura(msj);
    }
    if (strcmp(tipo, "riego") == 0) // EL MENSAJE RECIBIDO ES LA CONFIGURACIÓN DE LA DURACIÓN DEL RIEGO
    {
        Mqtt_Riego(msj);
    }
    if (strcmp(tipo, "aire") == 0) // EL MENSAJE RECIBIDO ES LA CONFIGURACIÓN DE LA DURACIÓN DEL RIEGO
    {
        Mqtt_Aire(msj);
    }
    if (strcmp(tipo, "inicio") == 0) // EL MENSAJE RECIBIDO ES LA NOTIFICACIÓN DE INICIO --> ENVIO TODOS LOS PARÁMETROS AL NODE-RED
    {
        Guardovalores("inicio", 0);
    }
    if (strcmp(tipo, "hs_upd") == 0) // EL MENSAJE RECIBIDO ES LA NOTIFICACIÓN DE INICIO --> ENVIO TODOS LOS PARÁMETROS AL NODE-RED
    {
        Mqtt_Hora_Update(msj);
    }
}

void RevisoConectividad(JsonDocument &msj)
{

    const char *emisor = msj["tx"]; // Extrae el valor de la clave "emisor"
    const char *tipo = msj["ty"];   // Extrae el valor de la clave "ty" (tipo)
    if (strcmp(tipo, "WIFI") == 0)  // Compara si ty=WIFI
    {
        const char *mensaje = msj["msj"]; // Extrae el valor de la clave "mensaje"
        if (mensaje)                      // Verifica si la clave "mensaje" existía
        {
            estado_wifi = String(mensaje); // Almacena el mensaje en msj_conectividad
        }
    }
    else
    {
        if (strcmp(tipo, "MQTT") == 0) // Compara si ty=MQTT
        {
            const char *mensaje = msj["msj"]; // Extrae el valor de la clave "mensaje"
            if (mensaje)                      // Verifica si la clave "mensaje" existía
            {
                estado_mqtt = String(mensaje); // Almacena el mensaje en msj_conectividad
                if (estado_mqtt == "Conectado")
                {
                    estado_wifi = "Conectado";
                }
            }
        }
    }
}

void Mqtt_Config_Dispositivos(JsonDocument &msj)
{
    // Pins relays
    // #define pinfotoperiodo 40 --> 0 en el array
    // #define pinextractor 41  --> 1 en el array
    // #define pinco2 42        --> 2 en el array
    // #define pinhumidificador 43> 3 en el array
    // #define pinriego 44      --> 4 en el array
    // #define intractor 45      --> 5 en el array
    const char *emisor = msj["tx"];
    const char *relay = msj["relay"];
    const byte valor = msj["valor"];
    if (strcmp(emisor, "node") == 0)
    {
        if (strcmp(relay, "luz") == 0)
        {
            RPines[0] = valor;
        }
        if (strcmp(relay, "ext") == 0)
        {
            RPines[1] = valor;
        }
        if (strcmp(relay, "co2") == 0)
        {
            RPines[2] = valor;
        }
        if (strcmp(relay, "humi") == 0)
        {
            RPines[3] = valor;
        }
        if (strcmp(relay, "riego") == 0)
        {
            RPines[4] = valor;
        }
        if (strcmp(relay, "int") == 0)
        {
            RPines[5] = valor;
        }
    }
    LeoDispositivos();
    Guardovalores("cfg_disp", 0);
}

void Mqtt_Fotoperiodo(JsonDocument &msj)
{
    const char *emisor = msj["tx"];
    const byte f1 = msj["f1"];
    const byte f2 = msj["f2"];
    if (strcmp(emisor, "node") == 0)
    {
        if (f1)
        {
            fotoperiodo1a = f1;
        }
        if (f2)
        {
            fotoperiodo1b = f2;
        }
        Guardovalores("foto", 0);
        LeoDispositivos();
    }
}
void Mqtt_Humedad(JsonDocument &msj)
{
    const char *emisor = msj["tx"];
    const byte h1 = msj["h1"];
    const byte h2 = msj["h2"];
    if (strcmp(emisor, "node") == 0)
    {
        if (h1)
        {
            HMin = h1;
        }
        if (h2)
        {
            HMax = h2;
        }
        Guardovalores("hum", 0);
        LeoDispositivos();
    }
}
void Mqtt_Temperatura(JsonDocument &msj)
{
    const char *emisor = msj["tx"];
    const byte t1 = msj["t1"];
    const byte t2 = msj["t2"];
    if (strcmp(emisor, "node") == 0)
    {
        if (t1)
        {
            TMin = t1;
        }
        if (t2)
        {
            TMax = t2;
        }
        Guardovalores("temp", 0);
        LeoDispositivos();
    }
}

void Mqtt_Riego(JsonDocument &msj)
{
    const char *emisor = msj["tx"];
    const int duracion_riego = msj["pulso"];
    const byte switch_riego = msj["switch_riego"];
    if (strcmp(emisor, "node") == 0)
    {
        if (duracion_riego) // Si está establecido al duración, la tomo
        {
            pulso_riego = duracion_riego;
        }
        if (switch_riego) // Si está establecido un cambio en el switch, opero.
        {
            if (switch_riego == 1)
            {
                estado_riego = true;
            }
        }
        Gestor_Riego();
        Guardovalores("riego", 0);
        Guardovalores("cfg_disp", 0);
        LeoDispositivos();
    }
}

void Mqtt_Aire(JsonDocument &msj)
{
    const char *emisor = msj["tx"];
    const int orden = msj["orden"];
    if (strcmp(emisor, "node") == 0)
    {
        Gestor_Aire(orden);
        Guardovalores("aire", 0);
    }
}

void Mqtt_Hora_Update(JsonDocument &msj)
{
    const byte y = msj["y"];
    const byte m = msj["m"];
    const byte d = msj["d"];
    const byte h = msj["h"];
    const byte mi = msj["mi"];
    const byte s = msj["s"];
    rtc.adjust(DateTime(y, m, d, h, mi, s));
}

// ------ INFO LISTA MENUS-------------
// MENU 1: CRÍTICOS Tº Y H%
// MENU 2: HUMEDAD
// MENU 3: DISPOSITIVOS
// MENU 4: FOTOPERIODO
// MENU 5: CO2
// MENU 6: CONECTIVIDAD
// MENU 7: RIEGO

void Agregar_Buffer(String msj_serial)
{
    if (index_buffer < 15)
    {

        // Serial.print("Agrego al buffer: ");
        // Serial.print(index_buffer);
        // Serial.print(" :  ");
        // Serial.println(msj_serial);
        buffer_serial[index_buffer] = msj_serial;
        index_buffer++;
    }
}
void Procesar_Buffer_Serial()
{
    // Verificar si ha pasado suficiente tiempo desde el último mensaje enviado
    if (index_buffer > 0 && (millis() - ultimo_serial >= 750))
    {
        // Enviar el primer mensaje del buffer
        Serial2.println(buffer_serial[0]);

        // Serial.print("Siendo el buffer: ");
        //  Serial.print(index_buffer);
        Serial.print("Envío el msj ");
        Serial.print(": ");
        Serial.println(buffer_serial[0]);

        // Desplazar los mensajes en el buffer
        for (int i = 0; i < index_buffer - 1; i++)
        {
            buffer_serial[i] = buffer_serial[i + 1];
        }

        // Reducir el tamaño del buffer
        index_buffer--;
        // Serial.print("Resto uno al buffer: ");
        // Serial.println(index_buffer);

        // Actualizar el tiempo del último mensaje enviado
        ultimo_serial = millis();
    }
}

void Aire_Off()
{
    const uint16_t OffCode[227] = {
        // Código para apagado de aire acondicionado marca Electra
        3050, 1650, 450, 1100, 450, 1100, 450, 200, 600, 200, 600, 200, 600, 1100, 450, 250, 550, 200, 600, 1100, 450, 1100, 450, 200, 600, 1100, 450, 300, 500, 300, 500, 1100, 500, 1050, 450, 200, 650, 1100, 450, 1100, 450, 200, 600, 250, 550, 1100, 450, 200, 600, 250, 550, 1100, 300, 750, 200, 250, 550, 200, 600, 200, 550, 300, 550, 200, 550, 350, 500, 250, 500, 300, 500, 300, 500, 300, 550, 250, 500, 300, 500, 200, 600, 300, 500, 300, 500, 200, 600, 300, 500, 300, 500, 350, 400, 1150, 450, 300, 500, 300, 500, 1100, 450, 250, 550, 300, 500, 350, 450, 350, 450, 350, 450, 350, 400, 400, 400, 400, 450, 350, 450, 250, 550, 1100, 400, 400, 450, 350, 450, 350, 400, 400, 450, 350, 450, 350, 450, 350, 400, 400, 400, 350, 450, 350, 450, 1150, 400, 350, 450, 300, 500, 350, 450, 350, 450, 250, 550, 350, 400, 400, 400, 400, 400, 400, 450, 350, 400, 400, 450, 350, 400, 400, 400, 300, 500, 400, 400, 400, 400, 350, 450, 350, 450, 350, 450, 350, 450, 350, 450, 350, 450, 350, 450, 350, 450, 350, 400, 400, 450, 350, 450, 350, 450, 350, 450, 350, 400, 400, 400, 400, 400, 1150, 400, 400, 400, 1150, 400, 1150, 400, 1150, 450, 1100, 450, 1150, 400, 1150, 400, 1150, 450

    };
    IrSender.sendRaw(OffCode, 227, 38);
    delay(1000);
    IrSender.sendRaw(OffCode, 227, 38);
    estado_aire = 0;
    ////////Último comando enviado ///////
}

void Aire_26()
{

    const uint16_t OnCode[227] =
        {
            // Código para encendido de aire acondicionado marca Electra
            3100, 1600, 450, 1150, 450, 1100, 450, 150, 650, 150, 650, 150, 650, 1100, 450, 150, 650, 150, 650, 1100, 450, 1100, 450, 150, 650, 1100, 450, 150, 650, 250, 550, 1100, 450, 1100, 450, 200, 600, 1100, 450, 1150, 400, 250, 600, 150, 650, 1100, 450, 150, 650, 200, 600, 1100, 450, 150, 650, 300, 500, 300, 500, 250, 550, 150, 600, 300, 500, 300, 500, 300, 550, 150, 600, 300, 550, 200, 600, 200, 550, 250, 550, 300, 500, 250, 550, 250, 550, 250, 550, 1100, 450, 250, 550, 250, 550, 1100, 450, 250, 550, 250, 550, 1050, 550, 200, 550, 300, 500, 300, 500, 300, 500, 300, 450, 350, 450, 350, 450, 1100, 500, 300, 500, 1050, 500, 300, 500, 300, 500, 300, 450, 350, 450, 350, 450, 300, 500, 300, 500, 300, 500, 300, 500, 300, 550, 200, 600, 1050, 500, 200, 600, 200, 600, 200, 550, 250, 550, 250, 550, 250, 550, 250, 550, 250, 550, 250, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 450, 350, 450, 350, 450, 350, 450, 350, 450, 350, 450, 350, 450, 350, 450, 350, 450, 1100, 450, 1100, 450, 1150, 450, 1100, 450, 1100, 450, 1100, 450, 1100, 450, 350, 500

        };
    IrSender.sendRaw(OnCode, 227, 38);
    delay(1000);
    IrSender.sendRaw(OnCode, 227, 38);
    delay(1000);
    if (estado_aire == 0) // Si el aire estaba apagado, al prenderlo apago el display. Si ya estaba prendido, el display ya hubo sido apagado.
    {
        Aire_Display();
    }
    Serial.println("Ejecuté Aire 26");
    estado_aire = 1;
}

void Aire_Deshu()
{

    const uint16_t OnCode[227] =
        {
            // Código para encendido de aire acondicionado marca Electra
            3100, 1600, 450, 1150, 450, 1100, 450, 150, 650, 150, 650, 150, 650, 1100, 450, 150, 650, 150, 650, 1100, 450, 1100, 500, 150, 650, 1100, 450, 150, 650, 150, 650, 1100, 500, 1050, 450, 200, 600, 1100, 500, 1100, 450, 250, 550, 150, 650, 1100, 450, 200, 600, 300, 500, 1100, 450, 150, 700, 150, 600, 150, 650, 300, 500, 150, 650, 300, 500, 150, 650, 300, 500, 150, 650, 300, 500, 200, 600, 150, 650, 150, 650, 150, 600, 200, 600, 300, 500, 300, 500, 1100, 500, 250, 500, 300, 500, 1150, 450, 250, 550, 250, 550, 250, 500, 1150, 450, 250, 550, 250, 550, 250, 550, 250, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 1100, 500, 250, 550, 250, 500, 300, 500, 200, 600, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 1100, 500, 250, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 450, 350, 450, 350, 450, 300, 500, 300, 500, 300, 550, 250, 550, 250, 550, 250, 550, 250, 550, 250, 550, 250, 550, 1050, 450, 1100, 500, 1050, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500

        };
    IrSender.sendRaw(OnCode, 227, 38);
    delay(1000);
    IrSender.sendRaw(OnCode, 227, 38);
    delay(1000);

    if (estado_aire == 0) // Si el aire estaba apagado, al prenderlo apago el display. Si ya estaba prendido, el display ya hubo sido apagado.
    {
        Aire_Display();
    }

    estado_aire = 2;
}
void Aire_Display() // CAMBIAR CÓDIGO X EL REAL
{

    const uint16_t OnCode[227] =
        {
            // Código para encendido de aire acondicionado marca Electra
            3100, 1650, 400, 1150, 450, 1100, 450, 250, 550, 150, 650, 200, 600, 1100, 450, 200, 600, 250, 550, 1100, 450, 1100, 500, 150, 600, 1150, 450, 200, 550, 300, 500, 1150, 400, 1150, 450, 200, 600, 1100, 450, 1100, 450, 300, 500, 200, 600, 1100, 450, 200, 600, 350, 450, 1150, 450, 150, 600, 250, 550, 300, 500, 200, 600, 300, 500, 350, 450, 350, 450, 350, 450, 200, 600, 200, 600, 200, 600, 200, 600, 200, 600, 300, 500, 200, 600, 300, 500, 200, 600, 1100, 450, 300, 500, 200, 600, 1100, 450, 1100, 500, 250, 550, 150, 650, 1100, 450, 250, 550, 250, 550, 250, 550, 250, 550, 250, 500, 200, 600, 200, 600, 300, 500, 300, 500, 1100, 450, 300, 500, 300, 500, 300, 550, 200, 550, 300, 550, 200, 550, 300, 550, 200, 550, 250, 550, 250, 550, 1100, 500, 200, 550, 250, 550, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 450, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 500, 300, 550, 200, 600, 200, 550, 250, 550, 250, 550, 250, 550, 250, 550, 1050, 500, 1050, 550, 1050, 450, 300, 500, 300, 500, 300, 500, 300, 500, 1100, 450, 300, 550

        };
    IrSender.sendRaw(OnCode, 227, 38);
}

void Gestor_Riego()
{
    DateTime now = rtc.now();
    if (estado_riego)
    {

        if (BoolPines[4] == false)
        {
            BoolPines[4] = true; // Activar el pin de riego
            // Guardovalores("cfg_disp", 0);
            // SOLO ENVÍO ACTUALIZACIÓN A NODE EN EL MOMENTO EN QUE CAMBIA EL BOOL 4, NO TODO EL TIEMPO
            if (ultimo_riego.unixtime() == 0)
            {
                ultimo_riego = rtc.now();
            }
        }

        // Obtener la fecha y hora actual del RTC

        // Calcular la diferencia de tiempo en segundos desde el inicio del riego
        tiempo_transcurrido_riego = (now.unixtime() - ultimo_riego.unixtime());

        if (tiempo_transcurrido_riego >= pulso_riego)
        {
            // Si ha pasado el tiempo de riego
            if (BoolPines[4] == true)
            {
                BoolPines[4] = false; // Desactivar el pin de riego
                estado_riego = false; // Desactivar el riego
                ultimo_riego = DateTime(0);
                tiempo_transcurrido_riego = 0;
                Guardovalores("cfg_disp", 0);
            }
        }
    }
    else
    {
        BoolPines[4] = false;
        tiempo_transcurrido_riego = 0;
        ultimo_riego = DateTime(0);
    }
}

void SensoresTyH()
{
    static int contadorErrorDHT1 = 0;
    static int contadorErrorDHT2 = 0;
    // static int contadorErrorDHT3 = 0;
    static int contadorErrorMHZ = 0;

    DateTime now = rtc.now();
    TimeSpan interval = now - UltimaLecturaSensores;
    TimeSpan tiempoTranscurridoDesdeInicio = now - InicioPrograma;   // Tiempo transcurrido desde el inicio del programa
    TimeSpan tiempoTranscurridoDesdeReinicio = now - UltimoReinicio; // Tiempo transcurrido desde el último reinicio

    if (interval.totalseconds() >= 2)
    {
        // Recojo CO2
        CO2 = myMHZ19.getCO2(false, true);
        Tmhz = myMHZ19.getTemperature();

        // Verificación de errores en MHZ19
        if ((Tmhz == 239 || Tmhz == 0) && tiempoTranscurridoDesdeReinicio.totalseconds() >= 900)
        {
            contadorErrorMHZ++;
        }
        else if (Tmhz != 239 && Tmhz != 0)
        {
            contadorErrorMHZ = 0;
        }

        // Recojo Temperaturas y Humedades
        float temp1 = dht1.readTemperature();
        if (t01 > 0)
        {
            byte DiferenciaLecturasT0 = temp1 - t01;
            if (DiferenciaLecturasT0 < 5)
            {
                t01 = temp1;
            }
        }
        else
        {
            t01 = temp1;
        }

        h01 = dht1.readHumidity();
        // Verificación de errores en DHT1
        if (t01 == 0 && tiempoTranscurridoDesdeReinicio.totalseconds() >= 900)
        {
            contadorErrorDHT1++;
        }
        else if (t01 != 0)
        {
            contadorErrorDHT1 = 0;
        }

        float temp2 = dht2.readTemperature();
        if (t02 > 0)
        {
            byte DiferenciaLecturasT2 = temp2 - t02;
            if (DiferenciaLecturasT2 < 5)
            {
                t02 = temp2;
            }
        }
        else
        {
            t02 = temp2;
        }

        h02 = dht2.readHumidity();
        // Verificación de errores en DHT2
        if (t02 == 0 && tiempoTranscurridoDesdeReinicio.totalseconds() >= 900)
        {
            contadorErrorDHT2++;
        }
        else if (t02 != 0)
        {
            contadorErrorDHT2 = 0;
        }

        // float temp3 = dht3.readTemperature();
        // if (t03 > 0) {
        //     byte DiferenciaLecturasT3 = temp3 - t03;
        //     if (DiferenciaLecturasT3 < 5) {
        //         t03 = temp3;
        //     }
        // } else {
        //     t03 = temp3;
        // }

        // h03 = dht3.readHumidity();
        // // Verificación de errores en DHT3
        // if (t03 == 0 && tiempoTranscurridoDesdeReinicio.totalseconds() >= 300) {
        //     contadorErrorDHT3++;
        // } else if (t03 != 0) {
        //     contadorErrorDHT3 = 0;
        // }

        // Verificar si hay que reiniciar los sensores
        if (contadorErrorDHT1 >= 3 || contadorErrorDHT2 >= 3 || contadorErrorMHZ >= 3)
        {
            digitalWrite(24, LOW); // Activa el pin 24
            Serial3.end();
            delay(1000);            // Espera 1 segundo
            digitalWrite(24, HIGH); // Desactiva el pin 24
            Serial3.begin(9600);
            myMHZ19.begin(Serial3);        // *Serial(Stream) refence must be passed to library begin().
            myMHZ19.autoCalibration(true); // Turn auto calibration ON (OFF autoCalibration(false))
            myMHZ19.setFilter(true, true);

            // Reinicia los contadores de error
            contadorErrorDHT1 = 0;
            contadorErrorDHT2 = 0;
            // contadorErrorDHT3 = 0;
            contadorErrorMHZ = 0;

            // Actualiza el tiempo del último reinicio
            UltimoReinicio = now;
        }

        // Actualiza el tiempo de la última lectura
        UltimaLecturaSensores = now;
    }
}

void Gestor_Aire(int orden)
{
    if (orden == 1)
    {
        Aire_26();
        estado_aire = 1;
    }
    if (orden == 2)
    {
        Aire_Deshu();
        estado_aire = 2;
    }
    if (orden == 0)
    {
        Aire_Off();
        estado_aire = 0;
    }
}
