

//IINICIO SISTEMA

// LIBRERÍAS Y VARIABLES
#include "WiFi.h"                 // Conection con red WiFi
#include "time.h"                 // Servidor NTP para obtener fecha y hora
#include "Adafruit_MQTT.h"        // Permite establecer la conexión con el servidor IoT 
#include "Adafruit_MQTT_Client.h" // Crear cliente para la publicación y suscripción para uso del protocolo MQTT
#include <Wire.h>                 // Libreria para bus I2C
#include <X9C.h>                  // Potenciometros digitales para control de iluminación
#include <BH1750.h>               // Librería sensor BH1750 Luxometro

#define WIFI_SSID       "BOY"                               // Credenciales de acceso a internet: Nombre de la red
#define WIFI_PASS       "51144258"                          // Credenciales de acceso a internet: Contraseña
#define AIO_SERVER      "io.adafruit.com"                   // Servidor IoT - MQTT
#define AIO_SERVERPORT  1883                                // Puerto a travez del cual se realiza la conexión con el servidor
#define AIO_USERNAME    "Dafenima97"                        // Credenciales de acceso al servidor: Usuario
#define AIO_KEY         "aio_uHMj91fsGqgTk9cXhNuIx6RDTyjo"  // Credenciales de acceso al servidor: Api KEY - que permite el envío de datos al servidor 
                                                            // (Tambien se entiende como la clave de acceso al servidor)

#define INC 4   // D1 Mini GPIO4
#define UD 0    // D1 Mini GPIO0
#define CS 2    // D1 Mini GPIO2
X9C pot;        // Se crea el objeto pot
BH1750 sensor;  // Se crea objeto sensor

#define D6T_ID 0x0A  // Se define el ID-dirección para el D6T
#define D6T_CMD 0x4C // Se define comando para tomar la info

// El sensor D6T envía 35 bytes de datos para procesar  // Se almacenan esas variables
int ReadBuffer[35];    // D6T Buffer
float ptat;            // Temperatura de referencia interna
float tdata[16];       // Data temporal de temperatura para 16px (4x4)

#define DHTPIN 23 
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);

//***********************************************************************
const int MQ1Pin = 39;
const int MQ2Pin = 35;
int LPin   = 34;
int POSICION_ESTIMADA = 0;
int pinDHT11 = 0;;
int NL = 0;
float h = 0;
float t = 0;
int MonitorSerial = 9600
//***********************************************************************
int HighH = 300;
int LowH = 250;
//***********************************************************************
String mensaje = "";
String mensajeN = "";
int Mensaje_Modo = 0;
int Mensaje_Valor = 0;
unsigned int lux = 0;
int luz_dim = 0;
int flag = 0;
//***********************************************************************
const int RL_MQ1 = 20;     // Resistencia RL del modulo en Kilo ohms
const int RL_MQ2 = 5;      // Resistencia RL del modulo en Kilo ohms
const int R0_MQ1 = 3.8;    // Resistencia R0 del sensor en Kilo ohms
const int R0_MQ2 = 10;     // Resistencia R0 del sensor en Kilo ohms
// Datos para lectura multiple
const int READ_SAMPLE_INTERVAL = 100;  // Tiempo entre muestras
const int READ_SAMPLE_TIMES = 5;       // Numero muestras
// Se ajustan los valores de al Datasheet y el tipo de gas que se quiere detectar
// En este caso para el sensor MQ-135 el gas propano
const float X0_MQ1 = 10;
const float Y0_MQ1 = 2.4;
const float X1_MQ1 = 200;
const float Y1_MQ1 = 0.8;
// En este caso para el sensor MQ-2 el gas CO2
const float X0_MQ2 = 200;
const float Y0_MQ2 = 1.7;
const float X1_MQ2 = 10000;
const float Y1_MQ2 = 0.28;
// Variables de cálculo sensores MQ-135 y MQ-2
float mq1_prom = 0;
float mq_1 = 0;
float mq2_prom = 0;
float mq_2 = 0;

//***********************************************************************

// Puntos de la curva de concentración {X, Y}
const float punto0_MQ1[] = { log10(X0_MQ1), log10(Y0_MQ1) };
const float punto1_MQ1[] = { log10(X1_MQ1), log10(Y1_MQ1) };

// Calcular pendiente y coordenada abscisas
const float scope_MQ1 = (punto1_MQ1[1] - punto0_MQ1[1]) / (punto1_MQ1[0] - punto0_MQ1[0]);
const float coord_MQ1 =  punto0_MQ1[1] - punto0_MQ1[0] * scope_MQ1;

// Puntos de la curva de concentración {X, Y}
const float punto0_MQ2[] = { log10(X0_MQ2), log10(Y0_MQ2) };
const float punto1_MQ2[] = { log10(X1_MQ2), log10(Y1_MQ2) };

// Calcular pendiente y coordenada abscisas
const float scope_MQ2 = (punto1_MQ2[1] - punto0_MQ2[1]) / (punto1_MQ2[0] - punto0_MQ2[0]);
const float coord_MQ2 =  punto0_MQ2[1] - punto0_MQ2[0] * scope_MQ2;

//***********************************************************************

WiFiClient client;
// Configuración del cliente MQTT
// Se envían como parámetros: el cliente WiFi, el servidor MQTT y los detalles de inicio de sesión.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
// envío Mqtt 
Adafruit_MQTT_Publish      Temperatura = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Temperatura");
Adafruit_MQTT_Publish          Humedad = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Humedad");
Adafruit_MQTT_Publish         NivelLuz = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/NivelLuz");
Adafruit_MQTT_Publish      Iluminacion = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Iluminacion");
Adafruit_MQTT_Publish   AirQuality_MQ1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/AirQuality_MQ1");
Adafruit_MQTT_Publish   AirQuality_MQ2 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/AirQuality_MQ2");
Adafruit_MQTT_Publish Posicion_Persona = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Posicion_Persona");
// recepción Mqtt 
Adafruit_MQTT_Subscribe DimMode = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/DimMode", MQTT_QOS_1);
Adafruit_MQTT_Subscribe ControlNivelLuz = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/ControlNivelLuz", MQTT_QOS_1);

//***********************************************************************
void Modo(char *data, uint16_t len) {
  Serial.print("Valor rebido del boton: "); Serial.println(data);
  mensaje = String(data);
  Mensaje_Modo = mensaje.toInt();
  //Serial.print("Modo String: ");  Serial.print(mensaje);
  //Serial.print("Modo INT: ");  Serial.print(Mensaje_Modo);
}
//***********************************************************************
void Nivel(char *data, uint16_t len) {
  Serial.print("Valor rebido del boton: "); Serial.println(data);
  mensajeN = String(data);
  Mensaje_Valor = mensajeN.toInt();
  //Serial.print("Valor String: ");Serial.print(mensajeN);
  //Serial.print("Valor INT: ");  Serial.print(Mensaje_Valor);
}
//***********************************************************************
// Para realizar la conexión con el servidor Adafruit
void connect() { 
  Serial.print(F("Conectando con Adafruit IO... "));
  int8_t ret;
  while ((ret = mqtt.connect()) != 0) {
    switch (ret) {
      case 1: Serial.println(F("Protocolo incorrecto")); break;
      case 2: Serial.println(F("ID rechazado")); break;
      case 3: Serial.println(F("Servidor inválido")); break;
      case 4: Serial.println(F("Usuario o contraseña erroneas")); break;
      case 5: Serial.println(F("No autorizado")); break;
      case 6: Serial.println(F("Suscripción fallida")); break;
      default: Serial.println(F("Connexión fallida")); break;
    }
    if (ret >= 0) 
      mqtt.disconnect();
    Serial.println(F("Reintentando Conexión..."));
    delay(10000);
  }
  Serial.println(F("Adafruit IO Conectado!"));
}
//***********************************************************************
void setup() {
//ALIDAS Y OBJETOS
  Serial.begin(MonitorSerial); // Se inicializa Monitor serial
  pinMode (2, INPUT);          // Se ideclara el led interno de la tarjeta como salida
                               // (Este será usado como indicador de conexión a la red WiFi
  dht.begin();                 // Se inicializa el objeto deht
  sensor.begin();              // Se inicializa sensor con valores por defecto
  pot.begin(CS, INC, UD);      // Se inicializa el objeto pot
  
//CONEXIÓN I2C                          
  Wire.begin();                // Se inicializa bus I2C
    
//CONEXIÓN WiFi
  CONEXION_WiFi();             // Se realiza la conexión con la red WiFi

//CONEXIÓN MQTT
  // Recibir Información desde el Dashboard - Protocolo MQTT
  DimMode.setCallback(Modo);          // El método setCallback permite la recepción de info. desde la plataforma -> se envia a la función void Modo donde se procesa
  ControlNivelLuz.setCallback(Nivel); // El método setCallback permite la recepción de info. desde la plataforma -> se envia a la función void Nivel donde se proces
  mqtt.subscribe(&DimMode);           // Se realiza la suscripción al feed DimMode
  mqtt.subscribe(&ControlNivelLuz);   // Se realiza la suscripción al feed ControlNivelLuz
  connect();
}
//LOOP
void loop() {
// ADQUISICIÓN DE VARIABLES AMBIENTALES   
  Variables_Ambientales(); 
  
// ALGORITMO DE CONTROL
 if (Modo_Control_Luz == 1){
       if (flag == 0 && Lux>HighH){ pot.setPot(0, true);flag=0;}
  else if (flag == 0 &&  Lux<LowH){pot.setPot(40, true);flag=1;}
  else if (flag == 0 &&  Lux>LowH){ pot.setPot(0, true);flag=0;}
  else if (flag == 1 &&  Lux>LowH){pot.setPot(40, true);flag=1;}
  else if (flag == 0 && Lux>HighH){ pot.setPot(0, true);flag=0;}
  else                            {pot.setPot(40, true);flag=0;}
  
    DETECION_D6T();
    POSICION();
  } 
  
  else{ 
    mqtt.processPackets(500); 
  }
  
// CONTROL DASHBOARD 
  int Modo_Control_Luz = Mensaje_Modo;  
  int Nivel_Luz_Deseado = Mensaje_Valor;
  
// CONTROL LUZ  
  CONTROL_LUZ(Modo_Control_Luz, Nivel_Luz_Deseado);  

// RE CONEXIÓN A RED WiFi
  if (WiFi.status() != WL_CONNECTED){
    CONEXION_WiFi();
    break;
  }
// COMUNICACIÓN AL SERVIDOR ADAFRUIT IO
  Servidor_ADAFRUIT();
  delay(500);
}

//
// FUNCIONES
void CONEXION_WiFi() {
  digitalWrite(2, LOW);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int cont = 0;
  while (x<=500){
  if (WiFi.status() != WL_CONNECTED) {
    delay(700);
    Serial.print("\nConnecting to WiFi...");
  }
  else{
    Serial.print("\nConnected to the WiFi network: ");
    Serial.print(WIFI_SSID);  Serial.print("\n");
    digitalWrite(2, HIGH);
  }
}
void CONTROL_LUZ(int Modo_Control_Luz, int Nivel_Luz_Deseado) {
    if (Modo_Control_Luz == 1) //Modo Automatico{
           if (pos == 3) {Serial.print(pos);pot.setPot(10, true);delay(500);}
      else if (pos == 2) {Serial.print(pos);pot.setPot(60, true);delay(500);}
      else if (pos == 1) {Serial.print(pos);pot.setPot(90, true);delay(500);}
    }
    //CONTROL DE ILUMINACIÓN DESDE DASHBOARD
    else if (Modo_Control_Luz == 0) //Modo Manual {
      pot.setPot(Nivel_Luz_Deseado, true);
    }
}

void DETECION_D6T() {  /*[1]*/
  int i;
  // Pregunta al sensor D6T si hay datos
  Wire.beginTransmission(D6T_ID);
  Wire.write(D6T_CMD);
  Wire.endTransmission();
  // Toma los datos del sensor
  Wire.requestFrom(D6T_ID,35);
  // Almacena los datos la memoria
  for(i=0; i<35; i++){ ReadBuffer[i] = Wire.read();}
  // Procesamiento de los datos:            Byte  0 -  1 --> Temperaura de Referencia
  // Byte  2 - 33 --> Dato de temperatura   Byte      34 --> Paquete Validación error
  ptat = (ReadBuffer[0]+(ReadBuffer[1]*256))*0.1;
  
  // Datos de Temperatura
  for(i=0; i<16; i++){
     tdata[i]= (ReadBuffer[(i*2+2)]+(ReadBuffer[(i*2+3)]*256))*0.1;
  }
  float tempF;
  // Imprimir en monitor Serial la temperatura en Farenheit
  if( ((tdata[0]*9.0/5.0)+32.0)>0 ){
    for (i=0; i<16; i++){
      tempF = (tdata[i]*9.0/5.0)+32.0;
      Serial.print(tempF);
      Serial.print(',');      
    }
    Serial.print((ptat*9.0/5.0)+32.0);
    Serial.print(',');    
    Serial.print(' ');
  }  
  Serial.print('\n');
}
void Servidor_ADAFRUIT(){  /*[3]*/
  Serial.print("Envío de variables ");
  // Se envía un ping a Adafruit IO varias veces para asegurar que la conexión permanece
  if (! mqtt.ping(3)) {
    // Reconectando con Adafruit IO
    if (! mqtt.connected())
      connect();
  }
  //Envío de información a Adafruit IO
  if (! Temperatura.publish(t))        {Serial.println(F("Envío Fallido"));}
  if (! Humedad.publish(h))            {Serial.println(F("Envío Fallido"));}
  if (! NivelLuz.publish(NL))          {Serial.println(F("Envío Fallido"));}
  if (! Iluminacion.publish(lux))      {Serial.println(F("Envío Fallido"));}
  if (! AirQuality_MQ1.publish(mq_1))  {Serial.println(F("Envío Fallido"));}
  if (! AirQuality_MQ2.publish(mq_2))  {Serial.println(F("Envío Fallido"));}
  if (! Posicion_Persona.publish(pos)) {Serial.println(F("Envío Fallido"));}
  else                                 {Serial.println(F("Envío Exitoso"));}
}
void POSICION(){
  int i;
    float h = 96.8;       // Temperatur a de referencia para detectar presencia
    float temp[16];       // Data temporal de temperatura para 16px (4x4) 
    for (i=0; i<15; i++){
      temp[i] =  (tdata[i]*9.0/5.0)+32.0;
    }
    Serial.print('\n');Serial.print(temp[1]);Serial.print('\n');

    if ((temp[5]>h) || (temp[6]>h) || (temp[9]>h) || (temp[10]>h)){
      POSICION_ESTIMADA = 1;
    }
    else if((temp[1]>h) || (temp[2]>h) || (temp[4]>h) || (temp[7]>h) || (temp[8]>h) || (temp[11]>h) || (temp[13]>h) || (temp[14]>h)){
      POSICION_ESTIMADA = 2;
    }
    else if((temp[0]>h) || (temp[3]>h) || (temp[12]>h) || (temp[15]>h)){
      POSICION_ESTIMADA = 3;
    }
    else{
      POSICION_ESTIMADA = 0; // NO HAY PRESENCIA
    }
    Serial.print("Posición Estimada: "); //Serial.println(POSICION_ESTIMADA);
    Serial.println(p);
}
void Variables_Ambientales(){ 
  h = dht.readHumidity();
  t = dht.readTemperature();
       if (pos == 3) {Serial.print(pos);NL=10; luz_dim=40;}
  else if (pos == 2) {Serial.print(pos);NL=60; luz_dim=90;}
  else if (pos == 1) {Serial.print(pos);NL=90;luz_dim=140;}
  lux = sensor.readLightLevel();
  lECTURA_mq1();  /*[2]*/
  LECTURA_mq2();  /*[2]*/
  pos = POSICION_ESTIMADA;
}
void lECTURA_mq1(){
   mq1_prom = readMQ1(MQ1Pin);             // Obtener la Rs promedio
   mq_1 = Concentracion1(mq1_prom/R0_MQ1); // Obtener la concentración
}
void LECTURA_mq2(){
   mq2_prom = readMQ2(MQ2Pin);             // Obtener la Rs promedio
   mq_2 = Concentracion2(mq2_prom/R0_MQ2); // Obtener la concentración
}

float readMQ1(int mq2_pin){
   float rs = 0;
   for (int i = 0;i<READ_SAMPLE_TIMES;i++) {
      rs += MQ1Resistance(analogRead(mq1_pin));
      delay(READ_SAMPLE_INTERVAL);
   }
   return rs/READ_SAMPLE_TIMES;
}
float MQ1Resistance(int raw_adc){
   return (((float)RL_MQ1 / 1000*(1023 - raw_adc) / raw_adc));
}
float Concentracion1(float rs_ro_ratio){
   return pow(10, coord_MQ1 + scope_MQ1 * log(rs_ro_ratio));
}

float readMQ2(int mq2_pin){
   float rs = 0;
   for (int i = 0;i<READ_SAMPLE_TIMES;i++) {
      rs += MQ2Resistance(analogRead(mq2_pin));
      delay(READ_SAMPLE_INTERVAL);
   }
   return rs/READ_SAMPLE_TIMES;
}
float MQ2Resistance(int raw_adc){
   return (((float)RL_MQ2 / 1000*(1023 - raw_adc) / raw_adc));
}
float Concentracion2(float rs_ro_ratio){
   return pow(10, coord_MQ2 + scope_MQ2 * log(rs_ro_ratio));
}
