#include <WebServer.h>
#include <WiFi.h>
#include <esp32cam.h>
#include <WiFiUdp.h> // creazione della socket udp per la comunicazione con matlab
#include <Wire.h> // per la comunicazione i2c
#include "esp_camera.h" // librerie per la camera
#include "soc/soc.h" // libreria per creare la socket 
#include "soc/rtc_cntl_reg.h"

#define MAT_PORT 5000 // porta su cui comunica la socket matlab
#define PY_PORT 4210 //  porta su cui riceve i dati da python

#define MAX_DISTANCE 1000 //cm  10 metri
#define SONIC_TIMEOUT (MAX_DISTANCE*60) // calcolo del timeout
#define VELOCITA_SUONO 340 //velocità del suono : 340m/s

struct I2cTxStruct { // struttura del pacchetto i2c
    unsigned long valA;          //  4
    unsigned long valB;          //  4
    unsigned long valC;          //  4
    unsigned long valD;          //  4
    unsigned long valE;          //  4
    byte padding[12];           // 12
                            //------
                            // 32
};
I2cTxStruct txData = {0,0,0,0,0}; //x,y,D,Dmin,stop.
bool newTxData = false;
TwoWire I2CSensors = TwoWire(0);
const byte otherAddress = 8;
// i2c configurazione FINE

// variabili di timing per il campionamento dei dati 
unsigned long prevUpdateTime = 0;
unsigned long updateInterval = 10;
unsigned long old_t = 0;
unsigned long old_t_2 = 0;
unsigned long T_samp = 1000;


//dichiarazione delle variabili globali 
int last_cmd;
int preferDistance;
int cmd_aux;
int pos_x;
int pos_y;
int distance;
int XcmVal , YcmVal;
int XcmOLD;
int center = 200;
int gap = 100;
int led_on ; 
int packetSize_M,packetSize_P;
int len;
int duration_us;//variabili del distanziometro
int distance_cm;
int ping;
int dist; // valore da passare al seriale di arduino 
int latency=0;
int pkt;
// array pacchetti di scambio con matlab
int pkt_O[6]={0} ; // pacchetto dati da inviare a MATLAB 
int pkt_I[3]={0} ; // pacchetto dati da riceve da MATLAB
int pkt_I_p[2]={0}; // riceve dallo script python solamente x e y 


const char* WIFI_SSID = "EXT";
const char* WIFI_PASS = "gestapo94";

WebServer server(80); // apro la porta per l interfaccia web
// comunicazione tra matlab e ESP
WiFiUDP Udp_M; // inizializza la socket udp MATLAB
WiFiUDP Udp_P; // inizializzo la socket udp PYTHON

char incomingPacket[255];  // buffer che contiene il testo del messaggio
// impostare con l'IP remoto del client che si vuole connettere 
// ip e porta per matlab 



IPAddress rip(192, 168, 1, 2); // IP Terminale di Destinazione (CAMBIARE QUESTO )
long rport = 5006; // Porta di Destinazione su cui mandare il pacchetto 



// assegnazione dei pin idle 
#define Toggle_Pin 4 //TOGGLE Pin  
#define I2C_SDA 14 // Pin di comunicazione con arduino
#define I2C_SCL 15 // pin di comunicazione con arduino
#define TRIG_PIN 13 // ESP32 pin GIOP23 connected to Ultrasonic Sensor's TRIG  
#define ECHO_PIN 2 // ESP32 pin GIOP22 connected to Ultrasonic Sensor's ECHO pin

static auto loRes = esp32cam::Resolution::find(320, 240);


void handleJpgLo(){
  if (!esp32cam::Camera.changeResolution(loRes)) {
    Serial.println("SET-LO-RES FAIL");
  }
  auto frame = esp32cam::capture();
  if (frame == nullptr) {
    Serial.println("CAPTURE FAIL");
    server.send(503, "", "");
    return;
  }
  server.setContentLength(frame->size());
  server.send(200, "image/jpeg");
  WiFiClient client = server.client();
  frame->writeTo(client);
}

 
 
void  setup(){
  Serial.begin(115200);
  Serial.println("");
  Wire.begin(I2C_SDA,I2C_SCL,115200); // join i2c bus
  
  //UltraSonic Sensor
  pinMode(TRIG_PIN, OUTPUT); // configure the trigger pin to output mod
  pinMode(ECHO_PIN, INPUT); // configure the echo pin to input mode
  
  //Configurazione toggle Pin (Alto in caso di caricamento , basso altrimenti )
  pinMode(Toggle_Pin, OUTPUT);
  digitalWrite(Toggle_Pin,LOW);
  
  {
    using namespace esp32cam;
    Config cfg;
    cfg.setPins(pins::AiThinker);
    cfg.setResolution(loRes);
    cfg.setBufferCount(2);
    cfg.setJpeg(80);
 
    bool ok = Camera.begin(cfg);
    Serial.println(ok ? "CAMERA OK" : "CAMERA FAIL");
  }
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA); // default
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  delay(1000);
  long int StartTime=millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
     if ((StartTime+10000) < millis()) 
      break;   
  } 
  Serial.print("http://");
  Serial.println(WiFi.localIP());
  Serial.println("  /cam-lo.jpg");

  server.on("/cam-lo.jpg", handleJpgLo);
 
  server.begin();
  int status = WL_IDLE_STATUS;
  Udp_M.begin(MAT_PORT); // socket per matlab
  Udp_P.begin(PY_PORT); // socket per python 
  Serial.printf(" porta di comunicazione con Matlab : %d\n",MAT_PORT);
  Serial.printf(" porta di comunicazione con Python : %d\n",PY_PORT);
}
 
void loop(){
   
   latency = (millis()-old_t_2); // latenza espressa in millisecondi  
   old_t_2 = millis();

  server.handleClient();

  // calcolo della distanza 
  digitalWrite(TRIG_PIN, HIGH); 
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  ping = pulseIn(ECHO_PIN, HIGH,SONIC_TIMEOUT); //misura la durata dell'impulso dal pin ECHO
  if(ping !=0)
    distance_cm = ((ping*VELOCITA_SUONO)/2)/10000;  // calcolo della distanza scalandola in cm
  else
    distance_cm = MAX_DISTANCE; // in caso di errore verrà scritto 1000 nella variabile  
  
  // ricezione dei pacchetti 
  // prende in inputo solamente i 2 valori x e y 
  packetSize_P = Udp_P.parsePacket(); // ricezione del pacchetto
  if(packetSize_P==8){// se arriva un pacchetto con 24 byte allora scrive 
    len = Udp_P.read((byte*)&pkt_I_p,packetSize_P);
    memcpy((void *)&pkt_O[3],&pkt_I_p[0],4);  // x
    memcpy((void *)&pkt_O[4],&pkt_I_p[1],4);  //y
  }
  // prende in input i soli 3 parametri di comando 
  packetSize_M = Udp_M.parsePacket(); // ricezione del pacchetto
  if(packetSize_M==8){// se arriva un pacchetto con 24 byte allora scrive 
    len = Udp_M.read((byte*)&pkt_I,packetSize_M);
    memcpy((void *)&pkt_O[0],&pkt_I[0],4);  // last_cmd
    memcpy((void *)&pkt_O[1],&pkt_I[1],4);  //prefer_distance 
  }
  pkt_O[2]=latency; // Distanza calcolata di su
  pkt_O[5]=distance_cm; // Distanza calcolata di su
// send del pacchetto a matlab
  if(millis()-old_t >= T_samp){
     old_t = millis(); 
     Udp_M.beginPacket(rip, rport); //inizializza un pacchetto
     Udp_M.write((byte*)pkt_O,sizeof(pkt_O)); // scrive il messaggio (modificare la write)
     Udp_M.endPacket(); // chiude il pacchetto e lo invia
     Udp_M.flush(); // attende l avvenuta uscita di tutti i pacchetti
  }
  // Cunicazione I2C INIZIO 
   if (millis() - prevUpdateTime >= updateInterval) {
        prevUpdateTime = millis();
            txData.valA= pkt_O[3]; // x 
            txData.valB= latency; // passo la latenza all arduino in quanto devo rispettare il teorema del campionamento  
            txData.valC=distance_cm; //distanza
            txData.valD=pkt_O[1];// distanza preferenziale
            txData.valE=pkt_O[0]; //Stop/Start
            Wire.beginTransmission(otherAddress); // trasmette pacchetto su seriale arduino
            Wire.write((byte*) &txData, sizeof(txData));
            Wire.endTransmission();    // this is what actually sends the data
    }
  // fine  comunicazione I2C
}

