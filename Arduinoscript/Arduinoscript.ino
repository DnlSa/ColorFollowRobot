#include <Wire.h>

struct I2cRxStruct {
    unsigned long Xcm;     //  4 
    unsigned long Ycm;     //  4          
    unsigned long distance;     //  4
    unsigned long dmin;      //  4
    unsigned long start;      //  4  
    byte padding[12];       // 12 // padding di completamenteo per la trasmissione dati 
                            //------
                            // 32    
};
I2cRxStruct rxData;
bool newRxData = false; // variabile flag per indicare se sono stati ricevuti i dati o no 
const byte thisAddress = 8; // byte di indirizzo per arduino 

//definizione dei pin
static int pinAcceleratore = A0; //pin analogico deputato a leggere i valori del potenziometro
static int mDXavanti = 12; //pin digitale per determinare gli stati logici da inviare al modulo
static int mDXdietro = 13; //pin digitale per determinare gli stati logici da inviare al modulo
static int pinPotenzaR = 11; //pin digitale tramite il quale inviare un segnale di tipo PWM tramite la funzione analgWrite()
static int mSXdietro = 8; //pin digitale per determinare gli stati logici da inviare al modulo
static int mSXavanti = 10; //pin digitale per determinare gli stati logici da inviare al modulo
static int pinPotenzaL = 9; //pin digitale tramite il quale inviare un segnale di tipo PWM tramite la funzione analgWrite()
const int start=4; //pin digitale per abilitare detect ed inseguimento, interruttore fisico sul robot
int StartEnable; // enable dei motori
int delay_sleep=2000;
int delay_add;

//variabili usate per gestire e mostrare i valori di operaizone
int accelerazione;  //valore letto dal pin A0, non utilizzato al momento
int potenza;  //valore in uscita dal pin 11

// Parametri legge di controllo proporzionale ADD
float Kv1=0.2; // costante di proporzionalità di velocità
float Kv2=0.001; //da determinare con più precisione
float v1;
float v2;
float vR=0; // vcalore da 0 a maxVr
float vL=0; // valore da 0 a maxVl
int max_Vr=35; // upper bound del del mapping motori 
int max_Vl=35;
int min_V =20; // valore minimo di potenza passabile al robot (al di sotto di essa il robot non si muove)
int last_back =0;
int flag = 1;
int ret ;  // valore di ritorno per il calcolo del valore di rotazione rispetto al centro dell immagine ESP
int potenzaL; //valore in uscita dal pin 11 
int potenzaR; //

//Test serial
int inByte; 
int var_in[3]; // array di serial input 
int T_samp=0; // tempo di campionamento 
unsigned long long old_t=0; // tempo restituito da millis per definire il tempo di attivita dell arduino e poter mandare i pacchetti (BUG : limite su la grandezza del dato  )
int k =3;


int lo_x=320; // largezza della risoluzione del robot  
int gap ; // differenza fra il centro dell immagine e l'oggetto 
int gap_rotate = 75; //limite di rotazione (se mi trovo in +20 o -20 il robot non dovrà ruotare ) 
int d_ref=40;
int d_min=20;
//Parametri I2C communication



//=================================
void setup() {

  Serial.begin(115200);
  Serial.println("\nStarting I2C SwapRoles demo\n");
  
  // setup della comunicazione  I2C
  Wire.begin(thisAddress); // join i2c bus
  Wire.onReceive(receiveEvent); // register function to be called when a message arrives

  //inizializzo variabili
  potenza = 0;
  //definisco tipologia pin
  pinMode(start, INPUT);//input di start detect ed inseguimento
  pinMode(pinAcceleratore, INPUT); //input in quanto legge il valore analogico del pin A0
  pinMode(mDXavanti, OUTPUT); //output perche' definisce lo stato logico del pin IN1 del modulo L298N
  pinMode(mDXdietro, OUTPUT); //output perche' definisce lo stato logico del pin IN2 del modulo L298N
  pinMode(pinPotenzaR, OUTPUT);  //output perche' definisce il valore PWM del pin EN1 del modulo L298N
  pinMode(mSXdietro, OUTPUT); //output perche' definisce lo stato logico del pin IN1 del modulo L298N
  pinMode(mSXavanti, OUTPUT); //output perche' definisce lo stato logico del pin IN2 del modulo L298N
  pinMode(pinPotenzaL, OUTPUT);  //output perche' definisce il valore PWM del pin EN1 del modulo L298N


  //pullDown sensore pinAcceleratore
  digitalWrite(start, LOW);
  //Definisco il senso di marcia del motore
  /*
    mA |   mB  | Evento
    -----|-------|----------------------
    LOW  | LOW   | fermo
    LOW  | HIGH  | Movimento in un senso
    HIGH | LOW   | Movimento senso opposto
    HIGH | HIGH  | Fermo
  */

 digitalWrite(mDXavanti, LOW);
 digitalWrite(mDXdietro, LOW);
 digitalWrite(mSXdietro, LOW);
 digitalWrite(mSXavanti, LOW);
}

void loop() {
    //parte di codice che fa il check di ricevimento informazioni e le stampa .
  if (newRxData == true) {
      showNewData(); 
      newRxData = false;
  }
 //StartEnable=rxData.start; //decommentare per azionare il robot da MATLAB
 StartEnable = 1;
 if(StartEnable){ // se viene inviato un segnale di enable questo if funziona 
    vR=max_Vl;
    vL=max_Vr;

  // filtro digitale sanifica buffer seriale  
   byte_serial  = Wire.available(); // restituisce il numero di byte accodati nel I2C 
   trash_byte = byte_serial-pkt_size ; 
   if(trash_byte>0){
     for(int i  = 0 ; i<trash_byte ; i++){ // for che consuma i byte obsoleti su seriale 
       Wire.read(); 
     }
   }
  read_pot =  analogRead(0);
  vR = map(read_pot, 0,1023, 0, max_Vr ); 
  read_pot =  analogRead(0);
  vL= map(read_pot, 0,1023, 0, max_Vl );
  vR = constrain(vR , 12 , max_Vr);
  vL = constrain(vL , 12 , max_Vl);
  read_pot =  analogRead(1);
  Kv2 = map(read_pot, 0,1023, 0, max_Vr);  

  read_pot =  analogRead(2);
  delay_add = map(read_pot, 0,1023, 0, 3000); 
  
  Serial.print( delay_add);
  Serial.print("\n");
  Serial.print(Kv1);
  Serial.print("\n");
  Serial.print(Kv2);
  Serial.print("\n");

    // porzione di codice utile a discriminare le incertezze di dei dati passati in input dall monitor seriale dell ESP . 
    // Es. nella serial vengono letti 2 o piu zeri dovuti a qualche problema allora viene non torno in fase di scanning di ambiente
    if ((gap <= 0) && rxData.Xcm == 0 ){ 
      if(k!=0){ 
        delay(1000);
        k--;
      }else{ // rotazione del robot per scansionare l ambiente (stato raggiunto ogni qual volta che vengono passati degli zeri in input)
        k=2;
        digitalWrite(mDXavanti, LOW); // rotazione in senso orario 
        digitalWrite(mDXdietro, HIGH);  
        digitalWrite(mSXavanti, HIGH);
        digitalWrite(mSXdietro, LOW); 
        move_robot();
        delay(300);
        digitalWrite(mDXavanti, HIGH);  // rotazione in senso antiorario
        digitalWrite(mDXdietro, LOW);  
        digitalWrite(mSXavanti, LOW);
        digitalWrite(mSXdietro, HIGH); 
        move_robot();
        delay(100);
        digitalWrite(mDXavanti, LOW);
        digitalWrite(mDXdietro, LOW);
        digitalWrite(mSXdietro, LOW);
        digitalWrite(mSXavanti, LOW);
        move_robot();
        delay(delay_add);
      }
    }else if( (gap > 0) && rxData.Xcm == 0){
      if(k!=0){ 
        delay(1000);
        k--;
      }else{
        k=2;
        digitalWrite(mDXavanti, HIGH);  // rotazione in senso antiorario
        digitalWrite(mDXdietro, LOW);  
        digitalWrite(mSXavanti, LOW);
        digitalWrite(mSXdietro, HIGH); 
        move_robot();
        delay(300);
        digitalWrite(mDXavanti, LOW); // rotazione in senso orario 
        digitalWrite(mDXdietro, HIGH);  
        digitalWrite(mSXavanti, HIGH);
        digitalWrite(mSXdietro, LOW); 
        move_robot();
        delay(100);
        digitalWrite(mDXavanti, LOW);
        digitalWrite(mDXdietro, LOW);
        digitalWrite(mSXdietro, LOW);
        digitalWrite(mSXavanti, LOW);
        move_robot();
        delay(delay_add);
      }
    }else{ 
         func_forward(); // il robot cammina 
         if(rxData.Xcm == 0){;} // utile a tenere l ultimo valore gap calcolato 
         else {gap = (lo_x/2)-rxData.Xcm ;} 
         vL = map(abs(gap), 0, 160, min_V, max_Vl-8 ); 
         vR = vL;

         if((abs(gap)>gap_rotate)&&(gap<0)){ // ruota in senso orario
            digitalWrite(mDXavanti, LOW);
            digitalWrite(mDXdietro, HIGH);  
            digitalWrite(mSXavanti, HIGH);
            digitalWrite(mSXdietro, LOW);  
            move_robot();
            delay(300);
            digitalWrite(mDXavanti, HIGH);  // rotazione in senso antiorario
            digitalWrite(mDXdietro, LOW);  
            digitalWrite(mSXavanti, LOW);
            digitalWrite(mSXdietro, HIGH); 
            move_robot();
            delay(100);
            digitalWrite(mDXavanti, LOW);
            digitalWrite(mDXdietro, LOW);
            digitalWrite(mSXdietro, LOW);
            digitalWrite(mSXavanti, LOW);
            move_robot();
            delay(delay_add/2);
         }else if ((abs(gap)>gap_rotate)&&(gap>0)){
             digitalWrite(mDXavanti, HIGH);  // rotazione in senso antiorario
             digitalWrite(mDXdietro, LOW);  
             digitalWrite(mSXavanti, LOW);
             digitalWrite(mSXdietro, HIGH); 
             move_robot();
             delay(300);
             digitalWrite(mDXavanti, LOW);
             digitalWrite(mDXdietro, HIGH);  
             digitalWrite(mSXavanti, HIGH);
             digitalWrite(mSXdietro, LOW);  
             move_robot();
             delay(100);
             digitalWrite(mDXavanti, LOW);
             digitalWrite(mDXdietro, LOW);
             digitalWrite(mSXdietro, LOW);
             digitalWrite(mSXavanti, LOW);
             move_robot();
             delay(delay_add/2);
             
          }else{;}   
    }if( rxData.distance < d_min && rxData.distance > 1 ){
      if(last_back!=0){ // evita che valori obsoleti facciano andare i robot indietro piu del dovuto 
        last_back--;
        delay(1000);
      }
      last_back =3;
      vR=35;
      vL=35;
      digitalWrite(mDXavanti, LOW); // manda indietro il robot 
      digitalWrite(mDXdietro, HIGH);
      digitalWrite(mSXdietro, HIGH);
      digitalWrite(mSXavanti, LOW);
      move_robot();
      delay(500);
      vR=15;
      vL=15;
      digitalWrite(mDXavanti, HIGH); // manda avanti il robot 
      digitalWrite(mDXdietro, LOW);
      digitalWrite(mSXdietro, LOW);
      digitalWrite(mSXavanti, HIGH);
      move_robot();
      delay(200);
      digitalWrite(mDXavanti, LOW);
      digitalWrite(mDXdietro, LOW);
      digitalWrite(mSXdietro, LOW);
      digitalWrite(mSXavanti, LOW);   
    }



  }else{ // se non viene mandato il segnale di enable allora  disarmo i motori
      digitalWrite(mDXavanti, LOW);
      digitalWrite(mDXdietro, LOW);
      digitalWrite(mSXdietro, LOW);
      digitalWrite(mSXavanti, LOW);    
  }
}  


void move_robot(){
  //Bisogna mappare correttamente la potenza. Trovare relazione omega/tensione
  vR=abs(vR);
  vL=abs(vL);
  potenzaL = map(vL, 0, max_Vl, 0, 255); 
  potenzaR = map(vR, 0, max_Vr, 0, 255);  

  //Valori max e min della potenza (deve stare sempre tra 0 e 255)
  if(potenzaL>255){
    potenzaL=255;
  }
  if(potenzaR>255){
    potenzaR=255;
  }
  if(potenzaL<0){
    potenzaL=0;
  }   
  if(potenzaR<0){
    potenzaR=0;
  }  
  analogWrite(pinPotenzaL, potenzaL);
  analogWrite(pinPotenzaR, potenzaR);
}


void func_forward(){

 if (rxData.distance  > d_ref && rxData.distance  < 1500) { // se la distanza rilevata e minore di 1500 cm va avanti 
      vR=30;
      vL=30;
      digitalWrite(mDXavanti, HIGH); // manda avanti il robot 
      digitalWrite(mDXdietro, LOW);
      digitalWrite(mSXdietro, LOW);
      digitalWrite(mSXavanti, HIGH);
      move_robot();
      delay(100);
 
  }else{ 
       digitalWrite(mDXavanti, LOW); // disattiva i motori 
       digitalWrite(mDXdietro, LOW);
       digitalWrite(mSXdietro, LOW);
       digitalWrite(mSXavanti, LOW);
       move_robot();
  }
}

void showNewData() {
    Serial.print("This just in    ");
    Serial.print(rxData.Xcm);
    Serial.print(' ');
    Serial.print(rxData.Ycm);
    Serial.print(' ');
    Serial.print(rxData.distance );
    Serial.print(' ');
    Serial.print(rxData.dmin);
    Serial.print(' ');
    Serial.print(rxData.start);
    Serial.print(' ');
    Serial.print("velocita: ");
    Serial.print("VR: ");
    Serial.print(vR);
    Serial.print(" ");
    Serial.print("VL: ");
    Serial.print(vL); 
    Serial.print(" ");
    Serial.print("V2: ");
    Serial.print(v2);
    Serial.print(" ");
    Serial.print("V1: ");
    Serial.print(v1);
    Serial.print("\n");
    Serial.print("potenzaR: ");
    Serial.print(potenzaR);
    Serial.print(' ');
    Serial.print("potenzaL: ");
    Serial.print(potenzaL); 
    Serial.print("\n");
 
}

// questa funzione è chiamata dalla librerire wire quando un messaggio e ricevuto 
void receiveEvent(int numBytesReceived) {
    if (newRxData == false) {
        Wire.readBytes( (byte*) &rxData, numBytesReceived); // legge i  byte e li inserisce in un array 
        newRxData = true; // set in true per stampare i dati 
        
    }else {
        while(Wire.available() > 0) { 
            byte c = Wire.read();
        }
    }
}       
