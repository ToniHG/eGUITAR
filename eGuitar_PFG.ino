#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <Arduino.h>
#include <dummy.h>
//BLE Scan and advertised device
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
//BT A2DP
#include <BluetoothA2DPSink.h>
#include <BluetoothA2DPSource.h>
#include <SoundData.h>
//BT SPP
#include <BluetoothSerial.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <math.h>
//watchdog control
#include <esp_task_wdt.h>

//BT HANDLER
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//Enum type for notes 
typedef enum{
  NOTE_c, NOTE_cs, NOTE_d, NOTE_eb, NOTE_e, NOTE_f, NOTE_fs, NOTE_g, NOTE_gs, NOTE_a, NOTE_bb, NOTE_b, NOTE_max
}note_gt;

//Frecuency for each note in the last octave
const double noteFreq[12] = {
//   C       Cs        D        Eb       E        F        Fs       G        Gs       A        Bb       B
  4186.01, 4434.92, 4698.63, 4978.03, 5274.04, 5587.65, 5919.91, 6271.93, 6644.88, 7040.00, 7458.62, 7902.13
};

//Structs for I/O strings
struct stringGuitar {
  String namest;      //Name assigned by the standard tuning of the guitar
  uint8_t PINp;       //Pin assigned to lineal potentiometer
  uint8_t PINpr;      //Pin assigned to photoresistor
  uint8_t stringPos;  //Position in the guitar
  bool touching;      //Variable of control
  note_gt notes[5];   //Each note in order of frets
  uint8_t octaves[5]; //Octave of each note 
  note_gt note;       //Note to play
  uint8_t octave;     //Octave of the note to play
};

//BT SERIAL AND BLE (MENU AND SCAN)
int numDev;              //Number of devices detected
String adDev[15]={""};   //Address of each device
String nameDev[15]={""}; //Name of each device
String remName = "SE-MJ561BT";
static uint8_t add[6] = {0x04,0x02,0xCA,0x00,0x06,0xB8};
int scanTime = 5;        //Time for scan in seconds
BLEScan* pBLEScan;       //BLE Scan
BluetoothSerial Sbt1;    //BT serial for menu

//Variable for jack and control
const uint8_t jackPIN = 16;      //Output for jack
const uint8_t ledChannelJack = 0;  //Channel of PWM for jack
const uint8_t switchJBTPIN = 5;      //PIN for switch JACK/BT
bool switchBTJack = false;         //False output by jack, true output by BT
bool playM = false;                //Control to playM by jack
const uint8_t restartPIN = 17;      //Pin for restart interrupt
bool restart = false;              //Control to restart esp32
uint8_t nextST = 0;                //Control to get next string or BT

//BT Audio (A2DP)
BluetoothA2DPSource a2dp_source;

//Tasks for get info from inputs and play music(JACK)
TaskHandle_t Task1;
TaskHandle_t Task2;

//Array of structs for the strings of the guitar
struct stringGuitar stringsGT[6];

//Constructor to load the data in the differents structs
void loadDataini(){ 
  uint8_t i;
  for(i=0 ; i < 6 ; i++){
    if(i == 0){
           stringsGT[i].namest = "mi4";
           stringsGT[i].PINp = 36;
           stringsGT[i].PINpr = 25;
           stringsGT[i].stringPos = 1;
           stringsGT[i].touching = false;
           stringsGT[i].notes[0] = NOTE_e;
           stringsGT[i].notes[1] = NOTE_f;
           stringsGT[i].notes[2] = NOTE_fs;
           stringsGT[i].notes[3] = NOTE_g;
           stringsGT[i].notes[4] = NOTE_gs;
           stringsGT[i].octaves[0] = 4;
           stringsGT[i].octaves[1] = 4;
           stringsGT[i].octaves[2] = 4;
           stringsGT[i].octaves[3] = 4;
           stringsGT[i].octaves[4] = 4;
    }
    else if(i == 1){
           stringsGT[i].namest = "si3";
           stringsGT[i].PINp = 39;
           stringsGT[i].PINpr = 26;
           stringsGT[i].stringPos = 2;
           stringsGT[i].touching = false;
           stringsGT[i].notes[0] = NOTE_b;
           stringsGT[i].notes[1] = NOTE_c;
           stringsGT[i].notes[2] = NOTE_cs;
           stringsGT[i].notes[3] = NOTE_d;
           stringsGT[i].notes[4] = NOTE_eb;
           stringsGT[i].octaves[0] = 3;
           stringsGT[i].octaves[1] = 4;
           stringsGT[i].octaves[2] = 4;
           stringsGT[i].octaves[3] = 4;
           stringsGT[i].octaves[4] = 4;
     }
     else if(i == 2){
           stringsGT[i].namest = "sol3";
           stringsGT[i].PINp = 34;
           stringsGT[i].PINpr = 27;
           stringsGT[i].stringPos = 3;
           stringsGT[i].touching = false;
           stringsGT[i].notes[0] = NOTE_g;
           stringsGT[i].notes[1] = NOTE_gs;
           stringsGT[i].notes[2] = NOTE_a;
           stringsGT[i].notes[3] = NOTE_bb;
           stringsGT[i].notes[4] = NOTE_b;
           stringsGT[i].octaves[0] = 3;
           stringsGT[i].octaves[1] = 3;
           stringsGT[i].octaves[2] = 3;
           stringsGT[i].octaves[3] = 3;
           stringsGT[i].octaves[4] = 4;
     }
     else if(i == 3){
           stringsGT[i].namest = "re3";
           stringsGT[i].PINp = 35;
           stringsGT[i].PINpr = 14;
           stringsGT[i].stringPos = 4;
           stringsGT[i].touching = false;
           stringsGT[i].notes[0] = NOTE_d;
           stringsGT[i].notes[1] = NOTE_eb;
           stringsGT[i].notes[2] = NOTE_e;
           stringsGT[i].notes[3] = NOTE_f;
           stringsGT[i].notes[4] = NOTE_fs;
           stringsGT[i].octaves[0] = 3;
           stringsGT[i].octaves[1] = 3;
           stringsGT[i].octaves[2] = 3;
           stringsGT[i].octaves[3] = 3;
           stringsGT[i].octaves[4] = 3;
     }
     else if(i == 4){
           stringsGT[i].namest = "la2";
           stringsGT[i].PINp = 32;
           stringsGT[i].PINpr = 12;
           stringsGT[i].stringPos = 5;
           stringsGT[i].touching = false;
           stringsGT[i].notes[0] = NOTE_a;
           stringsGT[i].notes[1] = NOTE_bb;
           stringsGT[i].notes[2] = NOTE_b;
           stringsGT[i].notes[3] = NOTE_c;
           stringsGT[i].notes[4] = NOTE_cs;
           stringsGT[i].octaves[0] = 2;
           stringsGT[i].octaves[1] = 2;
           stringsGT[i].octaves[2] = 2;
           stringsGT[i].octaves[3] = 2;
           stringsGT[i].octaves[4] = 3;
     }
     else if(i == 5){
           stringsGT[i].namest = "mi2";
           stringsGT[i].PINp = 33;
           stringsGT[i].PINpr = 13;
           stringsGT[i].stringPos = 6;
           stringsGT[i].touching = false;
           stringsGT[i].notes[0] = NOTE_e;
           stringsGT[i].notes[1] = NOTE_f;
           stringsGT[i].notes[2] = NOTE_fs;
           stringsGT[i].notes[3] = NOTE_g;
           stringsGT[i].notes[4] = NOTE_gs;
           stringsGT[i].octaves[0] = 2;
           stringsGT[i].octaves[1] = 2;
           stringsGT[i].octaves[2] = 2;
           stringsGT[i].octaves[3] = 2;
           stringsGT[i].octaves[4] = 2;
    }
  }
}

//Function to change restart status
void IRAM_ATTR switchBTJACK(){
  Serial.println("LOW");
  if(digitalRead(switchJBTPIN) == LOW){
    Serial.println("LOW");
    switchBTJack = false;
  }
  else{
    Serial.println("HIGH");
    switchBTJack = true;
  }
}

//Function to change restart status
void IRAM_ATTR restartESP(){
  Serial.println("Changing restart status");
  restart = true;
}

//Function for config for inputs fo the strings
void configIO(){
  uint8_t i;
  //Config for inputs of the strings
  for(i=0 ; i < 6 ; i++){
    pinMode(stringsGT[i].PINp, INPUT);
    pinMode(stringsGT[i].PINpr, INPUT);
  }
  //Config jack output PWM
  pinMode(jackPIN, OUTPUT);
  ledcAttachPin(jackPIN,ledChannelJack);
  //Config for switch
  pinMode(switchJBTPIN, INPUT);
  attachInterrupt(switchJBTPIN,switchBTJACK,CHANGE);
  //Config restart
  pinMode(restartPIN,INPUT_PULLUP);
  attachInterrupt(restartPIN,restartESP,FALLING);
}

//Callback function for BLEScan 
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks { //Cada vez que se detecta un dev nuevo se llama a esta funcion
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      if(numDev < 15){
        adDev[numDev] = advertisedDevice.getAddress().toString().c_str();
        Serial.println(adDev[numDev]);
        nameDev[numDev] = advertisedDevice.getName().c_str();
        Serial.println(nameDev[numDev]);
        numDev++;
      }
    }
};

//Function to scan devices
void scan(){
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value
  Serial.println("BLE configured");
  Serial.println("Scanning...");
  Sbt1.println("Scanning...");
  BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
  Serial.print("Devices found: ");
  Serial.println(String(numDev));
  Serial.println("Scan done!");
  Sbt1.println("Scan done!");
  pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory
}

//Function to clean data of devices
void cleanData(){
  int i;
  Serial.println("Cleaning data");
  numDev = 0;
  for(i = 0; i < nameDev[0].length(); i++){
    nameDev[i]="";
    adDev[i]="";
  }
}

//Function to show data of devices
void showDevs(){
  int i;
  Serial.println("Showing info");
  Sbt1.println("Showing info");
  for(i = 0 ; i < numDev ; i++){
    String mess = "Device " + String(i) + ": \tAddress: " +  adDev[i] + "\t/ Name: " + nameDev[i];
    Serial.println(mess);
    Sbt1.println(mess);
  }
}

//Funtcion to get frequency for BT
double get_actual_frequency(){
  double frequency = 0.0;
  if(!switchBTJack || !playM){
    return frequency;
  }
  else{
    if(stringsGT[nextST].touching){
      frequency = (double)noteFreq[stringsGT[nextST].note] / (double)(1 << (8 - stringsGT[nextST].octave));
    }
    else{
      frequency = 0.0;
    }
    nextST++;
    if(nextST == 6){
      nextST = 0;
    }
  }
  return frequency;
}

//Callback for a2dp source BT
int32_t get_data_channels(Channels *channels, int32_t channel_len) {
  static double m_time = 0.0;
  double m_amplitude = 10000.0;  // -32,768 to 32,767
  double m_deltaTime = 1.0 / 44100.0;
  double m_phase = 0.0;
  double double_Pi = PI * 2.0;
  // fill the channel data
  for (int sample = 0; sample < channel_len; ++sample) {
    double actual_frequency = get_actual_frequency();
    double angle = double_Pi * actual_frequency * m_time + m_phase;
    channels[sample].channel1 = m_amplitude * sin(angle);
    channels[sample].channel2 = channels[sample].channel1;
    m_time += m_deltaTime;
  }
  return channel_len;
}

//Function to connect a selected device
void connectToOtherDev(){
  a2dp_source.setResetBLE(false);
  esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
  Sbt1.println("Introduce el nombre del dispositivo.");
  delay(10000);
  char *nombre = "WONDERBOOM TONIK";
  String name_dev = "";
  char *name_dv;
  do{
    if(Sbt1.available()){
      name_dev += (char)Sbt1.read(); 
    }
    esp_task_wdt_reset();
  }while(Sbt1.available() && switchBTJack);
  name_dev.toCharArray(name_dv,sizeof(name_dev));
  a2dp_source.start(name_dv,get_data_channels);//Inicializa el modulo A2DP source
  delay(1000);
  if(a2dp_source.isConnected()){
    Serial.println("Conectado.");
  }
}

//Function to display BT Menu
void menuDisplayBT(){
  char entrada=NULL;
  if(Sbt1.hasClient()){
    Sbt1.println("Loop principal");
    Serial.println("Hay cliente conectado.");
    do{
      Sbt1.println("¡Hola! Te has conectado satisfactoriamente.");
      Sbt1.println("Introduce un 1 si quieres escanear los dispositivos disponibles.");
      Sbt1.println("Introduce un 2 si quieres los dispositivos escaneados los dispositivos disponibles.");
      Sbt1.println("Introduce un 3 si quieres seleccionar un dispositivo para reproducir el audio de la guitarra.");
      Sbt1.println("Introduce un 4 si quieres desconectarte, cambiar al modo de jack y salir del menú.");
      delay(5000);
      if(Sbt1.available()){
        entrada = Sbt1.read();
        Sbt1.write(entrada);
        Sbt1.write(10);
        Serial.println(String(entrada));
        Sbt1.flush();
      }
      else if(!Sbt1.available() || entrada < 49 || entrada > 52){
        Sbt1.println("Disculpa, pero no has enviado una opcion valida.");
        Sbt1.flush();
      }
    }while((entrada < 49 || entrada > 52) && Sbt1.hasClient());
    Sbt1.flush();
    switch(entrada){
      case '1':
        cleanData();
        scan();
        break;
      case '2':
        showDevs();
        break;
      case '3':
        connectToOtherDev();
        break;
      case '4':
        Sbt1.println("¡Hasta luego! Se han quedado guardados los cambios realizados, se procede a salir del menuBT y cargar la opcion JACK.");
        Serial.println("¡Hasta luego! Se han quedado guardados los cambios realizados, se procede a salir del menuBT y cargar la opcion JACK.");
        switchBTJack = false;
        nextST = 0;
        esp_a2d_source_deinit();
        break;
      default:
        Sbt1.println("No se ha recibido una opcion valida o se ha desconectado.");
        Serial.println("No se ha recibido una opcion valida o se ha desconectado.");
        break;
    }
  }
  else{
    Serial.println("No hay cliente conectado.");
  }
}

//Function to get note and octave
void IRAM_ATTR getNoteOct(uint8_t i){ //Only first 4 notes of each string
  uint16_t valuePot = analogRead(stringsGT[i].PINp);
  uint8_t fret = 6;
  if(0 <= valuePot && valuePot <= 200){
    Serial.println("0");
    fret = 0;
  }
  else if(3072 <= valuePot && valuePot <= 4095){
    Serial.println("1");
    fret = 1;
  }
  else if(2048 <= valuePot && valuePot <= 3071){
    Serial.println("2");
    fret = 2;
  }
  else if(1024 <= valuePot && valuePot <= 2047){
    Serial.println("3");
    fret = 3;
  }
  else if(201 <= valuePot && valuePot <= 1023){
    Serial.println("4");
    fret = 4;
  }
  else{
    fret = 5;
  }
  //Serial.println("Fret: " + String(fret) + "POT: " + String(valuePot));
  if(0 <= fret && fret <= 4){
    stringsGT[i].note = stringsGT[i].notes[fret];
    stringsGT[i].octave = stringsGT[i].octaves[fret];
    stringsGT[i].touching = true;
    playM = true;
    //Serial.println("Note: " + String(stringsGT[i].note) + " / Oct: " + String(stringsGT[i].octave));
  }
  else{
    stringsGT[i].touching = false;
  }
}

//Code for task1 and get strings and notes
void IRAM_ATTR readStringValues(void* parameter)
{
  for(;;){
    int i;
    playM = false;
    for(i = 0 ; i < 6 ; i++){
      if(digitalRead(stringsGT[i].PINpr) == LOW){
        getNoteOct(i);
      }
      else{
        stringsGT[i].touching = false;
      }
    }
    esp_task_wdt_reset();
    vTaskDelay(10);
  }
}

//Code for task2 and play music
void IRAM_ATTR playMusic(void* parameter){
  for(;;){
    double noteF = 0.0;
    if(playM && !switchBTJack){
      //Serial.println("Sending notes");
      int i;
      for(i = 0 ; i < 6 ; i++){
        if(stringsGT[i].touching && stringsGT[i].octave <= 8 && stringsGT[i].note < NOTE_max){
          noteF = (double)noteFreq[stringsGT[i].note] / (double)(1 << (8 - stringsGT[i].octave));
          //Serial.println("Frequency note: " + String(noteF));
          noteF = ledcWriteTone(ledChannelJack, noteF);
          delay(10);
        }
      }
    }
    else{
      ledcWriteTone(ledChannelJack, noteF);
    }
    esp_task_wdt_reset();
    vTaskDelay(10);
  }
}

//Setup function
void setup() {
  Serial.begin(115200);
  Serial.println("Setup function.");
  Sbt1.begin("ESP32testled");
  Serial.println("Serial bt configured");
  cleanData();
  
  //Functions with initial info
  loadDataini();
  configIO();
  //Creating task for core 0(reading data from inputs)
  xTaskCreatePinnedToCore(readStringValues,"rStrings",10000,NULL,1,&Task1,0);
  //Creating task for core 0(output for jack or BT)
  xTaskCreatePinnedToCore(playMusic,"pMusic",10000,NULL,1,&Task2,0);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Loop function.");
  if(restart){
    Serial.println("Restarting in 5 seconds.");
    delay(5000);
    ESP.restart();
  }
  if(switchBTJack){
    Serial.println("Entering BT menu.");
    delay(100);
    menuDisplayBT();
  }
  delay(2000);
}
