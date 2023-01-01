#include <ArduinoHttpClient.h>
#include <WiFi.h>
#include <driver/adc.h>
#include "arduino_secrets.h"
#include "SparkFun_Deploy_IMU_Library.h"
DEPLOY_IMU myIMU;
#include <SPI.h>
byte address=0x11;
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

char serverAddress[] = SECRET_ADDRESS;

int send_id = 3;// Left
int port = 22961;// Left
byte ps_data[251] = {byte(3)}; // Left

// int send_id = 2; //Right
// int port = 22960; // Right
// byte ps_data[251] = {byte(2)};// Right

int lines_ct = 3;
int decoders_ct = 8;
int decoders_num = 0;
int ps_ct = 0;
int imu_ct = 0;
int row_ct1 = 4;
int col_ct1 = 1;

int row_lines[3] = {27, 26, 25};
int col_lines[3] = {14, 15, 13};
int row_decoders[4] = {16, 5, 33, 32};

int biggest_readout = 0;
int step_ct = 0;
int cur_pot = 200;
int adjust_ct = 0;

int decoding[8][3] = {
  {0, 0, 0},
  {1, 0, 0},
  {0, 1, 0},
  {1, 1, 0},
  {0, 0, 1},
  {1, 0, 1},
  {0, 1, 1},
  {1, 1, 1},
};

uint16_t ground_analog;
WiFiClient wifi;
WebSocketClient client = WebSocketClient(wifi, serverAddress, port);

static const int spiClk = 1000000; // 1 MHz
SPIClass * vspi = NULL;
#define VSPI_MISO   MISO
#define VSPI_MOSI   MOSI
#define VSPI_SCLK   SCK
#define VSPI_SS     17

void setup() {

  Serial.begin(115200);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
  
  Wire.begin();
  if (myIMU.begin() == false)
  {
    Serial.println("IMU not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }
  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.enableLinearAccelerometer(1);  // m/s^2 no gravity
  myIMU.enableGyroIntegratedRotationVector(1); // quat

  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Establishing connection to WiFi..");
  }

  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  for (int i = 0; i < lines_ct; i++) {
    pinMode(col_lines[i], OUTPUT);
    pinMode(row_lines[i], OUTPUT);
    digitalWrite(col_lines[i], LOW);
    digitalWrite(row_lines[i], LOW);
  }

  for (int j = 0; j < decoders_ct; j++) {
    pinMode(row_decoders[j], OUTPUT);
    digitalWrite(row_decoders[j], HIGH);
  }

  pinMode(34,INPUT);
  
  vspi = new SPIClass(VSPI);
  Serial.println("1");
  vspi->begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS);
  pinMode(vspi->pinSS(), OUTPUT);

  Serial.println("2");
  spiCommand(vspi, cur_pot);
  delay(1000);
  Serial.println("3");

}

void loop() {
  
  Serial.println("starting WebSocket client");
  client.begin();

  
  while (client.connected()) {
    
    ps_ct = 1;
    imu_ct = 0;

    for (int j = 0; j < row_ct1; j++) {


      if (myIMU.dataAvailable() == true)

      {

          float ax, ay, az;
          byte linAccuracy = 0;

          myIMU.getLinAccel(ax, ay, az, linAccuracy);

          client.beginMessage(TYPE_TEXT);
          client.print(send_id);
          client.print(",");
          client.print(ax);
          client.print(",");
          client.print(ay);
          client.print(",");
          client.print(az);
          client.endMessage();
          
          float quatI = myIMU.getQuatI();
          float quatJ = myIMU.getQuatJ();
          float quatK = myIMU.getQuatK();
          float quatReal = myIMU.getQuatReal();
          float gyroX = myIMU.getFastGyroX();
          float gyroY = myIMU.getFastGyroY();
          float gyroZ = myIMU.getFastGyroZ();

          client.beginMessage(TYPE_TEXT);
          client.print(send_id);
          client.print(",");
          client.print(gyroX);
          client.print(",");
          client.print(gyroY);
          client.print(",");
          client.print(gyroZ);
          client.print(",");
          client.print(quatI);
          client.print(",");
          client.print(quatJ);
          client.print(",");
          client.print(quatK);
          client.print(",");
          client.print(quatReal);

          client.endMessage();
      }

      digitalWrite(row_decoders[j], LOW);

      if (j == 0) { decoders_num = 7; }
      else { decoders_num = 8; }

      for (int l = 0; l < decoders_num; l++) {

        digitalWrite(row_lines[0], decoding[l][0]);
        digitalWrite(row_lines[1], decoding[l][1]);
        digitalWrite(row_lines[2], decoding[l][2]);

        for (int p = 0; p < decoders_ct; p++) {

          digitalWrite(col_lines[0], decoding[p][0]);
          digitalWrite(col_lines[1], decoding[p][1]);
          digitalWrite(col_lines[2], decoding[p][2]);

          ground_analog = adc1_get_raw(ADC1_CHANNEL_6);

          ps_data[ps_ct] = map(ground_analog, 1200, 4095, 0, 255);
          ps_ct += 1;
  
        }
      }
      
       client.beginMessage(TYPE_BINARY);
       for (int16_t h = j * 50; h < j * 50 + 50; h++) { client.write(ps_data[h]); }
       client.endMessage();
      
      digitalWrite(row_decoders[j], HIGH);
      
    }

    client.beginMessage(TYPE_BINARY);

    for (int16_t j = 4 * 50; j < 4 * 50 + 50; j++) { client.write(ps_data[j]); }

    client.endMessage(); 
    
    if (adjust_ct < 100){

      for (int16_t p = 1; p < 249; p++){
        if(ps_data[p] > 70 && ps_data[p] < 250 ){ step_ct += 1; }
      }

      for (int16_t k = 1; k < 249; k++){
        if(ps_data[k] > biggest_readout && ps_data[k] < 253 ){ biggest_readout = ps_data[k]; }
      }
      

      if (step_ct > 40){

        if (biggest_readout < 240 && cur_pot > 30) { cur_pot = cur_pot - 10; }
        else if (biggest_readout > 250 && cur_pot < 230) { cur_pot = cur_pot + 10; }
        
        adjust_ct = adjust_ct + 1;
        spiCommand(vspi, cur_pot);
        Serial.print(adjust_ct);
        Serial.print(",");
        Serial.print(cur_pot);

      }

      biggest_readout = 0;
      step_ct = 0;
      
    }


    Serial.println("");
    
  }

  Serial.println("disconnected");
}

void spiCommand(SPIClass *spi, int data) {
  //use it as you would the regular arduino SPI API
  spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(spi->pinSS(), LOW); //pull SS slow to prep other end for transfer
  spi->transfer(B00010001);
  spi->transfer(data);
  digitalWrite(spi->pinSS(), HIGH); //pull ss high to signify end of data transfer
  spi->endTransaction();
}
