#include <Arduino.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////// // Scuderia UFABC //
// Universidade Federal do ABC //
// datalogger_rm03_v1 //
///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Hardware: Esp32-WROOM-DA Module;
// Módulo MicroSD
// MPU6050
// DS3231
// 5 x Potenciômetros lineares de 10k ohms
// 2 x Potenciômetros lineares de 01k ohms
// Software: Arduino IDE 2.2.2
// Library: Adafruit BusIO 1.14.5
// Adafruit MPU6050 2.2.6
// Adafruit Unified Sensor 1.1.14
// RTClib 2.1.3
// Última atualização: 07 de janeiro de 2024 por Carlos Saravia
///////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <RTClib.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#define chipSelectPin 5  // Pino do ESP32 conectado ao chip select (CS) do módulo do cartão SD
#define presFreDiaPin 36 // Pino do potenciômetro da pressão de fluido de freio dianteiro
#define presFreTraPin 39 // Pino do potenciômetro da pressão de fluido de freio traseiro
#define suspPosVolPin 25 // Pino do potenciômetro da posição do volante
#define interval 19      // Intervalo desejado entre leituras em milissegundos
#define dev

Adafruit_MPU6050 mpu;
RTC_DS3231 rtc;
File dataFile;
DateTime now;               // Variável global para armazenar a data e hora
sensors_event_t a, g, temp; // Variáveis globais para os dados do sensor

/// Define variáveis globais para armazenar os dados do MPU6050
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float tempC;
uint32_t previousMillis = 0; // Variável para armazenar o tempo da última leitura
uint16_t readingsCount = 0;  // Contador de leituras por segundo
uint32_t previousSecond = 0; // Variável para armazenar o tempo anterior

/// Define variáveis globais para armazenar os dados da Susp
uint32_t presFreDia = 0;
uint32_t presFreTra = 0;
uint32_t suspPosVol = 0;

void componentes()
{
  if (!rtc.begin())
  {
#ifdef dev
    Serial.println("Falha ao inicializar o RTC");
    delay(1000);
    //esp_restart();
#endif
  }
  if (!mpu.begin(0x69))
  { // Endereço definido apenas com VCC no pino AD0 do MPU6050
#ifdef dev
    Serial.println("Falha ao inicializar o MPU6050.");
// esp_restart();
#endif
  }
  if (!SD.begin(chipSelectPin))
  {
    Serial.println("Falha ao inicializar o cartão SD.");
    esp_restart();
  }
}

void configMPU()
{
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); //Define a faixa de medição do acelerômetro
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);      //Define a faixa de medição do giroscópio
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);   //Define a largura de banda do filtro do sensor
}

String nomeArquivo()
{
  // Criar uma string para armazenar o nome do arquivo
  String fileName = "/datalogger_";
  // Obter a data e hora atual do RTC
  DateTime now = rtc.now();
  // Adicionar ano, mês, dia, hora, minuto e segundo ao nome do arquivo
  fileName += String(now.year());
  fileName += "_";
  fileName += String(now.month());
  fileName += "_";
  fileName += String(now.day());
  fileName += "_";
  fileName += String(now.hour());
  fileName += "_";
  fileName += String(now.minute());
  fileName += "_";
  fileName += String(now.second());
  fileName += ".csv";
  return fileName;
}

void inicializaArquivo()
{
  dataFile = SD.open(nomeArquivo(), FILE_WRITE); // Abrir o arquivo com o nome criado para escrita
  if (dataFile)
  {
    Serial.println("Arquivo aberto. Salvando dados...");
  }
  else
  {
    Serial.println("Erro ao abrir o arquivo.");
    esp_restart();
  }
}

void verificaRTC()
{
  if (rtc.lostPower())
  {
    Serial.println("RTC perdeu energia. Defina a hora novamente.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  // Ajuste manual do horário do RTC DS3231
  // Para adiantar 1 minuto:
  // DateTime now = rtc.now();
  // rtc.adjust(DateTime(now.year(), now.month(), now.day(), now.hour(), now.minute() + 1, now.second()));
  // Ou para atrasar 1 minuto:
  // DateTime now = rtc.now();
  // rtc.adjust(DateTime(now.year(), now.month(), now.day(), now.hour(), now.minute() - 1, now.second()));
}

void calibraGiroscopio()
{
  // Realiza a calibração do acelerômetro:
  // calibrateAccel();
  // Realiza a calibração do giroscópio:
  // calibrateGyro();
}

void RTC()
{
  // Atualiza a variável global 'now' com a data e hora atuais
  now = rtc.now();
  DateTime now = rtc.now();
  Serial.print("RTC:");
  Serial.print(now.year());
  Serial.print(" ");
  Serial.print(now.month());
  Serial.print(" ");
  Serial.print(now.day());
  Serial.print(" ");
  Serial.print(now.hour());
  Serial.print(" ");
  Serial.print(now.minute());
  Serial.print(" ");
  Serial.print(now.second());
  Serial.print(" ");
}

void MPU()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  // Armazenar os valores lidos nas variáveis globais
  accelX = a.acceleration.x;
  accelY = a.acceleration.y;
  accelZ = a.acceleration.z;
  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;
  tempC = temp.temperature;
  // Mostra valores de aceleração em m/s²
  Serial.print("a:");
  Serial.print(a.acceleration.x);
  Serial.print(" ");
  Serial.print(a.acceleration.y);
  Serial.print(" ");
  Serial.print(a.acceleration.z);
  Serial.print(" ");
  // Mostra valores do giroscópio em rad/s
  Serial.print("g:");
  Serial.print(g.gyro.x);
  Serial.print(" ");
  Serial.print(g.gyro.y);
  Serial.print(" ");
  Serial.print(g.gyro.z);
  Serial.print(" ");
  // Mostra valor da temperatura da placa em degC
  Serial.print("T:");
  Serial.print(temp.temperature);
  Serial.print(" ");
}

void Freios()
{
  presFreDia = analogRead(presFreDiaPin); // Lê o valor do potenciômetro da pressão de fluido de freio dianteiro
  presFreTra = analogRead(presFreTraPin); // Lê o valor do potenciômetro da pressão de fluido de freio traseiro
  Serial.print("F:");
  Serial.print(presFreDia);
  Serial.print(" ");
  Serial.print(presFreTra);
  Serial.print(" ");
}

void Suspensao()
{
  suspPosVol = analogRead(suspPosVolPin); // Lê o valor do potenciômetro da posição do volante
  Serial.print("PV:");
  Serial.print(suspPosVol);
  Serial.print(" ");
}

void microSD()
{
  if (dataFile)
  {
    dataFile.print(now.year());
    dataFile.print(';');
    dataFile.print(now.month());
    dataFile.print(';');
    dataFile.print(now.day());
    dataFile.print(';');
    dataFile.print(now.hour());
    dataFile.print(';');
    dataFile.print(now.minute());
    dataFile.print(';');
    dataFile.print(now.second());
    dataFile.print(';');
    dataFile.print(accelX);
    dataFile.print(';');
    dataFile.print(accelY);
    dataFile.print(';');
    dataFile.print(accelZ);
    dataFile.print(';');
    dataFile.print(gyroX);
    dataFile.print(';');
    dataFile.print(gyroY);
    dataFile.print(';');
    dataFile.print(gyroZ);
    dataFile.print(';');
    dataFile.print(tempC);
    dataFile.print(';');
    dataFile.print(presFreDia);
    dataFile.print(';');
    dataFile.print(presFreTra);
    dataFile.print(';');
    dataFile.print(suspPosVol);
    dataFile.print(';');
    dataFile.println(); // Pula linha para proxima leitura
    dataFile.flush();   // Força a gravação dos dados no arquivo
  }
  else
  {
    Serial.println("Erro ao escrever no arquivo.");
  }
}

void microSDNovo()
{
  if (dataFile)
  {
    String data = String(now.year());
     data = +";";
     data = data + String(now.month());
     data = +";";
     data = data + String(now.day());
     data = +";";
     data = data + String(now.hour());
     data = +";";
     data = data + String(now.minute());
     data = +";";
     data = data + String(now.second());
     data = +";";
     data = data + String(accelX);
     data = +";";
     data = data + String(accelY);
     data = +";";
     data = data + String(accelZ);
     data = +";";
     data = data + String(gyroX);
     data = +";";
     data = data + String(gyroY);
     data = +";";
     data = data + String(gyroZ);
     data = +";";
     data = data + String(tempC);
     data = +";";
     data = data + String(presFreDia);
     data = +";";
     data = data + String(presFreTra);
     data = +";";
     data = data + String(suspPosVol);
     data = +";";
     data = +"\n";              // Pula linha para proxima leitura
    dataFile.print(data);
    dataFile.flush(); // Força a gravação dos dados no arquivo
  }
  else
  {
    Serial.println("Erro ao escrever no arquivo.");
  }
}

void setup()
{

#ifdef dev
  Serial.begin(115200); // Inicializa a comunicação serial
#endif
  componentes(); // Inicialização dos componentes
  configMPU();
  inicializaArquivo();
  verificaRTC();
  calibraGiroscopio();
  delay(10);
}

void loop()
{
  DateTime now = rtc.now(); // Obtém a hora atual do RTC

  if (now.second() != previousSecond) // Verifica se mudou o segundo para reiniciar o contador de leituras
  {
    previousSecond = now.second();
    readingsCount = 0;
  }

  if (readingsCount < 52) // Verifica se ainda não atingiu 10 leituras no segundo atual
  {
    unsigned long currentMillis = millis(); // Obtém o tempo atual em milissegundos
    if (currentMillis - previousMillis >= interval)
    {                                 // Verifica se é hora de fazer a próxima leitura
      previousMillis = currentMillis; // Atualiza o tempo da última leitura
      RTC();                          // Atualiza 'now' com os dados do RTC
      MPU();                          // Atualiza 'a', 'g', 'temp' com os dados do sensor MPU6050
      Freios();                       // Atualiza dados referentes aos freios
      Suspensao();                    // Atualiza dados referentes a suspensao
      microSD();                      // Salva os dados no cartão SD
      
      readingsCount++;                // Incrementa o contador de leituras
      Serial.print(" V/S:");
      Serial.print(readingsCount);
      Serial.println();               // Pula linha para próxima leitura
    }
  }
}
