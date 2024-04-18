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
const int chipSelectPin = 5; // Pino do ESP32 conectado ao chip select (CS) do módulo do cartão SD const int presFreDiaPin = 36; // Pino do potenciômetro da pressão de fluido de freio dianteiro const int presFreTraPin = 39; // Pino do potenciômetro da pressão de fluido de freio traseiro const int suspDiaDirPin = 34; // Pino do potenciômetro da suspensão dianteira da direita const int suspDiaEsqPin = 35; // Pino do potenciômetro da suspensão dianteira da esquerda const int suspTraDirPin = 32; // Pino do potenciômetro da suspensão traseira da direita const int suspTraEsqPin = 33; // Pino do potenciômetro da suspensão traseira da esquerda const int suspPosVolPin = 25; // Pino do potenciômetro da posição do volante 
int presFreDia, presFreTra, suspDiaDir, suspDiaEsq, suspTraDir, suspTraEsq, suspPosVol; 
Adafruit_MPU6050 mpu; 
RTC_DS3231 rtc; 
File dataFile; 
DateTime now; // Variável global para armazenar a data e hora 
sensors_event_t a, g, temp; // Variáveis globais para os dados do sensor 
// Define variáveis globais para armazenar os dados do MPU6050 
float accelX, accelY, accelZ; 
float gyroX, gyroY, gyroZ; 
float tempC;
unsigned long previousMillis = 0; // Variável para armazenar o tempo da última leitura const long interval = 80; // Intervalo desejado entre leituras em milissegundos int readingsCount = 0; // Contador de leituras por segundo 
int previousSecond = 0; // Variável para armazenar o tempo anterior void setup(void) { 
// Inicializa a comunicação serial 
Serial.begin(115200); 
while (!Serial) 
delay(10); 
// Inicialização dos componentes 
if (!rtc.begin()) { 
Serial.println("Falha ao inicializar o RTC"); 
while (1) { 
delay(10); 
} 
} 
if (!mpu.begin(0x69)) { // Endereço definido apenas com VCC no pino AD0 do MPU6050 Serial.println("Falha ao inicializar o MPU6050."); 
while (1) { 
delay(10); 
} 
} 
if (!SD.begin(chipSelectPin)) { 
Serial.println("Falha ao inicializar o cartão SD."); 
while (1) { 
delay(10); 
} 
} 
mpu.setAccelerometerRange(MPU6050_RANGE_8_G); //Define a faixa de medição do acelerômetro mpu.setGyroRange(MPU6050_RANGE_500_DEG); //Define a faixa de medição do giroscópio  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); //Define a largura de banda do filtro do sensor 
// Criar uma string para armazenar o nome do arquivo 
String fileName = "/datalogger_"; 
// Obter a data e hora atual do RTC 
DateTime now = rtc.now(); 
// Adicionar ano, mês, dia, hora, minuto e segundo ao nome do arquivo 
fileName += now.year(); 
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
fileName += ".txt"; 
// Abrir o arquivo com o nome criado para escrita 
dataFile = SD.open(fileName, FILE_WRITE); 
if (dataFile) { 
Serial.println("Arquivo aberto. Salvando dados..."); 
} else { 
Serial.println("Erro ao abrir o arquivo."); 
return; 
} 
if (rtc.lostPower()) { 
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
// Realiza a calibração do acelerômetro: 
// calibrateAccel(); 
// Realiza a calibração do giroscópio: 
// calibrateGyro(); 
delay(10); 
} 
void RTC() { 
// Atualiza a variável global 'now' com a data e hora atuais 
now = rtc.now(); 
DateTime now = rtc.now(); 
Serial.print(now.year()); Serial.print(" "); 
Serial.print(now.month()); Serial.print(" "); 
Serial.print(now.day()); Serial.print(" "); 
Serial.print(now.hour()); Serial.print(" "); 
Serial.print(now.minute()); Serial.print(" "); 
Serial.print(now.second()); Serial.print(" ");
} 
void MPU() { 
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
Serial.print(a.acceleration.x); Serial.print(" "); 
Serial.print(a.acceleration.y); Serial.print(" "); 
Serial.print(a.acceleration.z); Serial.print(" "); 
// Mostra valores do giroscópio em rad/s 
Serial.print(g.gyro.x); Serial.print(" "); 
Serial.print(g.gyro.y); Serial.print(" "); 
Serial.print(g.gyro.z); Serial.print(" "); 
// Mostra valor da temperatura da placa em degC 
Serial.print(temp.temperature); Serial.print(" "); 
} 
void Freios() { 
presFreDia = analogRead(presFreDiaPin); // Lê o valor do potenciômetro da pressão de fluido de freio dianteiro presFreTra = analogRead(presFreTraPin); // Lê o valor do potenciômetro da pressão de fluido de freio traseiro 
Serial.print(presFreDia); Serial.print(" "); 
Serial.print(presFreTra); Serial.print(" "); 
} 
void Suspensao() { 
suspDiaDir = analogRead(suspDiaDirPin); // Lê o valor do potenciômetro da suspensão dianteira da direita suspDiaEsq = analogRead(suspDiaEsqPin); // Lê o valor do potenciômetro da suspensão dianteira da esquerda suspTraDir = analogRead(suspTraDirPin); // Lê o valor do potenciômetro da suspensão traseira da direita suspTraEsq = analogRead(suspTraEsqPin); // Lê o valor do potenciômetro da suspensão traseira da esquerda suspPosVol = analogRead(suspPosVolPin); // Lê o valor do potenciômetro da posição do volante 
Serial.print(suspDiaDir); Serial.print(" "); 
Serial.print(suspDiaEsq); Serial.print(" "); 
Serial.print(suspTraDir); Serial.print(" ");
Serial.print(suspTraEsq); Serial.print(" "); 
Serial.print(suspPosVol); Serial.print(" "); 
} 
void microSD() { 
if (dataFile) { 
dataFile.print(now.year()); dataFile.print(' '); 
dataFile.print(now.month()); dataFile.print(' '); 
dataFile.print(now.day()); dataFile.print(' '); 
dataFile.print(now.hour()); dataFile.print(' '); 
dataFile.print(now.minute()); dataFile.print(' '); 
dataFile.print(now.second()); dataFile.print(' '); 
dataFile.print(accelX); dataFile.print(' '); 
dataFile.print(accelY); dataFile.print(' '); 
dataFile.print(accelZ); dataFile.print(' '); 
dataFile.print(gyroX); dataFile.print(' '); 
dataFile.print(gyroY); dataFile.print(' '); 
dataFile.print(gyroZ); dataFile.print(' '); 
dataFile.print(tempC); dataFile.print(' '); 
dataFile.print(presFreDia); dataFile.print(' '); 
dataFile.print(presFreTra); dataFile.print(' '); 
dataFile.print(suspDiaDir); dataFile.print(' '); 
dataFile.print(suspDiaEsq); dataFile.print(' '); 
dataFile.print(suspTraDir); dataFile.print(' '); 
dataFile.print(suspTraEsq); dataFile.print(' '); 
dataFile.print(suspPosVol); dataFile.print(' '); 
dataFile.println(); // Pula linha para proxima leitura 
dataFile.flush(); // Força a gravação dos dados no arquivo 
} else { 
Serial.println("Erro ao escrever no arquivo."); 
} 
} 
void loop() { 
DateTime now = rtc.now(); // Obtém a hora atual do RTC 
// Verifica se mudou o segundo para reiniciar o contador de leituras 
if (now.second() != previousSecond) { 
previousSecond = now.second(); 
readingsCount = 0; 
} 
// Verifica se ainda não atingiu 10 leituras no segundo atual 
if (readingsCount < 10) { 
unsigned long currentMillis = millis(); // Obtém o tempo atual em milissegundos if (currentMillis - previousMillis >= interval) { // Verifica se é hora de fazer a próxima leitura
previousMillis = currentMillis; // Atualiza o tempo da última leitura 
RTC(); // Atualiza 'now' com os dados do RTC 
MPU(); // Atualiza 'a', 'g', 'temp' com os dados do sensor MPU6050 Freios(); // Atualiza dados referentes aos freios 
Suspensao(); // Atualiza dados referentes a suspensao 
microSD(); // Salva os dados no cartão SD 
Serial.println(); // Pula linha para próxima leitura 
readingsCount++; // Incrementa o contador de leituras 
} 
} 
} 
