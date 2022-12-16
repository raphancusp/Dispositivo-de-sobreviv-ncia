# DispositivoDeSobrevivencia
Uma pochete com diversos sensores e display

## Descrição
O projeto consiste em um shoulder bag com sensores que mostram a temperatura e umidade local, a turbidez da agua, a localização em 
tempo real via satélites e um LDR que liga um lâmpada para iluminação. Informações são fornecidas ao usuário por um Display LCD que cicla entre elas.

Não há necessidade de conexão com a rede pois tudo roda localmente offline.

# Demonstração

https://user-images.githubusercontent.com/110292178/208086199-2af4d621-87a4-4e71-b90c-4cbe83f5e9d7.MOV

https://user-images.githubusercontent.com/110292178/208086544-f8e3392a-385b-46fb-9ab5-a715c023f259.MOV

# Arquitetura do Projeto

![image](https://user-images.githubusercontent.com/110292178/207584974-bca790b8-b7b6-4183-9179-8f3809ff6394.png)

# Lista de Materiais

| Item | Quantidade | Link Referência |
| --- | --- | --- |
| Power Bank | 1 | -- |
| Esp32 D1 Wemos Battery | 1 | https://www.casadarobotica.com/placas-embarcadas/esp/placas/esp32-d1-suporte-bateria-18650-wifi-ble-iot-wemos-cabo |
| Sensor Turbidez | 1 | https://www.eletrogate.com/sensor-de-turbidez-arduino-para-monitoramento-de-agua |
| Módulo GPS Neo-6mv2 | 1 | https://www.eletrogate.com/modulo-gps-neo-6m-com-antena |
| Módulo DHT22 | 1 | https://www.eletrogate.com/modulo-sensor-temperatura-e-umidade-dht22 |
| Display LCD 16x2 I2C | 1 | https://www.eletrogate.com/display-lcd-16x2-i2c-backlight-azul |
| Módulo LDR | 1 | https://proesi.com.br/modulo-sensor-de-luz-com-ldr-e-com-trimpot-gbk-robotics-modulo-p13.html |
| Módulo Relé 5V | 1 | https://www.eletrogate.com/modulo-rele-1-canal-5v |
| Protoboard | x | --- |
| Jumpers | x | --- |

## Montagem
A montagem foi feita utilizando protoboard e jumpers entre os componentes.
A alimentação se dá através de um Power Bank.

## Ligações
Os pinos de ligações dos respectivos módulos são:

| Item | Pino ESP32 | Pino Módulo |
| --- | --- | --- |
| Módulo GPS | GND | GND |
| Módulo GPS | 5V | 5V |
| Módulo GPS | GPIO 17 - TX | RX |
| Módulo GPS | GPIO 16 - RX | TX |
| Sensor Turbidez | 5V | 5V |
| Sensor Turbidez | GND | GND |
| Sensor Turbidez | GPIO 13 | OUT |
| Módulo DHT | 3.3V | 3.3V |
| Módulo DHT | GND | GND |
| Módulo DHT | GPIO 12 | OUT |
| Módulo LDR | 5V | 5V |
| Módulo LDR | GND | GND |
| Módulo LDR | GPIO 14 | OUT |
| Módulo Relé | 5V | 5V |
| Módulo Relé | GND | GND |
| Módulo Relé | GPIO 27 | IN |
| Display LCD | 5V | 5V |
| Display LCD | GND | GND |
| Display LCD | GPIO 21 | SDA |
| Display LCD | GPIO 22 | SCL |

# Explicação do Código

Inclusão de bibliotecas necessárias
```C++
//Projeto Mochila - Versão 3.0 - ESP32

//Bibliotecas GPS
#include <SoftwareSerial.h>
#include <TinyGPS.h>

//Bibliotecas Display
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//Biblioteca DHT - Sensor Temp e Umid
#include "DHT.h"
```

Função para objeto GPS
```C++
//Função GPS
SoftwareSerial serial1(16, 17); // 17 no RX, 16 no TX
TinyGPS gps1;
```
Função para objeto display
```C++
//Função Display
LiquidCrystal_I2C lcd(0x27, 16, 2);
```

Variáveis DHT22
```C++
//Variáveis DHT
#define DHTPIN 12 //Pino Sensor Temp e Umid

//Defina o modelo do DHT
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
```

Função para objeto DHT
```C++
//Função DHT
DHT dht(DHTPIN, DHTTYPE);
```

Variáveis Turbidez
```C++
//Variáveis Turbidez
int SensorTurbidez = 13;
int i;
float voltagem;
float NTU;
```
Variáveis LDR
```C++
//Variáveis LDR
int ldr = 14;
int luminosidade;
```
Variáveis Relé
```C++
//Variáveis Relé
int rele = 27;
```
## Setup

Inicializa o display
```C++
void setup() 
{
  // Inicia o display LCD
  lcd.init();
  lcd.backlight();
```

Inicializa o DHT22
```C++
//Inicia o dht;
dht.begin();
```

Inicializa o serial GPS
```C++
//Inicia o serial GPS
serial1.begin(9600);
```

Inicializa a comunicação serial
```C++
//Inicia o serial port
Serial.begin(9600);
```

Mostra mensagem de início no serial
```C++
//Mensagem de início
Serial.println("GPS buscando sinal de satelites...");
```
Mostra mensagem no display
```C++
//Mostra mensagem no Display
lcd.setCursor(0, 0);
lcd.print("Bem Vindo!");
lcd.setCursor(0, 1);
lcd.print("Iniciando...");
delay(2000);
lcd.clear();
```
Define inicialização dos pinos LDR e Relé
```C++
//Define modos dos pinos
pinMode(ldr, INPUT); 
pinMode(rele, OUTPUT);
digitalWrite(rele, LOW);
```
## Loop

Funções LDR e Relé:
Faz a leitura do pino do LDR e confere o valor lido, caso seja menor que 2000, desliga o relé 
```C++
void loop() 
{
  //============================================ Função LDR e Relé ================================================
  
  //Faz leitura sensor Luz - LDR
  luminosidade = analogRead(ldr);

  //Se valor analógico de luminosidade lido for maior que 2000
  if(luminosidade > 2000)
  {
    //Desliga rele
    digitalWrite(rele, LOW);
  }

  //Senão (valor analógico de luminosidade lido for menor que 2000)
  else
  {
   //Liga rele
   digitalWrite(rele, HIGH);
  }
```

Função DHT:
Faz a leitura do sensor LDR através das suas funções, após isso, verifica se o valor lido é coerente e então imprime essas no display 
```C++
//====================================== Funções DHT - Sensor Temp e Umid =======================================
  
  //Faz leituras do sensor
  float h = dht.readHumidity();         //Umidade
  float t = dht.readTemperature();      //Temperatura Celsius
  float f = dht.readTemperature(true);  //Temperatura Fahrenheit

  //Verifica se alguma leitura falhou para checar novamente
  if (isnan(h) || isnan(t) || isnan(f)) 
  {
    Serial.println(F("Falha de leitura!"));
    return;
  }

  //Imprime as informações na tela do LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(t);
  lcd.print("°C");
  lcd.setCursor(0, 1);
  lcd.print("Umidade: ");
  lcd.print(h);
  lcd.print("%");
  delay(3000);
```

Funções GPS:
Habilita o recebimento de informações do GPS através da comunicação UART e verifica se o recebido é válido, caso for, confere se 
essas informações são válidas (valores de latitude e longitude), caso for, imprime-ás no display, caso não, retorna e tenta novamente 
e mostra mensagem de que está sem sinal de GPS.
```C++
//=============================================== Funções GPS ====================================================
  
  bool recebido = false;

  //Enquanto não houver resposta válida do GPS
  while (serial1.available()) 
  {
   char cIn = serial1.read();
   recebido = gps1.encode(cIn);
  }

  //Se receber sinal do GPS
  if (recebido) 
  {
    //Latitude e Longitude
    long latitude, longitude;
    unsigned long idadeInfo;
    gps1.get_position(&latitude, &longitude, &idadeInfo);     

    //Se o valor de latitude for válido
    if (latitude != TinyGPS::GPS_INVALID_F_ANGLE) {
       Serial.print("Latitude: ");
       Serial.println(float(latitude) / 100000, 6);

       //Imprime as informações na tela do LCD
       lcd.clear();
       lcd.setCursor(0, 0);
       lcd.print("LAT: ");
       lcd.print(float(latitude) / 100000);
       delay(100);
    }

    //Se o valor de longitude for válido
    if (longitude != TinyGPS::GPS_INVALID_F_ANGLE) {
       Serial.print("Longitude: ");
       Serial.println(float(longitude) / 100000, 6);

       //Imprime as informações na tela do LCD
       lcd.clear();
       lcd.setCursor(0, 0);
       lcd.print("LAT: ");
       lcd.print(float(latitude) / 100000);
       lcd.setCursor(0, 1);
       lcd.print("LONG: ");
       lcd.print(float(longitude) / 100000);
       delay(3000);
    }

    //Se o valor de idade da informação for válido
    if (idadeInfo != TinyGPS::GPS_INVALID_AGE) {
       Serial.print("Idade da Informacao (ms): ");
       Serial.println(idadeInfo);
    }
  }
  
  //Se não receber sinal do GPS
  else
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sem Sinal GPS!");
    delay(2000);

    Serial.print("Sem Sinal GPS!");
  }
```

Funções Turbidez:
Faz leitura analógica do pino do sensor de turbidez e calcula o valor de NTU através da função, relacionando o valor lido aos valores base de NTU,
após calcular, mostra o valor de NTU no display.
```C++
  //============================================= Funções Turbidez =================================================
  
  //Inicia a leitura do pino em 0
  voltagem = 0;
 
  //Realiza "i" leituras de valores de voltagem
  for (i = 0; i < 800; i++) {
  voltagem += ((float)analogRead(SensorTurbidez) / 1023) * 5;
  }
 
  //Realiza a média entre os valores lidos na função acima
  voltagem = voltagem / 800;
  voltagem = ArredondarPara(voltagem, 1);
 
  // Se voltagem menor que 2.5 fixa o valor de NTU
  if (voltagem < 2.5) {
  NTU = 3000;
  }

  // Senão Se voltagem for maior que 4.2 fixa valor de NTU
  else if (voltagem > 4.2) {
  NTU = 0;
  voltagem = 4.2;
  }
 
  // Senão calcula o valor de NTU através da fórmula
  else {
  NTU = -1120.4 * (voltagem * voltagem) + 5742.3 * voltagem - 4353.8;
  }
 
  // Imprime as informações na tela do LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Turbidez:");
  lcd.print(NTU);
  lcd.print(" NTU");
  delay(3000);
}
```
Função de cálculo do valor lido no sensor turbidez:
Para uma medida mais apurada, é feito n leituras e então calculado um valor médio de tais, obtendo leituras mais homogêneas. 
```C++
  // Sistema de arredendamento para Leitura
  float ArredondarPara( float ValorEntrada, int CasaDecimal ) 
  {
   float multiplicador = powf( 10.0f, CasaDecimal );
   ValorEntrada = roundf( ValorEntrada * multiplicador ) / multiplicador;
   return ValorEntrada;
  }
```

# Referências

https://www.usinainfo.com.br/blog/sensor-de-turbidez-projeto-de-leitura-da-qualidade-da-agua/
https://randomnerdtutorials.com/guide-to-neo-6m-gps-module-with-arduino/#more-49434
https://blog.eletrogate.com/gps-neo-6m-com-arduino-aprenda-usar/
https://blog.eletrogate.com/sensores-dht11-dht22/
https://www.blogdarobotica.com/2022/05/02/como-utilizar-o-display-lcd-16x02-com-modulo-i2c-no-arduino/
