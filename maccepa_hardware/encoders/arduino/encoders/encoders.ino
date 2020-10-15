#include <SPI.h>

const int slavePin1 = 2;
const int slavePin2 = 10;

uint8_t data1[5];
uint8_t data2[5];

uint32_t result1 = 0;
uint32_t result2 = 0;

int error_bit1 = 0;
int error_bit2 = 0;

unsigned long time1 = 0;
unsigned long time2 = 0;

SPISettings settings(3000000, MSBFIRST, SPI_MODE0);

void setup() {
  Serial.begin(1000000);
  SPI.begin();
  SPI.beginTransaction(settings);
  pinMode(slavePin1, OUTPUT);
  pinMode(slavePin2, OUTPUT);
  delay(10);
}

uint32_t convert(uint8_t* arr){
  uint32_t res = 0;
  res = arr[0];
  res = res << 8;
  res = res | arr[1];
  res = res << 4;
  res = res | (arr[2] >> 4);
  return res;
}

uint32_t readAngle(uint8_t* data, int slavePin, int& error_bit){
  digitalWrite (slavePin, LOW);
  for(int i = 0; i < 5; i++){
    data[i] = SPI.transfer(0);
  }
  digitalWrite (slavePin, HIGH);
  error_bit = (data[2] & 0b00000100) >> 2;
  return convert(data);

}

void loop() {

  result1 = readAngle(data1, slavePin1, error_bit1);
  time1 = micros();
  result2 = readAngle(data2, slavePin2, error_bit2);
  time2 = micros();
  Serial.print(result1);
  Serial.print(" ");
  Serial.print(error_bit1);
  Serial.print(" ");
  Serial.print(time1);
  Serial.print(" ");
  Serial.print(result2);
  Serial.print(" ");
  Serial.print(error_bit2);
  Serial.print(" ");
  Serial.print(time2);
  Serial.println();
}
