These files are liquid and only work on the ZYC0076 robot.
Unidirectional wheels are used as wheels:

![image](https://github.com/Template58/ArduinoBOT/assets/173082842/2d4cfafd-b7a1-4d75-8435-c64dacdc96f9)

The ESP 32 is also connected with a sandwich board with USB-C
And the pins are connected UOR-tx OUT-rx gnd-gnd 5v-vcc

![image](https://github.com/Template58/ArduinoBOT/assets/173082842/16de0475-642c-4221-b09a-534f32af6370)

![image](https://github.com/Template58/ArduinoBOT/assets/173082842/3eb6039a-430d-4faa-8058-55122607c9a3)

sheme for esp32cam

![image](https://github.com/Template58/ArduinoBOT/assets/173082842/5300b7cf-22c8-467b-bec5-53d6f98f813c)
 
Example for arduino servo
#include "SoftServo.h"

SoftServo myservo;

void setup() {
  myservo.attach(5);
  
  // asyncMode - вызов tick не блокирует код на величину импульса (0.7-2.5 мс)
  // но работа будет нестабильной при наличии задержек в коде
  // в этом режиме tick вернёт true на период импульса, можно запрещать
  // тяжёлые функции на этот период 
  myservo.asyncMode();
  
  // delayMode - вызов tick блокирует код на величину импульса (0.7-2.5 мс) - по умолчанию  
  myservo.delayMode();  
}

int val = 0;
void loop() {
  // тикер - вызывать как можно чаще для каждого экземпляра
  myservo.tick();
  
  // двигаем туда сюда
  static uint32_t tmr;
  if (millis() - tmr >= 50) {
    tmr = millis();    
    static int dir = 5;
    val += dir;    
    if (val >= 180 || val <= 0) dir = -dir;   // разворачиваем
    myservo.write(val);
  }
}
