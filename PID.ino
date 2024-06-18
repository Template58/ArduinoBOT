#include <PIDController.h>
/* контакты ENCODER_A и ENCODER_B используются для считывания микроконтроллером
данных от энкодера, эти данные поступают очень быстро, поэтому необходимо
чтобы на них можно было использовать прерывания 
*/
#define ENCODER_A 2
#define ENCODER_B 3
/* контакты MOTOR_CW и MOTOR_CCW используются для управления двигателем
с помощью H-моста, на этих двух контактах должно быть доступно формирование
ШИМ сигнала, иначе код программы работать не будет
*/
#define MOTOR_CW 9
#define MOTOR_CCW 10
/*в этой части программы мы зададим коэффициенты усиления
для пропорциональной, интегральной и дифференциальной частей контроллера 
*/
#define __Kp 260 // пропорциональная константа
#define __Ki 2.7 // интегральная константа
#define __Kd 2000 // дифференциальная константа
volatile long int encoder_count = 0; // переменная счетчика энкодера
unsigned int integerValue = 0; // сохраняет значение, поступающее по последовательной связи. Максимальное значение - 65535
char incomingByte; // в этой переменной будут храниться символы, поступающие по последовательной связи
int motor_pwm_value = 255; // в этой переменной сохраняется значение, рассчитанное с помощью PID алгоритма
PIDController pidcontroller;
void setup() {
  Serial.begin(115200); // последовательная связь для целей отладки
 // устанавливаем режимы работы используемых контактов
  pinMode(ENCODER_A, INPUT); // ENCODER_A as Input
  pinMode(ENCODER_B, INPUT); // ENCODER_B as Input
  pinMode(MOTOR_CW, OUTPUT); // MOTOR_CW as Output
  pinMode(MOTOR_CCW, OUTPUT); // MOTOR_CW as Output
  /* делаем контакт ENCODER_A контактом обработки прерывания, при 
поступлении импульса с восходящим фронтом (RISING edge) вызывается функция encoder().
  */
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder, RISING);
  pidcontroller.begin(); // инициализируем PID контроллер
  pidcontroller.tune(260, 2.7, 2000); // настраиваем PID аргументы kP, kI, kD
  pidcontroller.limit(-255, 255); // ограничиваем выход PID значений от -255 до 255
}
void loop() {
  while (Serial.available() > 0) {
    integerValue = Serial.parseInt(); // сохраняем принятое значение в integerValue
    incomingByte = Serial.read(); // сохраняем символ /n 
    if (incomingByte == '\n') // если мы приняли символ новой строки newline character) мы будем продолжать цикл (loop)
      continue;
  }
  pidcontroller.setpoint(integerValue); // задаем цель, которую PID контроллер будет пытаться достигнуть
  Serial.println(integerValue); // печатаем поступающее значение для целей отладки
  motor_pwm_value = pidcontroller.compute(encoder_count);  //PID алгоритм рассчитывает оптимальное значение и мы сохраняем его в переменной 
  Serial.print(motor_pwm_value); // печатаем рассчитанное значение для целей отладки
  Serial.print("   ");
  if (motor_pwm_value > 0) // если значение motor_pwm_value больше нуля, мы вращаем двигатель по часовой стрелке
    motor_ccw(motor_pwm_value);
  else // иначе мы вращаем его против часовой стрелки
    motor_cw(abs(motor_pwm_value));
  Serial.println(encoder_count);// печатаем окончательное значение переменной encoder count
}
void encoder() {
  if (digitalRead(ENCODER_B) == HIGH) // если на контакте ENCODER_B уровень high увеличиваем count
    encoder_count++; // increment the count
  else // иначе уменьшаем count
    encoder_count--;  // decrement the count
}
void motor_cw(int power) {
  if (power > 100) {
    analogWrite(MOTOR_CW, power); //вращаем двигатель если значение больше чем 100
    digitalWrite(MOTOR_CCW, LOW); // make the other pin LOW
  }
  else {
    // both of the pins are set to low (останавливаем двигатель)   
    digitalWrite(MOTOR_CW, LOW);
    digitalWrite(MOTOR_CCW, LOW);
  }
}
void motor_ccw(int power) {
  if (power > 100) {
    analogWrite(MOTOR_CCW, power);
    digitalWrite(MOTOR_CW, LOW);
  }
  else {
    digitalWrite(MOTOR_CW, LOW);
    digitalWrite(MOTOR_CCW, LOW);
  }
}
