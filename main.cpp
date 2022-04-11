#include<Arduino.h>
// код  работает на 3 роботе // 74 дискреты за оборот
#include <math.h>
//библиотека с портами ввода вывода
#include <avr/io.h>
//задание тактовой частоты
#define F_CPU 16000000UL
//библиотека для работы с прерываниями
#include <avr/interrupt.h>
volatile boolean DegCount = false;
volatile boolean FlagMidleCheck = true;
volatile boolean FlagStopMidleCheck = false;
volatile boolean FlagDeg_true = false;
volatile boolean FlagStopDeg_true = false;
volatile boolean CheckEnd = false;



volatile float  deg1; // degree left counter
volatile float  deg2; // degree right counter
volatile float deg_true;

volatile float degree90 = 220;

volatile int32_t  enc1_cnt = 0; // счётчик правого энкодера
volatile int32_t  enc2_cnt = 0; // счётчик левого энкодера
volatile int32_t MidEnc = 0;

volatile float Check1; // расчёт правого энкодера
volatile float Check2; // рачёт леаого энкодера
volatile int32_t L = 50; // задаём расстояние в см здесь <
volatile int32_t l = 20;
volatile int32_t Lt = L * 0.7;
volatile int32_t Lk = L * 0.4;

volatile float MidleCheck;

volatile boolean L_A_new ; // левеое канал А новое
volatile boolean L_A_last ; // левое канал А предыдущее
volatile boolean L_B_new ; // левое канал В новое
volatile boolean L_B_last ; // левое канал В предыдущее

volatile boolean R_A_new ;// правое канал А новое
volatile boolean R_A_last ;// правое канал А предыдущее
volatile boolean R_B_new ;// правое канал В новое
volatile boolean R_B_last ; // правое канал В предыдущее

volatile int32_t err; //difference of sens
volatile int32_t P; // Proportional difference* rate
volatile float D; // Differencial
volatile float I; // Integral
volatile int32_t errold;
volatile int32_t X; // control


ISR(INT2_vect) //19                                                                   //_______правое ____________
{
  R_B_last = R_B_new ;  // предыдущее значение
  if (PIND & (1 << PD2))//считываем бит  19 порт
  {
    R_B_new =  1;
  }
  else
  {
    R_B_new =  0;
  }

  R_A_last = R_A_new ;  // предыдущее значение
  if (PIND & (1 << PD3))// считываем бит 18 порт
  {
    R_A_new =  1;
  }
  else
  {
    R_A_new =  0;
  }

  //для счётчика вращение в положительную
  if ((R_A_new == 0) && (R_B_new == 1) && (R_A_last == 1) && (R_B_last == 1))
  {


    enc1_cnt ++;
  }
  if ((R_A_new == 0) && (R_B_new == 0) && (R_A_last == 0) && (R_B_last == 1))
  {


    enc1_cnt ++;
  }
  if ((R_A_new == 1) && (R_B_new == 0) && (R_A_last == 0) && (R_B_last == 0))
  {


    enc1_cnt ++;
  }
  if ((R_A_new == 1) && (R_B_new == 1) && (R_A_last == 1) && (R_B_last == 0))
  {


    enc1_cnt ++;
  }

}


ISR(INT3_vect)//18
{
  R_B_last = R_B_new; // предыдущее значение
  if (PIND & (1 << PD2)) // считываем бит  19 порт
  {
    R_B_new =  1;
  }
  else
  {
    R_B_new =  0;
  }



  R_A_last = R_A_new ;  // предыдущее значемние
  if (PIND & (1 << PD3)) // считываем бит  18 порт
  {
    R_A_new =  1;
  }
  else
  {
    R_A_new =  0;
  }

  //для счётчика вращение в положительную
  if ((R_A_new == 0) && (R_B_new == 1) && (R_A_last == 1) && (R_B_last == 1))
  {

    enc1_cnt ++;
  }
  if ((R_A_new == 0) && (R_B_new == 0) && (R_A_last == 0) && (R_B_last == 1))
  {

    enc1_cnt ++;
  }
  if ((R_A_new == 1) && (R_B_new == 0) && (R_A_last == 0) && (R_B_last == 0))
  {

    enc1_cnt ++;
  }
  if ((R_A_new == 1) && (R_B_new == 1) && (R_A_last == 1) && (R_B_last == 0))
  {

    enc1_cnt ++;
  }

  //для счётчика  вращение в отрицательную
  if ((R_A_new == 0) && (R_B_new == 1) && (R_A_last == 0) && (R_B_last == 0))
  {
    //enc1_cnt --;
    enc1_cnt =  enc1_cnt - 2;
  }
  if ((R_A_new == 1) && (R_B_new == 1) && (R_A_last == 0) && (R_B_last == 1))
  {
    // enc1_cnt --;
    enc1_cnt =  enc1_cnt - 2;
  }
  if ((R_A_new == 1) && (R_B_new == 0) && (R_A_last == 1) && (R_B_last == 1))
  {
    // enc1_cnt --;
    enc1_cnt =  enc1_cnt - 2;
  }
  if ((R_A_new == 0) && (R_B_new == 0) && (R_A_last == 1) && (R_B_last == 0))
  {
    //enc1_cnt --;
    enc1_cnt =  enc1_cnt - 2;
  }

}


// вперёд 72 дискреты , назад 36
ISR(INT0_vect) // 21                                                                  //_______левое____________
{
  L_B_last = L_B_new ;  // предыдущее значение
  if (PIND & (1 << PD0)) // считываем бит 21 порт
  {
    L_B_new =  1;
  }
  else
  {
    L_B_new =  0;
  }

  L_A_last = L_A_new ;  // предыдущее значение
  if (PIND & (1 << PD1)) // считываем бит 20 порт
  {
    L_A_new =  1;
  }
  else
  {
    L_A_new =  0;
  }

  // для счётчика вращение в положительную
  if ((L_A_new == 0) && (L_B_new == 1) && (L_A_last == 1) && (L_B_last == 1))
  {
    enc2_cnt =  enc2_cnt - 4;

  }
  if ((L_A_new == 0) && (L_B_new == 0) && (L_A_last == 0) && (L_B_last == 1))
  {
    enc2_cnt =  enc2_cnt - 4;

  }
  if ((L_A_new == 1) && (L_B_new == 0) && (L_A_last == 0) && (L_B_last == 0))
  {
    enc2_cnt =  enc2_cnt - 4;

  }
  if ((L_A_new == 1) && (L_B_new == 1) && (L_A_last == 1) && (L_B_last == 0))
  {
    enc2_cnt =  enc2_cnt - 4;

  }


}


ISR(INT1_vect) // 20
{
  L_B_last = L_B_new ;  // предыдущее значение
  if (PIND & (1 << PD0)) // считываем бит 21 порт
  {
    L_B_new =  1;
  }
  else
  {
    L_B_new =  0;
  }

  L_A_last = L_A_new ;  // предыдущее значение
  if (PIND & (1 << PD1)) // считываем бит 20 порт
  {
    L_A_new =  1;
  }
  else
  {
    L_A_new =  0;
  }

  // для счётчика вращение в положительную
  if ((L_A_new == 0) && (L_B_new == 1) && (L_A_last == 1) && (L_B_last == 1))
  {
    // enc2_cnt --;
    enc2_cnt =  enc2_cnt - 2;
  }
  if ((L_A_new == 0) && (L_B_new == 0) && (L_A_last == 0) && (L_B_last == 1))
  {
    // enc2_cnt --;
    enc2_cnt =  enc2_cnt - 2;
  }
  if ((L_A_new == 1) && (L_B_new == 0) && (L_A_last == 0) && (L_B_last == 0))
  {
    // enc2_cnt --;
    enc2_cnt =  enc2_cnt - 2;
  }
  if ((L_A_new == 1) && (L_B_new == 1) && (L_A_last == 1) && (L_B_last == 0))
  {
    // enc2_cnt --;
    enc2_cnt =  enc2_cnt - 2;
  }


  //для счётчика вращение в отрицательную
  if ((L_A_new == 0) && (L_B_new == 1) && (L_A_last == 0) && (L_B_last == 0))
  {
    enc2_cnt =  enc2_cnt + 2;
  }
  if ((L_A_new == 1) && (L_B_new == 1) && (L_A_last == 0) && (L_B_last == 1))
  {
    enc2_cnt =  enc2_cnt + 2;
  }
  if ((L_A_new == 1) && (L_B_new == 0) && (L_A_last == 1) && (L_B_last == 1))
  {
    enc2_cnt =  enc2_cnt + 2;
  }
  if ((L_A_new == 0) && (L_B_new == 0) && (L_A_last == 1) && (L_B_last == 0))
  {
    enc2_cnt =  enc2_cnt + 2;
  }
}


int main()
{


  cli(); // запрещаем прерываение

  // два мотора

  DDRE |= (1 << PE4) | (1 << PE5); // настройка ножек на выход
  DDRL |= (1 << PL0); // правое колесо // 49 пин
  DDRB |= (1 << PB2); // левое колесо //51 пин

  //выбираем режим работы таймера Fast PWM (TOP = ICR3)
  TCCR3A = (1 << WGM31);
  TCCR3B = (1 << WGM32) | (1 << WGM33);
  //неинвертированный режим работы на каналах B и C
  TCCR3A |= (1 << COM3B1) | (1 << COM3C1);


  //настройка предделителя (prescaling ) clk / 1
  TCCR3B |= 1 << CS30;
  // записываем TOP value , таймер считает от 0 до ТОР
  ICR3 = 3199;


  // два энкодера

  //настройка ножек на вход
  DDRD &= ~((1 << PD3) | (1 << PD2)); //правое колесо
  DDRD &= ~((1 << PD1) | (1 << PD0));//левое колесо

  //настройка внешник прерываний int2 и int3 change
  EICRA |= ((1 << ISC20) | (1 << ISC30));
  //настройка внешних прерываний int1 и int0 change
  EICRA |= ((1 << ISC00) | (1 << ISC10));

  //включаем внешние прерывания int2 и int3
  EIMSK |= (1 << INT3) | (1 << INT2);
  //включаем внешние прерывания int1 и int0
  EIMSK |= (1 << INT1) | (1 << INT0);

  Serial.begin(9600);

  sei(); // разрешаем прерывание



  while (true)
  {

    if (DegCount == false)
    {
      // расчёт для левого энкодера
      Check2 = 3.14159 * 10 * ( (enc2_cnt) / 144.f);
      // расчёт для правого энкодера
      Check1 = 3.14159 * 10 * ( (enc1_cnt) / 144.f);
      MidEnc = (abs(enc1_cnt) + abs(enc2_cnt)) / 2;

      MidleCheck = (Check1 + Check2 ) / 2;
    }

    if (DegCount == true)
    {
      deg1 = (31.4159 * ( enc1_cnt / 144.f) * 360) / (2 * 3.14159 * 5); //угол правый
      deg2 = (31.4159 * ( enc2_cnt / 144.f) * 360) / (2 * 3.14159 * 5); //угол левый

      deg_true = ((abs(deg1) + abs(deg2)) / 2);
    }


    err = Check2 - Check1; //ошибка в показании энкодеров
    P = err * 40;// пропрциональная часть , коэффициент эмперически
    D = 3.5 * (err - errold); //дифференциальная часть
    I = 4 * 0.03 * err; // интегральная часть
    X = P + D + I; //регулятор скорости
    errold = err;



    // вывод проверки в монитор порта
    Serial.print(" R: ");
    Serial.print( enc1_cnt );
    Serial.print("  ");

    Serial.print(" L: ");
    Serial.print( enc2_cnt );
    Serial.print("  ");

    Serial.print( Check1 );
    Serial.print(" cm ");
    Serial.print("  ");

    Serial.print( Check2 );
    Serial.print(" cm ");
    Serial.print("  ");

    Serial.print(" mid: ");
    Serial.print(MidleCheck  );
    Serial.print("  ");

    Serial.print( deg1 );
    Serial.print(" ° ");
    Serial.print("  ");

    Serial.print( deg2 );
    Serial.print(" ° ");
    Serial.print("  ");

    Serial.print("true: ");
    Serial.print(deg_true );
    Serial.println("  ");



    //__________________________________________________________вперёд

    if ((MidleCheck <= L) )
    {

      //___ускорение___
      if (( L > 0) && (MidleCheck <= Lk))
      {
        Serial.print(" ускорение ");
        //правый
        // заполнение ШИМ (duty cycle) (от 0 до top)
        OCR3B = (600 + X) + (MidEnc * 4);
        PORTL &= ~(1 << PL0); // 49 LOW

        //левый
        // заполнение ШИМ (duty cycle) (от 0 до top)
        OCR3C = (600 - X) + (MidEnc * 4);
        PORTB &= ~(1 << PB2);


      }

      //___ движение___
      if (( L > 0) && (MidleCheck > Lk))
      {
        Serial.print(" движение ");
        //правый
        // заполнение ШИМ (duty cycle) (от 0 до top)
        OCR3B = 1020 + X;
        PORTL &= ~(1 << PL0); // 49 LOW

        //левый
        // заполнение ШИМ (duty cycle) (от 0 до top)
        OCR3C = 1200 -  X;
        PORTB &= ~(1 << PB2);


      }
      FlagMidleCheck = false;
      FlagStopMidleCheck = true;


    }

    //__________________________________________________________1 остановка

    if ((MidleCheck >= L) && (FlagStopMidleCheck == true) )
    {
      Serial.print(" остановка 1 ");
      OCR3B = 0;
      OCR3C = 0;

      FlagStopMidleCheck = false;
      FlagDeg_true = true;


      DegCount = true;
      enc1_cnt = 0;
      enc2_cnt = 0;

    }

    //__________________________________________________________поворот

    if ((degree90 > 0) && (FlagDeg_true == true))
    {

      if ( (deg_true < degree90) )                                          //движение
      {
        Serial.print(" поворот ");
        //правый вперёд
        // заполнение ШИМ (duty cycle) (от 0 до top)
        OCR3B = 1200 + X;
        PORTL &= ~(1 << PL0); // 49 LOW

        //левый назад
        OCR3C = 2500 - 1200 + X; // заполнение ШИМ (duty cycle) (от 0 до top)
        PORTB |= (1 << PB2);// движение назад

      }

      if ( (deg_true > degree90) )
      {
        Serial.print(" остановка 2 ");
        OCR3B = 0;
        OCR3C = 3199;
        FlagDeg_true = false;
        CheckEnd = true;

      }

    }

    // движение вперёд, остановка, обнуление, псмена счёта
    if (CheckEnd == true)
    {

      Serial.print(" end ");

      enc2_cnt = 0;
      enc1_cnt = 0;
      Check2 = 0;
      Check1 = 0;
      MidleCheck = 0;
      deg1 = 0;
      deg2 = 0;
      deg_true = 0;

      //правый

      OCR3B = 0;
      PORTL &= ~(1 << PL0); // 49 LOW

      //левый
      // заполнение ШИМ (duty cycle) (от 0 до top)
      OCR3C = 0;
      PORTB &= ~(1 << PB2);

      DegCount = false;

      CheckEnd = false;
    }

  }
}
