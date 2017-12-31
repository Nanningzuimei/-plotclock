#include <Time.h>
#include <TimeLib.h>
#include <math.h>
 #include <SoftwareSerial.h>
    SoftwareSerial BluetoothSerial(8, 9); 
int a,val;   //定义变量
float temperature;     //定义浮点型变量，用于存放转换后的温度
//int B=3975;              //热敏电阻的基础参考值B
float resistance;

// units: mm; microseconds; radians
// origin: bottom left of drawing surface

// time library see http://playground.arduino.cc/Code/time 

// delete or mark the next line as comment when done with calibration  
//#define CALIBRATION

// When in calibration mode, adjust the following factor until the servos move exactly 90 degrees
#define SERVOFAKTOR 650

// Zero-position of left and right servo
// When in calibration mode, adjust the NULL-values so that the servo arms are at all times parallel
// either to the X or Y axis
#define SERVOLEFTNULL 1980
#define SERVORIGHTNULL 1050

#define SERVOPINLIFT  2
#define SERVOPINLEFT  3
#define SERVOPINRIGHT 4

// lift positions of lifting servo
#define LIFT0 1100 // on drawing surface
#define LIFT1  750  // between numbers
#define LIFT2 570  // going towards sweeper

// speed of liftimg arm, higher is slower
#define LIFTSPEED 2000

// length of arms
#define L1 35
#define L2 55.2
#define L3 13.2


// origin points of left and right servo 
#define O1X 22
#define O1Y -25
#define O2X 47
#define O2Y -25



#include <Time.h>
#include <Servo.h>

int servoLift = 800;

Servo servo1;  // lifting servo
Servo servo2;  // left servo
Servo servo3;  // right servo

volatile double lastX = 70;
volatile double lastY = 42;

int last_min = 0;

void setup() 
{ Serial.begin(38400);
 analogReference(INTERNAL);  //调用板载1.1V基准源
  while (!Serial) {
            ; // 等待串口连接。
        }
        Serial.println("Serial Connected!");

  BluetoothSerial.begin(38400);
  // Set current time only the first to values, hh,mm are needed
  setTime(2,19,0,0,0,0);

  servo1.attach(SERVOPINLIFT);  //  lifting servo
  servo2.attach(SERVOPINLEFT);  //  left servo
  servo3.attach(SERVOPINRIGHT);  //  right servo
  lift( 2 );
  delay( 1000 );
  drawTo(0, 36);
  delay( 1000 );
  drawTo( 73.2, 36 );
  lift( 2 );
  delay(1000);

} 

void loop() 
{ while (Serial.available())
  { 
   int inByte = Serial.read();
            BluetoothSerial.write( inByte);//蓝牙模块将数据发送给单片机
   // Serial.println(inByte);//BluetoothSerial.write(inByte ); 
    if (BluetoothSerial.available()) //检测蓝牙模块串口状态
           {int value = BluetoothSerial.read();
            Serial.write(value); //单片机将指令发送到蓝牙模块
            
  };
    switch (inByte)
    {case 65:
    a=analogRead(A3);
  //resistance=(float)(1023-a)*10000/a; //计算出传感器的电阻值
  //temperature=1/(log(resistance/10000)/B+1/298.15)-273.15;//将电阻值转换成温度值
 temperature= (float)a/1024*105;//
  delay(500); //延时500毫秒
   Serial.print("Temp: ");  Serial.println(temperature);
//val=map(temperature,0,50,0,180); //将转换的温度值映射到舵机的角度值
//  myservo.write(val); //舵机转到相应的角度
if (1) //如需修改请复制上面代码  
  {  
    if (!servo1.attached()) servo1.attach(SERVOPINLIFT);  
    if (!servo2.attached()) servo2.attach(SERVOPINLEFT);  
    if (!servo3.attached()) servo3.attach(SERVOPINRIGHT);  
  
    lift(0);  
     int a=(int)temperature/10;
     int b=(int)temperature%10;
number(3, 3, 111, 1);   //擦黑板  
    number(5, 25, a, 0.9);  // 书写19:38的第一位，也就是写1  
    number(19, 25, b, 0.9); //提取并书写19:38的第二位，也就是写9  
    
    lift(2);  
    drawTo(70, 44);  //移动到70,44  
    lift(1);  
    servo1.detach();  
    servo2.detach();  
    servo3.detach(); };
    break;
    case 67:
  
 if (1) {

    if (!servo1.attached()) servo1.attach(SERVOPINLIFT);
    if (!servo2.attached()) servo2.attach(SERVOPINLEFT);
    if (!servo3.attached()) servo3.attach(SERVOPINRIGHT);

    lift(0);
    number(3, 3, 112, 1);
    drawTo(73.2, 49);
    lift(1);
      servo1.detach();
    servo2.detach();
    servo3.detach();}
break;
    
  case 66:


#ifdef CALIBRATION

  // Servohorns will have 90° between movements, parallel to x and y axis
  drawTo(-3, 29.2);
  delay(500);
  drawTo(74.1, 28);
  delay(500);

#else 

//if (last_min != minute())  //每分钟书写一次时间  
  //if  (1)                  //反复不间断的擦写模式  
    
  int i = 0;
  if (last_min != minute()) {

    if (!servo1.attached()) servo1.attach(SERVOPINLIFT);
    if (!servo2.attached()) servo2.attach(SERVOPINLEFT);
    if (!servo3.attached()) servo3.attach(SERVOPINRIGHT);

    lift(0);

    hour();
    while ((i+1)*10 <= hour())
    {
      i++;
    }

    number(3, 3, 111, 1);//number(起始x，起始y，书写内容，放大倍率）
    number(5, 26, i, 1.2);
    number(19, 25, (hour()-i*10), 1);
    number(29, 25, 11, 1);

    i=0;
    while ((i+1)*10 <= minute())
    {
      i++;
    }
    number(34, 25, i, 1);
    number(48, 25, (minute()-i*10), 1);
    lift(2);
    drawTo(73.2, 49);
    lift(1);
    last_min = minute();

    servo1.detach();
    servo2.detach();
    servo3.detach();
  };break;
  #endif
  
  }}



} 

// Writing numeral with bx by being the bottom left originpoint. Scale 1 equals a 20 mm high font.
// The structure follows this principle: move to first startpoint of the numeral, lift down, draw numeral, lift up

void number(float bx, float by, int num, float scale) {

  switch (num) {

  case 0:
    drawTo(bx + 12 * scale, by + 6 * scale);
    lift(0);
    beginACW(bx + 7 * scale, by + 10 * scale, 10 * scale, -0.8, 6.7, 0.5);
    lift(1);
    break;
  case 1:

    drawTo(bx + 10 * scale, by + 15 * scale);
    lift(0);
    drawTo(bx + 10 * scale, by + 20 * scale);
    drawTo(bx + 10 * scale, by + 0 * scale);
    lift(1);
    break;
  case 2:
    drawTo(bx + 2 * scale, by + 12 * scale);
    lift(0);
    beginCW(bx + 8 * scale, by + 14 * scale, 6 * scale, 3, -0.8, 1);
    drawTo(bx + 1 * scale, by + 0 * scale);
    drawTo(bx + 12 * scale, by + 0 * scale);
    lift(1);
    break;
  case 3:
    drawTo(bx + 2 * scale, by + 17 * scale);
    lift(0);
    beginCW(bx + 5 * scale, by + 15 * scale, 5 * scale, 3, -2, 1);
    beginCW(bx + 5 * scale, by + 5 * scale, 5 * scale, 1.57, -3, 1);
    lift(1);
    break;
  case 4:
    drawTo(bx + 10 * scale, by + 0 * scale);
    lift(0);
    drawTo(bx + 10 * scale, by + 20 * scale);
    drawTo(bx + 2 * scale, by + 6 * scale);
    drawTo(bx + 12 * scale, by + 6 * scale);
    lift(1);
    break;
  case 5:
    drawTo(bx + 2 * scale, by + 5 * scale);
    lift(0);
    beginACW(bx + 5 * scale, by + 6 * scale, 6 * scale, -2.5, 2, 1);
    drawTo(bx + 5 * scale, by + 20 * scale);
    drawTo(bx + 12 * scale, by + 20 * scale);
    lift(1);
    break;
  case 6:
    drawTo(bx + 2 * scale, by + 10 * scale);
    lift(0);
    beginCW(bx + 7 * scale, by + 6 * scale, 6 * scale, 2, -4.4, 1);
    drawTo(bx + 11 * scale, by + 20 * scale);
    lift(1);
    break;
  case 7:
    drawTo(bx + 2 * scale, by + 20 * scale);
    lift(0);
    drawTo(bx + 12 * scale, by + 20 * scale);
    drawTo(bx + 2 * scale, by + 0);
    lift(1);
    break;
  case 8:
    drawTo(bx + 5 * scale, by + 10 * scale);
    lift(0);
    beginCW(bx + 5 * scale, by + 15 * scale, 5 * scale, 4.7, -1.6, 1);
    beginACW(bx + 5 * scale, by + 5 * scale, 5 * scale, -4.7, 2, 1);
    lift(1);
    break;

  case 9:
    drawTo(bx + 9 * scale, by + 11 * scale);
    lift(0);
    beginCW(bx + 7 * scale, by + 15 * scale, 5 * scale, 4, -0.5, 1);
    drawTo(bx + 5 * scale, by + 0);
    lift(1);
    break;

  case 111:

    lift(0);
    drawTo(70, 46);
    drawTo(65, 43);

    drawTo(65, 49);
    drawTo(5, 49);
    drawTo(5, 45);
    drawTo(65, 45);
    drawTo(65, 40);

    drawTo(5, 40);
    drawTo(5, 35);
    drawTo(65, 35);
    drawTo(65, 30);

    drawTo(5, 30);
    drawTo(5, 25);
    drawTo(65, 25);
    drawTo(65, 20);

    drawTo(5, 20);
    drawTo(60, 44);

    drawTo(75, 45);
    lift(2);

    break;
    case 112:
    
    lift(1);
    drawTo(23,39);
    lift(0);
    drawTo(23,12);
    lift(1);
    drawTo(23,25);
    lift(0);
    drawTo(40,25);
    lift(1);
    drawTo(40,39);
    lift(0);
    drawTo(40,9);
    lift(2);
    drawTo(70,40);
   
    
     break;
  case 11:
    drawTo(bx + 5 * scale, by + 20 * scale);
    lift(0);
    beginACW(bx + 5 * scale, by + 20* scale, 0.1 * scale, 1, -1, 1);
    lift(1);
    drawTo(bx + 5 * scale, by + 5* scale);
    lift(0);
    beginACW(bx + 5 * scale, by + 5* scale, 0.1 * scale, 1, -1, 1);
    lift(1);
    break;

   
  }

}

void lift(char lift) {
  switch (lift) {
    // room to optimize  !
  case 0: //850
      if (servoLift >= LIFT0) {
      while (servoLift >= LIFT0) 
      {
        servoLift--;
        servo1.writeMicroseconds(servoLift);        
        delayMicroseconds(LIFTSPEED);
      }
    } 
    else {
      while (servoLift <= LIFT0) {
        servoLift++;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);
      }
    }
    break;

  case 1: //150
    if (servoLift >= LIFT1) {
      while (servoLift >= LIFT1) {
        servoLift--;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);
      }
    } 
    else {
      while (servoLift <= LIFT1) {
        servoLift++;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);
      }
    }
    break;

  case 2:
    if (servoLift >= LIFT2) {
      while (servoLift >= LIFT2) {
        servoLift--;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);
      }
    } 
    else {
      while (servoLift <= LIFT2) {
        servoLift++;
        servo1.writeMicroseconds(servoLift);        
        delayMicroseconds(LIFTSPEED);
      }
    }
    break;
  }
}

void beginCW(float bx, float by, float radius, int start, int ende, float sqee) {
  float inkr = -0.05;
  float count = 0;

  do {
    drawTo(sqee * radius * cos(start + count) + bx,
    radius * sin(start + count) + by);
    count += inkr;
  } 
  while ((start + count) > ende);

}

void beginACW(float bx, float by, float radius, int start, int ende, float sqee) {
  float inkr = 0.05;
  float count = 0;

  do {
    drawTo(sqee * radius * cos(start + count) + bx,
    radius * sin(start + count) + by);
    count += inkr;
  } 
  while ((start + count) <= ende);
}

void drawTo(double pX, double pY) {
  double dx, dy, c;
  int i;

  // dx dy of new point
  dx = pX - lastX;
  dy = pY - lastY;
  //path lenght in mm, times 4 equals 4 steps per mm
  c = floor(4 * sqrt(dx * dx + dy * dy));

  if (c < 1) c = 1;

  for (i = 0; i <= c; i++) {
    // draw line point by point
    set_XY(lastX + (i * dx / c), lastY + (i * dy / c));

  }

  lastX = pX;
  lastY = pY;
}

double return_angle(double a, double b, double c) {
  // cosine rule for angle between c and a
  return acos((a * a + c * c - b * b) / (2 * a * c));
}

void set_XY(double Tx, double Ty) 
{
  delay(1);
  double dx, dy, c, a1, a2, Hx, Hy;

  // calculate triangle between pen, servoLeft and arm joint
  // cartesian dx
  dx = Tx - O1X;
  dy = Ty - O1Y;

  // polar lemgth (c) and angle (a1)
  c = sqrt(dx * dx + dy * dy); // 
  a1 = atan2(dy, dx); //
  a2 = return_angle(L1, L2, c);

  servo2.writeMicroseconds(floor(((a2 + a1 - M_PI) * SERVOFAKTOR) + SERVOLEFTNULL));

  // calculate joinr arm point for triangle of the right servo arm
  a2 = return_angle(L2, L1, c);
  Hx = Tx + L3 * cos((a1 - a2 + 0.621) + M_PI); //36,5°
  Hy = Ty + L3 * sin((a1 - a2 + 0.621) + M_PI);

  // calculate triangle between pen joint, servoRight and arm joint
  dx = Hx - O2X;
  dy = Hy - O2Y;

  c = sqrt(dx * dx + dy * dy);
  a1 = atan2(dy, dx);
  a2 = return_angle(L1, (L2 - L3), c);

  servo3.writeMicroseconds(floor(((a1 - a2) * SERVOFAKTOR) + SERVORIGHTNULL));

}

