#include <Servo.h>
#include <SoftwareSerial.h>
#include "Pitches.h"


// Pin map
#define LIGHT    13  // Light 제어 핀
#define LED      13  // LED 제어 핀
#define M_PWM    5   // DC모터 PWM 핀
#define M_DIR1   7   // DC모터 DIR1 핀
#define M_DIR2   8   // DC모터 DIR2 핀
#define SERVO    9   // 서보모터 핀
#define BUZZER   3   // 버저 핀
#define BATTERY  A0  // 배터리 체크 핀
#define FC_TRIG 6   // 전방 초음파 센서 TRIG 핀
#define FC_ECHO 10  // 전방 초음파 센서 ECHO 핀
#define FL_TRIG 11  // 전방좌측 초음파 센서 TRIG 핀
#define FL_ECHO 12  // 전방좌측 초음파 센서 ECHO 핀
#define FR_TRIG 3   // 전방우측 초음파 센서 TRIG 핀
//#define FR_ECHO 4   // 전방우측 초음파 센서 ECHO 핀  --블루투스용 핀
#define L_TRIG  A2  // 좌측 초음파 센서 TRIG 핀
#define L_ECHO  A1  // 좌측 초음파 센서 ECHO 핀
//#define R_TRIG  2   // 우측 초음파 센서 TRIG 핀      --블루투스용 핀
#define R_ECHO  A5  // 우측 초음파 센서 ECHO 핀
//#define BC_TRIG A4  // 후방 초음파 센서 TRIG 핀
//#define BC_ECHO A3  // 후방 초음파 센서 ECHO 핀

#define MAX_DISTANCE  800 // 초음파 센서의 최대 감지거리(mm)

// 자동차 튜닝 파라미터
int servo_dir = 1; // 서보 회전 방향(동일: 1, 반대:-1)
int motor_dir = -1; // 모터 회전 방향(동일:1, 반대:-1)
int angle_limit = 25; // 서보 모터 회전 제한 각 (단위: 도
int angle_offset = 0; // 서보 모터 중앙각 오프셋 (단위: 도)
int max_rc_pwm = 255; // RC조종 모터 최대 출력 (0 ~ 255)
int min_rc_pwm = 110; // RC조종 모터 최소 출력 (0 ~ 255)
int punch_pwm = 255; // 정지 마찰력 극복 출력 (0 ~ 255)
int punch_time = 300; // 정지 마찰력 극복 시간 (단위 msec))
int stop_time = 300; // 전진후진 전환 시간 (단위 msec)
int melody_tempo = 3500; // 멜로디 연주 속도
int melody_num = 41; // 멜로디 음 개수
int battery_cell = 2; // 배터리 셀 개수
float voltage_error = 1.08; // 전압 오차 (1이 오차 없음)
// 자율주행 튜닝 파라미터
int max_ai_pwm = 180; // 자율주행 모터 최대 출력 (0 ~ 255)
int min_ai_pwm = 110; // 자율주행 모터 최소 출력 (0 ~ 255)
int center_detect = 200; // 전방 감지 거리 (단위: mm)
int center_start = 160; // 전방 출발 거리 (단위: mm)
int center_stop = 50; // 전방 멈춤 거리 (단위: mm)
int diagonal_detect = 80; // 대각 감지 거리 (단위: mm)
int diagonal_start = 120; // 대각 출발 거리 (단위: mm)
int diagonal_stop = 75; // 대각 멈춤 거리 (단위: mm)
int side_detect = 250; // 좌우 감지 거리 (단위: mm)
int side_start = 160; // 좌우 조향 시작 거리 (단위: mm)
int side_stop = 50; // 좌우 조향 끝 거리 (단위: mm)
float steering_gain = 1.5; // 좌우 조향 증폭상수

SoftwareSerial BTSerial(2, 4);            //2,4 번 핀은 블루투스용으로 남겨 두어야 한다.
Servo servo;

float cur_steering;
float cur_speed;
float max_pwm;
float min_pwm;

bool autoDriving = false;

unsigned long battery_time;
unsigned long rc_time;


float f_center;
float f_left;
float f_right;
float left;
float right;
float b_center;

// 배터리 체크, 저전압시 경고음
void CheckBattery()
{
    // 1초에 한번씩 체크
    if(millis() - battery_time < 1000)
        return;
  
    float voltage = (float)analogRead(BATTERY) / 1023;      
    voltage *= 5;                                           // 아두이노 5V
    voltage *= 3;                                           // 회로상 전압분배 비율 (VIN을 10K, 20K 저항으로 분배)
    voltage *= voltage_error;

    if(voltage < (2.8 * battery_cell))
    {
        while(true)
        {
            tone(BUZZER, NOTE_C5);
            digitalWrite(LED, HIGH);
            delay(1000);
            
            noTone(BUZZER);
            digitalWrite(LED, LOW);
            delay(1000);      
        }
    }
    else
    {
        Serial.print("B");
        Serial.print(battery_cell);
        Serial.print(":");
        Serial.println(voltage);
      }
    
    battery_time = millis();
    rc_time = millis();
}

// 초음파 거리측정
float GetDistance(int trig, int echo)
{ 
    digitalWrite(trig, LOW);
    delayMicroseconds(4);
    digitalWrite(trig, HIGH);
    delayMicroseconds(20);
    digitalWrite(trig, LOW);
    
    unsigned long duration = pulseIn(echo, HIGH, 5000);
    if(duration == 0)
        return MAX_DISTANCE;
    else
        return duration * 0.17;     // 음속 340m/s
}

// 자율주행 시작
void StartAutoDriving()
{
    autoDriving = true;
    max_pwm = max_ai_pwm;
    min_pwm = min_ai_pwm;
        
    SetSteering(0);
    SetSpeed(0);
    Serial.println("start auto driving");
}

// 자율주행 종료
void StopAutoDriving()
{
    autoDriving = false;
    max_pwm = max_rc_pwm;
    min_pwm = min_rc_pwm;
    SetSteering(0);
    SetSpeed(0);
}

// 앞바퀴 조향
void SetSteering(float steering)
{
    cur_steering = constrain(steering, -1, 1);
    
    float angle = cur_steering * angle_limit;
    if(servo_dir < 0)
        angle = -angle;
    
    int servoAngle = angle + 90;
    if(servo_dir < 0)
        servoAngle -= angle_offset;
    else
        servoAngle += angle_offset;
    servoAngle = constrain(servoAngle, 0, 180);
    
    servo.write(servoAngle);
}

// 뒷바퀴 모터회전
void SetSpeed(float speed)
{
    speed = constrain(speed, -1, 1);
    Serial.println("whiwhi");
    Serial.println(speed);
    Serial.println("areare");
    
    if((cur_speed * speed < 0) // 움직이는 중 반대 방향 명령이거나
        || (cur_speed != 0 && speed == 0)) // 움직이다가 정지라면
    {
        cur_speed = 0;
        digitalWrite(M_PWM, HIGH);
        digitalWrite(M_DIR1, LOW);
        digitalWrite(M_DIR2, LOW);
        if(stop_time > 0) 
          delay(stop_time);
    }

    if(cur_speed == 0 && speed != 0) // 정지상태에서 출발이라면
    {
        if(punch_time > 0)  //정지마찰력 
        {
            if(speed * motor_dir > 0)
            {
                analogWrite(M_PWM, punch_pwm);
                digitalWrite(M_DIR1, HIGH);
                digitalWrite(M_DIR2, LOW);
            }
            else if(speed * motor_dir < 0)
            {
                analogWrite(M_PWM, punch_pwm);
                digitalWrite(M_DIR1, LOW);
                digitalWrite(M_DIR2, HIGH);
            }
            
            delay(punch_time);
        }
    }

    if(speed != 0) // 명령이 정지가 아니라면
    {
        int pwm = abs(speed) * (max_pwm - min_pwm) + min_pwm;           // 0 ~ 255로 변환
        if(speed * motor_dir > 0)
        {
            analogWrite(M_PWM, pwm);
            digitalWrite(M_DIR1, HIGH);
            digitalWrite(M_DIR2, LOW);
        }
        else if(speed * motor_dir < 0)
        {
            analogWrite(M_PWM, pwm);
            digitalWrite(M_DIR1, LOW);
            digitalWrite(M_DIR2, HIGH);
        }
    }

    cur_speed = speed;
}

void AutoDriving(){


       if(!autoDriving)
            return;

       float compute_speed = cur_speed;
       float compute_steering = cur_steering;

      Serial.println("hello1111");
       if(cur_speed == 0){                                      //정지상태에서 출발시 
          f_center=GetDistance(FC_TRIG,FC_ECHO);
          if(f_center > center_start){
            compute_speed = 0.1;                                //왜 0.1 일까 
            compute_steering = 0 ;
          }

       }
       else if(cur_speed >0){                                  //정지 상태가 아닐때 
          f_center=GetDistance(FC_TRIG,FC_ECHO);
          if(f_center <= center_detect){                         //전방 장애물을 만났을때 
             compute_speed = 0;
             compute_steering = 0;
            
          }else{
            compute_speed=1;
            
          }

        
       }
      SetSpeed(compute_speed);
      SetSteering(compute_steering);
}





void setup() {
  // put your setup code here, to run once:
    max_pwm = max_rc_pwm;
    min_pwm = min_rc_pwm;
    
    servo.attach(SERVO);    
    pinMode(M_PWM, OUTPUT);
    pinMode(M_DIR1, OUTPUT);
    pinMode(M_DIR2, OUTPUT);
    pinMode(LIGHT, OUTPUT);
    pinMode(LED, OUTPUT);
    pinMode(FC_TRIG, OUTPUT);
    pinMode(FC_ECHO, INPUT);
    pinMode(FL_TRIG, OUTPUT);
    pinMode(FL_ECHO, INPUT);
    pinMode(FR_TRIG, OUTPUT);
//    pinMode(FR_ECHO, INPUT);              - 블루투스용 
    //pinMode(BC_TRIG, OUTPUT);
    //pinMode(BC_ECHO, INPUT);
    pinMode(L_TRIG, OUTPUT);
    pinMode(L_ECHO, INPUT);
 //   pinMode(R_TRIG, OUTPUT);              - 블루투스용
    pinMode(R_ECHO, INPUT);
    
    digitalWrite(LIGHT, LOW);
    SetSteering(0);
    SetSpeed(0);
    
    Serial.begin(9600);
    BTSerial.begin(9600);
    
    battery_time = millis();
}

void loop() {

    CheckBattery();         //배터리 체크

    if (BTSerial.available()){
       String packet = BTSerial.readStringUntil('\n');
       if(packet != 0)
        {
          int index = packet.indexOf(':');
          if(index >= 0)
            {
            String cmd = packet.substring(0, index);
            String param = packet.substring(index + 1);
           
           if(cmd.equals("S")){                            //테스트용 사운드 버튼
                 if(param.toInt() == 1){
                      servo.write(115);
                      Serial.println("hello");
                 }else
                      servo.write(90);    
              
            }
            else if(cmd.equals("A"))                    // 자율주행명령
            {
                if(param.toInt() == 1){
                    StartAutoDriving();
                 Serial.println("hello11");
                }else
                    StopAutoDriving();
                }
             
        }
     }
    }
     AutoDriving();
  
/*  if(Serial.available() > 0)
    {   
        String packet = Serial.readStringUntil('\n');
        if(packet != 0)
        {
            int index = packet.indexOf(':');
            if(index >= 0)
            {
            String cmd = packet.substring(0, index);
            String param = packet.substring(index + 1);
            if(cmd.equals("T") && !autoDriving)         // 조향명령
            {
                SetSteering(param.toFloat());
                rc_time = millis();
            }
            else if(cmd.equals("P") && !autoDriving)    // 속도명령
            {
                SetSpeed(param.toFloat());
                rc_time = millis();
            }
            else if(cmd.equals("A"))                    // 자율주행명령
            {
                if(param.toInt() == 1)
                    StartAutoDriving();
                else
                    StopAutoDriving();
                }
            else if(cmd.equals("S")){
                 if(param.toInt() == 1){
                      servo.write(180);
                      Serial.println("hello");
                 }else
                      servo.write(90);    
              
             }
            }
        }
    }

    AutoDriving();

    // 직접 컨트롤시 3초 동안 블루투스 신호 없으면 정지
    if(!autoDriving)
    {
        if(millis() - rc_time > 3000)
            SetSpeed(0);
    }

*/   





  

}
