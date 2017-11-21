#include <Servo.h>
#include <SoftwareSerial.h>
#include "Pitches.h"

// Pin map
#define LIGHT    4  // Light 제어 핀
#define LED      6  // LED 제어 핀
#define M_PWM    5   // DC모터 PWM 핀
#define M_DIR1   8   // DC모터 DIR1 핀
#define M_DIR2   7   // DC모터 DIR2 핀
#define SERVO    9   // 서보모터 핀
#define BUZZER   3   // 버저 핀
#define BATTERY A0  // 배터리 체크 핀
#define FC_TRIG A5   // 전방 초음파 센서 TRIG 핀
#define FC_ECHO A4 // 전방 초음파 센서 ECHO 핀
#define FL_TRIG A3  // 전방좌측 초음파 센서 TRIG 핀
#define FL_ECHO A2  // 전방좌측 초음파 센서 ECHO 핀
#define FR_TRIG 10   // 전방우측 초음파 센서 TRIG 핀
#define FR_ECHO 11  // 전방우측 초음파 센서 ECHO 핀
#define L_TRIG  A5  // 좌측 초음파 센서 TRIG 핀
#define L_ECHO  A4  // 좌측 초음파 센서 ECHO 핀
#define R_TRIG  12   // 우측 초음파 센서 TRIG 핀
#define R_ECHO  13  // 우측 초음파 센서 ECHO 핀



// 우측 초음파 센서 ECHO 핀
#define MAX_DISTANCE  800 // 초음파 센서의 최대 감지거리

// 자동차 튜닝 파라미터
int servo_dir = 1; // 서보 회전 방향(동일: 1, 반대:-1)
int motor_dir = 1; // 모터 회전 방향(동일:1, 반대:-1)   모터 선 위아래 바뀌어 뀌면 됨
int angle_limit = 35; // 서보 모터 회전 제한 각 (단위: 도)
int angle_offset = 0; // 서보 모터 중앙각 오프셋 (단위: 도)
int max_rc_pwm = 255; // RC조종 모터 최대 출력 (0 ~ 255)
int min_rc_pwm = 110; // RC조종 모터 최소 출력 (0 ~ 255)
int punch_pwm = 255; // 정지 마찰력 극복 출력 (0 ~ 255)
int punch_time = 200; // 정지 마찰력 극복 시간 (단위 msec)
int stop_time = 300; // 전진후진 전환 시간 (단위 msec)
int melody_tempo = 3500; // 멜로디 연주 속도
int melody_num = 41; // 멜로디 음 개수
int battery_cell = 2; // 배터리 셀 개수
float voltage_error = 1.08; // 전압 오차 (1이 오차 없음)

// 자율주행 튜닝 파라미터
int max_ai_pwm = 255; // 자율주행 모터 최대 출력 (0 ~ 255)
int min_ai_pwm = 110; // 자율주행 모터 최소 출력 (0 ~ 255)
int center_detect = 400; // 전방 감지 거리 (단위: mm)
int center_start = 160; // 전방 출발 거리 (단위: mm)
int center_stop = 50; // 전방 멈춤 거리 (단위: mm)
int diagonal_detect = 80; // 대각 감지 거리 (단위: mm)
int diagonal_start = 120; // 대각 출발 거리 (단위: mm)
int diagonal_stop = 75; // 대각 멈춤 거리 (단위: mm)
int side_detect = 250; // 좌우 감지 거리 (단위: mm)
int side_start = 160; // 좌우 조향 시작 거리 (단위: mm)
int side_stop = 50; // 좌우 조향 끝 거리 (단위: mm)
float steering_gain = 1.5; // 좌우 조향 증폭상수


// 멜로디 음계
int melody[] = {
  NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_B4, NOTE_D5, NOTE_C5, NOTE_A4, 0,
  NOTE_C4, NOTE_E4, NOTE_A4, NOTE_B4, 0, NOTE_E4, NOTE_GS4, NOTE_B4, NOTE_C5, 0,
  NOTE_E4, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_B4, NOTE_D5, NOTE_C5, NOTE_A4, 0,
  NOTE_C4, NOTE_E4, NOTE_A4, NOTE_B4, 0, NOTE_E4, NOTE_C5, NOTE_B4, NOTE_A4, 0
};
// 멜로디 음 빠르기
int duration[] = 
{
  16, 16, 16, 16, 16, 16, 16, 16, 8, 16,
  16, 16, 16, 8, 16, 16, 16, 16, 8, 16,
  16, 16, 16, 16, 16, 16, 16, 16, 16, 8, 16,
  16, 16, 16, 8, 16, 16, 16, 16, 4, 16
};


SoftwareSerial BTSerial(2, 4);            //2,4 번 핀은 블루투스용으로 남겨 두어야 한다.
Servo servo;
float cur_steering;
float cur_speed;
float max_pwm;
float min_pwm;
bool sound = false;
bool autoDriving = false;
bool backBool = false;
int melody_index = 0;
unsigned long melody_time;
unsigned long battery_time;
unsigned long rc_time;
float f_center;
float f_left;
float f_right;
float left;
float right;
float f_center1;
float f_left1;
float f_right1;
float left1;
float right1;
float f;
float fl;
float fr;
float l;
float r;





// 앞바퀴 조향   //offset 한번 확인해봐서 4도로 할지 0으로 할지 고려
void SetSteering(float steering)
{
  cur_steering = constrain(steering, -1, 1);

  float angle = cur_steering * angle_limit;
  if (servo_dir < 0)
    angle = -angle;

  int servoAngle = angle + 90;
  if (servo_dir < 0)
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

  if ((cur_speed * speed < 0) // 움직이는 중 반대 방향 명령이거나
      || (cur_speed != 0 && speed == 0)) // 움직이다가 정지라면
  {
    cur_speed = 0;
    digitalWrite(M_PWM, HIGH);
    digitalWrite(M_DIR1, LOW);
    digitalWrite(M_DIR2, LOW);
    if (stop_time > 0)
      delay(stop_time);
  }

  if (cur_speed == 0 && speed != 0) // 정지상태에서 출발이라면
  {
    if (punch_time > 0)
    {
      if (speed * motor_dir > 0)
      {
        analogWrite(M_PWM, punch_pwm);
        digitalWrite(M_DIR1, HIGH);
        digitalWrite(M_DIR2, LOW);
      }
      else if (speed * motor_dir < 0)
      {
        analogWrite(M_PWM, punch_pwm);
        digitalWrite(M_DIR1, LOW);
        digitalWrite(M_DIR2, HIGH);
      }

      delay(punch_time);
    }
  }

  if (speed != 0) // 명령이 정지가 아니라면
  {
    int pwm = abs(speed) * (max_pwm - min_pwm) + min_pwm;           // 0 ~ 255로 변환
    if (speed * motor_dir > 0)
    {
      analogWrite(M_PWM, pwm);
      digitalWrite(M_DIR1, HIGH);
      digitalWrite(M_DIR2, LOW);
    }
    else if (speed * motor_dir < 0)
    {
      analogWrite(M_PWM, pwm);
      digitalWrite(M_DIR1, LOW);
      digitalWrite(M_DIR2, HIGH);
    }
  }

  cur_speed = speed;
}


// 라이트 켜기
void LightON()
{
  digitalWrite(LIGHT, HIGH);
}



// 라이트 끄기
void LightOFF()
{
  digitalWrite(LIGHT, LOW);
}


// 멜로디 시작
void StartMelody()
{
  sound = true;
  melody_index = 0;
  melody_time = millis();
  tone(BUZZER, melody[melody_index]);
}

// 멜로디 종료
void StopMelody()
{
  sound = false;
  noTone(BUZZER);
}

// 멜로디
void PlayMelody()
{
  if (sound == false)
    return;

  unsigned long t = millis();
  if (t - melody_time >= (melody_tempo / duration[melody_index]))
  {
    melody_index++;
    if (melody_index >= melody_num)
      melody_index = 0;

    if (melody[melody_index] == 0)
      noTone(BUZZER);
    else
      tone(BUZZER, melody[melody_index]);
    melody_time = t;
  }
}



// 배터리 체크, 저전압시 경고음
void CheckBattery()
{
  // 1초에 한번씩 체크
  if (millis() - battery_time < 1000)
    return;

  float voltage = (float)analogRead(BATTERY) / 1023;
  voltage *= 5;                                           // 아두이노 5V
  voltage *= 3;                                           // 회로상 전압분배 비율 (VIN을 10K, 20K 저항으로 분배)
  voltage *= voltage_error;

  if (voltage < (2.8 * battery_cell))
  {
    while (true)
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
  if (duration == 0) // 멀어서 에코가 못받음
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
  LightOFF();
  StopMelody();

  SetSteering(0);
  SetSpeed(0);
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



//정지했을때를 인식하는 코드
//2개이상의 변수끼릴 비교를 해야함


//변수 비교 코드
float Diff(int trig, int echo)
{
  float firstd = GetDistance(trig, echo);
  delay(1000);
  float lastd = GetDistance(trig, echo);
  return (firstd - lastd); // 절대값 받아야 하는 경우 abs로 받기
}

void Stop()    // 변수 조정
{
  SetSteering(0); // 없앨지 말지 고민중
  SetSpeed(-0.2);
  delay(200);
}


//후진 코드
// 5로 할지 0으로 할지 고려해야함

void Backward()   // if문으로 바꿀지 생각해봐야함
{
  Stop();
  SetSpeed(0);
}


//회전
void Turn(float flag)
{
  if (flag == 1) // 우회전
  {
    cur_speed = 0.05;
    SetSpeed(cur_speed);  // 충돌방지
    SetSteering(-0.5);  // 원크게돌기위해
    delay(100);                                                                           //delay 값들 조정해야할지도
    cur_steering = 1;
    SetSteering(cur_steering);
    while (abs(right - left) > 50) //  좌우 센서로 받는게 최선인지 확인해야함       //회전후 어디로 쏠리느냐에 따라 right-left, left-right 결정해야할듯     //유턴할때 약간 코드가 겹칠거같기도하고
    {

      f_center1 = GetDistance(FC_TRIG, FC_ECHO);
      left1 = GetDistance(L_TRIG, L_ECHO);
      right1 = GetDistance(R_TRIG, R_ECHO);
      f_left1 = GetDistance(FL_TRIG, FL_ECHO);
      f_right1 = GetDistance(FR_TRIG, FR_ECHO);
      delay(100);
      f_center = GetDistance(FC_TRIG, FC_ECHO);
      left = GetDistance(L_TRIG, L_ECHO);
      right = GetDistance(R_TRIG, R_ECHO);
      f_left = GetDistance(FL_TRIG, FL_ECHO);
      f_right = GetDistance(FR_TRIG, FR_ECHO);


      f = f_center1 - f_center;
      l = left1 - left;
      r = right1 - right;
      fl = f_left1 - f_left;
      fr = f_right1 - f_right;

      cur_steering = cur_steering - 0.05;
      SetSteering(cur_steering);
      SetSpeed(cur_speed);

      if  ((f < 3) + (fl < 3) + (fr < 3) + (l < 3) + (r < 3) >= 4)//정지해있을때 나와야함
      {
        break;
      }
    }
    SetSteering(0);
    //직진코드 받기
  }
  else if (flag == -1) // 좌회전
  {
    cur_speed = 0.05;
    SetSpeed(cur_speed);  // 충돌방지
    SetSteering(0.5);
    delay(100);
    cur_steering = -1;
    SetSteering(cur_steering); //원크게돌기위해
    while ( abs(left - right) > 50)                                         
    {

      f_center1 = GetDistance(FC_TRIG, FC_ECHO);
      left1 = GetDistance(L_TRIG, L_ECHO);
      right1 = GetDistance(R_TRIG, R_ECHO);
      f_left1 = GetDistance(FL_TRIG, FL_ECHO);
      f_right1 = GetDistance(FR_TRIG, FR_ECHO);
      delay(100);
      f_center = GetDistance(FC_TRIG, FC_ECHO);
      left = GetDistance(L_TRIG, L_ECHO);
      right = GetDistance(R_TRIG, R_ECHO);
      f_left = GetDistance(FL_TRIG, FL_ECHO);
      f_right = GetDistance(FR_TRIG, FR_ECHO);




      f = f_center1 - f_center;
      l = left1 - left;
      r = right1 - right;
      fl = f_left1 - f_left;
      fr = f_right1 - f_right;
      cur_steering = cur_steering + 0.05;
      SetSteering(cur_steering);
      SetSpeed(cur_speed);

      if  ((f < 3) + (fl < 3) + (fr < 3) + (l < 3) + (r < 3) >= 4)//정지해있을때 나와야함
      {
        break;
      }
    }
    SetSteering(0);
    //직진코드 받기
  }
}








// 자율주행
void AutoDriving()
{
  if (!autoDriving)
    return;

  // 판단
  // 판단 근거가 없다면 이전 상태와 동일하게 수행



  //while 문에 무조건 Setspeed 넣어줘야 돌아감, 정지했을때 나올수있게 break 필수
  //속도가 계속 주어지나 확인



/*
  SetSpeed(cur_speed);
  delay(1000);
  */

  
  f_center1 = GetDistance(FC_TRIG, FC_ECHO);
  left1 = GetDistance(L_TRIG, L_ECHO);
  right1 = GetDistance(R_TRIG, R_ECHO);
  f_left1 = GetDistance(FL_TRIG, FL_ECHO);
  f_right1 = GetDistance(FR_TRIG, FR_ECHO);
  delay(100);
  f_center = GetDistance(FC_TRIG, FC_ECHO);
  left = GetDistance(L_TRIG, L_ECHO);
  right = GetDistance(R_TRIG, R_ECHO);
  f_left = GetDistance(FL_TRIG, FL_ECHO);
  f_right = GetDistance(FR_TRIG, FR_ECHO);




  f = f_center1 - f_center;
  l = left1 - left;
  r = right1 - right;
  fl = f_left1 - f_left;
  fr = f_right1 - f_right;



  if ((-fl > 50) || (f_left > 400 && f_right < 400) ) //좌전방이 트일때  좌회전  //유턴 고려 // 400 조정해야함
  {
    Turn(-1);
  }
  else if ( (-fr > 50) || (f_left < 400 && f_right > 400)) //우전방이 트일때  우회전
  {
    Turn(1);
  }
  else if ((f < 3) + (fl < 3) + (fr < 3) + (l < 3) + (r < 3) >= 4) //5개 센서중 4개 이상이 거리차가 없을때 후진
  {
    while (f_center<100)
    {
      Backward();  //backward 자체를 if로 바꿀지 고려해야함
      f_center=GetDistance(FC_TRIG,FC_ECHO);
    }
  }
  else //직진
  {
    if (f_center <= 600)  //전방 장애물을 만났을때                                          //숫자 조정필요할듯
    {
      SetSpeed(0);
      SetSteering(0);
      if (f_center <= 400 && backBool == false)                                           //숫자조정필요할듯
      {
        Stop();                                                                           //뒤로 가는지 정확하게 확인하기: 안가면 뒤로가는 코드 추가
        backBool == true;
      }
    }
    else  // 방향 조정 직진
    {
      if (left - right < 50 && left>right) // 우측쏠림
      {
        cur_steering = -0.2;                                                            // 조정 해야함
        cur_speed = 0.8;                                                                // 조정 해야함
        SetSteering(cur_steering);
        SetSpeed(cur_speed);
        while (cur_steering != 0)
        {
          cur_steering = cur_steering + 0.01;
          SetSteering(cur_steering);
          SetSpeed(cur_speed);

          f_center1 = GetDistance(FC_TRIG, FC_ECHO);
          left1 = GetDistance(L_TRIG, L_ECHO);
          right1 = GetDistance(R_TRIG, R_ECHO);
          f_left1 = GetDistance(FL_TRIG, FL_ECHO);
          f_right1 = GetDistance(FR_TRIG, FR_ECHO);
          delay(100);
          f_center = GetDistance(FC_TRIG, FC_ECHO);
          left = GetDistance(L_TRIG, L_ECHO);
          right = GetDistance(R_TRIG, R_ECHO);
          f_left = GetDistance(FL_TRIG, FL_ECHO);
          f_right = GetDistance(FR_TRIG, FR_ECHO);

          f = f_center1 - f_center;
          l = left1 - left;
          r = right1 - right;
          fl = f_left1 - f_left;
          fr = f_right1 - f_right;

          if  ((f < 3) + (fl < 3) + (fr < 3) + (l < 3) + (r < 3) >= 4)  //정지해있을때 나와야함
          {
            break;
          }
        }
      }
      else if (right - left < 50 && left<right)//좌측쏠림
      {
        cur_steering = 0.2;  // 조정 해야함
        cur_speed = 0.8; // 조정 해야함
        SetSteering(cur_steering);
        SetSpeed(cur_speed);
        while (cur_steering != 0)
        {
          cur_steering = cur_steering - 0.01;
          SetSteering(cur_steering);
          SetSpeed(cur_speed);

          f_center1 = GetDistance(FC_TRIG, FC_ECHO);
          left1 = GetDistance(L_TRIG, L_ECHO);
          right1 = GetDistance(R_TRIG, R_ECHO);
          f_left1 = GetDistance(FL_TRIG, FL_ECHO);
          f_right1 = GetDistance(FR_TRIG, FR_ECHO);
          delay(100);
          f_center = GetDistance(FC_TRIG, FC_ECHO);
          left = GetDistance(L_TRIG, L_ECHO);
          right = GetDistance(R_TRIG, R_ECHO);
          f_left = GetDistance(FL_TRIG, FL_ECHO);
          f_right = GetDistance(FR_TRIG, FR_ECHO);

          f = f_center1 - f_center;
          l = left1 - left;
          r = right1 - right;
          fl = f_left1 - f_left;
          fr = f_right1 - f_right;

          if  ((f < 3) + (fl < 3) + (fr < 3) + (l < 3) + (r < 3) >= 4)  //정지해있을때 나와야함
          {
            break;
          }
        }
      }
      else // 걍 직진
      {
        cur_speed = 0.8;
        cur_steering = 0;
        SetSpeed(cur_speed);  //최대속도 조정
        SetSteering(cur_steering);
      }
    }
  }
}









/*
  //회전 연습 코드

  SetSpeed(0.3);
  delay(1000);
  SetSpeed(0.01);
  delay(300);
  cur_steering = -0.5; //우회전 전에 크게 돌기
  SetSteering(cur_steering);
  delay(100)  ;
  cur_steering = 1;
  SetSteering(cur_steering) ;
  /*
    while ( cur_steering<1)   //
    {
    cur_steering += 0.1;
    SetSteering(cur_steering);
    delay(200); // 회전 변화를 위해서 필수
    }
  
  while ( cur_steering > 0) //핸들 풀기
  {
    cur_steering -= 0.05;  //1변수
    SetSteering(cur_steering);
    delay(200);  //2변수
  }

  delay(3000);

  SetSteering(0);
  SetSpeed(0.3);
  delay(1000);
  SetSpeed(0.01);
  delay(300);
  cur_steering = 0.5; //좌회전 전에 크게 돌기
  SetSteering(cur_steering);
  delay(100)  ;
  cur_steering = -1;
  SetSteering(cur_steering);
  while ( cur_steering < 0) //
  {
    cur_steering += 0.05;
    SetSteering(cur_steering);
    delay(200);
  }











  /*   메이키스트 코드
    float compute_speed = cur_speed;
    float compute_steering = cur_steering;

    if(cur_speed == 0) // 현재 정지 중인 상태이면
    {
      f_center = GetDistance(FC_TRIG, FC_ECHO);
      if(f_center > center_start) // 전방에 감지되는 것이 충분히 멀다면
      {
          // 출발한다
          compute_speed = 0.1;
          compute_steering = 0;
      }
    }
    else if(cur_speed > 0) // 현재 전진 중인 상태이면
    {
      f_center = GetDistance(FC_TRIG, FC_ECHO);
      f_left = GetDistance(FL_TRIG, FL_ECHO);
      f_right = GetDistance(FR_TRIG, FR_ECHO);
      if(f_center <= center_stop || f_left <= diagonal_stop || f_right <= diagonal_stop)  // 전방에 감지되면
      {
          // 후진한다
          compute_speed = -0.1;
          if(cur_steering > 0)
              compute_steering = -1;
          else
              compute_steering = 1;
      }
      else if(f_left <= diagonal_detect || f_right <= diagonal_detect) // 좌우측방 어느 곳이라도 감지된다면
      {
          if(f_center > center_detect) // 전방은 감지되지 않는다면
          {
              // 좌우측방 중 한 곳만 감지되었음
              if(f_left < f_right) // 좌측방이 감지되었다면
              {
                  // 우측으로 최대 조향
                  compute_steering = 1;
              }
              else if(f_right < f_left) // 우측방이 감지되었다면
              {
                  // 좌측으로 최대 조향
                  compute_steering = -1;
              }
          }
      }
      else
      {
          // 전방과 좌우 센서만으로 제어한다
          // 속도 결정
          if(f_center <= center_detect) // 전방에 감지된다면
          {
              // 거리에 따른 속도 결정
              compute_speed = (float)(f_center - center_stop) / (float)(center_detect - center_stop);
          }
          else // 전방에 없다면
          {
              // 최고 속도
              compute_speed = 1;
          }

          // 조향 결정
          left = GetDistance(L_TRIG, L_ECHO);
          right = GetDistance(R_TRIG, R_ECHO);
          if(left <= side_start && right <= side_start) // 좌우 모두 감지되면
          {
              // 거리차에 따라 조향한다
              float diff = (float)(right - side_stop) - (float)(left - side_stop);
              diff /= (float)(side_start - side_stop);
              // 결과에 Gain을 적용하여 반응성을 높일 수 있다
              compute_steering = diff * steering_gain;
          }
          else if(f_center <= center_detect && (left > side_detect || right > side_detect))
          // 전방은 감지되는데 어느 한쪽이라도 감지되지 않는다면
          {
              if(left <= side_detect) // 좌측이 감지된다면
              {
                  // 우측으로 최대 조향
                  compute_steering = 1;
              }
              else if(right <= side_detect) // 우측이 감지된다면
              {
                  // 좌측으로 최대 조향
                  compute_steering = -1;
              }
          }
      }
    }
    else
    {
      // 현재 후진 중인 상태이면
      f_center = GetDistance(FC_TRIG, FC_ECHO);
      f_right = GetDistance(FR_TRIG, FR_ECHO);
      f_left = GetDistance(FL_TRIG, FL_ECHO);
      if(f_center > center_start && f_left > diagonal_start && f_right > diagonal_start) // 전방에 감지되지 않으면
      {
          // 조향을 반대로 꺾고 회피 종료
          compute_speed = 0.1;
          if(cur_steering > 0) // 우 조향 중이면
              compute_steering = -1;
          else // 좌 조향 중이면
           compute_steering = 1;
      }
    }

    // 제어
    SetSteering(compute_steering);
    SetSpeed(compute_speed);
  */
}








void setup()
{
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
  pinMode(FR_ECHO, INPUT);
  pinMode(L_TRIG, OUTPUT);
  pinMode(L_ECHO, INPUT);
  pinMode(R_TRIG, OUTPUT);
  pinMode(R_ECHO, INPUT);

  digitalWrite(LIGHT, LOW);
  SetSteering(0);
  SetSpeed(0);

  Serial.begin(9600);
  BTSerial.begin(9600);

  battery_time = millis();
}

void loop()
{
  
  //CheckBattery();

  /*
      수신데이터 샘플
      T:1
      P:1
      L:1
      S:1
      A:1
  */
  if (BTSerial.available() > 0)
  {
    String packet = BTSerial.readStringUntil('\n');
    if (packet != 0)
    {
      int index = packet.indexOf(':');
      if (index >= 0)
      {
        String cmd = packet.substring(0, index);
        String param = packet.substring(index + 1);
        if (cmd.equals("T") && !autoDriving)        // 조향명령
        {
          SetSteering(param.toFloat());
          rc_time = millis();
        }
        else if (cmd.equals("P") && !autoDriving)   // 속도명령
        {
          SetSpeed(param.toFloat());
          rc_time = millis();
        }
        else if (cmd.equals("L") && !autoDriving)   // 라이트명령
        {
          if (param.toInt() == 1)
            LightON();
          else
            LightOFF();
        }
        else if (cmd.equals("S") && !autoDriving)   // 사운드명령
        {
          if (param.toInt() == 1)
            StartMelody();
          else
            StopMelody();
        }
        else if (cmd.equals("A"))                   // 자율주행명령
        {
          if (param.toInt() == 1)
            StartAutoDriving();
          else
            StopAutoDriving();
        }
      }
    }
  }

  PlayMelody();

  AutoDriving(); //자율주행 시작

  // 직접 컨트롤시 3초 동안 블루투스 신호 없으면 정지
  if (!autoDriving)
  {
    if (millis() - rc_time > 3000)
      SetSpeed(0);
  }
}
