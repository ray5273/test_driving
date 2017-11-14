#include <Servo.h>

// Pin map
#define FC_TRIG 6   // 전방 초음파 센서 TRIG 핀
#define FC_ECHO 10 // 전방 초음파 센서 ECHO 핀
#define FL_TRIG 11  // 전방좌측 초음파 센서 TRIG 핀
#define FL_ECHO 12  // 전방좌측 초음파 센서 ECHO 핀
#define FR_TRIG 3   // 전방우측 초음파 센서 TRIG 핀
#define FR_ECHO 4   // 전방우측 초음파 센서 ECHO 핀
#define L_TRIG  A2  // 좌측 초음파 센서 TRIG 핀
#define L_ECHO  A1  // 좌측 초음파 센서 ECHO 핀
#define R_TRIG  A4   // 우측 초음파 센서 TRIG 핀
#define R_ECHO  A5  // 우측 초음파 센서 ECHO 핀
#define M_PWM   5   // DC모터 PWM 핀
#define M_DIR1  8   // DC모터 DIR1 핀
#define M_DIR2  7  // DC모터 DIR2 핀
#define SERVO   9   // 서보모터 핀
#define BUZZER   3   // 버저 핀
#define BATTERY  A0  // 배터리 체크 핀

#define MAX_DISTANCE  800 // 초음파 센서의 최대 감지거리

// 자동차 튜닝 파라미터
int servo_dir = 1; // 서보 회전 방향(동일: 1, 반대:-1)
int motor_dir = 1; // 모터 회전 방향(동일:1, 반대:-1)
int angle_limit = 35; // 서보 모터 회전 제한 각 (단위: 도)
int angle_offset = 0; // 서보 모터 중앙각 오프셋 (단위: 도)
int max_pwm = 200; // 모터 최대 출력 (0 ~ 255)
int min_pwm = 110; // 모터 최소 출력 (0 ~ 255)
int punch_pwm = 255; // 정지 마찰력 극복 출력 (0 ~ 255)
int punch_time = 200; // 정지 마찰력 극복 시간 (단위 msec)
int stop_time = 300; // 전진후진 전환 시간 (단위 msec)
int battery_cell = 2; // 배터리 셀 개수
float voltage_error = 1.08; // 전압 오차 (1이 오차 없음)
// 자율주행 튜닝 파라미터
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


Servo servo;
float f_center;
float f_left;
float f_right;
float left;
float right;
float cf_left;
float cf_right;


float cur_steering;
float cur_speed;
unsigned long battery_time;



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

void SetSpeed(float speed)
{
  speed = constrain(speed, -1, 1);

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
    if(punch_time > 0)
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
    int pwm = abs(speed) * (max_pwm - min_pwm) + min_pwm;
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



//회전
void Turn(float flag)
{
  SetSpeed(0);
  if (flag==1) // 우회전
  {
    SetSteering(-0.5);  // 원크게돌기위해
    SetSpeed(0.1);  // 충돌방지
    while (-20<f_left-f_right<20)   //
    {
      
      cur_steering=cur_steering+0.05;
      SetSteering(cur_steering);
      
    }
  }
  else if (flag==-1) // 좌회전
  {
    SetSteering(0.5);  // 원크게돌기위해
    SetSpeed(0.1);  // 충돌방지
    while (-20<f_left-f_right<20)   //
    {
      
      cur_steering=cur_steering-0.05;
      SetSteering(cur_steering);
    }
  }
}






float GetDistance(int trig, int echo)
{ 
  // Range: 3cm ~ 75cm
  digitalWrite(trig, LOW);
  delayMicroseconds(4);
  digitalWrite(trig, HIGH);
  delayMicroseconds(20);
  digitalWrite(trig, LOW);
  
  unsigned long duration = pulseIn(echo, HIGH, 5000);
 
  if(duration == 0) // 멀어서 에코가 못받음
      return MAX_DISTANCE;
  else
      return duration * 0.17;
}

/*void CheckBattery()
{
  if(millis() - battery_time < 1000)
    return;
  
  float voltage = (float)analogRead(BATTERY) / 1023;
  voltage *= 5;
  voltage *= 3;
  voltage *= voltage_error;

  if(voltage < (2.8 * battery_cell))
  {
    while(true)
    {
      tone(BUZZER, 523);
      delay(1000);

      noTone(BUZZER);
      delay(1000);      
    }
  }

  battery_time = millis();
}
*/
void setup()
{
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
  
  servo.attach(SERVO);    
  pinMode(M_PWM, OUTPUT);
  pinMode(M_DIR1, OUTPUT);
  pinMode(M_DIR2, OUTPUT);

  SetSteering(0);
  SetSpeed(0);

  battery_time = millis();
}

void loop()
{
//CheckBattery();

  // 판단
  // 히스테리시스 법칙에 따른다 (판단 근거가 없다면 이전 상태와 동일하게 수행)
  float compute_speed = cur_speed;
  float compute_steering = cur_steering;


//직진 기본루프
f_center=GetDistance(FC_TRIG,FC_ECHO);
left=GetDistance(L_TRIG,L_ECHO);
right=GetDistance(R_TRIG,R_ECHO);
f_left=GetDistance(FL_TRIG,FL_ECHO);
f_right=GetDistance(FR_TRIG,FR_ECHO);

if (f_center<500) // 전방에감지될때
{
  SetSpeed(0);
}
else 
{
  SetSpeed(0.5); //최대속도 정해야함
  //조향   
cf_left=GetDistance(FL_TRIG,FL_ECHO);
cf_right=GetDistance(FR_TRIG,FR_ECHO);
if (cf_left-f_left>50)  // 좌전방이 트일때 좌회전
{
  Turn(1);
}
 else if (cf_right-f_right>50)
 {
   Turn(-1);
 }


  
  if (left-right<50) // 우측쏠림
  {
    cur_steering=-0.3;
    SetSteering(cur_steering);
    while (cur_steering==0)
    {
      cur_steering=cur_steering+0.01;
      SetSteering(cur_steering);
    }
  }
  else if (right-left<50) //좌측쏠림
  {
    cur_steering=0.3;
    SetSteering(cur_steering);
    while (cur_steering==0)
    {
      cur_steering=cur_steering-0.01;
      SetSteering(cur_steering);
    }
  }
  
}



/*
//조향   cf 기존값
cf_left=GetDistance(FL_TRIG,FL_ECHO);
cf_right=GetDistance(FR_TRIG,FR_ECHO);


if (cf_left-f_left>50)  // 좌전방이 트일때 좌회전
{
  Turn(1);
}
 else if (cf_right-f_right>50)
 {
   Turn(-1);
 }










/*
  
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



  
  else if(cur_speed > 0) // 현재 주행 중인 상태이면
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


    
    else if(f_left <= diagonal_detect || f_right <= diagonal_detect) //  좌우측방 어느 곳이라도 감지된다면
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
  SetSpeed(compute_speed);.
  */


}
