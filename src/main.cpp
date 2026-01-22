#include <Pixy2.h>
#include <powerTrain.h>
#include <purePursuit.h>
#include <geometry2D.h>
#include "robotFSM.h"

Pixy2 pixy;

#define THROTTLE_BASE     0.35f
#define LOOKAHEAD_MIN_M   0.032f

#define THROTTLE_SEARCH   0.33f
#define STEER_SEARCH      0.6f      // rotire pe loc

extern float debug_left_motor;
extern float debug_right_motor;

RobotFSM fsm;


void setup()
{
  Serial.begin(115200);
  Serial.println("Starting Pixy2 Tracking + PurePursuit + FSM");

  pixy.init();
  pixy.changeProg("line");

  pinMode(3, OUTPUT);   // stanga spate
  pinMode(5, OUTPUT);   // stanga fata
  pinMode(6, OUTPUT);   // dreapta spate
  pinMode(9, OUTPUT);   // dreapta fata

  FSM_Init(&fsm);
}



void loop()
{
  int8_t result = pixy.line.getMainFeatures();
  bool lineDetected = (result > 0 && pixy.line.numVectors > 0);

  float Kp_center = 0.018f;     
float Kd_center = 0.0035f;   

float last_center_error = 0.0f;
unsigned long last_pid_time = 0;


  if (!lineDetected)
  {
    // stop dupa 300ms de cautat linie
    if (FSM_ShouldStop(&fsm))
    {
      write_powertrain(0.0f, 0.0f);
      Serial.println("STOP (300ms no line)");
      return;
    }

    // altfel: cautare
    FSM_OnLineLost(&fsm);

    if (fsm.currentState == FSM_SEARCH_LEFT)
    {
      write_powertrain(THROTTLE_SEARCH, -STEER_SEARCH);
      Serial.println("SEARCH_LEFT");
    }
    else
    {
      write_powertrain(THROTTLE_SEARCH, +STEER_SEARCH);
      Serial.println("SEARCH_RIGHT");
    }

    delay(40);
    return;
  }
  
  //linie gasita

  // avem linie, resetam timerul
  fsm.currentState = FSM_FOLLOW_LINE;
  fsm.lastSeenTime = millis();

  float centerX = 39.0f;
  float bottomY= 51.0f; //centrul camerei

  // vectorul cel mai apropiat de centru
  int bestIndex = -1;
  float bestScore = 1e9f;





  for (int i = 0; i < pixy.line.numVectors; i++)
  {
    int vx0 = pixy.line.vectors[i].m_x0;
    int vx1 = pixy.line.vectors[i].m_x1;



    float score = fabsf(vx0 - centerX) + fabsf(vx1 - centerX);

    if (score < bestScore)
    {
      bestScore = score;
      bestIndex = i;
    }
  }

  if (bestIndex < 0)
  {
    write_powertrain(0.0f, 0.0f);
    Serial.println("STOP (no valid vector)");
    return;
  }

  int x0 = pixy.line.vectors[bestIndex].m_x0;
  int y0 = pixy.line.vectors[bestIndex].m_y0;
  int x1 = pixy.line.vectors[bestIndex].m_x1;
  int y1 = pixy.line.vectors[bestIndex].m_y1;

  // ==================== PID LATERAL ======================
float line_center = (x0 + x1) * 0.5f;
float center_error = (centerX - line_center);  // PIXELI

unsigned long now = millis();
float dt = (now - last_pid_time) * 0.001f;
if (dt > 0.2f) dt = 0.2f;  // protecție

float derivative = (center_error - last_center_error) / dt;

float correction = Kp_center * center_error + Kd_center * derivative;

last_center_error = center_error;
last_pid_time = now;

// ========================================================


  Serial.print("Vector central: ");
  Serial.print(x0); Serial.print(", "); Serial.print(y0);
  Serial.print(" -> ");
  Serial.print(x1); Serial.print(", "); Serial.println(y1);

  LineABC lane = points2lineABC({(float)x0, (float)y0},
                                {(float)x1, (float)y1});

  Point2D rearAxe = {39.0f, 51.0f};

  PurePursuitInfo info = purePursuitComputeABC(
      rearAxe, lane, WHEELBASE_M, LOOKAHEAD_MIN_M
  );

 float steering = info.steeringAngle + correction;

// Limitare siguranță
if (steering > 0.7f) steering = 0.7f;
if (steering < -0.7f) steering = -0.7f;

fsm.lastSteering = steering;


  Serial.print("Steer deg: ");
  Serial.println(degrees(steering));


  write_powertrain(THROTTLE_BASE, steering);

  Serial.print("L: ");
  Serial.print(debug_left_motor);
  Serial.print("  R: ");
  Serial.println(debug_right_motor);

  delay(40);
}
