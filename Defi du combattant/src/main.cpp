#include <Arduino.h>
#include <LibRobus.h>

void MOTOR_SetSpeed(uint8_t id, float speed);
void SERVO_Enable(uint8_t id);
bool ROBUS_IsBumper(uint8_t id);
int32_t ENCODER_Read(uint8_t id);

int CalculerAngleDepart(int *sensAngle, int AngleInitial, int couleur);
float LireDistance();

void defiParcours();

//void acceleration(void);
void Mouvement(float dist);
void Tourner(int dir, int Angle);
float FonctionPID(float distMotDroite, float distMotGauche); 

#define ANGLE_INITIAL_ROBOT_A 90
#define ANGLE_INITIAL_ROBOT_B 270

#define DISTANCE_DU_CENTRE 30 //La distance initiale entre les roues et le centre de l arene en cm

#define ROUGE 0
#define VERT 1
#define BLEU 2
#define JAUNE 3

#define ANGLE_ROUGE 45
#define ANGLE_VERT 135
#define ANGLE_BLEU 225
#define ANGLE_JAUNE 315
int TableauAnglesCouleurs[4] = {ANGLE_ROUGE, ANGLE_VERT, ANGLE_BLEU, ANGLE_JAUNE};

int Start = 0;
int Bumper3 = 0;
int Bumper2 = 0;
float Distactuel = 0;
int Distvoulu = 0;
int AngleActuel = 0;
float VitesseactuelG = 0;
float VitesseactuelD = 0;
float VitesseactuelG2 = 0;
float VitesseactuelD2 = 0;  
int32_t EncoderG = 0;
int32_t EncoderD = 0;

float distTotMotDroite = 0;
float distTotMotGauche = 0;

void setup() {
  // put your setup code here, to run once:
  BoardInit();
  Serial.begin(9600);
  delay(1500);
  MOTOR_SetSpeed(0, 0); // Moteur gauche
  MOTOR_SetSpeed(1, 0); // Moteur droit
}

void loop() 
{
  // put your main code here, to run repeatedly:

  if(ROBUS_IsBumper(3))
  {
    Mouvement(DISTANCE_DU_CENTRE);

    int sensAngleDepart = 1;

    int angleInitial = ANGLE_INITIAL_ROBOT_A;
    int couleur = ROUGE;

    int angleDepart = CalculerAngleDepart(&sensAngleDepart, angleInitial, couleur);

    Tourner(sensAngleDepart, angleDepart);
  }
}

//Calcule l'angle le plus court que peut faire le robot en fonction de son angle initial et de la couleur qu'il veut atteindre
int calculerAngleDepart(int *sensAngle, int angleInitial, int couleur)
{
  int angle = TableauAnglesCouleurs[couleur] - angleInitial;
  *sensAngle = 1;

  if(angle < 0)
  {
    angle *= -1;
    *sensAngle = -1;
  }

  if(angle > 180)
  {
    angle = 360 - angle;
    *sensAngle = -1;
  }

  return angle;
}

float LireDistance()
{
  float brut = ROBUS_ReadIR(0) / 200.0;
  Serial.println(brut);
  Serial.println(analogRead(0));
  double distance, step1, step2, step3;
  step1 = (brut - 49.8115) / (brut - 0.230724);
  step2 = 0.679454 * -1 * step1;
  step3 = pow(step2, 0.899171);
  
  /*Serial.print("\n"); Serial.print(step1);
  Serial.print("\n"); Serial.print(step2);
  Serial.print("\n"); Serial.print(step3);*/

  distance = pow(0.679454 * -1 * ((brut-49.8115) / (brut - 0.230724)), (125000.0 / 139017.0));

  return distance;
  }

void Tourner(int dir, int Angle) //dir = -1 pour tourner a gauche et dir = 1 pour tourner à droite
{
  AngleActuel = 0;

  ENCODER_ReadReset(0);
  ENCODER_ReadReset(1);

  while(AngleActuel <= Angle && !ROBUS_IsBumper(2))
  {
    EncoderG = ENCODER_Read(0);
    EncoderD = ENCODER_Read(1);

    AngleActuel=(EncoderG)/(22.0418)*dir;

    /*Serial.println(EncoderG);
    Serial.println(EncoderD);
    Serial.println("\n");*/

    if (dir < 0)
    {
      MOTOR_SetSpeed(0,-0.2); // Moteur gauche
      MOTOR_SetSpeed(1, 0.2); // Moteur droit
    }
    else
    {
      MOTOR_SetSpeed(0, 0.2); // Moteur gauche
      MOTOR_SetSpeed(1, -0.2); // Moteur droit
    }
    //delay(75);
  }

  MOTOR_SetSpeed(0, 0); // Moteur gauche
  MOTOR_SetSpeed(1, 0); // Moteur droit

  delay(50);
}


void Mouvement(float dist)
{ 
  double accel = 0;
  double distAccel = 30;

  ENCODER_ReadReset(0);
  ENCODER_ReadReset(1);

  while(Distactuel <= dist && !ROBUS_IsBumper(2))
  {
    if(distAccel >= (dist/2))
    {
      if(Distactuel < dist/2)
      {
        accel = (Distactuel / distAccel)*0.8;
       //Serial.println("1");
      }
      else

      {
        accel = ((dist-Distactuel)/distAccel)*0.8;
        //Serial.println("2");
      }
    }
    else if(Distactuel < distAccel)
    {
      accel = (Distactuel / distAccel)*0.8 ;
      //Serial.println("3");
    }
    else if(dist-Distactuel <= distAccel)
    {
      accel = ((dist-Distactuel)/distAccel)*0.8 ;
      //Serial.println("4");
    }
    else
    {
      accel = 0.8;
    }
    EncoderG=ENCODER_Read(0);
    EncoderD=ENCODER_Read(1);
    Distactuel=(EncoderG)/(133.6675);
    // Serial.println(EncoderD);
    // Serial.println(EncoderG);
    //Serial.println("\n");

    MOTOR_SetSpeed(0, accel + 0.1 - FonctionPID(ENCODER_Read(0),ENCODER_Read(1))); // Moteur gauche
    MOTOR_SetSpeed(1, accel + 0.1 + FonctionPID(ENCODER_Read(0),ENCODER_Read(1))); // Moteur droit
    //delay(50);
  }
  
  MOTOR_SetSpeed(0, 0); // Moteur gauche
  MOTOR_SetSpeed(1, 0); // Moteur droit
  Distactuel = 0;

  delay(100);
}

float FonctionPID(float distMotDroite, float distMotGauche)
{ 
  float kp = 0.001;
  float ki = 0.002;
  float diffDist = 0;
  float P = 0; // P = Produit de la difference dans un PID
  float diffDistTotal = 0;
  float I = 0; // I = Integrale de la difference dans un PID
  float vitMot1 = 0;
  float distMD = 0;
  float distMG = 0;
  
  distMD = distMotDroite - distTotMotDroite;
  distMG = distMotGauche - distTotMotGauche;

  diffDist = distMD - distMG; 
  diffDistTotal = distMotDroite - distMotGauche;

  P = diffDist * kp;
  I = diffDistTotal * ki;
  vitMot1 = (P+I)/2;

  distTotMotDroite = distMotDroite;
  distTotMotGauche = distMotGauche;

  return vitMot1;
}

void defiParcours()
{
    //Aller
    Mouvement(230);
    Tourner(-1, 88);
    Mouvement(100);
    Tourner(1, 85);
    Mouvement(45);
    Tourner(1, 85);
    Mouvement(55);
    Tourner(-1,88);
    Mouvement(105);
    Tourner(1,85);
    Mouvement(60);
    Tourner(-1, 81);
    Mouvement(122.5);

    //Ballon
    Tourner(-1, 177);
    Mouvement(225);
    Tourner(-1, 177);

    //Retour
    Mouvement(95);
    Tourner(-1,88);
    Mouvement(50);
    Tourner(-1,88);
    Mouvement(105);
    Tourner(1,85);
    Mouvement(55);
    Tourner(-1,88);
    Mouvement(45);
    Tourner(-1,88);
    Mouvement(100);
    Tourner(1,88);
    Mouvement(245);
    Tourner(-1,720);
}

/* 
void acceleration(void)
{
  Start = ROBUS_IsBumper(3);
  Bumper2 = ROBUS_IsBumper(2);
  //VitesseactuelG = ENCODER_Read(0);//moteur gauche
  //VitesseactuelD = ENCODER_Read(1);//moteur droit
  if(Start)
  {
    VitesseactuelG=0.1; 
    VitesseactuelD=0.1;
    delay(500);
  }

  if(VitesseactuelG>0 && VitesseactuelD>0)
  { 
    EncoderG=ENCODER_Read(0);
    EncoderD=ENCODER_Read(1);
    Distactuel=(EncoderG)/(133.6675);
    Serial.println(Distactuel);
    Distvoulu=300;
    if((Distvoulu/2)<=Distactuel)
    {
      VitesseactuelG=0;
      VitesseactuelD=0;
    }  
    if(Distactuel>=Distvoulu)
    {
      MOTOR_SetSpeed(0, 0); // Moteur gauche
      MOTOR_SetSpeed(1, 0); // Moteur droit
    }

    if(VitesseactuelG<=1 && VitesseactuelD<=1)
    {
      if(VitesseactuelG<1 && VitesseactuelD<1)
      {  
        VitesseactuelD=VitesseactuelD+0.1;
        VitesseactuelG=VitesseactuelG+0.1;
        MOTOR_SetSpeed(0, VitesseactuelG); // Moteur gauche
        MOTOR_SetSpeed(1, VitesseactuelD); // Moteur droit
        delay(500);
        VitesseactuelG2 = VitesseactuelG;
        VitesseactuelD2 = VitesseactuelD;
      }
    }
  }   

      if(VitesseactuelG>=1 && VitesseactuelD>=1)
      {
        if(VitesseactuelG2 >= 0.1 && VitesseactuelD2 >= 0.1)
        {
          VitesseactuelD2=VitesseactuelD2-0.1;
          VitesseactuelG2=VitesseactuelG2-0.1;

          MOTOR_SetSpeed(0, VitesseactuelG2); // Moteur gauche
          MOTOR_SetSpeed(1, VitesseactuelD2); // Moteur droit
          delay(500);
        } 
        else
        {
          //VitesseactuelG = 0;
          //VitesseactuelD = 0;
          MOTOR_SetSpeed(0, VitesseactuelD2); // Moteur gauche
          MOTOR_SetSpeed(1, VitesseactuelD2); // Moteur droit
        }
      } 

  if(Bumper2)
  {
    MOTOR_SetSpeed(0, 0); // Moteur gauche
    MOTOR_SetSpeed(1, 0); // Moteur droit
  }

  Serial.println(Bumper3);
}
*/