#include <PID_v1.h>
#include <Encoder.h>
#define LEFT  0
#define RIGHT 1
//Frecuencia a la que se llama al PID para recalcular la accion de control
#define CONTROL_MOTOR_SPEED_PERIOD       10  //hz
//Pines donde están conectados los motores
int MotorR = 6; 
int MotorL = 7;
//Handler para los encoders y pines donde se encuentran conectados
Encoder leftEnc(18, 19);
Encoder rightEnc(2, 3);
//-------------MOTOR CONTROL VARIABLES--------------------------------------------
long leftOldPosition, leftNewPosition, rightOldPosition, rightNewPosition  = 0;
double leftVelocity, rightVelocity = 0;
unsigned long t1, t2, previousSample;
int update_d = 1;
double difLeftPulses, difRightPulses, dift;
double SetLeftVelocity, OutputLeftMotor, SetRightVelocity, OutputRightMotor;
//Specify the links and initial tuning parameters
double Kp = 0.005, Ki = 0.035, Kd = 0.0005;
int samplingTime = 100;
double vLeft, vRight = 0;
int vEncLeft, vEncRight = 0;
double prev_vLeft, prev_vRight = 0;
PID leftPID(&vLeft, &OutputLeftMotor, &SetLeftVelocity, Kp, Ki, Kd, DIRECT);
PID rightPID(&vRight, &OutputRightMotor, &SetRightVelocity, Kp, Ki, Kd, DIRECT);
float LinearVelocity, AngularVelocity = 0;
const float diameterWheel = 0.135; //diameter in meters
const float radiusWheel=0.0675;
const float wheelDistance = 0.275; // distance between both wheels in meters
const float TICK2RAD = 0.00236922;
//Warning: Bigger than 127.5 going backward
//         Smaller than 127.5 going forward
float pwmR = 127.5;
float pwmL = 127.5;
//================================================================================

void setup() {
   // pines de los motores
   pinMode(MotorR, OUTPUT);
   pinMode(MotorL, OUTPUT);
   // pines de los encoders(2 por encoder)
   pinMode(2, INPUT);
   pinMode(3, INPUT);
   pinMode(18, INPUT);
   pinMode(19, INPUT);
   leftPID.SetMode(AUTOMATIC);
   rightPID.SetMode(AUTOMATIC);
   //Dejar los motores parados al principio
   analogWrite( MotorR, 127.5);
   analogWrite( MotorL, 127.5);
}

void loop() {
   //Al robot se le dan comandas de velocidad lineal (LinearVelocity) y 
   // velocidad angular (AngularVelocity) y mediante esta fórmula se determina
   //la velocidad angular de cada rueda en [pulsos_encoder/segundo]
   vRight = LinearVelocity + (AngularVelocity * wheelDistance) / 2;
   vLeft = LinearVelocity - (AngularVelocity * wheelDistance) / 2;
   SetLeftVelocity = (vLeft * 2652) / ((diameterWheel) * 3.1416);
   SetRightVelocity = (vRight * 2652) / ((diameterWheel) * 3.1416);   
   // Dependiendo de como este conectados los motores al arduino habrá que cambiar
   // los signos, asi para unos robots no hara falta cambiar el signo y para otros si.
   SetLeftVelocity = -SetLeftVelocity;
   SetRightVelocity = -SetRightVelocity;
  
   //Dependiendo de si la velocidad es positiva o negativa habrá que configurar el PID de
   //cada rueda para acotar las acciones de control.
   if (SetLeftVelocity >= 0)
     {
      leftPID.SetOutputLimits(127.5, 255);
     }
   if (SetLeftVelocity < 0)
     {
      leftPID.SetOutputLimits(0, 127.5);
     }
   if (SetRightVelocity >= 0)
     {
      rightPID.SetOutputLimits(127.5, 255);
     }
   if (SetRightVelocity < 0)
     {
      rightPID.SetOutputLimits(0, 127.5);
     }     
    //Leer estado de los encoder en periodo 1 
    if (update_d==1)
     {    
       tTime[0] = millis(); 
       update_d=0;
       rightOldPosition = rightEnc.read();
       leftOldPosition =  leftEnc.read();
     }    
     t2=millis();
     //Una vez hallán transcurrido [1000 / CONTROL_MOTOR_SPEED_PERIOD], volver 
     // a leer estado de los encoders, calcular velocidad ruedas, y llamar al PID
     // para calcular la accion de control.
     if ((t2-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_PERIOD))
      {    
       update_d=1;
       rightNewPosition = rightEnc.read();
       leftNewPosition = leftEnc.read();
          
       difRightPulses= rightNewPosition-rightOldPosition ;
       dift=(t2-tTime[0]);
       vRight = difRightPulses/dift; //Pulses/second
       vRight = difRightPulses/dift*1000;
       vEncRight = vRight;
      
       difLeftPulses= leftNewPosition-leftOldPosition ;
       vLeft = difLeftPulses/dift; //Pulses/second
       vLeft = difLeftPulses/dift*1000;
       vEncLeft = vLeft;
       
       rightPID.Compute();
       leftPID.Compute();
       
       // Aplicar acción de control en los motores       
       analogWrite(MotorL, OutputLeftMotor);
       analogWrite(MotorR, OutputRightMotor); 
      }
}
