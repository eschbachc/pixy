// PixyRacer_1
// Huai-Ti Lin [Mar 2016]
// Janelia Mechatronics & Robotics Club
// This is a demonstration code for controlling the differential drive system for the Pixy Racer Robot (Arduino Uno)
// v3 demonstrate the object following + line following behavior

//// Set up Pixy
#include <SPI.h>  
#include <Pixy.h>
Pixy pixy;

#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)       
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)

class ServoLoop
{
public:
  ServoLoop(int32_t pgain, int32_t dgain);

  void update(int32_t error);
   
  int32_t m_pos;
  int32_t m_prevError;
  int32_t m_pgain;
  int32_t m_dgain;
};

ServoLoop panLoop(300, 500); // define objects
ServoLoop tiltLoop(500, 700);

ServoLoop::ServoLoop(int32_t pgain, int32_t dgain)
{
  m_pos = PIXY_RCS_CENTER_POS;
  m_pgain = pgain; // head turning gain
  m_dgain = dgain;
  m_prevError = 0x80000000L;
}
int dt = 20; // 20ms time interval for 50Hz
int timeout = dt*3; // check timeout
long currentTime =0;
long lastTime =0;
//boolean block_exist;

//// Setup motor system
#define leftWheelforward 5
#define leftWheelbackward 6
#define rightWheelforward 9
#define rightWheelbackward 10

int LMotor = 0;
int RMotor =0; 
float deadband = 0; // 5% drive is daedband
float LDrive = 0;  // initialize the left wheel state 
float RDrive = 0;  // initialize the right wheel state 
float synDrive = 0; // synchronous drive level
float throttle = 0; // this is the total drive level [-100~100]; abs(throttle)<30 doesn't do much
float driveGain = 1;
float bias = 0.5;  // this ratio determines the differential drive [0~1]
float diffGain = 0; // this is the drive level allocated for turning [0~1]; dynamically modulate this!!
int input;  
int duty;
int dr; // test drive level
float h_pgain = 0.5; // body turning gain
float h_dgain = 0;  // body turning gain
float targetSize = 20; // distance tracking target size: 16cm for orange cone width  
float targetSize2 = 10; 

void setup() {
  Serial.begin(9600);
  pixy.init();
  analogWrite(leftWheelforward, 0); // set all motor to zero
  analogWrite(leftWheelbackward, 0);
  analogWrite(rightWheelforward, 0);
  analogWrite(rightWheelbackward, 0);
} 

void loop() {
  
  currentTime= millis();
    
    static int i = 0;
    int j;
    uint16_t blocks;
    char buf[32]; 
    String sig1_list, sig2_list, sig3_list, sig4_list; 
    int sig1, sig2, sig3, sig4;
    int32_t panError, tiltError;
    int32_t turnError, distError;
    
    blocks = pixy.getBlocks();
  //  Serial.println(blocks);
    if (blocks)  {      
      lastTime = currentTime;
      for (int k=0; k < blocks && k < 10; k++) { // we only evaluate 10 blocks or all blocks whichever smaller
        int sig = pixy.blocks[k].signature;
        if (sig == 1) { //signature 1 is chasing object
        sig1 = k; 
//        sig1_list.concat(k);
        }
        else if (sig == 2) { //signature 2 is mid-line
        sig2 = k; 
//        sig2_list.concat(k);
        }
        else if (sig == 3) { //signature 3 is left mark
        sig3 = k;
//        sig3_list.concat(k);
        } 
        else if (sig == 4) { //signature 4 is right mark
        sig4 = k;
//        sig4_list.concat(k);
        }       
      }
      
      if (pixy.blocks[0].signature == 1) { // if the largest block is the object to pursue, then prioritize this behavior
        panError = X_CENTER-pixy.blocks[0].x;
        tiltError = pixy.blocks[0].y-Y_CENTER;
        if (pixy.blocks[0].width < targetSize) { //the target is far and we must advance
          throttle = 100; // charge forward
          distError = targetSize - pixy.blocks[0].width;
          diffGain = 1- driveGain * float(distError) / targetSize; // this is in float format
        }
        else if (pixy.blocks[0].width > targetSize) { //the target is too close and we must back off
          throttle = -100; // retreat
          distError = pixy.blocks[0].width - targetSize;
          diffGain = 1- driveGain * float(distError) / targetSize; // this is in float format
        }
      }
      else if (pixy.blocks[0].signature == 2) { // this is line following algorithm
        panError = X_CENTER-pixy.blocks[0].x;
        tiltError = pixy.blocks[0].y-Y_CENTER;
        throttle = 100; // charge forward
        diffGain = 0.3;
      }
//      else if (sig3-sig2 >= 1 && sig4-sig3 >= 1) { // this is center-lane algorithm pixy.blocks[0].signature == 3 && 
//        panError = X_CENTER - int(float(pixy.blocks[sig2+1].x + pixy.blocks[sig3+1].x)/2);
//        tiltError = pixy.blocks[sig2+1].y-Y_CENTER;
//        throttle = 100; // charge forward
//        diffGain = 0.3;
//      }      
      else { // if none of the blocks make sense, just pause
        panError = 0;
        tiltError = 0;
        throttle = 0;
        diffGain = 1;      
      }
      
      panLoop.update(panError);
      tiltLoop.update(tiltError);     
      pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
      
      i++;
      
//      // do this (print) every 50 frames because printing every
//      // frame would bog down the Arduino
//      if (i%50==0) 
//      {
//        sprintf(buf, "Detected %d:\n", blocks);
//        Serial.print(buf);
//        for (j=0; j<blocks; j++)
//        {
//          sprintf(buf, "  block %d: ", j);
////          Serial.print(buf); 
////          pixy.blocks[j].print();
////          Serial.println(panLoop.m_pos);
////          Serial.println(tiltLoop.m_pos);
////          Serial.println(bias);
////          Serial.println(throttle);
////          Serial.println(sig1_list[1]);
////          Serial.println(sig2_list[0]);
//        }
//      }
    } 
    
      
  if (currentTime-lastTime >= timeout) { // if Pixy sees nothing recognizable, don't move.
  throttle = 0;
  diffGain = 1;
  }
//    else if (blocks == 0) {
//    throttle = 0;  
//  ////        Serial.println(bias);
//  ////        Serial.println(throttle);
//    }
    
  //  panLoop.m_pos  // 0~1000; <500 is to the right
  //  PIXY_RCS_CENTER_POS // 500 is center
  //  tiltLoop.m_pos // 447~900; 1000 is all the way pitch down
    if (panLoop.m_pos > PIXY_RCS_CENTER_POS) { //this is turning to left
    turnError = panLoop.m_pos - PIXY_RCS_CENTER_POS; // should be still int32_t
//    bias = 0.5 - float(turnError) / float(PIXY_RCS_CENTER_POS) * h_pgain;// <0.5 is turning left 
    bias = - float(turnError) / float(PIXY_RCS_CENTER_POS) * h_pgain;// <0.5 is turning left 
    }
    else if (panLoop.m_pos < PIXY_RCS_CENTER_POS) { //this is turning to right
    turnError = PIXY_RCS_CENTER_POS - panLoop.m_pos; // should be still int32_t
//    bias = 0.5 + float(turnError) / float(PIXY_RCS_CENTER_POS) * h_pgain;// <0.5 is turning left
    bias = float(turnError) / float(PIXY_RCS_CENTER_POS) * h_pgain;// <0.5 is turning left 
    }
    
    Drive();
}
