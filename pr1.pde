import de.voidplus.leapmotion.*;
import processing.serial.*;

float INTERACTION_SPACE_WIDTH = 200; // left-right from user
float INTERACTION_SPACE_DEPTH = 150; // away-and-toward user
float FINGER_DOT = 30;

float initPosY;
float initPosX;
float initPosYIndex;
float initPosYPinky;
float difPosY;
float difPosX;
float difPosYIndex;
float difPosYPinky;
int flag = 1;

Serial myPort;
LeapMotion leap;

void setup() {
  size(800, 500, OPENGL);
  leap = new LeapMotion(this);
  textAlign(CENTER);
  myPort = new Serial(this, "COM4", 9600);
  myPort.bufferUntil('n');
}


void draw() {
  background(100);

  // FPS
  int fps = leap.getFrameRate();
  fill(#00E310);
  text(fps + " fps", 20, 20);
  
  char send=0;

  for (Hand hand : leap.getHands ()) {
    
    PVector middleTip = hand.getMiddleFinger().getRawPositionOfJointTip();
    PVector indexTip = hand.getIndexFinger().getRawPositionOfJointTip();
    PVector pinkyTip = hand.getPinkyFinger().getRawPositionOfJointTip();
    float handGrab = hand.getGrabStrength();
    
    if(handGrab>=0.3 && flag==1){
      initPosY=handYPosition(middleTip);
      initPosX=handXPosition(middleTip);
      initPosYIndex=handYPosition(indexTip);
      initPosYPinky=handYPosition(pinkyTip);
      flag = 0;
    }
    else if(handGrab<0.3 && flag==0){
      flag = 1;
    }
    
    handleFinger(middleTip, "mdl");
    handleFinger(indexTip,"indx");
    handleFinger(pinkyTip,"pinky");
    
    if (handGrab >= 0.3) {
      difPosY=initPosY-handYPosition(middleTip);
      difPosX=initPosX-handXPosition(middleTip);
      difPosYIndex=initPosYIndex-handYPosition(indexTip);
      difPosYPinky=initPosYPinky-handYPosition(pinkyTip);
      text(difPosX + " x difference", 100, 40);
      
      if (difPosX<-50){
        send = '3';
      }
      else if (difPosX>50){
        send = '4';
      }
      else if(difPosY>50 ){ //tolerance of 50
        send = '1';
      }
      else if (difPosY<-50){ //tolerance of 50
        send = '2';
      }
      else{
        send = '0';
      }
      
  }
   myPort.write(send);
   hand.draw();
  }
}

void handleFinger(PVector pos, String id) {

  // map finger tip position to 2D surface
  float x = map(pos.x, -INTERACTION_SPACE_WIDTH,INTERACTION_SPACE_WIDTH, 0, width);
  float y = map(pos.z, -INTERACTION_SPACE_DEPTH,INTERACTION_SPACE_DEPTH, 0, height);

  fill(#00E310);
  noStroke();
  ellipse(x, y, FINGER_DOT, FINGER_DOT);

  // ID
  fill(0);
  text(id, x, y + 5);
  fill(#00E310);
  text("y: " + y, 100, 20);
}

float handYPosition(PVector pos) {

  // map finger tip position to 2D surface
  float y = map(pos.z, -INTERACTION_SPACE_DEPTH,INTERACTION_SPACE_DEPTH, 0, height);

  return y;
}
float handXPosition(PVector pos) {

  // map finger tip position to 2D surface
  float x = map(pos.x, -INTERACTION_SPACE_WIDTH,INTERACTION_SPACE_WIDTH, 0, width);

  return x;
}
