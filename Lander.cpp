/*
	Lander Control simulation.

	Updated by F. Estrada for CSC C85, Oct. 2013
	Updated by Per Parker, Sep. 2015

	Learning goals:

	- To explore the implementation of control software
	  that is robust to malfunctions/failures.

	The exercise:

	- The program loads a terrain map from a .ppm file.
	  the map shows a red platform which is the location
	  a landing module should arrive at.
	- The control software has to navigate the lander
	  to this location and deposit the lander on the
	  ground considering:

	  * Maximum vertical speed should be less than 10 m/s at touchdown
	  * Maximum landing angle should be less than 15 degrees w.r.t vertical

	- Of course, touching any part of the terrain except
	  for the landing platform will result in destruction
	  of the lander

	This has been made into many videogames. The oldest one
	I know of being a C64 game called 1985 The Day After.
        There are older ones! (for bonus credit, find the oldest
        one and send me a description/picture plus info about the
        platform it ran on!)

	Your task:

	- These are the 'sensors' you have available to control
          the lander.

	  Velocity_X();  - Gives you the lander's horizontal velocity
	  Velocity_Y();	 - Gives you the lander's vertical velocity
	  Position_X();  - Gives you the lander's horizontal position (0 to 1024)
	  Position Y();  - Gives you the lander's vertical position (0 to 1024)

          Angle();	 - Gives the lander's angle w.r.t. vertical in DEGREES (upside-down = 180 degrees)

	  SONAR_DIST[];  - Array with distances obtained by sonar. Index corresponds
                           to angle w.r.t. vertical direction measured clockwise, so that
                           SONAR_DIST[0] is distance at 0 degrees (pointing upward)
                           SONAR_DIST[1] is distance at 10 degrees from vertical
                           SONAR_DIST[2] is distance at 20 degrees from vertical
                           .
                           .
                           .
                           SONAR_DIST[35] is distance at 350 degrees from vertical

                           if distance is '-1' there is no valid reading. Note that updating
                           the sonar readings takes time! Readings remain constant between
                           sonar updates.

          RangeDist();   - Uses a laser range-finder to accurately measure the distance to ground
                           in the direction of the lander's main thruster.
                           The laser range finder never fails (probably was designed and
                           built by PacoNetics Inc.)

          Note: All sensors are NOISY. This makes your life more interesting.

	- Variables accessible to your 'in flight' computer

	  MT_OK		- Boolean, if 1 indicates the main thruster is working properly
	  RT_OK		- Boolean, if 1 indicates the right thruster is working properly
	  LT_OK		- Boolean, if 1 indicates thr left thruster is working properly
          PLAT_X	- X position of the landing platform
          PLAY_Y        - Y position of the landing platform

	- Control of the lander is via the following functions
          (which are noisy!)

	  Main_Thruster(double power);   - Sets main thurster power in [0 1], 0 is off
	  Left_Thruster(double power);	 - Sets left thruster power in [0 1]
	  Right_Thruster(double power);  - Sets right thruster power in [0 1]
	  Rotate(double angle);	 	 - Rotates module 'angle' degrees clockwise
					   (ccw if angle is negative) from current
                                           orientation (i.e. rotation is not w.r.t.
                                           a fixed reference direction).

 					   Note that rotation takes time!


	- Important constants

	  G_ACCEL = 8.87	- Gravitational acceleration on Venus
	  MT_ACCEL = 35.0	- Max acceleration provided by the main thruster
	  RT_ACCEL = 25.0	- Max acceleration provided by right thruster
	  LT_ACCEL = 25.0	- Max acceleration provided by left thruster
          MAX_ROT_RATE = .075    - Maximum rate of rotation (in radians) per unit time

	- Functions you need to analyze and possibly change

	  * The Lander_Control(); function, which determines where the lander should
	    go next and calls control functions
          * The Safety_Override(); function, which determines whether the lander is
            in danger of crashing, and calls control functions to prevent this.

	- You *can* add your own helper functions (e.g. write a robust thruster
	  handler, or your own robust sensor functions - of course, these must
	  use the noisy and possibly faulty ones!).

	- The rest is a black box... life sometimes is like that.

        - Program usage: The program is designed to simulate different failure
                         scenarios. Mode '1' allows for failures in the
                         controls. Mode '2' allows for failures of both
                         controls and sensors. There is also a 'custom' mode
                         that allows you to test your code against specific
                         component failures.

			 Initial lander position, orientation, and velocity are
                         randomized.

	  * The code I am providing will land the module assuming nothing goes wrong
          with the sensors and/or controls, both for the 'easy.ppm' and 'hard.ppm'
          maps.

	  * Failure modes: 0 - Nothing ever fails, life is simple
			   1 - Controls can fail, sensors are always reliable
			   2 - Both controls and sensors can fail (and do!)
			   3 - Selectable failure mode, remaining arguments determine
                               failing component(s):
                               1 - Main thruster
                               2 - Left Thruster
                               3 - Right Thruster
                               4 - Horizontal velocity sensor
                               5 - Vertical velocity sensor
                               6 - Horizontal position sensor
                               7 - Vertical position sensor
                               8 - Angle sensor
                               9 - Sonar

        e.g.

             Lander_Control easy.ppm 3 1 5 8

             Launches the program on the 'easy.ppm' map, and disables the main thruster,
             vertical velocity sensor, and angle sensor.

		* Note - while running. Pressing 'q' on the keyboard terminates the 
			program.

        * Be sure to complete the attached REPORT.TXT and submit the report as well as
          your code by email. Subject should be 'C85 Safe Landings, name_of_your_team'

	Have fun! try not to crash too many landers, they are expensive!

  	Credits: Lander image and rocky texture provided by NASA
		 Per Parker spent some time making sure you will have fun! thanks Per!
*/

/*
  Standard C libraries
*/
#include <math.h>

#include "Lander_Control.h"
#include <cstdio>

double DD = -1;
double ST_ANG = -1;
double POS_X[22] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
double POS_Y[22] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 
double VEL_X[22] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 
double VEL_Y[22] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 

int VELOCITY_X_OK = 1;
int VELOCITY_Y_OK = 1;
int POSITION_X_OK = 1;
int POSITION_Y_OK = 1;
int ANGLE_OK = 1;

int FLAGPOSX = 1; 
int FLAGPOSY = 1;
int FLAGVELOX = 1;
int FLAGVELOY = 1;
int FLAGANGLE = 1;


int count = 90;


double (*Velocity_X_alt)(void) = &Velocity_X;
double (*Velocity_Y_alt)(void) = &Velocity_Y;
double (*Position_X_alt)(void) = &Position_X;
double (*Position_Y_alt)(void) = &Position_Y;
double (*Angle_alt)(void) = &Angle;
double (*RangeDist_alt)(void) = &RangeDist;


void Faulty_Checker(void) {
  int faulty_pos_x_counter = 0;
  int faulty_pos_y_counter = 0;
  int faulty_velo_x_counter = 0;
  int faulty_velo_y_counter = 0;
  int faulty_angle_counter = 0;
  for (int i = 0; i < 25; i++ ) {
    if (POSITION_X_OK && (Position_X() - Position_X()) > EPSILON_POSITION_X) {
      faulty_pos_x_counter++;
    }
    if (POSITION_Y_OK && fabs(Position_Y() - Position_Y()) > EPSILON_POSITION_Y) {
      faulty_pos_y_counter++;
    }
    if (VELOCITY_X_OK && fabs(Velocity_X() - Velocity_X()) > EPSILON_VELOCITY_X) {
      faulty_velo_x_counter++;
    } 
    if (VELOCITY_Y_OK && fabs(Velocity_Y() - Velocity_Y()) > EPSILON_VELOCITY_Y) {
      faulty_velo_y_counter++;
    }
    if (ANGLE_OK && fabs(Angle() - Angle()) > EPSILON_ANGLE) {
      faulty_angle_counter++;
    }
  }

  if (faulty_pos_x_counter >= AMOUNT_OF_FAULTY) POSITION_X_OK = 0;
  if (faulty_pos_y_counter >= AMOUNT_OF_FAULTY) POSITION_Y_OK = 0;
  if (faulty_velo_x_counter >= AMOUNT_OF_FAULTY) VELOCITY_X_OK = 0;
  if (faulty_velo_y_counter >= AMOUNT_OF_FAULTY) VELOCITY_Y_OK = 0;
  if (faulty_angle_counter >= AMOUNT_OF_FAULTY) ANGLE_OK = 0;
  return;
}


void Setting_Up_Arrays(void) {
  // move elements array to the right;
  for (int i = 21; i > 0; i--) {
    POS_X[i] = POS_X[i-1];
    POS_Y[i] = POS_Y[i-1];
  }
  
  // get new data point by taking average of 100000 measurements
  double new_reading_x = 0;
  double new_reading_y = 0;
  for(int a = 0; a < 1000000; a++){
      new_reading_x += Position_X_alt();
      new_reading_y += Position_Y_alt();
  }
  
  new_reading_x = new_reading_x / 1000000;
  POS_X[0] = new_reading_x;
  new_reading_y = new_reading_y / 1000000;
  POS_Y[0] = new_reading_y;
  
  if(count % 500 == 0){
    /*
    printf("------------------------------------------------------------------- \n");
    printf("PLAT X: %f, PLAT Y: %f\n", PLAT_X, PLAT_Y);
    printf("X_Position before: %f,  X_Position after: %f, \n", POS_X[1], POS_X[0]);
    printf("Velocity : %f || Current Velocity : %f \n", Velocity_X(), Velocity_X_alt());
    printf("Y_Position before: %f,  Y_Position after: %f, \n", POS_Y[1], POS_Y[0]);
    printf("Velocity : %f || Current Velocity : %f \n", Velocity_Y(), Velocity_Y_alt());
    printf("------------------------------------------------------------------- \n  ");*/
  }
  count++;
  // printf("------------------------------------------------------------------- \n  ");
  // printf("TESTING FOR ROBUST POSITION SENSOR BASED ON VELOCITY SENSOR THAT WORKS!");
  // printf("X_Position_Using_sensor: %f, X_Position_Using_robust: %f \n", POS_X[0], Robust_Position_X());
  // printf("Y_Position_Using_sensor: %f, Y_Position_Using_robust: %f \n", POS_Y[0], Robust_Position_Y());
  // printf("------------------------------------------------------------------- \n  ");
}



double Robust_Velocity_X(void) {
  // makes use of position_x
  double distance = 0;
  double count = 0;
  for (int i = 0; i < 21 ; i++) {
    if(POS_X[i] == 0 || POS_X[i+1] == 0) continue;
     distance += (POS_X[i] - POS_X[i+1]);
     count++; 
  }
  
  return (distance/count)/(T_STEP)/(S_SCALE);
}

double Robust_Velocity_Y(void) {
  // makes use of position_y
  double distance = 0;
  double count = 0;
  for (int i = 0; i < 21; i++) {
    if(POS_Y[i] == 0 || POS_Y[i+1] == 0) continue;
     distance += -((POS_Y[i] - POS_Y[i+1]));
     count++; 
  }
  
  return (distance/count)/(T_STEP)/(S_SCALE);
}

double Robust_Position_X(void) {
  double velocity = Velocity_X_alt();
  return POS_X[0] + velocity*T_STEP*S_SCALE;
}

double Robust_Position_Y(void) {
  double velocity = Velocity_Y_alt();
  return POS_Y[0] + velocity*T_STEP*S_SCALE;
}

void Sensor_Adjustment(void) {
  // replacing velocity_x sensor;
  if (!VELOCITY_X_OK) {
    Velocity_X_alt = &Robust_Velocity_X;
  }
  if (!VELOCITY_Y_OK) {
    Velocity_Y_alt = &Robust_Velocity_Y;
  }
  if (!POSITION_X_OK) {
    Position_X_alt = &Robust_Position_X;
  }
  if (!POSITION_Y_OK) {
    Position_Y_alt = &Robust_Position_Y;
  }
  return;
}





void Lander_Control(void)
{
 Faulty_Checker();
 Sensor_Adjustment();
 Setting_Up_Arrays();
 
 if (!POSITION_X_OK && FLAGPOSX) {
  //printf("The X_POSITION sensor is broken! \n");
  FLAGPOSX = 0;
 }

 if (!POSITION_Y_OK && FLAGPOSY) {
  //printf("The Y_POSITION sensor is broken! \n");
  FLAGPOSY = 0;
 }

 if (!VELOCITY_X_OK && FLAGVELOX) {
  //printf("The X_Velocity sensor is broken! \n");
  FLAGVELOX = 0;
 }

 if (!VELOCITY_Y_OK && FLAGVELOY) {
  //printf("The Y_Velocity sensor is broken! \n");
  FLAGVELOY = 0;
 }
 if (!ANGLE_OK && FLAGANGLE) {
  //printf("The angle sensor is broken! \n");
  FLAGANGLE = 0;
 }
  
 //if(MT_OK && RT_OK && LT_OK) Lander_Control_N();
 if(MT_OK) Lander_Control_M();
 else if(RT_OK) Lander_Control_R();
 else if(LT_OK) Lander_Control_L();
}

double Robust_VX(void){
  return Velocity_X_alt();
	//return Velocity_X();
}

double Robust_VY(void){
  return Velocity_Y_alt();
	//return Velocity_Y();
}

double Robust_PX(void){
  return Position_X_alt();
	//return Position_X();
}

double Robust_PY(void){
  return Position_Y_alt();
	//return Position_Y();
}

double Robust_Ang(void){
  return Angle();
}


void Robust_Rot(double ang){
    Rotate(ang);
}
void Rotate_to(double from, double to){
  if(fabs(from-to) <= 180) Robust_Rot(to-from);
  else Robust_Rot(-360+to-from);
}

void Rotate_to(double dest){
  if(fabs(Robust_Ang() - dest) <= 1) return;
  if(fabs(dest - Robust_Ang()) <= 180){
    Robust_Rot(dest-Robust_Ang());
    //printf("HEY\t");
  }
  else{
    Robust_Rot(360-Robust_Ang()+dest);
    //printf("TAYO\t");
  }
  //printf("dest : %f\t Angle : %f\n", dest, Robust_Ang());
}


void Lander_Control_R(void){
  double VXlim;
	double VYlim;

  if(Robust_PX() - PLAT_X < -20) VXlim = 10; // If lander on the left of platform
	else if (Robust_PX()-PLAT_X>200) VXlim=15;
 	else if (Robust_PX()-PLAT_X>100) VXlim=10;
  else VXlim = 5;
	//else if (Robust_PX() -PLAT_X > 20)VXlim=5;
  //else VXlim = 0;

 	if (PLAT_Y-Robust_PY()>200) VYlim=-16;
 	else if (PLAT_Y-Robust_PY()>100) VYlim=-7;  // These are negative because they
 	else VYlim=-2;

  if(Robust_VX() - PLAT_X < -20){
      VXlim = 5;
  }


	if (fabs(PLAT_X-Robust_PX())/fabs(Robust_VX())>1.25*fabs(PLAT_Y-Robust_PY())/fabs(Robust_VY())){ VYlim=0; VXlim=0;}

  if(Robust_VY()<VYlim){
         
         Right_Thruster(1);
         if(fabs(PLAT_X-Robust_PX()) < 40 && fabs(PLAT_Y-Robust_PY())<30){Right_Thruster(0); return;}
         if(Robust_Ang() < 89 || Robust_Ang() > 91){
          if(Robust_Ang() < 270) Robust_Rot(90-Robust_Ang());
          else Robust_Rot(450-Robust_Ang());
          return;
         }
         else Right_Thruster(1);
         return;
 }
 else{
         Right_Thruster(0);
 }
  if((Robust_PX() - PLAT_X)> -20 && (Robust_PX() - PLAT_X) < 25 && fabs(Robust_PY() - PLAT_Y)>200) return;
 else if(Robust_PX() - PLAT_X > 0 && Robust_PX() - PLAT_X < 15) return;
 
if ((Robust_PX()-PLAT_X>15) && Robust_VX() > -VXlim)
 {  
    if(Robust_VX() < 0){Right_Thruster(0); return;}
    Right_Thruster((VXlim+fmin(0,Robust_VX())));
    if(Robust_Ang() < 359 && Robust_Ang() > 1){
        
		if(Robust_Ang() >= 180) Robust_Rot(360-Robust_Ang());
    else Robust_Rot(-Robust_Ang());
    printf("Putar 1\n");
    return;
  }
 }
 // Left of plat
 else if((PLAT_X-Robust_PX() > 15) && Robust_VX() < VXlim)
 {
    
    if(Robust_VX() > 0){Right_Thruster(0);return;}
    Right_Thruster((VXlim-fmax(0,Robust_VX())));
    if(Robust_Ang() < 179 || Robust_Ang() > 181){
    Robust_Rot(180-Robust_Ang());
    ("Putar 2\n");
    return;
 }
} 
 else Right_Thruster(0);
         if(Robust_Ang() < 89 || Robust_Ang() > 91){
          if(Robust_Ang() < 270) Robust_Rot(90-Robust_Ang());
          else Robust_Rot(450-Robust_Ang());
         }

}

void Lander_Control_L(void){
	double VXlim;
	double VYlim;

	if (fabs(Robust_PX()-PLAT_X)>200) VXlim=10;
 	else if (fabs(Robust_PX()-PLAT_X)>100) VXlim=15;
	else if (fabs(Robust_PX() - PLAT_X) > 40) VXlim=10;
  else VXlim = 5;

 	if (PLAT_Y-Robust_PY()>200) VYlim=-16;
 	else if (PLAT_Y-Robust_PY()>100) VYlim=-7;  // These are negative because they
 	else VYlim=-2;


	if (fabs(PLAT_X-Robust_PX())/fabs(Robust_VX())>1.25*fabs(PLAT_Y-Robust_PY())/fabs(Robust_VY())){ VYlim=0;VXlim=0;}
//if(fabs(Robust_PY() - PLAT_Y) < 25 && fabs(Robust_PX() - PLAT_X) < 30) return;
  if(Robust_VY()<VYlim){
         Left_Thruster(1);
         if(fabs(PLAT_X-Robust_PX()) < 40 && fabs(PLAT_Y-Robust_PY())<30){Left_Thruster(0);return;}
         if(Robust_Ang() < 269 || Robust_Ang() > 271){
          if(Robust_Ang() > 90) Robust_Rot(270-Robust_Ang());
          else Robust_Rot(-90-Robust_Ang());
          return;
         }
         else Left_Thruster(1);
         return;
 }
 else{
         Left_Thruster(0);
 }
  if((Robust_PX() - PLAT_X)> -20 && (Robust_PX() - PLAT_X) < 25 && fabs(Robust_PY() - PLAT_Y)>200) return;
 else if(fabs(Robust_PX() - PLAT_X) < 15) return;
 
if ((Robust_PX()-PLAT_X>20) && Robust_VX() > -VXlim)
 {
    if(Robust_VX() < 0){Left_Thruster(0); return;}
    Left_Thruster(1);
    if(Robust_Ang() < 179 || Robust_Ang() > 181) Robust_Rot(180-Robust_Ang());
    return;
 }
 // Left of plat
 else if((PLAT_X-Robust_PX() > 15) && Robust_VX() < VXlim)
 {
    
    if(Robust_VX() > 0){Left_Thruster(0);return;}
    Left_Thruster(1);
    if(Robust_Ang() < 359|| Robust_Ang() > 1){
    if(Robust_Ang() >= 180) Robust_Rot(360-Robust_Ang());
    else Robust_Rot(-Robust_Ang());
    return;
 }
} 
 else Left_Thruster(0);
  if(Robust_Ang() < 269 || Robust_Ang() > 271){
     if(Robust_Ang() > 90) Robust_Rot(270-Robust_Ang());
      else Robust_Rot(-90-Robust_Ang());
  }

}


void Lander_Control_N(void)
{
 
 double VXlim;
 double VYlim;

 // Set velocity limits depending on distance to platform.
 // If the module is far from the platform allow it to
 // move faster, decrease speed limits as the module
 // approaches landing. You may need to be more conservative
 // with velocity limits when things fail.
 if (fabs(Position_X()-PLAT_X)>200) VXlim=25;
 else if (fabs(Position_X()-PLAT_X)>100) VXlim=15;
 else VXlim=5;

 if (PLAT_Y-Position_Y()>200) VYlim=-20;
 else if (PLAT_Y-Position_Y()>100) VYlim=-10;  // These are negative because they
 else VYlim=-4;				       // limit descent velocity

 // Ensure we will be OVER the platform when we land
 if (fabs(PLAT_X-Position_X())/fabs(Velocity_X())>1.25*fabs(PLAT_Y-Position_Y())/fabs(Velocity_Y())) VYlim=0;

 // IMPORTANT NOTE: The code below assumes all components working
 // properly. IT MAY OR MAY NOT BE USEFUL TO YOU when components
 // fail. More likely, you will need a set of case-based code
 // chunks, each of which works under particular failure conditions.

 // Check for rotation away from zero degrees - Robust_Rot first,
 // use thrusters only when not rotating to avoid adding
 // velocity components along the rotation directions
 // Note that only the latest Robust_Rot() command has any
 // effect, i.e. the rotation angle does not accumulate
 // for successive calls.

 if (Angle()>1&&Angle()<359)
 {
  if (Angle()>=180) Robust_Rot(360-Angle());
  else Robust_Rot(-Angle());
  return;
 }

 // Module is oriented properly, check for horizontal position
 // and set thrusters appropriately.
 if (Position_X()>PLAT_X)
 {
  // Lander is to the LEFT of the landing platform, use Right thrusters to move
  // lander to the left.
  Left_Thruster(0);	// Make sure we're not fighting ourselves here!
  if (Velocity_X()>(-VXlim)) Right_Thruster((VXlim+fmin(0,Velocity_X()))/VXlim);
  else
  {
   // Exceeded velocity limit, brake
   Right_Thruster(0);
   Left_Thruster(fabs(VXlim-Velocity_X()));
  }
 }
 else
 {
  // Lander is to the RIGHT of the landing platform, opposite from above
  Right_Thruster(0);
  if (Velocity_X()<VXlim) Left_Thruster((VXlim-fmax(0,Velocity_X()))/VXlim);
  else
  {
   Left_Thruster(0);
   Right_Thruster(fabs(VXlim-Velocity_X()));
  }
 }

 // Vertical adjustments. Basically, keep the module below the limit for
 // vertical velocity and allow for continuous descent. We trust
 // Safety_Override() to save us from crashing with the ground.
 if (Velocity_Y()<VYlim) Main_Thruster(1.0);
 else Main_Thruster(0);
}

void Safety_Override_N(void)
{

 double DistLimit;
 double Vmag;
 double dmin;

 // Establish distance threshold based on lander
 // speed (we need more time to rectify direction
 // at high speed)
 Vmag=Velocity_X()*Velocity_X();
 Vmag+=Velocity_Y()*Velocity_Y();

 DistLimit=fmax(75,Vmag);

 // If we're close to the landing platform, disable
 // safety override (close to the landing platform
 // the Control_Policy() should be trusted to
 // safely land the craft)
 if (fabs(PLAT_X-Position_X())<150&&fabs(PLAT_Y-Position_Y())<150) return;

 // Determine the closest surfaces in the direction
 // of motion. This is done by checking the sonar
 // array in the quadrant corresponding to the
 // ship's motion direction to find the entry
 // with the smallest registered distance

 // Horizontal direction.
 dmin=1000000;
 if (Velocity_X()>0)
 {
  for (int i=5;i<14;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 else
 {
  for (int i=22;i<32;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 // Determine whether we're too close for comfort. There is a reason
 // to have this distance limit modulated by horizontal speed...
 // what is it?
 if (dmin<DistLimit*fmax(.25,fmin(fabs(Velocity_X())/5.0,1)))
 { // Too close to a surface in the horizontal direction
  if (Angle()>1&&Angle()<359)
  {
   if (Angle()>=180) Robust_Rot(360-Angle());
   else Robust_Rot(-Angle());
   return;
  }

  if (Velocity_X()>0){
   Right_Thruster(1.0);
   Left_Thruster(0.0);
  }
  else
  {
   Left_Thruster(1.0);
   Right_Thruster(0.0);
  }
 }

 // Vertical direction
 dmin=1000000;
 if (Velocity_Y()>5)      // Mind this! there is a reason for it...
 {
  for (int i=0; i<5; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
  for (int i=32; i<36; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 else
 {
  for (int i=14; i<22; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 if (dmin<DistLimit)   // Too close to a surface in the horizontal direction
 {
  if (Angle()>1||Angle()>359)
  {
   if (Angle()>=180) Robust_Rot(360-Angle());
   else Robust_Rot(-Angle());
   return;
  }
  if (Velocity_Y()>2.0){
   Main_Thruster(0.0);
  }
  else
  {
   Main_Thruster(1.0);
  }
 }
}


void Lander_Control_M(void){
 double VXlim;
 double VYlim;

 if(Robust_PX() - PLAT_X < -20) VXlim = 10; 
 else if (fabs(Robust_PX()-PLAT_X)>200) VXlim=15;
 else if (fabs(Robust_PX()-PLAT_X)>100) VXlim=10;
 else VXlim=5;

 if (PLAT_Y-Robust_PY()>200) VYlim=-20;
 else if (PLAT_Y-Robust_PY()>100) VYlim=-10;  // These are negative because they
 else VYlim=-4;				       // limit descent velocity

 // Ensure we will be OVER the platform when we land
 if (fabs(PLAT_X-Robust_PX())/fabs(Robust_VX())>1.25*fabs(PLAT_Y-Robust_PY())/fabs(Robust_VY())) VYlim=0;
 if (Robust_VY()<VYlim){
  Main_Thruster(1);
  if(Robust_Ang() > 1 && Robust_Ang() < 369){
    if(Robust_Ang() >= 180) Robust_Rot(360-Robust_Ang());
    else Robust_Rot(-Robust_Ang());
    return;
  }
	else Main_Thruster(1);
   return;
 }
 else{
	 Main_Thruster(0);
 }
 //&& fabs(Robust_PY() - PLAT_Y) > 200
 if(fabs(Robust_PX() - PLAT_X ) < 30 ){
    //Main_Thruster(0);
   return;
 }
 else if(fabs(Robust_PX()- PLAT_X) < 20) return;

 //else if(fabs(Robust_PY() - PLAT_Y)< 100) return;
//Right of plat
 if ((Robust_PX()-PLAT_X>20) && Robust_VX() > -VXlim)
 {  
    if(Robust_VX() < 0){Main_Thruster(0); return;}
    //Main_Thruster((VXlim+fmin(0,Robust_VX())));
    Main_Thruster(1);
    if(Robust_Ang() < 269 || Robust_Ang() > 271){
		if(Robust_Ang() >= 90) Robust_Rot(270-Robust_Ang());
		else Robust_Rot(-90-Robust_Ang());
    
    printf("Putar 1\n");
    return;
  }
 }
 // Left of plat
 else if((PLAT_X-Robust_PX() > 20) && Robust_VX() < VXlim)
 {
	  //Main_Thruster((VXlim-fmax(0,Robust_VX())));
    if(Robust_VX() > 0){Main_Thruster(0);return;}
    //Main_Thruster((VXlim-fmax(0,Robust_VX())));
    Main_Thruster(1);
    if(Robust_Ang() < 269 || Robust_Ang() > 271){
    if(Robust_Ang() >= 270) Robust_Rot(450-Robust_Ang());
       else Robust_Rot(90-Robust_Ang());
    printf("Putar 2\n");
    return;
 }
} 
 else Main_Thruster(0);
  if(Robust_Ang() > 1 && Robust_Ang() < 359){
    if(Robust_Ang() >= 180) Robust_Rot(360-Robust_Ang());
    else Robust_Rot(-Robust_Ang());
  }
}

void Safety_Override(void){
  //if(MT_OK && RT_OK && LT_OK) Safety_Override_N();
	if(MT_OK) Safety_Override_M();
	else if(RT_OK) Safety_Override_R();
	else if(LT_OK) Safety_Override_L();
}

void Safety_Override_M(void){
 double DistLimit;
 double Vmag;
 double dmin;
 int ang;
 Vmag=Robust_VX()*Robust_VX();
 Vmag+=Robust_VY()*Robust_VY();

 DistLimit=fmax(75,Vmag);
 //if(fabs(PLAT_X-Robust_PX()) < 150)return;
 //&&(fabs(PLAT_Y-Robust_PY())<200)
 if (fabs(PLAT_X-Robust_PX())<100 && PLAT_Y-Robust_PY()<200){
	 //printf("Here : \n");
	 return;
 }
 
 dmin=1000000;
///fabs(PLAT_X-Robust_PX())<50 && fabs(PLAT_Y-Robust_PY())<200
 if(Robust_VX() > 0)
 {
  for (int i=5;i<14;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin){
	   dmin=SONAR_DIST[i];
  	   ang = 10*i;
   }
 }
 else if(Robust_VX() > 0 && (PLAT_X - Robust_PX()) > 15)
 {
  for (int i=22;i<32;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) {
	dmin=SONAR_DIST[i];
   	ang = 10*i;
  }
 }
 if (dmin<DistLimit*fmax(.25,fmin(fabs(Robust_VX())/5.0,1)))
 { // Too close to a surface in the horizontal direction
  //if((Robust_VX()>0 && ang < 140) || (Robust_VX() < 0 && ang > 220)){
 if(dmin < fmin(DistLimit,fabs
  (PLAT_X - Robust_PX()))){
		//Main_Thruster(1);
    if(ang < 140 && Robust_VX() >0) Main_Thruster(1);
    else if(ang > 220 && Robust_VX() < 0) Main_Thruster(1);
    else{ Main_Thruster(0); return;}
    if(Robust_Ang() > ang){
		        //printf("Deon\n");
			  Rotate(180 + ang - Robust_Ang());
		}
		else{
			//printf("Eond\n");       
			Rotate(-180+ang-Robust_Ang()); 
    }
	}
 }
dmin=1000000;
 if (Robust_VY()>5)      // Mind this! there is a reason for it...
 {
  for (int i=0; i<5; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) {
	   dmin=SONAR_DIST[i];
	   ang = 10*i;
   }
  for (int i=32; i<36; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin){
	  ang = 10 *i;
	  dmin=SONAR_DIST[i];
   }
 }
 else
 {
  for (int i=14; i<22; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin){
	   ang = 10*i;
	   dmin=SONAR_DIST[i];
   }
 }
 if (dmin<DistLimit)   // Too close to a surface in the horizontal direction
 {
  if(fabs(PLAT_X - Robust_PX()) > 30){Main_Thruster(1);}
  
  if(Robust_Ang() < 359 && Robust_Ang() > 1){
	  if(Robust_Ang() >= 180) Rotate(360-Robust_Ang());
	  else Rotate(-Robust_Ang());
    //printf("Putar 4\n");
    return;
  }
  if (Robust_VY()>1.0){
   Main_Thruster(0.0);
  }
  else
  {
   Main_Thruster(1.0); 
  }
  return;
 }
 else return;
}

void Safety_Override_L(void){
  
double DistLimit;
 double Vmag;
 double dmin;
 int ang;
 Vmag=Robust_VX()*Robust_VX();
 Vmag+=Robust_VY()*Robust_VY();

 DistLimit=fmax(75,Vmag);
 
 //Rotate when landing
 if (fabs(PLAT_X-Robust_PX())<50&&fabs(PLAT_Y-Robust_PY())<200){
         if(fabs(PLAT_X-Robust_PX()) < 50 && fabs(PLAT_Y-Robust_PY())<30){
		    //Right_Thruster(0);
          //printf("Ready_R\n");
		    if(Robust_Ang() > 0.5 && Robust_Ang() < 359.5){
			      Left_Thruster(0);
          if(Robust_Ang() >= 180) Rotate(360-Robust_Ang());
          else Rotate(-Robust_Ang());
		    }
	 }
	 return;
 }

 dmin=1000000;
 if (Robust_VX()>0)
 {
  for (int i=5;i<14;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin && Robust_VX() > 0){
           dmin=SONAR_DIST[i];
           ang = 10*i;
 }
 }
 else
 {
  for (int i=22;i<32;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin && Robust_VX() < 0) {
        dmin=SONAR_DIST[i];
        ang = 10*i;
   }
 }

 if (dmin<DistLimit*fmax(.25,fmin(fabs(Robust_VX())/5.0,1)))
 { 
  if(dmin < fmin(DistLimit,fabs
  (PLAT_X - Robust_PX()))){
		//Main_Thruster(1);
    if(ang < 140 && Robust_VX() >0) Left_Thruster(1);
    else if(ang > 220 && Robust_VX() < 0) Left_Thruster(1);
    else{ Left_Thruster(0); return;}
    if(Robust_Ang() > ang){
      Robust_Rot(-90+ang-Robust_Ang());
    }
    else Robust_Rot(90+ang-Robust_Ang());
	}
  }

  dmin=1000000;
  if (Robust_VY()>5)      // Mind this! there is a reason for it...
  {
  for (int i=0; i<5; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) {
           dmin=SONAR_DIST[i];
           ang = 10*i;
   }
  for (int i=32; i<36; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin){
          ang = 10 *i;
          dmin=SONAR_DIST[i];
   }
  }
  else
 {
  for (int i=14; i<22; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin){
           ang = 10*i;
           dmin=SONAR_DIST[i];
   }
 }
 if (dmin<DistLimit)   // Too close to a surface in the horizontal direction
 {
  if(fabs(PLAT_X - Robust_PX()) > 150)Left_Thruster(1);

  //Rotate to push against 
         if(Robust_Ang() < 269 || Robust_Ang() > 271){
          if(Robust_Ang() > 90) Rotate(270-Robust_Ang());
          else Rotate(-90-Robust_Ang());
          return;
         }
  if (Robust_VY()>1.0){
   Left_Thruster(0.0);
  }
  else  {
   Left_Thruster(1.0);
   //printf("431\n");
  }
  return;
 }
 else return;
}

/**/
void Safety_Override_R(void)
{

double DistLimit;
 double Vmag;
 double dmin;
 int ang;
 Vmag=Robust_VX()*Robust_VX();
 Vmag+=Robust_VY()*Robust_VY();

 DistLimit=fmax(75,Vmag);
 
 //Rotate when landing
 if (fabs(PLAT_X-Robust_PX())<50&&fabs(PLAT_Y-Robust_PY())<200){
         if(fabs(PLAT_X-Robust_PX()) < 40 && fabs(PLAT_Y-Robust_PY())<30){
		    //Right_Thruster(0);
          //printf("Ready_R\n");
		    if(Robust_Ang() > 0.5 && Robust_Ang() < 359.5){
			      Right_Thruster(0);
          if(Robust_Ang() >= 180) Robust_Rot(360-Robust_Ang());
          else Robust_Rot(-Robust_Ang());
		    }
	 }
	 return;
 } 
    

  //return;
 dmin=1000000;
 if (Robust_VX()>0)
 {
  for (int i=5;i<14;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin && Robust_VX() > 0){
           dmin=SONAR_DIST[i];
           ang = 10*i;
 }
 }
 else
 {
  for (int i=22;i<32;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin && Robust_VX() < 0) {
        dmin=SONAR_DIST[i];
        ang = 10*i;
   }
 }




 if (dmin<DistLimit*fmax(.25,fmin(fabs(Robust_VX())/5.0,1)))
 { 
  if(dmin < fmin(DistLimit,fabs
  (PLAT_X - Robust_PX()))){
		//Main_Thruster(1);
    if(ang < 140 && Robust_VX() >0) Right_Thruster(1);
    else if(ang > 220 && Robust_VX() < 0) Right_Thruster(1);
    else{ Right_Thruster(0); return;}
    if(Robust_Ang() > ang){
		        //printf("Deon\n");
			  Robust_Rot(90 + ang - Robust_Ang());
		}
		else{
			//printf("Eond\n");       
			Robust_Rot(-90+ang-Robust_Ang()); 
    }
	}
  }


dmin=1000000;
 if (Robust_VY()>5)      // Mind this! there is a reason for it...
 {
  for (int i=0; i<5; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) {
           dmin=SONAR_DIST[i];
           ang = 10*i;
   }
  for (int i=32; i<36; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin){
          ang = 10 *i;
          dmin=SONAR_DIST[i];
   }
 }
 else
 {
  for (int i=14; i<22; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin){
           ang = 10*i;
           dmin=SONAR_DIST[i];
   }
 }
 if (dmin<DistLimit)   // Too close to a surface in the horizontal direction
 {
  if(fabs(PLAT_X - Robust_PX()) > 150)Right_Thruster(1);
  //Rotate to push against 
         if(Robust_Ang() < 89 || Robust_Ang() > 91){
          if(Robust_Ang() < 270) Robust_Rot(90-Robust_Ang());
          else Robust_Rot(450-Robust_Ang());
          return;
         }
  if (Robust_VY()>1.0){
   Right_Thruster(0.0);
  }
  else
  {
   Right_Thruster(0.8);
   //printf("431\n");
  }
  return;
 }
 else return;



}

void vv(void){return;}