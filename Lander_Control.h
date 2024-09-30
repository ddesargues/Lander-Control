#ifndef _LANDER_CONTROL_H
#define _LANDER_CONTROL_H

// Simulation parameters - YOU MUST NOT CHANGE ANY OF THESE
#define G_ACCEL 8.87
#define MT_ACCEL 35.0
#define RT_ACCEL 25.0
#define LT_ACCEL 25.0
#define MAX_ROT_RATE .075
#define SONAR_RANGE 9.0     
#define NP1 .05
#define NP2 .05
#define T_STEP .005
#define S_SCALE 5.0
#define PI 3.14159265359
#define DISPLAY_LATENCY 10
#define HIST 180
#define EPSILON_VELOCITY_X 5
#define EPSILON_VELOCITY_Y 5
#define EPSILON_POSITION_X 50
#define EPSILON_POSITION_Y 50
#define EPSILON_ANGLE 5
#define AMOUNT_OF_FAULTY 1

// Global variables accessible to your flight computer
extern int MT_OK;
extern int RT_OK;
extern int LT_OK;
extern double PLAT_X;
extern double PLAT_Y;
extern double SONAR_DIST[36];
extern double DD;
extern double ST_ANG;

extern double POS_X[22]; // use position sensors
extern double POS_Y[22]; // use position sensors
extern double VEL_X[22]; 
extern double VEL_Y[22];



extern int VELOCITY_X_OK;
extern int VELOCITY_Y_OK;
extern int POSITION_X_OK;
extern int POSITION_Y_OK;
extern int ANGLE_OK;
extern int RANGEDIST_OK;

extern int FLAGPOSX;
extern int FLAGPOSY;
extern int FLAGVELOX;
extern int FLAGVELOY;
extern int FLAGANGLE;
extern int FLAGRANGEDIST;

extern double PREVIOUS_X;
// Flight controls
void Main_Thruster(double power);
void Left_Thruster(double power);
void Right_Thruster(double power);
void Rotate(double angle);
double Velocity_X(void);
double Velocity_Y(void);
double Position_X(void);
double Position_Y(void);
double Angle(void);
double RangeDist(void);


void Faulty_Checker(void);
void Setting_Up_Arrays(void);
double Robust_Velocity_X(void);
double Robust_Velocity_Y(void);
double Robust_Position_X(void);
double Robust_Position_Y(void);

extern double (*Velocity_X_alt)(void);
extern double (*Velocity_Y_alt)(void);
extern double (*Position_X_alt)(void);
extern double (*Position_Y_alt)(void);
extern double (*Angle_alt)(void);
extern double (*RangeDist_alt)(void);



void Rotate_to(double from, double to);

double Robust_VX(void);
double Robust_VY(void);
double Robust_PX(void);
double Robust_PY(void);
double Robust_Ang(void);

// Function prototypes for code you need to look at
void Lander_Control(void);
void Safety_Override(void);
void Robust_Rot(double);
void vv(void);

void Lander_Control_M(void);
void Lander_Control_R(void);
void Lander_Control_L(void);
void Lander_Control_N(void);
void Safety_Override_M(void);
void Safety_Override_L(void);
void Safety_Override_R(void);
void Safety_Override_N(void);

void CondAng(double from, double to);

void Safety_Swap(void);
void Lander_Swap(void);


void Rotate_to(double dest);
#endif
