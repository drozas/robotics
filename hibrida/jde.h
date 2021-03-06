#include <unistd.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <pthread.h>
#include <math.h>

#define DEGTORAD     (3.14159264 / 180.0)
#define RADTODEG     (180.0 /3.14159264)
#define MAX_USERS 3

#ifndef TRUE
#define TRUE         1
#endif
#ifndef FALSE
#define FALSE        0
#endif
 
#ifndef tvoxel
#define tvoxel
typedef struct voxel{
  float x;
  float y;}Tvoxel;
#endif

typedef void (*intcallback)(int i);
typedef void (*arbitration)(void);
extern void null_arbitration();
extern void put_state(int numschema,int newstate);

typedef enum {SCH_SONARS,
	      SCH_LASER,
	      SCH_ENCODERS,
	      SCH_PANTILTENCODERS,
	      SCH_IMAGEA,
	      SCH_IMAGEB,
	      SCH_MOTORS,
	      SCH_PANTILTMOTORS,
	      NUM_BASICSCHEMAS,
	      SCH_GUIXFORMS,
	      SCH_MYSCHEMA,
	      NUM_SCHEMAS}Schemas;
extern int debug[NUM_SCHEMAS];
extern int state[NUM_SCHEMAS];
enum states {slept,active,notready,ready,forced,winner};
extern pthread_mutex_t mymutex[NUM_SCHEMAS];
extern pthread_cond_t condition[NUM_SCHEMAS];
extern pthread_t schema[NUM_SCHEMAS];
extern void jdeshutdown(int sig);
extern void us2xy(int numsensor, float d,float phi, Tvoxel *point);
extern void laser2xy(int reading, float d, Tvoxel *point);
extern float fpsA, fpsB, fpssonars, fpsencoders, fpslaser,fpspantiltencoders,fpsmotors,fpspantiltmotors;
extern char *greyA;
extern char *greyB;

/***************** API of variables ***************/
#define NUM_LASER 180
#define NUM_SONARS 16
#define NUM_BUMPERS 10
#define MAX_VEL 1000 /* mm/sec, hardware limit: 1800 */
#define MAX_RVEL 180 /* deg/sec, hardware limit: 360 */
/* SIF image size */
#define SIFNTSC_ROWS 240
#define SIFNTSC_COLUMNS 320
/* directed perception pantilt limits */
#define MAX_PAN_ANGLE 158. /* degrees */
#define MIN_PAN_ANGLE -158. /* degrees */
#define MAX_TILT_ANGLE 30. /* degrees */
#define MIN_TILT_ANGLE -46. /* degrees */
#define MAX_SPEED_PANTILT 205.89

extern float laser_coord[5];
extern float us_coord[NUM_SONARS][5];/* sensor positions */
extern float camera_coord[5]; /* camera position */

extern float robot[5]; 
extern unsigned long int encoders_clock;
extern void encoders_suspend(int i);
extern int encoders_resume(arbitration fn);

extern int laser[NUM_LASER];
extern unsigned long int laser_clock;
extern void laser_suspend(int i);
extern int laser_resume(arbitration fn);

extern float us[NUM_SONARS];
extern unsigned long int us_clock[NUM_SONARS];
extern void sonars_suspend(int i);
extern int sonars_resume(intcallback fn);

extern char *colorA; /* sifntsc image itself */
extern unsigned long int imageA_clock;
extern int imageA_resume(arbitration fn);
extern void imageA_suspend(int);

extern char *colorB; /* sifntsc image itself */
extern unsigned long int imageB_clock;
extern int imageB_resume(arbitration fn);
extern void imageB_suspend(int);

extern float pan_angle, tilt_angle;  /* degs */
extern unsigned long int pantiltencoders_clock;
extern int pantiltencoders_resume(arbitration fn);
extern void pantiltencoders_suspend(int);

extern float v; /* mm/s */
extern float w; /* deg/s*/
extern void motors_suspend();
extern void motors_resume(int *brothers, arbitration fn);
extern int motors_cycle;

extern float longitude; /* degs, pan angle */
extern float latitude; /* degs, tilt angle */
extern void pantiltmotors_suspend();
extern void pantiltmotors_resume(int *brothers, arbitration fn);
extern float longitude_speed;
extern float latitude_speed;
extern int pantiltmotors_cycle;



#include <forms.h>
#include "guixforms.h"
#include "myschema.h"

/* Variables para controlar las pintadas de las fuerzas */
float x_robot; /* Posicion actual del robot*/
float y_robot;

float x_frep; /* Posicion de la fuerza repulsiva */
float y_frep;

float x_fatrac;	/* Posicion de la fuerza atractiva */
float y_fatrac;

float x_ffin; /* Posicion de la fuerza final*/
float y_ffin;

/*Variables globales para vectores*/
double f_repulsiva[2];
double f_atractiva[2];
double f_final[2];

#include <forms.h>
#include "guixforms.h"
#include "myschema.h"
#define N_PUNTOS 3

/*float x_destino;
float y_destino;*/

struct tipoPunto{
    double x;
    double y;
};
typedef struct tipoPunto tPunto;

tPunto destino;
tPunto lista[N_PUNTOS];
