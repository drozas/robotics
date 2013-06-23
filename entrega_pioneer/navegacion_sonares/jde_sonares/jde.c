#define thisrelease "basic JDE 3.4"

/*
3.4: jmplaza local pantilt support

- -D CONFIGDIR='"/etc/jde-robot"', the directory in which searches config 
  files (jde.conf) can be changed at compile time
*/

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include "jdeservers.h"
#include "jde.h"


#ifdef JDEVIDEO4LINUX
#define _LINUX_TIME_H 1
#define _DEVICE_H_ 1
/* before videodev.h. It avoids /usr/include/linux/time.h:9: redefinition of `struct timespec' */
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/videodev.h>

/* Color conversion, taken from xawtv */
#define CLIP         320
# define RED_NULL    128  /* 137 */
# define BLUE_NULL   128  /* 156 */
# define LUN_MUL     256  /* 360 */
# define RED_MUL     512
# define BLUE_MUL    512
#define GREEN1_MUL  (-RED_MUL/2)
#define GREEN2_MUL  (-BLUE_MUL/6)
#define RED_ADD     (-RED_NULL  * RED_MUL)
#define BLUE_ADD    (-BLUE_NULL * BLUE_MUL)
#define GREEN1_ADD  (-RED_ADD/2)
#define GREEN2_ADD  (-BLUE_ADD/6)
/* lookup tables for color conversion*/
static unsigned int  ng_yuv_gray[256];
static unsigned int  ng_yuv_red[256];
static unsigned int  ng_yuv_blue[256];
static unsigned int  ng_yuv_g1[256];
static unsigned int  ng_yuv_g2[256];
static unsigned int  ng_clip[256 + 2 * CLIP];
#define GRAY(val)		ng_yuv_gray[val]
#define RED(gray,red)		ng_clip[ CLIP + gray + ng_yuv_red[red] ]
#define GREEN(gray,red,blue)	ng_clip[ CLIP + gray + ng_yuv_g1[red] + ng_yuv_g2[blue] ]
#define BLUE(gray,blue)		ng_clip[ CLIP + gray + ng_yuv_blue[blue] ]
#endif


#ifdef JDEFIREWIRE
#include <libraw1394/raw1394.h>
#include <libdc1394/dc1394_control.h>


/* color conversion functions from Bart Nabbe.*/
/* corrected by Damien: bad coeficients in YUV2RGB*/
#define YUV2RGB(y, u, v, r, g, b)\
  r = y + ((v*1436) >> 10);\
  g = y - ((u*352 + v*731) >> 10);\
  b = y + ((u*1814) >> 10);\
  r = r < 0 ? 0 : r;\
  g = g < 0 ? 0 : g;\
  b = b < 0 ? 0 : b;\
  r = r > 255 ? 255 : r;\
  g = g > 255 ? 255 : g;\
  b = b > 255 ? 255 : b
  

void uyvy2rgb (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels)
{
  register int i = (NumPixels << 1)-1;
  register int j = NumPixels + ( NumPixels << 1 ) -1;
  register int y0, y1, u, v;
  register int r, g, b;

  while (i > 0) {
    y1 = (unsigned char) src[i--];
    v  = (unsigned char) src[i--] - 128;
    y0 = (unsigned char) src[i--];
    u  = (unsigned char) src[i--] - 128;
    YUV2RGB (y1, u, v, r, g, b);
    dest[j--] = r;
    dest[j--] = g;
    dest[j--] = b;
    YUV2RGB (y0, u, v, r, g, b);
    dest[j--] = r;
    dest[j--] = g;
    dest[j--] = b;
  }
}
#endif 




char *greyA;
char greyAA[SIFNTSC_COLUMNS*SIFNTSC_ROWS*1]; /* sifntsc image itself */
char *greyB;
char greyBB[SIFNTSC_COLUMNS*SIFNTSC_ROWS*1]; /* sifntsc image itself */

char *colorA;
char colorAA[SIFNTSC_COLUMNS*SIFNTSC_ROWS*3]; /* sifntsc image itself */
unsigned long int imageA_clock;
float fpsA=0;
int kA=0;
arbitration imageA_callbacks[MAX_USERS];
int imageA_users=0;

char *colorB;
char colorBB[SIFNTSC_COLUMNS*SIFNTSC_ROWS*3]; /* sifntsc image itself */
unsigned long int imageB_clock;
float fpsB=0;
int kB=0;
arbitration imageB_callbacks[MAX_USERS];
int imageB_users=0;

float pan_angle, tilt_angle;  /* degs */
unsigned long int pantiltencoders_clock;
float fpspantiltencoders=0;
int kpantiltencoders=0;
arbitration pantiltencoders_callbacks[MAX_USERS];
int pantiltencoders_users=0;

int laser[NUM_LASER];
unsigned long int laser_clock;
int klaser=0;
float fpslaser=0.;
arbitration laser_callbacks[MAX_USERS];
int laser_users=0;

float us[NUM_SONARS];
unsigned long int us_clock[NUM_SONARS];
int ksonars=0;
float fpssonars=0.;
intcallback sonars_callbacks[MAX_USERS];
int sonars_users=0;

float robot[5]; /* mm, mm, rad */
unsigned long int encoders_clock;
float tspeed, rspeed; /* mm/s, deg/s */
unsigned long lasttime; /* microsecs */
int kencoders=0;
float fpsencoders=0.;
arbitration encoders_callbacks[MAX_USERS];
int encoders_users=0;

float latitude,longitude;
float latitude_speed,longitude_speed;
int pantiltmotors_cycle=100; /* ms */
float fpspantiltmotors=0;
int kpantiltmotors=0;

float v,w;
int motors_cycle=100; /* ms */
float ac=0.;
float fpsmotors=0;
int kmotors=0;


/* sensor positions */
float laser_coord[5];
float us_coord[NUM_SONARS][5]; 
   /* Estructura para poder cambiar medidas de sensor a coordenadas locales al robot, y de estas al sist ref inicial: xsensor, ysensor, orientsensor,cossensor y sinsensor del sensor respecto del sistema solidario con el robot. Es fija. */
float camera_coord[5];


/* sources for sensor data, and sinks for motor orders */
typedef enum {none,
	      otos,
	      srisim,
	      stage,
	      sip,
	      seriallaser,
	      serialpantilt,
	      oculo1,
	      oculo2,
	      file1,
	      file2,
	      video4linux1,
	      video4linux2,
	      firewire,
	      NUM_SOURCES}PhysicalDevice;
PhysicalDevice source[NUM_BASICSCHEMAS];
Schemas served[NUM_SOURCES][NUM_BASICSCHEMAS];
int imageAdevice,imageBdevice; /* inside its serving oculo or firewire bus, if 0 or 1 camera */
int state[NUM_SCHEMAS];
pthread_t schema[NUM_SCHEMAS];
pthread_mutex_t mymutex[NUM_SCHEMAS];
pthread_cond_t condition[NUM_SCHEMAS];
int debug[NUM_SCHEMAS];
int trace=0;
char *name[NUM_SCHEMAS]={"sonars","laser","encoders","pantiltencoders","imageA","imageB","motors","pantiltmotors","NUM_BASICSCHEMAS","guixforms","myschema"};
#define FPS_CYCLE 2000 /* ms, to compute fps*/


pthread_t cronos_th;

char otos_host[MAX_MESSAGE];
int otos_port;
pthread_t otos_thread;
pthread_mutex_t otos_mutex;
int sock_otos;
char otos_out[MAX_MESSAGE];
char otos_in[MAX_MESSAGE];
int otos_marca; /* "puntero de insercion" en el input_buffer, por si hay mensajes muy largos, completar el mensaje con varias lecturas del socket */
int otos_service[NUM_SCHEMAS];

typedef enum {mensajes,unaimagen}ModosLecturaSocket;
char oculo1_host[MAX_MESSAGE];
int oculo1_port;
pthread_t oculo1_thread;
pthread_mutex_t oculo1_mutex;
int sock_oculo1;
int oculo1_connected=0;
char oculo1_in[MAX_MESSAGE];
char oculo1_out[MAX_MESSAGE];
long int oculo1_marca;
int oculo1_lastimage=0;
ModosLecturaSocket modolectura1=mensajes;
char mmbuf1[SIFNTSC_COLUMNS*SIFNTSC_ROWS*3+MAX_MESSAGE]; /* memoria para la imagen que viene por el socket. Hay que pedir un poco mas por la lectura asincrona en tacadas que se hace del socket para recibir los datos que llegan */
int oculo1_service[NUM_SCHEMAS];

char oculo2_host[MAX_MESSAGE];
int oculo2_port;
pthread_t oculo2_thread;
pthread_mutex_t oculo2_mutex;
int sock_oculo2;
int oculo2_connected=0;
char oculo2_in[MAX_MESSAGE];
char oculo2_out[MAX_MESSAGE];
long int oculo2_marca;
int oculo2_lastimage=0;
ModosLecturaSocket modolectura2=mensajes;
char mmbuf2[SIFNTSC_COLUMNS*SIFNTSC_ROWS*3+MAX_MESSAGE]; /* memoria para la imagen que viene por el socket. Hay que pedir un poco mas por la lectura asincrona en tacadas que se hace del socket para recibir los datos que llegan */
int oculo2_service[NUM_SCHEMAS];


#ifdef JDEVIDEO4LINUX
#define MAXCAM 2
pthread_t v4l_thread[MAXCAM];
char v4l_filename[MAXCAM][MAX_MESSAGE]; 
static char *map[MAXCAM]; /* local memory for v4l capture */ 
int fdv4l[MAXCAM];
struct video_capability cap[MAXCAM];
struct video_channel chan[MAXCAM];
struct video_picture vpic[MAXCAM];
static struct video_mbuf gb_buffers[MAXCAM]; 
struct video_window win[MAXCAM];
struct video_mmap gb[MAXCAM];
#endif


char file1_filename[MAX_MESSAGE];
char file2_filename[MAX_MESSAGE];


#ifdef JDEFIREWIRE
/* uncomment the following to drop frames to prevent delays */
#define DROP_FRAMES 1
#define MAX_PORTS 3
#define MAX_CAMERAS 8
#define NUM_BUFFERS 8
/* declarations for libdc1394 */
int numPorts = MAX_PORTS;
raw1394handle_t handles[MAX_PORTS];
int numCameras = 0;
dc1394_cameracapture cameras[MAX_CAMERAS];
nodeid_t *camera_nodes;
dc1394_feature_set features;
/* declarations for video1394 */
char device_name[]="/dev/video1394/0";
int firewire_fps;
int firewire_res;
pthread_t firewire_th;

void firewirecleanup(void) {
  int i;
  for (i=0; i < numCameras; i++)
    {
      dc1394_dma_unlisten( handles[cameras[i].port], &cameras[i] );
      dc1394_dma_release_camera( handles[cameras[i].port], &cameras[i]);
    }
  for (i=0; i < numPorts; i++)
    raw1394_destroy_handle(handles[i]);
}
#endif 

#include <termios.h>
#define PANTILTENCODERS_POLLING 100 /* period to ask for new pantilt encoders (ms) */
pthread_t serialpantilt_readth,serialpantilt_pollth;
char pantilt_device[MAX_MESSAGE]; 
int pantilt_serialport;
pthread_mutex_t serialpantilt_mutex;
char pantilt_in[MAX_MESSAGE];
char pantilt_out[MAX_MESSAGE];
int pantilt_marca;
#define RS232_BAUD_RATE B9600
#define ENCOD_TO_DEG (3.086/60.) /* from encoder units to deg, also 205.89/4002 */
float longitude_last=0., latitude_last=0.;
float max_pan, min_pan, max_tilt, min_tilt; 
/* real limits in encoders units. MAX_PAN_ANGLE, MAX_TILT_ANGLE... are approximate limits to be used by the client of oculo */


extern void jdeshutdown(int sig);
extern void SendCmd(char *cmd);

void update_greyA(void)
{
  int i;
  for(i=0; i<(SIFNTSC_COLUMNS*SIFNTSC_ROWS);i++)
    {
      /* this pixel conversion is a little bit tricky: it MUST pass through the (unsigned char) in order to get the right number for values over 128 in colorA[3*i]. it MUST pass through the (unsigned int) in order to allow the addition of three values, which may overflow the 8 bits limit of chars and unsigned chars */
      greyA[i]=(char)(((unsigned int)((unsigned char)(colorA[3*i]))+(unsigned int)((unsigned char)(colorA[3*i+1]))+(unsigned int)((unsigned char)(colorA[3*i+2])))/3);
    }
}

void imageA_suspend(int i)
{
  int j;
  pthread_mutex_lock(&mymutex[SCH_IMAGEA]);
  if ((i>imageA_users)||(i<0)) 
    {printf("bad range in imageA_suspend\n"); return;}

  for(j=i+1;j<imageA_users;j++)
    imageA_callbacks[j-1]=imageA_callbacks[j];
  imageA_users--;
  /* this thread never stops */
  state[SCH_IMAGEA]=slept;
  printf("imageA schema off\n");
  pthread_mutex_unlock(&mymutex[SCH_IMAGEA]); 
}

int imageA_resume(arbitration fn)
{
  int output;
  int i,leidos, marca,c,r,last=0;
  int f2;
  char buff[MAX_MESSAGE];
 
  if (imageA_users<MAX_USERS)
    {imageA_callbacks[imageA_users++]=fn;
    output=imageA_users-1;
    }
  else output=-1; 

  if (state[SCH_IMAGEA]==slept)
    {
      state[SCH_IMAGEA]=active; 
      printf("imageA schema on\n");
      /* watch out with several resumes in sequence, only the first one should
	 really send the message to the server */
      if (source[SCH_IMAGEA]==oculo1)
      {
	pthread_mutex_lock(&oculo1_mutex); 
	sprintf(oculo1_out,"%d %d\n",OCULO_rgb24bpp_sifntsc_image_query,imageAdevice);
	write(sock_oculo1,oculo1_out,strlen(oculo1_out)); 
	pthread_mutex_unlock(&oculo1_mutex); 
      }
      else if (source[SCH_IMAGEA]==oculo2)
      {
	pthread_mutex_lock(&oculo2_mutex); 
	sprintf(oculo2_out,"%d %d\n",OCULO_rgb24bpp_sifntsc_image_query,imageAdevice);
	write(sock_oculo2,oculo2_out,strlen(oculo2_out)); 
	pthread_mutex_unlock(&oculo2_mutex); 
      }
      else if ((source[SCH_IMAGEA]==file1)||(source[SCH_IMAGEA]==file2))
	{
	  /* read the image from the file */
	  if (source[SCH_IMAGEA]==file1) f2=open(file1_filename,O_RDONLY);
	  else if (source[SCH_IMAGEA]==file2) f2=open(file2_filename,O_RDONLY);
	  /*lseek(f2,SEEK_SET,0);	  */

	  if (f2==-1) 
	    fprintf(stderr,"I can't open the image file %s\n",file1_filename);
	  else 
	    {
	      i=0;
	      while(i<3) 
		/* three head lines in ppm file:  
		   "P6\n%d %d\n%d\n",width,height,255 */
		{
		  marca=0; buff[marca]='\n';
		  do
		    {
		      leidos=read(f2,&(buff[marca]),1);
		      if (leidos>0) marca+=leidos;
		      if (marca>0) last=marca-1;
		    } 
		  while((buff[last]!='\n')||(leidos<=0));
		  buff[last]='\0';
		  if (buff[0]!='#') 
		    {
		      i++; /* to skip comment lines */
		      /*printf("input %d: %s\n",i,buff); */
		      if (i==1) 
			{
			  if (strcmp(buff,"P6")!=0) 
			    fprintf(stderr,"file %s: non supported image format, must be raw PPM\n",file1_filename);
			}
		      else if (i==2)
			{
			  if ((sscanf(buff,"%d %d",&c,&r)!=2)||(c!=SIFNTSC_COLUMNS)||(r!=SIFNTSC_ROWS)) 
			    fprintf(stderr,"file %s: non supported image size, must be 320x240\n",file1_filename);
			}
		    }
		}	
	      for(i=0;i<SIFNTSC_ROWS*SIFNTSC_COLUMNS;i++)
		{
		  read(f2,(void *)buff,(size_t)3);
		  colorA[i*3]=buff[2];
		  colorA[i*3+1]=buff[1];
		  colorA[i*3+2]=buff[0];
		}
	      close(f2);
	      update_greyA();
	    }
	}}
  return output;
}


void update_greyB(void)
{
  int i;
  for(i=0; i<(SIFNTSC_COLUMNS*SIFNTSC_ROWS);i++)
    {
      /* this pixel conversion is a little bit tricky: it MUST pass through the (unsigned char) in order to get the right number for values over 128 in colorA[3*i]. it MUST pass through the (unsigned int) in order to allow the addition of three values, which may overflow the 8 bits limit of chars and unsigned chars */
      greyB[i]=(char)(((unsigned int)((unsigned char)(colorB[3*i]))+(unsigned int)((unsigned char)(colorB[3*i+1]))+(unsigned int)((unsigned char)(colorB[3*i+2])))/3);
    }
}

void imageB_suspend(int i)
{
  int j;
  pthread_mutex_lock(&mymutex[SCH_IMAGEB]);
  if ((i>imageB_users)||(i<0)) 
    {printf("bad range in imageB_suspend\n"); return;}

  for(j=i+1;j<imageB_users;j++)
    imageB_callbacks[j-1]=imageB_callbacks[j];
  imageB_users--;
  /* this thread never stops */
  state[SCH_IMAGEB]=slept;
  printf("imageB schema off\n");
  pthread_mutex_unlock(&mymutex[SCH_IMAGEB]); 
}

int imageB_resume(arbitration fn)
{
  int output;
  int i,leidos, marca,c,r,last=0;
  int f2;
  char buff[MAX_MESSAGE];
 
  if (imageB_users<MAX_USERS)
    {imageB_callbacks[imageB_users++]=fn;
    output=imageB_users-1;
    }
  else output=-1; 

  if (state[SCH_IMAGEB]==slept)
    {
      state[SCH_IMAGEB]=active; 
      printf("imageB schema on\n");
      /* watch out with several resumes in sequence, only the first one should
	 really send the message to the server */
      if (source[SCH_IMAGEB]==oculo1)
      {
	pthread_mutex_lock(&oculo1_mutex); 
	sprintf(oculo1_out,"%d %d\n",OCULO_rgb24bpp_sifntsc_image_query,imageBdevice);
	write(sock_oculo1,oculo1_out,strlen(oculo1_out)); 
	pthread_mutex_unlock(&oculo1_mutex); 
      }
      else if (source[SCH_IMAGEB]==oculo2)
      {
	pthread_mutex_lock(&oculo2_mutex); 
	sprintf(oculo2_out,"%d %d\n",OCULO_rgb24bpp_sifntsc_image_query,imageBdevice);
	write(sock_oculo2,oculo2_out,strlen(oculo2_out)); 
	pthread_mutex_unlock(&oculo2_mutex); 
      }
      else if ((source[SCH_IMAGEB]==file1)||(source[SCH_IMAGEB]==file2))
	{
	  /* read the image from the file */
	  if (source[SCH_IMAGEB]==file1) f2=open(file1_filename,O_RDONLY);
	  else if (source[SCH_IMAGEB]==file2) f2=open(file2_filename,O_RDONLY);
	  /*lseek(f2,SEEK_SET,0);	  */
	  
	  if (f2==-1) 
	    fprintf(stderr,"I can't open the image file %s\n",file1_filename);
	  else 
	    {
	      i=0;
	      while(i<3) 
		/* three head lines in ppm file:  
		   "P6\n%d %d\n%d\n",width,height,255 */
		{
		  marca=0; buff[marca]='\n';
		  do
		    {
		      leidos=read(f2,&(buff[marca]),1);
		      if (leidos>0) marca+=leidos;
		      if (marca>0) last=marca-1;
		    } 
		  while((buff[last]!='\n')||(leidos<=0));
		  buff[last]='\0';
		  if (buff[0]!='#') 
		    {
		      i++; /* to skip comment lines */
		      /* printf("line %d: %s\n",i,buff);*/
		      if (i==1) 
			{
			  if (strcmp(buff,"P6")!=0) 
			    fprintf(stderr,"file %s: non supported image format, must be raw PPM\n",file1_filename);
			}
		      else if (i==2)
			{
			  if ((sscanf(buff,"%d %d",&c,&r)!=2)||(c!=SIFNTSC_COLUMNS)||(r!=SIFNTSC_ROWS)) 
			    fprintf(stderr,"file %s: non supported image size, must be 320x240\n",file1_filename);
			}
		    }
		}	
	      for(i=0;i<SIFNTSC_ROWS*SIFNTSC_COLUMNS;i++)
		{
		  read(f2,(void *)buff,(size_t)3);
		  colorB[i*3]=buff[2];
		  colorB[i*3+1]=buff[1];
		  colorB[i*3+2]=buff[0];
		}
	      close(f2);
	      update_greyB();
	    }
	}}
  return output;
}


void image_completed(PhysicalDevice o)
{ 
  int i;
  int receivedA=FALSE, receivedB=FALSE;


  if (o==oculo1)
    {  
      /* printf("oculo1 %d, %d\n",oculo1,oculo1_lastimage);*/
      if ((source[SCH_IMAGEA]==oculo1)&&(imageAdevice==oculo1_lastimage)) receivedA=TRUE;
      else if ((source[SCH_IMAGEB]==oculo1)&&(imageBdevice==oculo1_lastimage)) receivedB=TRUE;
    }
  else if (o==oculo2)
    { 
      /* printf("oculo2 %d %d\n",oculo2,oculo2_lastimage);*/
      if ((source[SCH_IMAGEA]==oculo2)&&(imageAdevice==oculo2_lastimage)) receivedA=TRUE;
      else if ((source[SCH_IMAGEB]==oculo2)&&(imageBdevice==oculo2_lastimage)) receivedB=TRUE;
    }
 
  
  if (receivedA==TRUE)
    {
      /* printf("receivedA\n");*/
      kA+=1.;
      
      if (o==oculo1) memcpy(colorA,mmbuf1,SIFNTSC_COLUMNS*SIFNTSC_ROWS*3);
      else if (o==oculo2) memcpy(colorA,mmbuf2,SIFNTSC_COLUMNS*SIFNTSC_ROWS*3);
      
      for(i=0;i<imageA_users;i++)
	if (imageA_callbacks[i]!=NULL) imageA_callbacks[i]();

      update_greyA();
    }
  
  else if (receivedB==TRUE)
    { 
      kB+=1.;
      /* printf("receivedB\n");*/
      if (o==oculo1) memcpy(colorB,mmbuf1,SIFNTSC_COLUMNS*SIFNTSC_ROWS*3);
      else if (o==oculo2) memcpy(colorB,mmbuf2,SIFNTSC_COLUMNS*SIFNTSC_ROWS*3);
     
      for(i=0;i<imageB_users;i++)
	if (imageB_callbacks[i]!=NULL) imageB_callbacks[i]();

      update_greyB();
    }     
  
  
  /* Ask for the next image */
  /* no big problem with race conditions when reading state[SCH_IMAGEA]*/
  if ((o==oculo1) &&
      ((receivedA==TRUE)||(receivedB==TRUE)))
    { if ((source[SCH_IMAGEA]==oculo1)&&
	  (imageAdevice==oculo1_lastimage)&&
	  (state[SCH_IMAGEA]==active))
      {
	/*printf("ask for A in oculo1, %d\n",imageAdevice);*/
	pthread_mutex_lock(&oculo1_mutex); 
	sprintf(oculo1_out,"%d %d\n",OCULO_rgb24bpp_sifntsc_image_query,imageAdevice);
	write(sock_oculo1,oculo1_out,strlen(oculo1_out)); 
	pthread_mutex_unlock(&oculo1_mutex); 
      }
    else if ((source[SCH_IMAGEB]==oculo1)&&
	     (imageBdevice==oculo1_lastimage)&&
	     (state[SCH_IMAGEB]==active)) 
      {
	/*printf("ask for B in oculo1, %d\n",imageBdevice);*/
	pthread_mutex_lock(&oculo1_mutex); 
	sprintf(oculo1_out,"%d %d\n",OCULO_rgb24bpp_sifntsc_image_query,imageBdevice);
	write(sock_oculo1,oculo1_out,strlen(oculo1_out)); 
	pthread_mutex_unlock(&oculo1_mutex); 
      }
    }
  
  else if ((o==oculo2) &&
	   ((receivedA==TRUE)||(receivedB==TRUE)))
    { if ((source[SCH_IMAGEA]==oculo2)&&
	  (imageAdevice==oculo2_lastimage)&&
	  (state[SCH_IMAGEA]==active))
      {
	pthread_mutex_lock(&oculo2_mutex); 
	sprintf(oculo2_out,"%d %d\n",OCULO_rgb24bpp_sifntsc_image_query,imageAdevice);
	write(sock_oculo2,oculo2_out,strlen(oculo2_out)); 
	pthread_mutex_unlock(&oculo2_mutex); 
      }
    else if ((source[SCH_IMAGEB]==oculo2)&&
	     (imageBdevice==oculo2_lastimage)&&
	     (state[SCH_IMAGEB]==active)) 
      {
	pthread_mutex_lock(&oculo2_mutex); 
	sprintf(oculo2_out,"%d %d\n",OCULO_rgb24bpp_sifntsc_image_query,imageBdevice);
	write(sock_oculo2,oculo2_out,strlen(oculo2_out)); 
	pthread_mutex_unlock(&oculo2_mutex); 
      }
    }
}

void pantiltencoders_suspend(int i)
{
  int j;
 
  if ((i>pantiltencoders_users)||(i<0)) 
    {printf("bad range in pantiltencoders_suspend\n"); return;}

  for(j=i+1;j<pantiltencoders_users;j++)
    pantiltencoders_callbacks[j-1]=pantiltencoders_callbacks[j];
  pantiltencoders_users--;
  /* this thread never stops */

  /* unsubscribe to pantiltencoders */
 if (source[SCH_PANTILTENCODERS]==oculo1)
    {
      pthread_mutex_lock(&oculo1_mutex);  
      sprintf(oculo1_out,"%d\n",OCULO_unsubscribe_pantilt_encoders); 
      write(sock_oculo1,oculo1_out,strlen(oculo1_out));
      printf("pantiltencoders schema off\n");
      state[SCH_PANTILTENCODERS]=slept;
      pthread_mutex_unlock(&oculo1_mutex); 
    }
 else if (source[SCH_PANTILTENCODERS]==oculo2)
    {
      pthread_mutex_lock(&oculo2_mutex);  
      sprintf(oculo2_out,"%d\n",OCULO_unsubscribe_pantilt_encoders); 
      write(sock_oculo2,oculo2_out,strlen(oculo2_out));
      printf("pantiltencoders schema off\n");
      state[SCH_PANTILTENCODERS]=slept;
      pthread_mutex_unlock(&oculo2_mutex); 
    }
 else if (source[SCH_PANTILTENCODERS]==serialpantilt)
   {
     pthread_mutex_unlock(&mymutex[SCH_PANTILTENCODERS]);
     state[SCH_PANTILTENCODERS]=slept;
     printf("pantiltencoders schema off\n");
     pthread_mutex_unlock(&mymutex[SCH_PANTILTENCODERS]);
   }
}

int pantiltencoders_resume(arbitration fn)
{
  int output;

  if (pantiltencoders_users<MAX_USERS)
    {pantiltencoders_callbacks[pantiltencoders_users++]=fn;
    output=pantiltencoders_users-1;
    }
  else output=-1; 

  /* subscribe to pantiltencoders */
  if (source[SCH_PANTILTENCODERS]==oculo1)
    {
      pthread_mutex_lock(&oculo1_mutex);  
      sprintf(oculo1_out,"%d\n",OCULO_subscribe_pantilt_encoders); 
      write(sock_oculo1,oculo1_out,strlen(oculo1_out));
      printf("pantiltencoders schema on\n");
      state[SCH_PANTILTENCODERS]=active;
      pthread_mutex_unlock(&oculo1_mutex); 
    }
  else if (source[SCH_PANTILTENCODERS]==oculo2)
    {
      pthread_mutex_lock(&oculo2_mutex);  
      sprintf(oculo2_out,"%d\n",OCULO_subscribe_pantilt_encoders); 
      write(sock_oculo2,oculo2_out,strlen(oculo2_out));
      printf("pantiltencoders schema on\n");
      state[SCH_PANTILTENCODERS]=active;
      pthread_mutex_unlock(&oculo2_mutex); 
    } 
  else if (source[SCH_PANTILTENCODERS]==serialpantilt)
    {
      pthread_mutex_unlock(&mymutex[SCH_PANTILTENCODERS]);
      state[SCH_PANTILTENCODERS]=active;
      printf("pantiltencoders schema on\n");
      pthread_cond_signal(&condition[SCH_PANTILTENCODERS]);
      pthread_mutex_unlock(&mymutex[SCH_PANTILTENCODERS]);
    }
  return output;
}


int serve_oculomessage(PhysicalDevice o,char *mensaje)
{
  int tipo;
  int i2,i3,i4,i5;
  int j=0;
  unsigned long int reloj_oculo;

  if (sscanf(mensaje,"%d %lu",&tipo,&reloj_oculo)!=2) 
    {printf("No entiendo el mensaje del servidor: (%s)\n",mensaje);
    return -1;
    }
  else {
    /* avanza el "puntero j" sobre el mensaje hasta comenzar en el primer dato sensorial, saltandose el reloj y el tipo */
    while((mensaje[j]!='\n')&&(mensaje[j]!=' ')&&(mensaje[j]!='\0')) j++;
    j++;
    while((mensaje[j]!='\n')&&(mensaje[j]!=' ')&&(mensaje[j]!='\0')) j++;

    if (tipo == OCULO_y8bpp_sifntsc_image) {
      sscanf(&mensaje[++j],"%d %d %d %d\n",&i2,&i3,&i4,&i5);
      /*printf("Recibido mensaje preimagen %d (%d x %d, %d Bpp)\n",i4,i1,i2,i3);*/}
    else if (tipo == OCULO_rgb24bpp_sifntsc_image) {
      sscanf(&mensaje[++j],"%d %d %d %d\n",&i2,&i3,&i4,&i5);
      /*printf("Recibido mensaje preimagen %d (%d x %d, %d Bpp)\n",reloj_oculo1,i2,i3,i4);*/
      if (o==oculo1) oculo1_lastimage=i2;
      else if (o==oculo2) oculo2_lastimage=i2;

      if ((source[SCH_IMAGEA]==o)&&(imageAdevice==i2)) 
	imageA_clock=reloj_oculo;    
      else if ((source[SCH_IMAGEB]==oculo1)&&(imageBdevice==i2)) 
	imageB_clock=reloj_oculo; 
    }
    else if (tipo == OCULO_pantilt_encoders)
      {
	if (source[SCH_PANTILTENCODERS]==o)
	  {
	    sscanf(&mensaje[++j],"%f %f\n",&pan_angle, &tilt_angle);
	    /*printf("pan=%.1f tilt=%1.f\n",pan_angle,tilt_angle);*/
	    pantiltencoders_clock=reloj_oculo;
	    kpantiltencoders++;
	  }
      }
    else if (tipo == OCULO_pantilt_limits);
    else 
      {
	tipo=-1; printf("unknown oculo1 message\n");
      }
    return tipo;
  }
}

void *sensationsoculo1_thread(void *not_used) 
{
  int j=0;
  long int leidos=0,bytes_leidos=0,comienzo=0;
  char lastmessage[MAX_MESSAGE];
  /*  fd_set dummy_set,read_set;*/

   /* Escucha si el servidor le ha mandado algo. Si se bloquea esperando se para todo el programa !!. De momento atender al servidor es exclusivamente sacar por pantalla lo que este le envie. */

  for(;;)
    {
      /*
      FD_ZERO(&dummy_set);
      FD_ZERO(&read_set);
      FD_SET(sock_oculo1,&read_set);
      select(FD_SETSIZE,&read_set,&dummy_set,&dummy_set,NULL);
      */

      if (modolectura1==mensajes) 
	leidos=read(sock_oculo1,&(oculo1_in[oculo1_marca]),MAX_MESSAGE-oculo1_marca);
      else if (modolectura1==unaimagen) 
	{leidos=read(sock_oculo1,&(mmbuf1[oculo1_marca]),SIFNTSC_COLUMNS*SIFNTSC_ROWS*3+MAX_MESSAGE-oculo1_marca);
	}

      bytes_leidos=leidos;
      
      switch(leidos)
	{case 0: printf("oculo1 has closed its socket\n"); exit(1); break;
	case -1: break; /* no hay nada que leer */
	default: 
	  if (modolectura1==unaimagen) { /* no se acabo de leer la imagen */
	    if ((oculo1_marca+bytes_leidos)<(SIFNTSC_COLUMNS*SIFNTSC_ROWS*3)) 
	      { 
		oculo1_marca+=bytes_leidos;
	      } 
	    else 
	      {modolectura1=mensajes;
	      /*complete image. Now we put in message mode */
	      strncpy(oculo1_in,&(mmbuf1[SIFNTSC_COLUMNS*SIFNTSC_ROWS*3]),oculo1_marca+bytes_leidos-SIFNTSC_COLUMNS*SIFNTSC_ROWS*3);
	      bytes_leidos=oculo1_marca+bytes_leidos-SIFNTSC_COLUMNS*SIFNTSC_ROWS*3;
	      oculo1_marca=0; comienzo=0; 
	      /* lo deja preparado, para entrar a analizar los mensajes que se hayan recibido despues de la imagen */
	      image_completed(oculo1); /* tiene que ser despues de modolectura1=mensajes, para que no se salte la funcion */
	      }
	  }
	  
	  if (modolectura1==mensajes) {
	    comienzo=0;
	    for(j=oculo1_marca; (j<MAX_MESSAGE)&&(j<oculo1_marca+bytes_leidos)&&(modolectura1==mensajes);j++)
	      {if (oculo1_in[j]=='\n') 
		{strncpy(lastmessage,&oculo1_in[comienzo],j-comienzo);
		lastmessage[j-comienzo]='\0';
		comienzo=j+1;
		if (serve_oculomessage(oculo1,lastmessage)==OCULO_rgb24bpp_sifntsc_image) 
		  {
		    if ((oculo1_marca+bytes_leidos-comienzo)<SIFNTSC_COLUMNS*SIFNTSC_ROWS*3)
		      {strncpy(mmbuf1,&(oculo1_in[comienzo]),oculo1_marca+bytes_leidos-comienzo);
		      modolectura1=unaimagen; 
		      /*printf("Lectura imagen, trozo de %ld\n",oculo1_marca+bytes_leidos-comienzo);*/
		      oculo1_marca=oculo1_marca+bytes_leidos-comienzo;
		      /* Sale del for y del switch se salta la comprobacion de si relleno el oculo1_in para mensajes, y el desplazamiento del trozo de mensaje al comienzo del oculo1_in. Este oculo1_in se utiliza solo para mensajes. */ 
		      break; 
		      }
		    else /* (oculo1_marca+bytes_leidos-comienzo)>=SIFNTSC_COLUMNS*SIFNTSC_ROWS*1 */
		      {strncpy(mmbuf1,&(oculo1_in[comienzo]),SIFNTSC_COLUMNS*SIFNTSC_ROWS*3);
		      comienzo=comienzo+SIFNTSC_COLUMNS*SIFNTSC_ROWS*3;
		      j=j+SIFNTSC_COLUMNS*SIFNTSC_ROWS*3;
		      image_completed(oculo1);
		      /* DING: tenemos una nueva imagen completa */
		      /* Copiamos la imagen a la memoria de la imagen y seguimos analizando el resto de mensajes posteriores */
		      }
		  }
		}
	      }
	  }
	  
	  if (modolectura1==mensajes) {
	    if ((oculo1_marca+bytes_leidos-comienzo)>= MAX_MESSAGE) {oculo1_marca=0; printf("Se desbordo el buffer de recepcion\n");}
	    else if ((oculo1_marca+bytes_leidos-comienzo)>0) 
	      strncpy(oculo1_in,&(oculo1_in[comienzo]),oculo1_marca+bytes_leidos-comienzo); 
	    /* desplazo el resto de la tacada de bytes (que no completa un mensaje, pues si no hubiera sido procesado) al comienzo del buffer de rececpcion */
	    oculo1_marca=oculo1_marca+bytes_leidos-comienzo;
	  }
	  
	}
    
}}



void *sensationsoculo2_thread(void *not_used) 
{
  int j=0;
  long int leidos=0,bytes_leidos=0,comienzo=0;
  char lastmessage[MAX_MESSAGE];
  /*  fd_set dummy_set,read_set;*/

   /* Escucha si el servidor le ha mandado algo. Si se bloquea esperando se para todo el programa !!. De momento atender al servidor es exclusivamente sacar por pantalla lo que este le envie. */

  for(;;)
    {
      /*
      FD_ZERO(&dummy_set);
      FD_ZERO(&read_set);
      FD_SET(sock_oculo1,&read_set);
      select(FD_SETSIZE,&read_set,&dummy_set,&dummy_set,NULL);
      */

      if (modolectura2==mensajes) 
	leidos=read(sock_oculo2,&(oculo2_in[oculo2_marca]),MAX_MESSAGE-oculo2_marca);
      else if (modolectura2==unaimagen) 
	{leidos=read(sock_oculo2,&(mmbuf2[oculo2_marca]),SIFNTSC_COLUMNS*SIFNTSC_ROWS*3+MAX_MESSAGE-oculo2_marca);
	}

      bytes_leidos=leidos;
      
      switch(leidos)
	{case 0: printf("oculo2 has closed its socket\n"); exit(1); break;
	case -1: break; /* no hay nada que leer */
	default: 
	  if (modolectura2==unaimagen) { /* no se acabo de leer la imagen */
	    if ((oculo2_marca+bytes_leidos)<(SIFNTSC_COLUMNS*SIFNTSC_ROWS*3)) 
	      { 
		oculo2_marca+=bytes_leidos;
	      } 
	    else 
	      {modolectura2=mensajes;
	      /*complete image. Now we put in message mode */
	      strncpy(oculo2_in,&(mmbuf2[SIFNTSC_COLUMNS*SIFNTSC_ROWS*3]),oculo2_marca+bytes_leidos-SIFNTSC_COLUMNS*SIFNTSC_ROWS*3);
	      bytes_leidos=oculo2_marca+bytes_leidos-SIFNTSC_COLUMNS*SIFNTSC_ROWS*3;
	      oculo2_marca=0; comienzo=0; 
	      /* lo deja preparado, para entrar a analizar los mensajes que se hayan recibido despues de la imagen */
	      image_completed(oculo2); /* tiene que ser despues de modolectura2=mensajes, para que no se salte la funcion */
	      }
	  }
	  
	  if (modolectura2==mensajes) {
	    comienzo=0;
	    for(j=oculo2_marca; (j<MAX_MESSAGE)&&(j<oculo2_marca+bytes_leidos)&&(modolectura2==mensajes);j++)
	      {if (oculo2_in[j]=='\n') 
		{strncpy(lastmessage,&oculo2_in[comienzo],j-comienzo);
		lastmessage[j-comienzo]='\0';
		comienzo=j+1;
		if (serve_oculomessage(oculo2,lastmessage)==OCULO_rgb24bpp_sifntsc_image) 
		  {
		    if ((oculo2_marca+bytes_leidos-comienzo)<SIFNTSC_COLUMNS*SIFNTSC_ROWS*3)
		      {strncpy(mmbuf2,&(oculo2_in[comienzo]),oculo2_marca+bytes_leidos-comienzo);
		      modolectura2=unaimagen; 
		      /*printf("Lectura imagen, trozo de %ld\n",oculo2_marca+bytes_leidos-comienzo);*/
		      oculo2_marca=oculo2_marca+bytes_leidos-comienzo;
		      /* Sale del for y del switch se salta la comprobacion de si relleno el oculo2_in para mensajes, y el desplazamiento del trozo de mensaje al comienzo del oculo2_in. Este oculo2_in se utiliza solo para mensajes. */ 
		      break; 
		      }
		    else /* (oculo2_marca+bytes_leidos-comienzo)>=SIFNTSC_COLUMNS*SIFNTSC_ROWS*1 */
		      {strncpy(mmbuf2,&(oculo2_in[comienzo]),SIFNTSC_COLUMNS*SIFNTSC_ROWS*3);
		      comienzo=comienzo+SIFNTSC_COLUMNS*SIFNTSC_ROWS*3;
		      j=j+SIFNTSC_COLUMNS*SIFNTSC_ROWS*3;
		      image_completed(oculo2);
		      /* DING: tenemos una nueva imagen completa */
		      /* Copiamos la imagen a la memoria de la imagen y seguimos analizando el resto de mensajes posteriores */
		      }
		  }
		}
	      }
	  }
	  
	  if (modolectura2==mensajes) {
	    if ((oculo2_marca+bytes_leidos-comienzo)>= MAX_MESSAGE) {oculo2_marca=0; printf("Se desbordo el buffer de recepcion\n");}
	    else if ((oculo2_marca+bytes_leidos-comienzo)>0) 
	      strncpy(oculo2_in,&(oculo2_in[comienzo]),oculo2_marca+bytes_leidos-comienzo); 
	    /* desplazo el resto de la tacada de bytes (que no completa un mensaje, pues si no hubiera sido procesado) al comienzo del buffer de rececpcion */
	    oculo2_marca=oculo2_marca+bytes_leidos-comienzo;
	  }
	  
	}
    
}}



void us2xy(int numsensor, float d,float phi, Tvoxel *point) 

/*  Calcula la posicion respecto de sistema de referencia inicial (sistema odometrico) del punto detectado en el sistema de coordenadas solidario al sensor. OJO depende de estructura posiciones y de por el sensor, sabiendo que:
   a) el robot se encuentra en robot[0], robot[1] con orientacion robot[2] respecto al sistema de referencia externo,
   b) que el sensor se encuentra en xsen, ysen con orientacion asen respecto del sistema centrado en el robot apuntando hacia su frente, 
   c) el punto esta a distancia d del sensor en el angulo phi 
*/ 
{
  float  Xp_sensor, Yp_sensor, Xp_robot, Yp_robot;

  Xp_sensor = d*cos(DEGTORAD*phi);
  Yp_sensor = d*sin(DEGTORAD*phi);
  /* Coordenadas del punto detectado por el US con respecto al sistema del sensor, eje x+ normal al sensor */
  Xp_robot = us_coord[numsensor][0] + Xp_sensor*us_coord[numsensor][3] - Yp_sensor*us_coord[numsensor][4];
  Yp_robot = us_coord[numsensor][1] + Yp_sensor*us_coord[numsensor][3] + Xp_sensor*us_coord[numsensor][4];
  /* Coordenadas del punto detectado por el US con respecto al robot */
  (*point).x = Xp_robot*robot[3] - Yp_robot*robot[4] + robot[0];
  (*point).y = Yp_robot*robot[3] + Xp_robot*robot[4] + robot[1];
  /* Coordenadas del punto con respecto al origen del SdeR */
}



void laser2xy(int reading, float d, Tvoxel *point)

/*  Calcula la posicion respecto de sistema de referencia inicial (sistema odometrico) del punto detectado en el sistema de coordenadas solidario al sensor. OJO depende de estructura posiciones y de por el sensor, sabiendo que:
   a) el robot se encuentra en robot[0], robot[1] con orientacion robot[2] respecto al sistema de referencia externo,
   b) que el sensor se encuentra en xsen, ysen con orientacion asen respecto del sistema centrado en el robot apuntando hacia su frente, 
*/ 
{
  float  Xp_sensor, Yp_sensor, Xp_robot, Yp_robot,phi;
  
  phi=-90.+180.*reading/NUM_LASER;
  Xp_sensor = d*cos(DEGTORAD*phi);
  Yp_sensor = d*sin(DEGTORAD*phi);
  Xp_robot = laser_coord[0] + Xp_sensor*laser_coord[3] - Yp_sensor*laser_coord[4];
  Yp_robot = laser_coord[1] + Yp_sensor*laser_coord[3] + Xp_sensor*laser_coord[4];
  /* Coordenadas del punto detectado por el laser con respecto al robot */
  (*point).x = Xp_robot*robot[3] - Yp_robot*robot[4] + robot[0];
  (*point).y = Yp_robot*robot[3] + Xp_robot*robot[4] + robot[1];
  /* Coordenadas del punto con respecto al origen del SdeR */
}


void serve_otosmessage(char *mensaje)
{
  unsigned long int reloj_otos;
  int tipo,l; 
  int j=0,sensor, i;
  float medida;
  unsigned long nowtime; /* microsecs */
  float nowx, nowy, nowtheta; 
  float a1,a2,a3;

  /* uncomment this when scanf requests a "," (instead of a dot) between 
     the integer part and the decimal part of a real number. 
     Typically this appears as "unexpected conversion failure in 
     encoders data" */

  /*  for (i=0;i<=strlen(mensaje);i++)
      if (mensaje[i]=='.') mensaje[i]=',';*/
 
  if (sscanf(mensaje,"%d %lu",&tipo,&reloj_otos)!=2) printf("No entiendo el mensaje del servidor OTOS: (%s)\n",mensaje);
  else {
  /* avanza el "puntero j" sobre el mensaje hasta comenzar en el primer dato sensorial, saltandose el reloj y el tipo */
  while((mensaje[j]!='\n')&&(mensaje[j]!=' ')&&(mensaje[j]!='\0')) j++;
  j++;
  while((mensaje[j]!='\n')&&(mensaje[j]!=' ')&&(mensaje[j]!='\0')) j++;
  
  
  if (tipo==OTOS_sonars)  
    { 
      while (mensaje[j]==' ')  /* leo el par sensor-medida. Cuando buffer[j]=='\n' o =='\0' entonces se para */
      {if (sscanf(&mensaje[++j],"%d %f",&sensor, &medida)!=2) printf(" Fallo de conversion en mensaje de sonars \n");
      else { /* incorpora la medida y avanza el "puntero j" sobre el mensaje  */
	while((mensaje[j]!='\n')&&(mensaje[j]!=' ')&&(mensaje[j]!='\0')) j++;
	j++;
	while((mensaje[j]!='\n')&&(mensaje[j]!=' ')&&(mensaje[j]!='\0')) j++;
	ksonars++;
	us[sensor]=medida;
	us_clock[sensor]=reloj_otos;
	if (debug[SCH_SONARS]) printf(" sonar %d %1.1f\n",sensor,medida);
   
	for(i=0;i<sonars_users;i++)
	  if (sonars_callbacks[i]!=NULL) sonars_callbacks[i](sensor);
      }}
    }

  else if (tipo==OTOS_laser)  
    { 
      klaser++;
      sensor=0;

      if (debug[SCH_LASER]) printf("laser ");
      while (mensaje[j]==' ')  
	/* leo una medida. Cuando buffer[j]=='\n' o =='\0' se para */
      {if (sscanf(&mensaje[++j],"%d ",&l)!=1) printf(" Fallo de conversion en mensaje laser\n");
      else { /* incorpora la medida y avanza el "puntero j" sobre el mensaje  */
	while((mensaje[j]!='\n')&&(mensaje[j]!=' ')&&(mensaje[j]!='\0')) j++;
	/*j++;*/
	laser[sensor++]=l; 
	laser_clock=reloj_otos;
	if (debug[SCH_LASER]) printf("%d ",l);
      }
      }

      if (sensor>(NUM_LASER-2))
	/* only calls for callbacks if the laser message was correctly received. That means NUM_LASER readings (181 in the robot, 179 in the simulator) */
	{
	  for(i=0;i<laser_users;i++)
	    if (laser_callbacks[i]!=NULL) laser_callbacks[i]();
	}

 
      if (debug[SCH_LASER]) printf("\n");
    }

  else if (tipo==OTOS_encoders) 
{ 
/* Actualizo las variables internas de posicion */
    if (sscanf(&mensaje[++j],"%f %f %f %lu",&nowx,&nowy,&nowtheta,&nowtime)!=4) 
      {printf("Conversion failure in encoders reading\n");
      /* watch out with LOCALE configuration. With bad settings it expects float 
	 numbers to be composed of an integer part and a decimal part separated
	 by a single comma (not a single dot). In such a case the sscanf fails 
	 when processing real numbers as those involved in encoders data, because
	 otos sends them using the dot */
      printf(" This is a float for me: %f\n",3.141592654);
      }
    else {
      kencoders++;
      encoders_clock=reloj_otos;
      if (nowtheta <=0) nowtheta=nowtheta+360; 
      if (nowtime>lasttime) 
	{
	  tspeed=(1000000*sqrt((nowx-robot[0])*(nowx-robot[0])+(nowy-robot[1])*(nowy-robot[1])))/(nowtime-lasttime);
	  if (((nowx-robot[0])*robot[3]+(nowy-robot[1])*robot[4])<0) tspeed=-tspeed;

	  /* square to avoid function calling and its delay. Division by DEGTOREAD introduces some little error */
	  a1=(robot[2]/DEGTORAD-nowtheta)*(robot[2]/DEGTORAD-nowtheta);
	  a2=(robot[2]/DEGTORAD-nowtheta-360)*(robot[2]/DEGTORAD-nowtheta-360);
	  a3=(robot[2]/DEGTORAD-nowtheta+360)*(robot[2]/DEGTORAD-nowtheta+360);
	  if ((a1<a2)&&(a1<a3)) 
	    rspeed=(1000000*(nowtheta-robot[2]/DEGTORAD))/(nowtime-lasttime);
	  else if ((a2<a1)&&(a2<a3)) 
	    rspeed=(1000000*(nowtheta-360-robot[2]/DEGTORAD))/(nowtime-lasttime);
	  else 
	    rspeed=(1000000*(nowtheta+360-robot[2]/DEGTORAD))/(nowtime-lasttime);
	}

      /* El resto de la linea ya esta leido del fichero al leer la linea completa*/
     
      lasttime=nowtime;
      robot[0] = nowx;
      robot[1] = nowy;
      robot[2] = nowtheta * DEGTORAD;
      robot[3] = cos(robot[2]);
      robot[4] = sin(robot[2]);

      for(i=0;i<encoders_users;i++)
	if (encoders_callbacks[i]!=NULL) encoders_callbacks[i]();
      
      if (debug[SCH_ENCODERS]) printf(" encoders %.2f %.2f %.2f %lu\n",robot[0],robot[1],robot[2],nowtime);

     }}

  else printf("Codigo de mensaje no reconocido\n");
  }
}


void sonars_suspend(int i)
{
  int j;

  if ((i>sonars_users)||(i<0)) 
    {printf("bad range in sonars_suspend\n"); return;}

  for(j=i+1;j<sonars_users;j++)
    sonars_callbacks[j-1]=sonars_callbacks[j];
  sonars_users--;

  state[SCH_SONARS]=slept;
  printf("sonars schema off\n");

 if (source[SCH_SONARS]==otos)
   {
     pthread_mutex_lock(&otos_mutex);
     sprintf(otos_out,"%d\n",OTOS_unsubscribe_us);
     write(sock_otos,otos_out,strlen(otos_out));
     pthread_mutex_unlock(&otos_mutex);
   }
}


int sonars_resume(intcallback fn)
{
int output;

 if (sonars_users<MAX_USERS)
   {
     sonars_callbacks[sonars_users++]=fn;
     output = sonars_users-1;
   }
 else output=-1;

 state[SCH_SONARS]=active;
 printf("sonars schema on\n");

 if (source[SCH_SONARS]==otos)
   {
     pthread_mutex_lock(&otos_mutex);
     sprintf(otos_out,"%d\n",OTOS_subscribe_us);
     write(sock_otos,otos_out,strlen(otos_out));
     pthread_mutex_unlock(&otos_mutex);
   }
 return output;
}


void laser_suspend(int i)
{
  int j;

  if ((i>laser_users)||(i<0)) 
    {printf("bad range in laser_suspend\n"); return;}

  for(j=i+1;j<laser_users;j++)
    laser_callbacks[j-1]=laser_callbacks[j];
  laser_users--;

  state[SCH_LASER]=slept;
  printf("laser schema off\n");
  if (source[SCH_LASER]==otos)
    {
      pthread_mutex_lock(&otos_mutex);
      sprintf(otos_out,"%d\n",OTOS_unsubscribe_laser);
      write(sock_otos,otos_out,strlen(otos_out));
      pthread_mutex_unlock(&otos_mutex);
    }
}

int laser_resume(arbitration fn)
{
int output;

 if (laser_users<MAX_USERS)
   {
     laser_callbacks[laser_users++]=fn;
     output = laser_users-1;
   }
 else output=-1;

 state[SCH_LASER]=active;
 printf("laser schema on\n");

 if (source[SCH_LASER]==otos)
   {
     pthread_mutex_lock(&otos_mutex); 
     sprintf(otos_out,"%d\n",OTOS_subscribe_laser);
     write(sock_otos,otos_out,strlen(otos_out));
     pthread_mutex_unlock(&otos_mutex);
   }
 return output;
}


void encoders_suspend(int i)
{
  int j;

  if ((i>encoders_users)||(i<0)) 
    {printf("bad range in encoders_suspend\n"); return;}
  for(j=i+1;j<encoders_users;j++)
    encoders_callbacks[j-1]=encoders_callbacks[j];
  encoders_users--;

  state[SCH_ENCODERS]=slept;
  printf("encoders schema off\n");

  if (source[SCH_ENCODERS]==otos)
    {
      /* se desuscribe encoders de odometria */
      pthread_mutex_lock(&otos_mutex);
      sprintf(otos_out,"%d\n", OTOS_unsubscribe_encoders);
      write(sock_otos,otos_out,strlen(otos_out));
      pthread_mutex_unlock(&otos_mutex);
    }
}

int encoders_resume(arbitration fn)
{
int output;

 if (encoders_users<MAX_USERS)
   {
     encoders_callbacks[encoders_users++]=fn;
     output = encoders_users-1;
   }
 else output=-1;

 state[SCH_ENCODERS]=active;
 printf("encoders schema on\n");
 if (source[SCH_ENCODERS]==otos)
    {
      /* se suscribe (conexion directa) encoders de odometria */
      pthread_mutex_lock(&otos_mutex);
      sprintf(otos_out,"%d\n", OTOS_subscribe_encoders);
      write(sock_otos,otos_out,strlen(otos_out));
      pthread_mutex_unlock(&otos_mutex);
    }
 return output;
}




void *sensationsotos_thread(void *not_used) 
{
  int leidos, beginning,j;
  /*  fd_set dummy_set,read_set;*/
  
  /* Escucha si el servidor le ha mandado algo. Si se bloquea esperando se para todo el programa !! */

  for(;;)
    {

      /*      FD_ZERO(&dummy_set);
      FD_ZERO(&read_set);
      FD_SET(sock_otos,&read_set);
      printf("leo \n");      
      select(FD_SETSIZE,&read_set,&dummy_set,&dummy_set,NULL);
      */
    
      leidos=read(sock_otos,&otos_in[otos_marca],MAX_MESSAGE); 
      /* no hay desbordamiento al poner MAX_MESSAGE. Como de momento no hay granularidad de mensaje, las lineas las leo por trozos. Si me ha venido una linea muy larga el siguiente trozo lo leere en el siguiente read_from_socket_poll, de momento se queda en el buffer asociado al socket que tenga el sistema operativo.*/
      switch(leidos) { 
      case 0: close(sock_otos); break; /* Se mando EOF: borramos socket de la lista */
      case -1: ; break; /* Por ejemplo cuando no hay nada que leer break; */
      default: 
	/* Hay algo en ese socket, si hemos completado un mensaje (= ha llegado \n) se pasa a procesarlo. En caso contrario simplemente anyadimos el trozo leido al input_buffer y ya terminaremos de leer el mensaje entero en proximas llamadas. Si hay varios mensajes completos seguidos se atienden a todos. Asume que el mensaje en el buffer comienza en justo en el origen del buffer, no mas alla. */
	{ beginning=0;
	
        for(j=otos_marca; (j<MAX_MESSAGE)&&(j<otos_marca+leidos);j++)
	  { if (otos_in[j]=='\n') {serve_otosmessage(&otos_in[beginning]);
	  beginning=j+1;}}
	
        if (beginning==0) /* no se acabo ningun mensaje en esta recepcion */
          {if (otos_marca+leidos >= MAX_MESSAGE-1) otos_marca=0; 
          /* El buffer de recepcion se ha llenado con los trozos del mensaje recibidos, pero no se ha completado ningun mensaje entero => se descarta el actual mensaje. OJO el mensaje siguiente comienzara mal pues tiene como inicio el resto del mensaje anterior no leido, y tambien sera desechada por incomprensible. Asi cuando hay un desbordamiento pierdo el mensaje desbordado y el siguiente. No obstante no espero que haya ningun desbordamiento. */
          else otos_marca+=leidos;
          }
        else /* (beginning!=0) */
          {strncpy(otos_in,&(otos_in[beginning]),otos_marca+leidos-beginning); 
          otos_marca = otos_marca+leidos-beginning;
          beginning=0; /* para la siguiente tacada de recepcion */
          }
	}
      }
    }
}


void motors_iteration()
{  
  kmotors++;
  if (source[SCH_MOTORS]==otos)
    {
      /* Manda al servidor OTOS el control de traccion en velocidad mm/s, mm/s2, a traves de la interfaz socket */
      pthread_mutex_lock(&otos_mutex);
      sprintf(otos_out,"%d %1.1f %1.1f\n",OTOS_drive_speed,v,ac);
      write(sock_otos,otos_out,strlen(otos_out));
      sprintf(otos_out,"%d %1.1f %1.1f\n",OTOS_steer_speed,w,ac);
      write(sock_otos,otos_out,strlen(otos_out));
      pthread_mutex_unlock(&otos_mutex);
    }
 if (debug[SCH_MOTORS]) printf("motors\n"); 
}


void motors_suspend()
{
  pthread_mutex_lock(&mymutex[SCH_MOTORS]);
  state[SCH_MOTORS]=slept;
  printf("motors schema off\n");
  pthread_mutex_unlock(&mymutex[SCH_MOTORS]);

  v=0; w=0;
  if (source[SCH_MOTORS]==otos)
    {
      pthread_mutex_lock(&otos_mutex);
      sprintf(otos_out,"%d %1.1f %1.1f\n",OTOS_drive_speed,0.,0.);
      write(sock_otos,otos_out,strlen(otos_out));
      sprintf(otos_out,"%d %1.1f %1.1f\n",OTOS_steer_speed,0.,0.);
      write(sock_otos,otos_out,strlen(otos_out));
      pthread_mutex_unlock(&otos_mutex);
    }
  /* safety stop: really sends the stop speeds before really suspend this schema. The real suspend will take place in the next motors_iteration */
}

void motors_resume(int *brothers, arbitration fn)
{
  pthread_mutex_lock(&mymutex[SCH_MOTORS]);
  state[SCH_MOTORS]=winner;
  printf("motors schema on\n");
  pthread_cond_signal(&condition[SCH_MOTORS]);
  pthread_mutex_unlock(&mymutex[SCH_MOTORS]);
}

void *motors_thread(void *not_used) 
{
  struct timeval a,b;
  long diff, next;

  for(;;)
    {
      pthread_mutex_lock(&mymutex[SCH_MOTORS]);

      if (state[SCH_MOTORS]==slept) 
	{
	  /*printf("motors schema off\n");*/
	  pthread_cond_wait(&condition[SCH_MOTORS],&mymutex[SCH_MOTORS]);
	  /*printf("motors schema on\n");*/
	}
      else 
	{
	  gettimeofday(&a,NULL);
	  motors_iteration();
	  gettimeofday(&b,NULL);  
	  diff = (b.tv_sec-a.tv_sec)*1000000+b.tv_usec-a.tv_usec;
	  
	  next = motors_cycle*1000-diff-10000; 
	  /* discounts 10ms taken by calling usleep itself */
	  if (next>0) usleep(motors_cycle*1000-diff);
	  else {printf("time interval violated: motors\n"); usleep(motors_cycle*1000);}
	}
      pthread_mutex_unlock(&mymutex[SCH_MOTORS]);
    }
}


void pantiltmotors_iteration()
{  
  float longspeed, latspeed, longcommand, latcommand;

  kpantiltmotors++;
  if (source[SCH_PANTILTMOTORS]==oculo1)
    {
      /* Manda al servidor OCULO el control en posicion, en degs, a traves de la interfaz socket */   
      pthread_mutex_lock(&oculo1_mutex);
      sprintf(oculo1_out,"%ld %1.1f %1.1f %1.1f %1.1f\n", (long int)OCULO_pantilt_position,longitude,latitude,longitude_speed,latitude_speed);
      /*  sprintf(oculo1_out,"%ld %1.1f %1.1f\n", (long int)OCULO_pantilt_position,longitude,latitude);*/
      write(sock_oculo1,oculo1_out,strlen(oculo1_out));
      pthread_mutex_unlock(&oculo1_mutex);
    }
  else if (source[SCH_PANTILTMOTORS]==oculo2)
    {
      pthread_mutex_lock(&oculo2_mutex);
      sprintf(oculo2_out,"%ld %1.1f %1.1f %1.1f %1.1f\n", (long int)OCULO_pantilt_position,longitude,latitude,longitude_speed,latitude_speed);
      /*  sprintf(oculo2_out,"%ld %1.1f %1.1f\n", (long int)OCULO_pantilt_position,longitude,latitude);*/
      write(sock_oculo2,oculo2_out,strlen(oculo1_out));
      pthread_mutex_unlock(&oculo2_mutex);
    }
  else if (source[SCH_PANTILTMOTORS]==serialpantilt)
    {
      /*printf("JDE: longitude speed %.2f, latitude speed %.2f\n",longitude_speed,latitude_speed);*/
      /* truncate both speed and position inside permited values*/
      if (longitude_speed < 0.0)     longspeed=0.0;
      else if (longitude_speed > MAX_SPEED_PANTILT)  longspeed=MAX_SPEED_PANTILT;
      else longspeed=longitude_speed;
      if (latitude_speed < 0.0)    latspeed=0.0;
      else if (latitude_speed > MAX_SPEED_PANTILT) latspeed=MAX_SPEED_PANTILT;
      else latspeed=latitude_speed;

      if (longitude > max_pan) longcommand = max_pan;
      else if (longitude < min_pan) longcommand = min_pan;
      else longcommand=longitude;
      if (latitude > max_tilt)  latcommand = max_tilt;
      else if (latitude < min_tilt) latcommand = min_tilt;
      else latcommand=latitude;
      

      pthread_mutex_lock(&serialpantilt_mutex);
      if (longcommand!=longitude_last) {
	sprintf(pantilt_out,"PS%d PP%d\n",(int)(longspeed/ENCOD_TO_DEG),(int)(longcommand/ENCOD_TO_DEG));
	SendCmd(pantilt_out);
	longitude_last=longcommand;
      }
      if (latcommand!=latitude_last) {
	sprintf(pantilt_out,"TS%d TP%d\n",(int)(latspeed/ENCOD_TO_DEG),(int)(latcommand/ENCOD_TO_DEG));
	SendCmd(pantilt_out);
	latitude_last=latcommand;
      }
      pthread_mutex_unlock(&serialpantilt_mutex);
    }

  if (debug[SCH_PANTILTMOTORS]) printf("pantiltmotors:  %1.1f %1.1f %1.1f %1.1f \n",latitude,longitude,latitude_speed,longitude_speed); 
}

void pantiltmotors_suspend()
{
  pthread_mutex_lock(&mymutex[SCH_PANTILTMOTORS]);
  state[SCH_PANTILTMOTORS]=slept;
  printf("pantiltmotors schema off\n");
  pthread_mutex_unlock(&mymutex[SCH_PANTILTMOTORS]);
}

void pantiltmotors_resume(int *brothers, arbitration fn)
{
  latitude=pan_angle;
  longitude=tilt_angle;
  /* unfortunately it reads pan_angle an tilt_angle before sensationsoculo1
     has updated them for the first time, so this schema always starts 
     positioning the pantilt unit at (0,0) angles */ 
  pthread_mutex_lock(&mymutex[SCH_PANTILTMOTORS]);
  state[SCH_PANTILTMOTORS]=winner;
  printf("pantiltmotors schema on\n");
  pthread_cond_signal(&condition[SCH_PANTILTMOTORS]);
  pthread_mutex_unlock(&mymutex[SCH_PANTILTMOTORS]);
}

void *pantiltmotors_thread(void *not_used) 
{
  struct timeval a,b;
  long diff, next;

  for(;;)
    {
      pthread_mutex_lock(&mymutex[SCH_PANTILTMOTORS]);

      if (state[SCH_PANTILTMOTORS]==slept) 
	{
	  /* printf("pantiltmotors off\n");*/
	  pthread_cond_wait(&condition[SCH_PANTILTMOTORS],&mymutex[SCH_PANTILTMOTORS]);
	  /* printf("pantiltmotors on\n");*/
	}
      else 
	{
	  gettimeofday(&a,NULL);
	  pantiltmotors_iteration();
	  gettimeofday(&b,NULL);  
	  diff = (b.tv_sec-a.tv_sec)*1000000+b.tv_usec-a.tv_usec;
	  
	  next = pantiltmotors_cycle*1000-diff-10000; 
	  /* discounts 10ms taken by calling usleep itself */
	  if (next>0) usleep(pantiltmotors_cycle*1000-diff);
	  else {printf("time interval violated: pantiltmotors\n"); usleep(pantiltmotors_cycle*1000);}
	}
      pthread_mutex_unlock(&mymutex[SCH_PANTILTMOTORS]);
    }
}


#ifdef JDEVIDEO4LINUX
void *video4linux1_thread(void *not_used) 
{ int cam=0;
 int f,c,yy,j;
 unsigned char y,u,v;
 unsigned char *src, *dest; 
 for(;;)
   {
     if (((served[video4linux1][SCH_IMAGEA]==TRUE) &&
	  (state[SCH_IMAGEA]==active))        || 
	 ((served[video4linux1][SCH_IMAGEB]==TRUE) &&
	  (state[SCH_IMAGEB]==active)))
       { /* copy local video4linux image to imageA or image B*/
	 src = map[cam]+gb_buffers[cam].offsets[gb[cam].frame];
	 if (served[video4linux1][SCH_IMAGEA]==TRUE) dest= (unsigned char*)colorA;
	 else if (served[video4linux1][SCH_IMAGEB]==TRUE) dest=(unsigned char*) colorB;
	 
	 if (vpic[cam].palette==VIDEO_PALETTE_YUV420P)
	   {
	     for (j = 0; j < SIFNTSC_COLUMNS*SIFNTSC_ROWS; j++) 
	       {
		 f=j/SIFNTSC_COLUMNS; c=j%SIFNTSC_COLUMNS;
		 y=src[j];
		 u=src[SIFNTSC_COLUMNS*SIFNTSC_ROWS+(f/2)*SIFNTSC_COLUMNS/2+c/2];
		 v=src[SIFNTSC_COLUMNS*SIFNTSC_ROWS*5/4+(f/2)*SIFNTSC_COLUMNS/2+c/2];
		 yy = GRAY(y);
		 dest[3*j]=BLUE(yy,u);
		 dest[3*j+1]=GREEN(yy,v,u);
		 dest[3*j+2]=RED(yy,v);	
	       }
	   }
	 else if (vpic[cam].palette==VIDEO_PALETTE_RGB565)
	   {
	     for (j = 0; j < SIFNTSC_COLUMNS*SIFNTSC_ROWS; j++) 
	       {dest[3*j]=  (0x1F&src[2*j])<<3; /* blue */
	       dest[3*j+1]= ((0x07&src[2*j+1])<<5) | ((0xE0&src[2*j])>>3); /* green */
	       dest[3*j+2]= (0xF8&(src[2*j+1]));  /* red */
	       }
	   }
	 else if (vpic[cam].palette==VIDEO_PALETTE_RGB24)
	   {
	     memcpy(dest,src,SIFNTSC_COLUMNS*SIFNTSC_ROWS*3);
	   }

	 /* fps counter and callbacks for imageA or imageB */
	 if (served[video4linux1][SCH_IMAGEA]==TRUE)
	   {
	     kA+=1.;
	     for(j=0;j<imageA_users;j++)
	       if (imageA_callbacks[j]!=NULL) imageA_callbacks[j]();
	   }
	 else if (served[video4linux1][SCH_IMAGEB]==TRUE)
	   {
	     kB+=1.;
	     for(j=0;j<imageB_users;j++)
	       if (imageB_callbacks[j]!=NULL) imageB_callbacks[j]();
	   }

	 /* asks for next capture */
	 gb[cam].frame=(gb[cam].frame+1)%2;
	 if (-1 == ioctl(fdv4l[cam],VIDIOCMCAPTURE,&gb[cam])) {
	   perror("VIDIOCMCAPTURE");
	 } 
	 
	 /* "libera" ese frame para que el driver pueda cargar ahi nueva imagen.*/
	 if (-1 == ioctl(fdv4l[cam],VIDIOCSYNC,&gb[cam].frame)) {
	   perror("VIDIOCSYNC");
	 }
       }
     else usleep(300000);
}}

void *video4linux2_thread(void *not_used) 
{ int cam=1;
 int f,c,yy,j;
 unsigned char y,u,v;
 unsigned char *src, *dest; 
 for(;;)
   {
       if (((served[video4linux2][SCH_IMAGEA]==TRUE) &&
	  (state[SCH_IMAGEA]==active))        || 
	 ((served[video4linux2][SCH_IMAGEB]==TRUE) &&
	  (state[SCH_IMAGEB]==active)))
       {
	 /* copy local video4linux image to imageA or image B*/
	 src = map[cam]+gb_buffers[cam].offsets[gb[cam].frame];
	 if (served[video4linux2][SCH_IMAGEA]==TRUE) dest= (unsigned char*)colorA;
	 else if (served[video4linux2][SCH_IMAGEB]==TRUE) dest=(unsigned char*) colorB;
	 
	 if (vpic[cam].palette==VIDEO_PALETTE_YUV420P)
	   {
	     for (j = 0; j < SIFNTSC_COLUMNS*SIFNTSC_ROWS; j++) 
	       {
		 f=j/SIFNTSC_COLUMNS; c=j%SIFNTSC_COLUMNS;
		 y=src[j];
		 u=src[SIFNTSC_COLUMNS*SIFNTSC_ROWS+(f/2)*SIFNTSC_COLUMNS/2+c/2];
		 v=src[SIFNTSC_COLUMNS*SIFNTSC_ROWS*5/4+(f/2)*SIFNTSC_COLUMNS/2+c/2];
		 yy = GRAY(y);
		 dest[3*j]=BLUE(yy,u);
		 dest[3*j+1]=GREEN(yy,v,u);
		 dest[3*j+2]=RED(yy,v);	
	       }
	   }
	 else if (vpic[cam].palette==VIDEO_PALETTE_RGB565)
	   {
	     for (j = 0; j < SIFNTSC_COLUMNS*SIFNTSC_ROWS; j++) 
	       {dest[3*j]=  (0x1F&src[2*j])<<3; /* blue */
	       dest[3*j+1]= ((0x07&src[2*j+1])<<5) | ((0xE0&src[2*j])>>3); /* green */
	       dest[3*j+2]= (0xF8&(src[2*j+1]));  /* red */
	       }
	   }
	 else if (vpic[cam].palette==VIDEO_PALETTE_RGB24)
	   {
	     memcpy(dest,src,SIFNTSC_COLUMNS*SIFNTSC_ROWS*3);
	   }
	 
	 /* fps counter and callbacks for imageA or imageB */
	 if (served[video4linux2][SCH_IMAGEA]==TRUE)
	   {
	     kA+=1.;
	     for(j=0;j<imageA_users;j++)
	       if (imageA_callbacks[j]!=NULL) imageA_callbacks[j]();
	   }
	 else if (served[video4linux2][SCH_IMAGEB]==TRUE)
	   {
	     kB+=1.;
	     for(j=0;j<imageB_users;j++)
	       if (imageB_callbacks[j]!=NULL) imageB_callbacks[j]();
	   }

	 /* asks for next capture */
	 gb[cam].frame=(gb[cam].frame+1)%2;
	 if (-1 == ioctl(fdv4l[cam],VIDIOCMCAPTURE,&gb[cam])) {
	   perror("VIDIOCMCAPTURE");
	 } 
	 
	 /* "libera" ese frame para que el driver pueda cargar ahi nueva imagen.*/
	 if (-1 == ioctl(fdv4l[cam],VIDIOCSYNC,&gb[cam].frame)) {
	   perror("VIDIOCSYNC");        
	 }
       }
     else usleep(300000);
}}
#endif 


#ifdef JDEFIREWIRE
void *firewire_thread(void *not_used) 
{
  int i,j;
  for(;;)
    {
      /* printf("iteration %d\n",k++);*/

      if (((served[firewire][SCH_IMAGEA]==TRUE) &&
	  (state[SCH_IMAGEA]==active))        || 
	 ((served[firewire][SCH_IMAGEB]==TRUE) &&
	  (state[SCH_IMAGEB]==active)))
       {
	 dc1394_dma_multi_capture(cameras, numCameras);
	 for (i = 0; i < numCameras; i++)
	   {
	     if ((i==imageAdevice) && 
		 (served[firewire][SCH_IMAGEA]==TRUE) &&
		 (state[SCH_IMAGEA]==active))  
	       {kA++;
	       uyvy2rgb((unsigned char*)cameras[i].capture_buffer,(unsigned char*)colorA,SIFNTSC_COLUMNS*SIFNTSC_ROWS);
	       for(j=0;j<imageA_users;j++)
		 if (imageA_callbacks[j]!=NULL) imageA_callbacks[j]();
	       /*update_greyA();*/
	       }
	     if ((i==imageBdevice) && 
		 (served[firewire][SCH_IMAGEB]==TRUE) &&
		 (state[SCH_IMAGEB]==active))  
	       {kB++;
	       uyvy2rgb((unsigned char*)cameras[i].capture_buffer,(unsigned char*)colorB,SIFNTSC_COLUMNS*SIFNTSC_ROWS);
	       for(j=0;j<imageB_users;j++)
		 if (imageB_callbacks[j]!=NULL) imageB_callbacks[j]();
	       /*update_greyB();*/
	       }
	     dc1394_dma_done_with_buffer(&cameras[i]);
	   }
       }
      else usleep(100000);
    }
}
#endif


int openRaw(char* filename, mode_t io_flags)
/*                                                        */
/*  Opens passed filename and sets its device to raw mode.*/
/*  Returns file descriptor.                              */
/*                                                        */
/*  raw mode is defined as:                               */
/*  Off: echo, canonical input, extended processing,      */
/*       signals, break key, parity, 8th bit strip,       */
/*       flow control, output post processing             */
/*   On: 8 bit size                                       */
{
  struct termios term_info;
  int fd;

  fd = open(filename,io_flags);
  if (fd == -1)
    {
      /* maybe complain if TRACE is on */
      return -1;
    }
  
  if(tcgetattr(fd,&term_info) <0)
    {
      /* complain - fd is not a terminal */
      return -1;
    }

  /* turn off echo, canonical mode, extended processing, signals */
  term_info.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG); 
  
  /* turn off break sig, cr->nl, parity off, 8 bit strip, flow control */
  term_info.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

  /* clear size, turn off parity bit */
  term_info.c_cflag &= ~(CSIZE | PARENB);

  /* set size to 8 bits */
  term_info.c_cflag |= CS8;

  /* turn output processing off */
  term_info.c_oflag &= ~(OPOST);

  /* Set time and bytes to read at once */
  term_info.c_cc[VTIME] = 0;
  term_info.c_cc[VMIN] = 0;

  if(tcsetattr(fd,TCSAFLUSH,&term_info) <0)
    return -1;

  return fd;
}

int setBaudRate (int fd, speed_t baudRate)
/*  Set the baud rate of an open device.                  */
/*  Baud rate constants are found in <termios.h>.         */
/*  B9600 == 9600 baud                                    */
{
   struct termios terminfo;
   int error;
     
   error = tcgetattr(fd, &terminfo);
   if (error)
      {
         perror("tcgetattr()");
         return(-1);
      }
   error = cfsetospeed(&terminfo, baudRate);
   if (error)
      {
         fprintf(stderr, "cfsetospeed(%ld): ",
                 (long)baudRate);
         perror(NULL);
         return(-1);
      }
   error = cfsetispeed(&terminfo, baudRate);
   if (error)
      {
         fprintf(stderr, "cfsetispeed(%ld): ",
                 (long)baudRate);
         perror(NULL);
         return(-1);
      }
   error = tcsetattr(fd, TCSANOW, &terminfo);
   if (error)
      {
         perror("tcsetattr()");
         return(-1);
      }
   return(1);
}

void SendCmd(char *cmd){
  write(pantilt_serialport,cmd,strlen(cmd));  
  /* printf("serialpantilt sends: %s",cmd);*/
}

void serve_serialpantilt_message(char *mensaje)
{
  int pan_encoders, tilt_encoders,kk; /* encoders units */
  /* printf("Recibi:%s\n",mensaje); */

  if (sscanf(mensaje,"* Current Pan position is %d",&pan_encoders)==1) 
    {
      pan_angle=(float)pan_encoders*ENCOD_TO_DEG; 
      if (source[SCH_PANTILTENCODERS]==serialpantilt)
	kpantiltencoders++;
      /*printf("PAN=%.2f\n",pan_angle);*/
    }
  else if (sscanf(mensaje,"* Current Tilt position is %d",&tilt_encoders)==1)
   {tilt_angle=(float)tilt_encoders*ENCOD_TO_DEG;
   /*printf("TILT=%.2f\n",tilt_angle);*/}
  else if (sscanf(mensaje,"* Minimum Pan position is %d",&kk)==1)
    { min_pan=(float)kk*ENCOD_TO_DEG; 
    printf(" Minimum Pan = %.2f\n",min_pan);
    }
  else if (sscanf(mensaje,"* Maximum Pan position is %d",&kk)==1) 
    { max_pan=(float)kk*ENCOD_TO_DEG; 
    printf(" Maximum Pan = %.2f\n",max_pan);
    }
  else if (sscanf(mensaje,"* Minimum Tilt position is %d",&kk)==1) 
    { min_tilt=(float)kk*ENCOD_TO_DEG; 
    printf(" Minimun Tilt = %.2f\n",min_tilt);
    }
  else if (sscanf(mensaje,"* Maximum Tilt position is %d",&kk)==1) 
    { max_tilt=(float)kk*ENCOD_TO_DEG; 
    printf(" Maximun Tilt = %.2f\n",max_tilt);
    }
}


void *serialpantilt_pollthread(void *not_used) 
{
  struct timeval a,b;
  long diff, next;

  for(;;)
    {
      pthread_mutex_lock(&mymutex[SCH_PANTILTENCODERS]);

      if (state[SCH_PANTILTENCODERS]==slept) 
	  pthread_cond_wait(&condition[SCH_PANTILTENCODERS],&mymutex[SCH_PANTILTENCODERS]);
      else 
	{
	  gettimeofday(&a,NULL);
	  SendCmd("PP\nTP\n");
	  gettimeofday(&b,NULL);  
	  diff = (b.tv_sec-a.tv_sec)*1000000+b.tv_usec-a.tv_usec;
	  
	  next = PANTILTENCODERS_POLLING*1000-diff-10000; 
	  /* discounts 10ms taken by calling usleep itself */
	  if (next>0) usleep(PANTILTENCODERS_POLLING*1000-diff);
	  else 
	    {
	      printf("time interval violated: pantiltencoders\n"); 
	      usleep(PANTILTENCODERS_POLLING*1000);
	    }
	}
      pthread_mutex_unlock(&mymutex[SCH_PANTILTENCODERS]);
    }
}

void *serialpantilt_readthread(void *not_used) 
{ 
  int leidos=0, comienzo=0, j=0;
  
 for(;;)
   {
     usleep(PANTILTENCODERS_POLLING*0.75*1000);
     /* this read should be blocking, but awfully it seems it is not */
     leidos=read(pantilt_serialport,&(pantilt_in[pantilt_marca]),MAX_MESSAGE-pantilt_marca-1);
     /* printf("desbloqueo\n");*/

     switch(leidos)
       {case 0: /* printf(" Se cerro el puerto serie\n"); exit(1);  El puerto serie nunca se puede cerrar */ break;
       case -1: break; /* no hay nada que leer */
       default: 
	 comienzo=0;
	 for(j=pantilt_marca; (j<MAX_MESSAGE)&&(j<pantilt_marca+leidos);j++)
	   if (pantilt_in[j]=='\n') 
	     {
	     serve_serialpantilt_message(&pantilt_in[comienzo]);
	     comienzo=j+1;
	     }
	 
	 if (comienzo==0) 
	   {if ((pantilt_marca+leidos)> MAX_MESSAGE-1) pantilt_marca=0; /* Se desbordo el buffer */
	   else pantilt_marca+=leidos;
	   }
	 else 
	   {strncpy(pantilt_in,&(pantilt_in[comienzo]),pantilt_marca+leidos-comienzo);
	   pantilt_marca=pantilt_marca+leidos-comienzo;
	   comienzo=0;
	   }}
   }
}

 
void *cronos_thread(void *not_used) 
{
  struct timeval tlast,tnow;
  long diff;
 
 /* frame rate computing */   
  for(;;)
    {
      gettimeofday(&tnow,NULL); 
      diff = (tnow.tv_sec-tlast.tv_sec)*1000000+tnow.tv_usec-tlast.tv_usec;
      
      fpsA=kA*1000000/diff; 
      fpsB=kB*1000000/diff;
      fpssonars=ksonars*1000000/(NUM_SONARS*diff);
      fpslaser=klaser*1000000/diff;
      fpsencoders=kencoders*1000000/diff;
      fpspantiltencoders=kpantiltencoders*1000000/diff;
      fpsmotors=kmotors*1000000/diff;
      fpspantiltmotors=kpantiltmotors*1000000/diff;
      kA=0.; 
      kB=0.;
      klaser=0;
      ksonars=0;
      kencoders=0;
      kpantiltencoders=0;
      kmotors=0;
      kpantiltmotors=0;
      tlast=tnow;
      /* discounts 10ms taken by calling usleep itself */
      usleep(FPS_CYCLE*1000-10000);
    }

}

void put_state(int numschema, int newstate)
{
  int old,i;
  static long int oldtrace=0;
  long int newtrace=0;
  old=state[numschema];
  state[numschema]=newstate;

  if (trace)
    {
      /* only some changes are relevant. For instance change of one motor schema from active to ready is not, because it happens every iteration */
      if (
	  (newstate==winner) || 
	  (newstate==slept) || 
	  (newstate==forced)|| 
	  (newstate==active))
	{
	  for(i=0;i<NUM_SCHEMAS;i++)
	    {
	    if (state[i]==slept) newtrace+=0;
	    else if (state[i]==active) newtrace+=1;
	    else if (state[i]==notready) newtrace+=2;
	    else if (state[i]==ready) newtrace+=3;
	    else if (state[i]==forced) newtrace+=4;
	    else if (state[i]==winner) newtrace+=5;
	    newtrace = newtrace*6; 
	    }
	  
	  /* If state to be displayed is the same we showed last time we don't do it once more, to avoid repetitions */
	  if (newtrace!=oldtrace) 
	    {;
	    oldtrace=newtrace;
	    printf("\n");
	    for(i=0;i<NUM_SCHEMAS;i++)
	    {
	      if (i==numschema) printf("*");
	      if ((state[i]==slept)&&(i==numschema)&&(old!=slept)) printf("%s slept\n",name[i]);
	      else if (state[i]==active) printf("%s active\n",name[i]);
	      else if (state[i]==notready) printf("%s notready\n",name[i]);
	      else if (state[i]==ready) printf("%s ready\n",name[i]);
	      else if (state[i]==forced) printf("%s forced\n",name[i]);
	      else if (state[i]==winner) printf("%s winner\n",name[i]);
	    }
	    printf("\n");
	    }
	}
    }
}

void null_arbitration()
{
  printf("NULL arbitration\n");
}



int serve_keyboardmessage(char *mensaje)
{
 char word[256],word2[256];

 /*printf("Command from keyboard: %s\n",mensaje); */
  if (sscanf(mensaje,"%s %s",word,word2)==2) 
    {
      if (strcmp(word,"resume")==0)
	{
	if (strcmp(word2,"guixforms")==0)
	  guixforms_resume();
	}
      else if (strcmp(word,"suspend")==0)
	{
	if (strcmp(word2,"guixforms")==0)
	  guixforms_suspend();
	}
      else if (strcmp(word,"debug")==0)
	{
	if (strcmp(word2,"guixforms")==0)
	  debug[SCH_GUIXFORMS]=TRUE;
	else if (strcmp(word2,"pantiltencoders")==0)
	  debug[SCH_PANTILTENCODERS]=TRUE;
	else if (strcmp(word2,"pantiltmotors")==0)
	  debug[SCH_PANTILTMOTORS]=TRUE;
	}
      else if (strcmp(word,"nodebug")==0)
	{
	if (strcmp(word2,"guixforms")==0)
	  debug[SCH_GUIXFORMS]=FALSE;
	else if (strcmp(word2,"pantiltencoders")==0)
	  debug[SCH_PANTILTENCODERS]=FALSE;
	else if (strcmp(word2,"pantiltmotors")==0)
	  debug[SCH_PANTILTMOTORS]=FALSE;
	}
      return 0;
    }
  else if (sscanf(mensaje,"%s",word)==1) 
    {
      if (strcmp(word,"quit")==0)
	jdeshutdown(0);
      return 0;
    }
  else return -1;
}

int jde_readline(FILE *myfile)
     /* It reads a single line from config file, parses it and do the right thing. Returns EOF in detects end of such file. Otherwise returns 0.*/
     /* To startup non-basic schemas, just raise the flag, putting the in active state. It will effectively start up in main, after the "startup" of all basic schemas */

{
  char word[256],word2[256],word3[256],word4[256];
  int i=0,j=0;
  char buffer_file[256]; 

  buffer_file[0]=fgetc(myfile);
  if (buffer_file[0]==EOF) return EOF;
  if (buffer_file[0]=='#') {while(fgetc(myfile)!='\n'); return 0;}
  if (buffer_file[0]==' ') {while(buffer_file[0]==' ') buffer_file[0]=fgetc(myfile);}

  /* Captura la linea y luego leeremos de la linea con sscanf, comprobando en la linea que el ultimo caracter es \n. No lo podemos hacer directamente con fscanf porque esta funcion no distingue espacio en blanco de \n */
  while(buffer_file[i]!='\n') buffer_file[++i]=fgetc(myfile);
  buffer_file[++i]='\0';
  
  if (sscanf(buffer_file,"%s",word)!=1) return 0; 
  /* return EOF; empty line*/
  else {
    if (strcmp(word,name[SCH_SONARS])==0) {
      while((buffer_file[j]!='\n')&&(buffer_file[j]!=' ')&&(buffer_file[j]!='\0')) j++;
      sscanf(&buffer_file[j],"%s",word);
      if (strcmp(word,"otos")==0)
	{
	  sscanf(&buffer_file[j],"%s %s %s",word3,word,word2);
	  strcpy(otos_host,word);
	  otos_port=atoi(word2);
	  served[otos][SCH_SONARS]=TRUE;
	  source[SCH_SONARS]=otos;

	  printf("otos item for sonars: %s %d\n",otos_host,otos_port);
	}
    }
    else if (strcmp(word,name[SCH_LASER])==0) {
      while((buffer_file[j]!='\n')&&(buffer_file[j]!=' ')&&(buffer_file[j]!='\0')) j++;
      sscanf(&buffer_file[j],"%s",word);
      if (strcmp(word,"otos")==0)
	{
	  sscanf(&buffer_file[j],"%s %s %s",word3,word,word2);
	  strcpy(otos_host,word);
	  otos_port=atoi(word2);
	  served[otos][SCH_LASER]=TRUE;
	  source[SCH_LASER]=otos;
	}
    }
    else if (strcmp(word,name[SCH_ENCODERS])==0) {
      while((buffer_file[j]!='\n')&&(buffer_file[j]!=' ')&&(buffer_file[j]!='\0')) j++;
      sscanf(&buffer_file[j],"%s",word);
      if (strcmp(word,"otos")==0)
	{
	  sscanf(&buffer_file[j],"%s %s %s",word3,word,word2);
	  strcpy(otos_host,word);
	  otos_port=atoi(word2);
	  served[otos][SCH_ENCODERS]=TRUE;
	  source[SCH_ENCODERS]=otos;
	}
    }
    else if (strcmp(word,name[SCH_MOTORS])==0) {
      while((buffer_file[j]!='\n')&&(buffer_file[j]!=' ')&&(buffer_file[j]!='\0')) j++;
      sscanf(&buffer_file[j],"%s",word);
      if (strcmp(word,"otos")==0)
	{
	  sscanf(&buffer_file[j],"%s %s %s",word3,word,word2);
	  strcpy(otos_host,word);
	  otos_port=atoi(word2);
	  served[otos][SCH_MOTORS]=TRUE;
	  source[SCH_MOTORS]=otos;
	}
    }
    else if (strcmp(word,name[SCH_IMAGEA])==0) {
      if (source[SCH_IMAGEA]!=none) printf("Repeated imageA item ignored\n");
      else 
	{
	  while((buffer_file[j]!='\n')&&(buffer_file[j]!=' ')&&(buffer_file[j]!='\0')) j++;
	  sscanf(&buffer_file[j],"%s",word);
	  if (strcmp(word,"oculo")==0)
	    {
	      served[oculo1][SCH_IMAGEA]=TRUE; 
	      source[SCH_IMAGEA]=oculo1;
	      sscanf(&buffer_file[j],"%s %s %s %s",word3,word,word2,word4);
	      strcpy(oculo1_host,word);
	      oculo1_port=atoi(word2);	 
	      imageAdevice=atoi(word4);
	    }
	  else if (strcmp(word,"firewire")==0)
	    {
	      served[firewire][SCH_IMAGEA]=TRUE; 
	      source[SCH_IMAGEA]=firewire;
	      sscanf(&buffer_file[j],"%s %s",word3,word);
	      imageAdevice=atoi(word);
	    }
	  else if (strcmp(word,"video4linux")==0)
	    {
	      sscanf(&buffer_file[j],"%s %s",word,word2);
	      if (served[video4linux1][SCH_IMAGEB]!=TRUE) 
		{served[video4linux1][SCH_IMAGEA]=TRUE;
		source[SCH_IMAGEA]=video4linux1;
#ifdef JDEVIDEO4LINUX
		strcpy(v4l_filename[0],word2);
#endif
		}
	      else if (served[video4linux2][SCH_IMAGEB]!=TRUE) 
		{served[video4linux2][SCH_IMAGEA]=TRUE;
		source[SCH_IMAGEA]=video4linux2;
#ifdef JDEVIDEO4LINUX
		strcpy(v4l_filename[1],word2);
#endif
		}
	    }
	  else if (strcmp(word,"file")==0)
	    {
	      sscanf(&buffer_file[j],"%s %s",word,word2);
	      if (served[file1][SCH_IMAGEB]!=TRUE) 
		{served[file1][SCH_IMAGEA]=TRUE;
		source[SCH_IMAGEA]=file1;
		strcpy(file1_filename,word2);
		}
	      else if (served[file2][SCH_IMAGEB]!=TRUE) 
		{served[file2][SCH_IMAGEA]=TRUE;
		source[SCH_IMAGEA]=file2;
		strcpy(file2_filename,word2);
		}
	    }
	}}
    else if (strcmp(word,name[SCH_IMAGEB])==0) {
      if (source[SCH_IMAGEB]!=none) printf("Repeated imageB item ignored\n");
      else 
	{
	  while((buffer_file[j]!='\n')&&(buffer_file[j]!=' ')&&(buffer_file[j]!='\0')) j++;
	  sscanf(&buffer_file[j],"%s",word);
	  if (strcmp(word,"oculo")==0)
	    {
	      served[oculo2][SCH_IMAGEB]=TRUE; 
	      source[SCH_IMAGEB]=oculo2;
	      sscanf(&buffer_file[j],"%s %s %s %s",word3,word,word2,word4);
	      strcpy(oculo2_host,word);
	      oculo2_port=atoi(word2);	 
	      imageBdevice=atoi(word4);
	    }
	  else if (strcmp(word,"firewire")==0)
	    {
	      served[firewire][SCH_IMAGEB]=TRUE; 
	      source[SCH_IMAGEB]=firewire;
	      sscanf(&buffer_file[j],"%s %s",word3,word);
	      imageBdevice=atoi(word);
	    }
	  else if (strcmp(word,"video4linux")==0)
	    {
	      sscanf(&buffer_file[j],"%s %s",word,word2);
	      if (served[video4linux2][SCH_IMAGEA]!=TRUE) 
		{served[video4linux2][SCH_IMAGEB]=TRUE;
		source[SCH_IMAGEB]=video4linux2;
#ifdef JDEVIDEO4LINUX
		strcpy(v4l_filename[1],word2);
#endif
		}
	      else if (served[video4linux1][SCH_IMAGEB]!=TRUE) 
		{served[video4linux1][SCH_IMAGEA]=TRUE;
		source[SCH_IMAGEA]=video4linux1;
#ifdef JDEVIDEO4LINUX
		strcpy(v4l_filename[0],word2);
#endif
		}
	    }
	  else if (strcmp(word,"file")==0)
	    {
	      sscanf(&buffer_file[j],"%s %s",word,word2);
	      if (served[file2][SCH_IMAGEA]!=TRUE) 
		{served[file2][SCH_IMAGEB]=TRUE;
		source[SCH_IMAGEB]=file2;
		strcpy(file2_filename,word2);
		}
	      else if (served[file1][SCH_IMAGEA]!=TRUE) 
		{served[file1][SCH_IMAGEB]=TRUE;
		source[SCH_IMAGEB]=file1;
		strcpy(file1_filename,word2);
		}
	    }
	}}
    else if (strcmp(word,name[SCH_PANTILTMOTORS])==0) {
      while((buffer_file[j]!='\n')&&(buffer_file[j]!=' ')&&(buffer_file[j]!='\0')) j++;
      sscanf(&buffer_file[j],"%s",word);
      if (strcmp(word,"oculo")==0)
	{
	  sscanf(&buffer_file[j],"%s %s %s",word3,word,word2);
	  strcpy(oculo1_host,word);
	  oculo1_port=atoi(word2);
	  served[oculo1][SCH_PANTILTMOTORS]=TRUE;
	  source[SCH_PANTILTMOTORS]=oculo1;
	}
      else if (strcmp(word,"serial")==0)
	{
	  sscanf(&buffer_file[j],"%s %s",word3,word);
	  strcpy(pantilt_device,word);
	  served[serialpantilt][SCH_PANTILTMOTORS]=TRUE;
	  source[SCH_PANTILTMOTORS]=serialpantilt;
	}
    }
    else if (strcmp(word,name[SCH_PANTILTENCODERS])==0) {
      while((buffer_file[j]!='\n')&&(buffer_file[j]!=' ')&&(buffer_file[j]!='\0')) j++;
      sscanf(&buffer_file[j],"%s",word);
      if (strcmp(word,"oculo")==0)
	{
	  sscanf(&buffer_file[j],"%s %s %s",word3,word,word2);
	  strcpy(oculo1_host,word);
	  oculo1_port=atoi(word2);
	  served[oculo1][SCH_PANTILTENCODERS]=TRUE;
	  source[SCH_PANTILTENCODERS]=oculo1;
	}
      else if (strcmp(word,"serial")==0)
	{
	  sscanf(&buffer_file[j],"%s %s",word3,word);
	  strcpy(pantilt_device,word);
	  served[serialpantilt][SCH_PANTILTENCODERS]=TRUE;
	  source[SCH_PANTILTENCODERS]=serialpantilt;
	}
    }

    else if (strcmp(word,name[SCH_GUIXFORMS])==0) state[SCH_GUIXFORMS]=active;
    else if (strcmp(word,name[SCH_MYSCHEMA])==0) state[SCH_MYSCHEMA]=active;
    else if (strcmp(word,"trace")==0) trace=1;
    else if (strcmp(word,"debug")==0)
      {
	while((buffer_file[j]!='\n')&&(buffer_file[j]!=' ')&&(buffer_file[j]!='\0')) j++;
	sscanf(&buffer_file[j],"%s",word);
	printf("debug %s\n",word);
	if (strcmp(word,name[SCH_SONARS])==0) debug[SCH_SONARS]=1;
	else if (strcmp(word,name[SCH_LASER])==0) debug[SCH_LASER]=1;
	else if (strcmp(word,name[SCH_ENCODERS])==0) debug[SCH_ENCODERS]=1;
	else if (strcmp(word,name[SCH_PANTILTENCODERS])==0) debug[SCH_PANTILTENCODERS]=1;
	else if (strcmp(word,name[SCH_IMAGEA])==0) debug[SCH_IMAGEA]=1;
	else if (strcmp(word,name[SCH_IMAGEB])==0) debug[SCH_IMAGEB]=1;
	else if (strcmp(word,name[SCH_MOTORS])==0) debug[SCH_MOTORS]=1;
	else if (strcmp(word,name[SCH_PANTILTMOTORS])==0) debug[SCH_PANTILTMOTORS]=1;
	else if (strcmp(word,name[SCH_GUIXFORMS])==0) debug[SCH_GUIXFORMS]=1;
	else if (strcmp(word,name[SCH_MYSCHEMA])==0) debug[SCH_MYSCHEMA]=1;
      }
    else printf("I don't know what to do with: %s\n",word); return 0;
  }
}



int jdeinit(void)
  /* init of physical device threads for basic schemas, that is,
     sources for sensor data, and sinks for motor orders */
{
  int i,ii,n,k;
#ifdef JDEFIREWIRE
  int p;
  raw1394handle_t raw_handle;
  struct raw1394_portinfo ports[MAX_PORTS];
  unsigned int channel;
  unsigned int speed;
#endif

  char client_name[MAX_CLIENT_NAME]="basicjde";
  /* Debe enviarse un client_name de cliente aunque en la linea de comandos no se especifique. El servidor otos SIEMPRE espera un client_name de 20 caracteres al inicio de la conexion, y si no se envia, entonces el primer mensaje del cliente se toma como tal client_name */
  struct sockaddr_in server;
  struct hostent *hp; 


  /* otos server */ 
  for(i=0;i<NUM_BASICSCHEMAS;i++)
    {
      if (served[otos][i]==TRUE)
	{
	  /* setup the network connection with otos */  
	  sock_otos = socket(AF_INET, SOCK_STREAM, 0);
	  if (sock_otos < 0) 
	    {perror("opening stream socket");
	    exit(1);
	    }
	  
	  server.sin_family = AF_INET;
	  hp = gethostbyname(otos_host);
	  if (hp == 0)
	    { perror("Unknown host");
	    exit(1);
	    }
	  bcopy((char *)hp ->h_addr, (char *)&server.sin_addr, hp -> h_length); 
	  server.sin_port = htons(otos_port);
	  
	  if (connect(sock_otos, (struct sockaddr *)&server,sizeof server ) >= 0)
	    write(sock_otos,client_name,MAX_CLIENT_NAME);
	  /* Debe enviarse un nombre de cliente aunque en la linea de comandos no se especifique. El servidor SIEMPRE espera un nombre de 10 caracteres al inicio de la conexion, y si no se envia, entonces el primer mensaje del cliente se toma como tal nombre */
	  
	  /* This socket is a blocking one, by default. Uncomment the following if you prefer a non-blocking one. */
	  /*if(fcntl(sock_otos, F_SETFL, O_NONBLOCK) < 0) { printf("fcntl FSETFL, O_NONBLOCK\n");  exit(1);}*/  

	  printf("connecting to otos server in %s:%d ",otos_host,otos_port);  
	  if (sock_otos<0) 
	    {printf("...failed\n");
	    /*exit(-1);*/
	    }
	  else 
	    {
	      printf("...Ok\n");
	      pthread_mutex_init(&otos_mutex,NULL);
	      pthread_create(&otos_thread,NULL,sensationsotos_thread,NULL);  
	      
	      if (served[otos][SCH_SONARS]==TRUE)
		printf("sonars schema started up\n");
	      if (served[otos][SCH_LASER]==TRUE)
		printf("laser schema started up\n");
	      if (served[otos][SCH_ENCODERS]==TRUE)
		printf("encoders schema started up\n");
	      if (served[otos][SCH_MOTORS]==TRUE)
		{
		  pthread_mutex_lock(&mymutex[SCH_MOTORS]);
		  state[SCH_MOTORS]=slept;
		  printf("motors schema started up\n");
		  pthread_create(&schema[SCH_MOTORS],NULL,motors_thread,NULL);
		  pthread_mutex_unlock(&mymutex[SCH_MOTORS]);
		}
	    }
	  break;
	}
    }
  
  /* oculo1 */
  for(i=0;i<NUM_BASICSCHEMAS;i++)
    {
      if (served[oculo1][i]==TRUE)
	{
	  /* setup the network connection with oculo */  
	  sock_oculo1 = socket(AF_INET, SOCK_STREAM, 0);
	  if (sock_oculo1 < 0) 
	    {perror("opening stream socket");
	    exit(1);
	    }
	  
	  server.sin_family = AF_INET;
	  hp = gethostbyname(oculo1_host);
	  if (hp == 0)
	    { perror("Unknown host");
	    exit(1);
	    }
	  bcopy((char *)hp ->h_addr, (char *)&server.sin_addr, hp -> h_length); 
	  server.sin_port = htons(oculo1_port);
	  
	  if (connect(sock_oculo1, (struct sockaddr *)&server,sizeof server ) >= 0)
	    write(sock_oculo1,client_name,MAX_CLIENT_NAME);
	  /* Debe enviarse un nombre de cliente aunque en la linea de comandos no se especifique. El servidor SIEMPRE espera un nombre de 10 caracteres al inicio de la conexion, y si no se envia, entonces el primer mensaje del cliente se toma como tal nombre */
	  
	  /* This socket is a blocking one, by default. Uncomment the following if you prefer a non-blocking one. */
	  /*if(fcntl(sock_otos, F_SETFL, O_NONBLOCK) < 0) { printf("fcntl FSETFL, O_NONBLOCK\n");  exit(1);}*/
	  printf("connecting to oculo server in %s:%d ",oculo1_host,oculo1_port);
	  if (sock_oculo1<0) 
	    {printf("...failed\n");
	    /*exit(-1);*/
	    }
	  else {
	    printf("...Ok\n");
	    pthread_mutex_init(&oculo1_mutex,NULL);
	    pthread_create(&oculo1_thread,NULL,sensationsoculo1_thread,NULL);

	    if (served[oculo1][SCH_IMAGEA]==TRUE)
	      printf("imageA schema started up\n");
	    if (served[oculo1][SCH_IMAGEB]==TRUE)
	      printf("imageB schema started up\n");
	    if (served[oculo1][SCH_PANTILTENCODERS]==TRUE)
	      printf("pantiltencoders schema started up\n");
	    if (served[oculo1][SCH_PANTILTMOTORS]==TRUE)
	     { printf("pantiltmotors schema started up\n");
	     pthread_mutex_lock(&mymutex[SCH_PANTILTMOTORS]);
	     state[SCH_PANTILTMOTORS]=slept;
	     pthread_create(&schema[SCH_PANTILTMOTORS],NULL,pantiltmotors_thread,NULL);
	     pthread_mutex_unlock(&mymutex[SCH_PANTILTMOTORS]);
	     }
	  }
	  break;
	}
    }


  /* oculo2 */
  for(i=0;i<NUM_BASICSCHEMAS;i++)
    {
      if (served[oculo2][i]==TRUE)
	{
	  /* setup the network connection with oculo2 */  
	  sock_oculo2 = socket(AF_INET, SOCK_STREAM, 0);
	  if (sock_oculo2 < 0) 
	    {perror("opening stream socket");
	    exit(1);
	    }
	  
	  server.sin_family = AF_INET;
	  hp = gethostbyname(oculo2_host);
	  if (hp == 0)
	    { perror("Unknown host");
	    exit(1);
	    }
	  bcopy((char *)hp ->h_addr, (char *)&server.sin_addr, hp -> h_length); 
	  server.sin_port = htons(oculo2_port);
	  
	  if (connect(sock_oculo2, (struct sockaddr *)&server,sizeof server ) >= 0)
	    write(sock_oculo2,client_name,MAX_CLIENT_NAME);
	  /* Debe enviarse un nombre de cliente aunque en la linea de comandos no se especifique. El servidor SIEMPRE espera un nombre de 10 caracteres al inicio de la conexion, y si no se envia, entonces el primer mensaje del cliente se toma como tal nombre */
	  
	  /* This socket is a blocking one, by default. Uncomment the following if you prefer a non-blocking one. */
	  /*if(fcntl(sock_otos, F_SETFL, O_NONBLOCK) < 0) { printf("fcntl FSETFL, O_NONBLOCK\n");  exit(1);}*/
	  printf("connecting to oculo server in %s:%d ",oculo2_host,oculo2_port);
	  if (sock_oculo2<0) 
	    {printf("...failed\n");
	    exit(-1);
	    }
	  else {
	    printf("...Ok\n");
	    pthread_mutex_init(&oculo2_mutex,NULL);
	    pthread_create(&oculo2_thread,NULL,sensationsoculo2_thread,NULL);

	    if (served[oculo2][SCH_IMAGEA]==TRUE)
	      printf("imageA schema started up\n");
	    if (served[oculo2][SCH_IMAGEB]==TRUE)
	      printf("imageB schema started up\n");
	    if (served[oculo2][SCH_PANTILTENCODERS]==TRUE)
	      printf("pantiltencoders schema started up\n");
	    if (served[oculo2][SCH_PANTILTMOTORS]==TRUE)
	       { printf("pantiltmotors schema started up\n");
	       pthread_mutex_lock(&mymutex[SCH_PANTILTMOTORS]);
	       state[SCH_PANTILTMOTORS]=slept;
	       pthread_create(&schema[SCH_PANTILTMOTORS],NULL,pantiltmotors_thread,NULL);
	       pthread_mutex_unlock(&mymutex[SCH_PANTILTMOTORS]);
	     }
	  }
	  break;
	}
    }


  /* video4linux1 and video4linux2 */
  for(ii=0;ii<NUM_BASICSCHEMAS;ii++)
    {
      if (source[ii]==video4linux1) n=0;
      else if (source[ii]==video4linux2) n=1;

      if ((source[ii]==video4linux1)||(source[ii]==video4linux2))
	{
#ifdef JDEVIDEO4LINUX
	  /* Apertura e inicializacion del dispositivo de videoforlinux */
	  fdv4l[n] = open(v4l_filename[n], O_RDWR);
	  if (fdv4l[n] < 0) {perror(v4l_filename[n]); return -1;}
	  
	  if (ioctl(fdv4l[n], VIDIOCGCAP, &cap[n]) < 0) 
	    {perror("VIDIOGCAP"); return -1;}
	  else 
	    {
	      if (debug[ii]) printf("CAPABILITY: \n name=%s \n type=%d \n channels=%d \n audios=%d \n maxwidth=%d \n maxheight=%d \n minwidth=%d \n minheight=%d \n\n", cap[n].name, cap[n].type, cap[n].channels, cap[n].audios, cap[n].maxwidth, cap[n].maxheight, cap[n].minwidth, cap[n].minheight);
	      printf("%s detected\n",cap[n].name);
	    }
	  
	  /*  fcntl(fdv4l[n],F_SETFD,FD_CLOEXEC); */
	  for(i=0; i<cap[n].channels; i++)
	    {
	      chan[n].channel=(int) i;
	      if (ioctl(fdv4l[n], VIDIOCGCHAN, &chan[n]) == -1) 
		{
		  perror("VIDIOCGCHAN");
		  close(fdv4l[n]);
		  return -1;
		}
	      else if (debug[ii]) printf(" CHANNEL %d: \n name=%s \n tuners=%d \n type=%d \n\n",chan[n].channel,chan[n].name,chan[n].tuners,chan[n].type);
	    }
	  
	  if ((strcmp(cap[n].name,"BT878(Hauppauge new)")==0) && (cap[n].channels == 3)) 
	    {
	      /* Selecciona el canal composite-video (=1) como canal activo. Si pongo canal Television (=0) tambien funciona. Si pongo s-video (=2) se ve, pero en niveles de gris nada mas */
	      chan[n].channel=1;
	      if (ioctl(fdv4l[n], VIDIOCSCHAN, &chan[n]) == -1) 
		{ perror("VIDIOCSCHAN"); return(-1);}
	    }
	  
	  /*image size*/
	  win[n].width=SIFNTSC_COLUMNS;
	  win[n].height=SIFNTSC_ROWS;
	  win[n].x=0;
	  win[n].y=0; 
	  win[n].chromakey=0;
	  win[n].clipcount=0; 
	  
	  if ( (strcmp(cap[n].name,"Philips 740 webcam")==0) || (strcmp(cap[n].name,"Philips 730 webcam")==0) )
	    
#ifdef PWCX
	    /* the driver with compression (pwcx) allows higher frame rates */
	    win[n].flags = 30 << 16; 
#else
	  /* Philips 740 webcam need to configure framerate at 5 to deliver 320x240 images. For higher speeds (including default ones) the maximum image size is 160x120. This doesn't affect to other v4l devices, as long as they don't use bits 16..22 of flags field */
	  /* withour pwcx, requiring 30 fps causes small image sizes */
	  win[n].flags = 5 << 16;
#endif 
	  else win[n].flags =0;
	  
	  if (ioctl(fdv4l[n], VIDIOCSWIN, &win[n]) < 0) perror("VIDIOCSWIN");
	  if (ioctl(fdv4l[n], VIDIOCGWIN, &win[n]) < 0) {perror("VIDIOCGWIN");return -1;}
	  else
	    {
	      if (debug[ii]) printf("WINDOW \n x=%d y=%d \n width=%d, height=%d \n chromakey=%d \n flags=%d \n clipcount=%d \n\n", win[n].x, win[n].y, win[n].width, win[n].height, win[n].chromakey, win[n].flags, win[n].clipcount);
	    }
	  printf("video4linux: capture window = %d x %d \n",win[n].width,win[n].height);
	  
	  
	  /* Caracteristicas de la imagen, incluyendo formato de pixels!!. Se solicitan al driver y este nos dice que si puede darnoslo o no */
	  if (ioctl(fdv4l[n], VIDIOCGPICT, &vpic[n]) < 0) {perror("VIDIOCGPICT"); return -1;}
	  
	  /*
	    vpic[n].depth=8;
	    vpic[n].palette=VIDEO_PALETTE_YUV420P;
	    if(ioctl(fdv4l[n], VIDIOCSPICT, &vpic[n]) < 0) 
	    {perror("v4l: unable to find a supported capture format"); return -1;}
	  */
	  
	  vpic[n].depth=8;
	  vpic[n].palette=VIDEO_PALETTE_YUV420P;
	  if(ioctl(fdv4l[n], VIDIOCSPICT, &vpic[n])<0) 
	    {
	      vpic[n].depth=16;
	      vpic[n].palette=VIDEO_PALETTE_RGB565;
	      if(ioctl(fdv4l[n], VIDIOCSPICT, &vpic[n]) < 0) 
		{
		  vpic[n].depth=24;
		  vpic[n].palette=VIDEO_PALETTE_RGB24;
		  if(ioctl(fdv4l[n], VIDIOCSPICT, &vpic[n]) < 0) 
		    {perror("v4l: unable to find a supported capture format"); return -1;}
		}      
	    }
	  
	  if (debug[ii]) printf("VIDEO_PICTURE \n brightness=%d \n hue=%d \n colour=%d \n contrast=%d \n whiteness=%d \n depth=%d \n palette=%d \n\n",vpic[n].brightness,vpic[n].hue,vpic[n].colour,vpic[n].contrast,vpic[n].whiteness,vpic[n].depth,vpic[n].palette);
	  printf("video4linux: capture palette %d\n",vpic[n].palette);
	  
	  gb_buffers[n].frames=2;
	  if (cap[n].type & VID_TYPE_CAPTURE) 
	    {
	      /* map grab buffer */
	      if (-1 == ioctl(fdv4l[n],VIDIOCGMBUF,&gb_buffers[n])) 
		{perror("ioctl VIDIOCGMBUF"); return -1;}
	      
	      map[n] = mmap(0,gb_buffers[n].size,PROT_READ|PROT_WRITE,MAP_SHARED,fdv4l[n],0);
	      if ((char*)-1 != map[n]) 
		printf("Memory mapped: %d Bytes for %d frames\n",gb_buffers[n].size,gb_buffers[n].frames);
	      else 
		{ perror("mmap"); return -1;}
	    }
	  gb[n].frame=0;
	  gb[n].width=win[n].width;
	  gb[n].height=win[n].height;
	  gb[n].format=vpic[n].palette;
	  
	  /* init Lookup tables for color conversion */
	  for (k = 0; k < 256; k++) {
	    ng_yuv_gray[k] = k * LUN_MUL >> 8;
	    ng_yuv_red[k]  = (RED_ADD    + k * RED_MUL)    >> 8;
	    ng_yuv_blue[k] = (BLUE_ADD   + k * BLUE_MUL)   >> 8;
	    ng_yuv_g1[k]   = (GREEN1_ADD + k * GREEN1_MUL) >> 8;
	    ng_yuv_g2[k]   = (GREEN2_ADD + k * GREEN2_MUL) >> 8;
	  }
	  for (k = 0; k < CLIP; k++)
	    ng_clip[k] = 0;
	  for (; k < CLIP + 256; k++)
	    ng_clip[k] = k - CLIP;
	  for (; k < 2 * CLIP + 256; k++)
	    ng_clip[k] = 255;

	  
	  if (source[ii]==video4linux1) pthread_create(&v4l_thread[0],NULL,video4linux1_thread,NULL);
	  else if (source[ii]==video4linux2) pthread_create(&v4l_thread[1],NULL,video4linux2_thread,NULL);
#else
	  printf("This jde was compiled WITHOUT video4linux support\n");
#endif
	}
    }


  /* file1 and file2 don't require any thread */
  /* firewire */
  for(ii=0;ii<NUM_BASICSCHEMAS;ii++)
    {
      if (source[ii]==firewire) 
#ifdef JDEFIREWIRE
	{
	  firewire_res = MODE_320x240_YUV422;
	  firewire_fps = FRAMERATE_30;

	  /* get the number of ports (cards) */
	  raw_handle = raw1394_new_handle();
	  if (raw_handle==NULL) {
	    perror("Unable to aquire a raw1394 handle\n");
	    perror("did you load the drivers?\n");
	    exit(-1);
	  }
	  
	  numPorts = raw1394_get_port_info(raw_handle, ports, numPorts);
	  raw1394_destroy_handle(raw_handle);
	  printf("number of ports = %d\n", numPorts);
	  
	  /* get dc1394 handle to each port */
	  for (p = 0; p < numPorts; p++)
	    {
	      int camCount;
	      
	      handles[p] = dc1394_create_handle(p);
	      if (handles[p]==NULL) {
		perror("Unable to aquire a raw1394 handle\n");
		perror("did you load the drivers?\n");
		firewirecleanup();
		exit(-1);
	      }
	      
	      /* get the camera nodes and describe them as we find them */
	      camera_nodes = dc1394_get_camera_nodes(handles[p], &camCount, 1);
	      
	      /* setup cameras for capture */
	      for (i = 0; i < camCount; i++)
		{	
		  cameras[numCameras].node = camera_nodes[i];
		  
		  if(dc1394_get_camera_feature_set(handles[p], cameras[numCameras].node, &features) !=DC1394_SUCCESS) 
		    {
		      printf("unable to get feature set\n");
		    } else {
		      dc1394_print_feature_set(&features);
		    }
		  
		  if (dc1394_get_iso_channel_and_speed(handles[p], cameras[numCameras].node,
						       &channel, &speed) != DC1394_SUCCESS) 
		    {
		      printf("unable to get the iso channel number\n");
		      firewirecleanup();
		      exit(-1);
		    }
		  
		  if (dc1394_dma_setup_capture(handles[p], cameras[numCameras].node, i+1 /*channel*/,
					       FORMAT_VGA_NONCOMPRESSED, firewire_res,
					       SPEED_400, firewire_fps, NUM_BUFFERS, DROP_FRAMES,
					       device_name, &cameras[numCameras]) != DC1394_SUCCESS) 
		    {
		      fprintf(stderr, "unable to setup camera- check line %d of %s to make sure\n",
			      __LINE__,__FILE__);
		      perror("that the video mode,framerate and format are supported\n");
		      printf("is one supported by your camera\n");
		      firewirecleanup();
		      exit(-1);
		    }
		  
		  
		  /*have the camera start sending us data*/
		  if (dc1394_start_iso_transmission(handles[p], cameras[numCameras].node) !=DC1394_SUCCESS) 
		    {
		      perror("unable to start camera iso transmission\n");
		      firewirecleanup();
		      exit(-1);
		    }
		  numCameras++;
		}
	    }
	  
	  fflush(stdout);
	  if (numCameras < 1) {
	    perror("no cameras found :(\n");
	    firewirecleanup();
	    exit(-1);
	  }
	  pthread_create(&firewire_th,NULL,firewire_thread,NULL);  
	}
#else
	printf("This jde was compiled WITHOUT firewire support\n");
#endif
    }

  /* serial port*/ 
  for(ii=0;ii<NUM_BASICSCHEMAS;ii++)
    {
      if (source[ii]==serialpantilt) 
 	{
	  printf("pantilt device on %s serial port\n",pantilt_device); 
	  if ((pantilt_serialport = openRaw(pantilt_device, O_RDWR)) <= 0) 
	    {perror("Cannot open serial port");return -1;}
	  if (!setBaudRate(pantilt_serialport,(speed_t) RS232_BAUD_RATE))
	    {perror("Unable to set baud rate of the serial port");return -1;}

	  /* lo pone no bloqueante 
	     if(fcntl(pantilt_serialport, F_SETFL, O_NONBLOCK) < 0) 
	     { printf("fcntl FSETFL, O_NONBLOCK\n");  exit(1);}
	  */

	  pthread_mutex_lock(&serialpantilt_mutex);
	  /*  Remove whatever commands might be buffered.  */
	  SendCmd("\xFF\xFF\n");
	  /*  Make sure we start in immediate mode.  */
	  SendCmd("I\n");
	  SendCmd("CI\n");
	  /*  Set base speeds.  */
	  SendCmd("PB1000\n");
	  SendCmd("TB1000\n");
	  /*  Set upper and lower speed limits  */
	  SendCmd("PU6000\n");
	  SendCmd("TU6000\n");
	  SendCmd("PL31\n");
	  SendCmd("TL31\n");
	  /*  Set target speeds and accelarations */
	  SendCmd("PS2900\n");
	  SendCmd("TS2900\n");
	  latitude_speed=2900*ENCOD_TO_DEG;
	  longitude_speed=2900*ENCOD_TO_DEG;
	  SendCmd("PA9000\n"); /* 2000 */
	  SendCmd("TA9000\n"); /* 2000 */
 	  /*  Set move and hold powers to low setting.  */
	  SendCmd("PHL\n");
	  SendCmd("PML\n");
	  SendCmd("THL\n");
	  SendCmd("TML\n");
	  /*  Move to center.  */
	  SendCmd("PP0\n");
	  SendCmd("TP0\n");
	  pan_angle=0.;
	  tilt_angle=0.;
	  /*  Enforce soft limits.  */
	  SendCmd("LE\n");
	  /* para leer los limites de posicion de la unidad*/
	  SendCmd("PN\nPX\nTN\nTX\n"); 
	
	  pthread_mutex_unlock(&serialpantilt_mutex);

	  if (served[serialpantilt][SCH_PANTILTENCODERS]==TRUE)
	    {
	      printf("pantiltencoders schema started up\n");
	      pthread_create(&serialpantilt_readth,NULL,serialpantilt_readthread,NULL);
	      /*usleep(100000);*/
	      pthread_mutex_lock(&mymutex[SCH_PANTILTENCODERS]);
	      state[SCH_PANTILTENCODERS]=slept;
	      pthread_create(&serialpantilt_pollth,NULL,serialpantilt_pollthread,NULL);
	      pthread_mutex_unlock(&mymutex[SCH_PANTILTENCODERS]);

	    }
	  if (served[serialpantilt][SCH_PANTILTMOTORS]==TRUE)
	    {
	      printf("pantiltmotors schema started up\n");
	      pthread_mutex_lock(&mymutex[SCH_PANTILTMOTORS]);
	      state[SCH_PANTILTMOTORS]=slept;
	      pthread_create(&schema[SCH_PANTILTMOTORS],NULL,pantiltmotors_thread,NULL);
	      pthread_mutex_unlock(&mymutex[SCH_PANTILTMOTORS]);
	    }
	  break;
	}
    }

  return 0;
}


void jdeshutdown(int sig)
{
  int i;
  guixforms_shutdown();

  /*motors_shutdown*/
  if (source[SCH_MOTORS]==otos)
    {
      /*pthread_mutex_lock(&otos_mutex);*/
      sprintf(otos_out,"%d %1.1f %1.1f\n",OTOS_drive_speed,0.,0.);
      write(sock_otos,otos_out,strlen(otos_out));
      sprintf(otos_out,"%d %1.1f %1.1f\n",OTOS_steer_speed,0.,0.);
      write(sock_otos,otos_out,strlen(otos_out));
      sprintf(otos_out,"%d\n",OTOS_goodbye); 
      write(sock_otos,otos_out,strlen(otos_out));
      close(sock_otos); 
      /*pthread_mutex_unlock(&otos_mutex);*/
    }

  /*pantiltmotors shutdown*/
  if (source[SCH_PANTILTMOTORS]==oculo1)
    {
      /* pthread_mutex_lock(&oculo1_mutex);*/
      sprintf(oculo1_out,"%ld %1.1f %1.1f\n",(long int)OCULO_pantilt_position,0.,0.);
      write(sock_oculo1,oculo1_out,strlen(oculo1_out));
      /* pthread_mutex_unlock(&oculo1_mutex);*/
    }
  else if (source[SCH_PANTILTMOTORS]==oculo2)
    {
      /* pthread_mutex_lock(&oculo2_mutex);*/
      sprintf(oculo2_out,"%ld %1.1f %1.1f\n",(long int)OCULO_pantilt_position,0.,0.);
      write(sock_oculo2,oculo2_out,strlen(oculo1_out));
      /* pthread_mutex_unlock(&oculo2_mutex);*/
    }

  for(i=0;i<NUM_BASICSCHEMAS;i++)
    {
      if (served[otos][i]==TRUE)
	{
	  sprintf(otos_out,"%d\n",OTOS_goodbye); 
	  write(sock_otos,otos_out,strlen(otos_out));
	  close(sock_otos); 
	  printf("Closed connection with otos at %s:%d\n",otos_host,otos_port);
	  break;
	}
    }


  for(i=0;i<NUM_BASICSCHEMAS;i++)
    {
      if (served[oculo1][i]==TRUE)
	{
	  sprintf(oculo1_out,"%d\n",OCULO_goodbye); 
	  write(sock_oculo1,oculo1_out,strlen(oculo1_out));
	  close(sock_oculo1); 
	  printf("Closed connection with oculo at %s:%d\n",oculo1_host,oculo1_port);
	  break;
	}}

  for(i=0;i<NUM_BASICSCHEMAS;i++)
    {
      if (served[oculo2][i]==TRUE)
	{
	  sprintf(oculo2_out,"%d\n",OCULO_goodbye); 
	  write(sock_oculo2,oculo2_out,strlen(oculo2_out));
	  close(sock_oculo2); 
	  printf("Closed connection with oculo at %s:%d\n",oculo2_host,oculo2_port);
	  break;
	}}
#ifdef JDEFIREWIRE
 for(i=0;i<NUM_BASICSCHEMAS;i++)
    {
      if (served[firewire][i]==TRUE)
	{
	  firewirecleanup();
	  break;
	}}
#endif
#ifdef JDEVIDEO4LINUX
 for(i=0;i<NUM_BASICSCHEMAS;i++)
    {
      if (served[video4linux1][i]==TRUE)
	{close(fdv4l[0]);
	  break;
	}}

 for(i=0;i<NUM_BASICSCHEMAS;i++)
    {
      if (served[video4linux2][i]==TRUE)
	{close(fdv4l[0]);
	break;
	}}
#endif


 /* closing the serialpantilt_port */
 for(i=0;i<NUM_BASICSCHEMAS;i++)
   {
     if (source[i]==serialpantilt) 
       {
	 pthread_mutex_lock(&serialpantilt_mutex);
	 close(pantilt_serialport);
	 pthread_mutex_unlock(&serialpantilt_mutex);
	 break;
       }
   }

  exit(0);
}



int main(int argc, char** argv)
{
  int i,j,marca,leidos,last=0;
  char buff[MAX_MESSAGE];
  FILE *config;
  char configfile[MAX_MESSAGE]; 
 
  
  signal(SIGTERM, &jdeshutdown); /* kill interrupt handler */
  signal(SIGINT, &jdeshutdown); /* control-C interrupt handler */
  signal(SIGABRT, &jdeshutdown); /* failed assert handler */
  signal(SIGPIPE, SIG_IGN);

 /* Pablo Barrera: Por alguna razón libforms hace que fprintf("%f") ponga 
    una coma en vez de un punto como separador si las locales son españolas. 
     Con esto las ponemos a POSIX. Al salir en entorno es el normal */
  /*unsetenv("LANG");*/
  setenv("LC_ALL","POSIX",1);

  printf("%s\n",thisrelease);



  if (argv[1]==NULL) { /* search in default locations */
    if ((config=fopen("./jde.conf","r"))==NULL){
      sprintf(configfile,"%s%s",CONFIGDIR,"/jde.conf");
      if ((config=fopen(configfile,"r"))==NULL){
       printf("Can not find any config file\n");
       exit(-1);
      }
      else printf("Configuration from %s\n",configfile);
    }
    else printf("Configuration from ./jde.conf\n");
  }
  else {
    if ((strcmp(argv[1],"--help")==0) ||
	(strcmp(argv[1],"-h")==0))
      {printf("Use: jde [config.file]\n");
      exit(0);}
    
    else if ((strcmp(argv[1],"--version")==0) ||
	     (strcmp(argv[1],"-v")==0))
      {printf("%s\n",thisrelease);
      exit(0);}
    
    else {
      if ((config=fopen(argv[1],"r"))==NULL){
	printf("Can not open config file: %s\n",argv[1]);
	exit(-1);
      }
      else printf("Configuration from %s\n",argv[1]);
    }
  }
 
  for(i=0; i<NUM_SCHEMAS; i++)
    { debug[i]=0;
    state[i]=slept;
    pthread_mutex_init(&mymutex[i],NULL);
    pthread_cond_init(&condition[i],NULL);
    }
 

/*  Posicion y orientacion de los sensores con respecto al centro del eje trasero del robot. Dado un sistema de coordenadas con la X en la direccion de movimiento del robot, los angulos se miden considerando que el eje X toma valor 0 y siendo positivos cuando se gira en sentido contrario al de movimiento de las agujas del reloj. Se utiliza para cambiar las distancias sensoriales al sistema de referencia local, solidario con el robot-enclosure. la rellena con milimetros y grados.   */ 
  us_coord[0][0]=115.; us_coord[0][1]=130.; us_coord[0][2]=90.; 
  us_coord[1][0]=155.; us_coord[1][1]=115.; us_coord[1][2]=50.; 
  us_coord[2][0]=190.; us_coord[2][1]=80.; us_coord[2][2]=30.; 
  us_coord[3][0]=210.; us_coord[3][1]=25.; us_coord[3][2]=10.; 
  us_coord[4][0]=210.; us_coord[4][1]=-25.; us_coord[4][2]=350.; 
  us_coord[5][0]=190.; us_coord[5][1]=-80.; us_coord[5][2]=330.; 
  us_coord[6][0]=155.; us_coord[6][1]=-115.; us_coord[6][2]=310;
  us_coord[7][0]=115.; us_coord[7][1]=-130.; us_coord[7][2]=270.; 
  us_coord[8][0]=-115.; us_coord[8][1]=-130.; us_coord[8][2]=270.; 
  us_coord[9][0]=-155.; us_coord[9][1]=-115.; us_coord[9][2]=230.; 
  us_coord[10][0]=-190.; us_coord[10][1]=-80.; us_coord[10][2]=210.; 
  us_coord[11][0]=-210.; us_coord[11][1]=-25.; us_coord[11][2]=190.; 
  us_coord[12][0]=-210.; us_coord[12][1]=25.; us_coord[12][2]=170.; 
  us_coord[13][0]=-190.; us_coord[13][1]=80.; us_coord[13][2]=150.;  
  us_coord[14][0]=-155.; us_coord[14][1]=115.; us_coord[14][2]=130.; 
  us_coord[15][0]=-115.; us_coord[15][1]=130.; us_coord[15][2]=90.; 

  for(j=0;j<NUM_SONARS;j++)
    { 
      us_coord[j][3]= cos(us_coord[j][2]*DEGTORAD);
      us_coord[j][4]= sin(us_coord[j][2]*DEGTORAD);
    }

  laser_coord[0]=19.; laser_coord[1]=0.; laser_coord[2]=0.; 
  laser_coord[3]=cos(laser_coord[2]*DEGTORAD);
  laser_coord[4]=sin(laser_coord[2]*DEGTORAD);

  camera_coord[0]=190.; 
  camera_coord[1]=0.; 
  camera_coord[2]=0.; 
  camera_coord[3]= cos(camera_coord[2]*DEGTORAD);
  camera_coord[4]= sin(camera_coord[2]*DEGTORAD);


  /* initial values for sensor variables*/
  colorA=colorAA;
  colorB=colorBB;
  greyA=greyAA;
  greyB=greyBB;
  for(i=0; i<(SIFNTSC_COLUMNS*SIFNTSC_ROWS);i++)
    {colorA[i*3]=(char) 200; /* blue */ 
    colorA[i*3+1]=0; /* green */
    colorA[i*3+2]=0; /* red */
    colorB[i*3]= 0;/* blue */ 
    colorB[i*3+1]=0; /* green */
    colorB[i*3+2]=(char)(((i%SIFNTSC_COLUMNS)%30)*255/30);/* red */
    greyA[i]=(char)(((i%SIFNTSC_COLUMNS)%30)*255/30);
    greyB[i]=(char)(((i/SIFNTSC_COLUMNS)%30)*255/30);
    }
  v=0.; w=0;


  /* read the configuration file and so does the allocation: basicschemas-sources */
  for(j=0;j<NUM_SOURCES;j++)
    for(i=0;i<NUM_BASICSCHEMAS;i++)
      served[j][i]=FALSE;
  for(i=0;i<NUM_BASICSCHEMAS;i++)
    source[i]=none;
  do {
    i=jde_readline(config);
  }while(i!=EOF);
 
 
  /* init of physical device threads for basic schemas, that is,
     sources for sensor data, and sinks for motor orders */
  jdeinit();

  /* starting up the non-basic schemas. When the configuration file was parsed, the state of corresponding schemas were set to active */
  if (state[SCH_MYSCHEMA]==active) 
    {state[SCH_MYSCHEMA]=slept;
    myschema_startup();
    }
    
  if (state[SCH_GUIXFORMS]==active) 
    { guixforms_startup(); 
    guixforms_cycle=100;
    }
    
  /* start the cronos thread */
  pthread_create(&cronos_th,NULL,cronos_thread,NULL);  
	
  /* blocking read commands from keyboard */
  for(;;)
    {
      /* leo hasta agotar todo lo que hay en ese descriptor */
      last=0;marca=0;
      do
	{
	  leidos=read(0,&(buff[marca]),1);
	  /*printf("desbloqueo 2\n");*/
	  if (leidos>0) marca+=leidos;
	  if (marca>0) last=marca-1;
	} 
      while((buff[last]!='\n')||(leidos<=0));
      buff[last]='\0';
      serve_keyboardmessage(buff);
    }
       
  pthread_exit(0); 
  /* If we don't need this thread anymore, but want the others to keep running */

}



