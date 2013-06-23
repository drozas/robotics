#include "jde.h"
#include "guixformsf.h"
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xos.h>
#include <X11/Xatom.h>
#ifndef _icon
#define _icon
#include <X11/bitmaps/icon>
#endif

int guixforms_cycle=200; /* ms */
#define FORCED_REFRESH 5000 /* ms */ 
/*Every forced_refresh the display is drawn from scratch. If it is too small it will cause flickering with grid display. No merece la pena una hebra de "display_lento" solo para repintar completamente la pantalla. */


#define joystick_maxRotVel 30 /* deg/sec */
#define joystick_maxTranVel 200 /* mm/sec */

float joystick_x, joystick_y;
float pt_joystick_x, pt_joystick_y;
float mouse_x, mouse_y;
int mouse_new=0;
int back=0;

#define PUSHED 1
#define RELEASED 0

int pantiltencoders_cb=0;
int laser_cb=0;
int sonars_cb=0;
int encoders_cb=0;
int imageA_cb=0;
int imageB_cb=0;

/* Necesarias para las Xlib */
static Display* display;
static GC image_gc;
static int screen;
static Window image_win; /* image window */
static int vmode;
static XImage *imagenA,*imagenB; 
static char *imagenA_buf, *imagenB_buf; /* puntero a memoria para la imagen a visualizar en el servidor X. No compartida con el servidor */
long int tabla[256]; 
/* tabla con la traduccion de niveles de gris a numero de pixel en Pseudocolor-8bpp. Cada numero de pixel apunta al valor adecuado del ColorMap, con el color adecuado instalado */
int pixel8bpp_rojo, pixel8bpp_blanco, pixel8bpp_amarillo;

FD_guixformsf *fd_guixforms;
GC      canvas_gc;
Window  canvas_win; /* canvas window */
float   escala, width, height;


float odometrico[5];


unsigned long display_state;
int visual_refresh=FALSE;
int track_robot=FALSE;
int iteracion_display=0;

#define EGOMAX NUM_SONARS+5
XPoint ego[EGOMAX];
float last_heading; /* ultima orientacion visualizada del robot */
int numego=0;
int visual_delete_ego=FALSE;

XPoint laser_dpy[NUM_LASER];
int visual_delete_laser=FALSE;

XPoint us_dpy[NUM_SONARS*2];
int visual_delete_us=FALSE;

#define RANGO_MAX 20000. /* en mm */
#define RANGO_MIN 500. /* en mm */ 
#define RANGO_INICIAL 4000. /* en mm */
float rango=RANGO_INICIAL; /* Rango de visualizacion en milimetros */

char fpstext[80]="";


const char *rangoenmetros(FL_OBJECT *ob, double value, int prec)
{
static char buf[32];

sprintf(buf,"%.1f",value/1000.);
return buf;
}

int xy2canvas(Tvoxel point, XPoint* grafico)
     /* return -1 if point falls outside the canvas */
{
float xi, yi;

xi = (point.x * odometrico[3] - point.y * odometrico[4] + odometrico[0])*escala;
yi = (point.x * odometrico[4] + point.y * odometrico[3] + odometrico[1])*escala;
/* Con esto cambiamos al sistema de referencia de visualizacion, centrado en algun punto xy y con alguna orientacion definidos por odometrico. Ahora cambiamos de ese sistema al del display, donde siempre hay un desplazamiento a la esquina sup. izda. y que las y se cuentan para abajo. */

grafico->x = xi + width/2;
grafico->y = -yi + height/2;

 if ((grafico->x <0)||(grafico->x>width)) return -1; 
 if ((grafico->y <0)||(grafico->y>height)) return -1; 
 return 0;
}

/* ************************************/
/* FUNCION PARA PINTAR LA TRAYECTORIA	*/
/**************************************/

void pinta_trayecto_negro(float xa,float ya,float xb,float yb)
{
	float x1_canvas,x2_canvas;
  float y1_canvas,y2_canvas;
	/*printf("Pintamos negro[%f,%f] - [%f,%f]\n",xa,ya,xb,yb);*/

/* Transformamos puntos a canvas */
	x1_canvas = (xa * odometrico[3] - ya * odometrico[4] + odometrico[0])*escala;
  y1_canvas = (xa * odometrico[4] + ya * odometrico[3] + odometrico[1])*escala;

	x2_canvas = (xb * odometrico[3] - yb * odometrico[4] + odometrico[0])*escala;
  y2_canvas = (xb * odometrico[4] + yb * odometrico[3] + odometrico[1])*escala;

    x1_canvas = x1_canvas + width/2;
    y1_canvas = -y1_canvas + height/2;

    x2_canvas = x2_canvas + width/2;
    y2_canvas = -y2_canvas + height/2;

	fl_set_foreground(canvas_gc, FL_BLACK);
	XDrawLine(display,canvas_win,canvas_gc,x1_canvas,y1_canvas,x2_canvas,y2_canvas);


}

void pinta_trayecto_verde(float xa,float ya,float xb,float yb)
{
	float x1_canvas,x2_canvas;
  float y1_canvas,y2_canvas;
	/*printf("Pintamos verde[%f,%f] - [%f,%f]\n",xa,ya,xb,yb);*/

/* Transformamos puntos a canvas */
	x1_canvas = (xa * odometrico[3] - ya * odometrico[4] + odometrico[0])*escala;
  y1_canvas = (xa * odometrico[4] + ya * odometrico[3] + odometrico[1])*escala;

	x2_canvas = (xb * odometrico[3] - yb * odometrico[4] + odometrico[0])*escala;
  y2_canvas = (xb * odometrico[4] + yb * odometrico[3] + odometrico[1])*escala;

    x1_canvas = x1_canvas + width/2;
    y1_canvas = -y1_canvas + height/2;

    x2_canvas = x2_canvas + width/2;
    y2_canvas = -y2_canvas + height/2;

	fl_set_foreground(canvas_gc, FL_GREEN);
	XDrawLine(display,canvas_win,canvas_gc,x1_canvas,y1_canvas,x2_canvas,y2_canvas);


}


int image_displaysetup() 
/* Inicializa las ventanas, la paleta de colores y memoria compartida para visualizacion*/ 
{
    XGCValues gc_values;
    XWindowAttributes win_attributes;
    XColor nuevocolor;
    int pixelNum, numCols;
    int allocated_colors=0, non_allocated_colors=0;
   

    image_win= FL_ObjWin(fd_guixforms->ventana0);
    XGetWindowAttributes(display, image_win, &win_attributes);  
    screen = DefaultScreen(display);

    XMapWindow(display, image_win);
    
    /*XSelectInput(display, image_win, ButtonPress|StructureNotifyMask);*/
   
    gc_values.graphics_exposures = False;
    image_gc = XCreateGC(display, image_win, GCGraphicsExposures, &gc_values);  
    
    /* Utilizan el Visual (=estructura de color) y el colormap con que este operando el programa principal con su Xforms. No crea un nuevo colormap, sino que modifica el que se estaba usando a traves de funciones de Xforms*/
    vmode= fl_get_vclass();
    if ((vmode==TrueColor)&&(fl_state[vmode].depth==16)) 
      {printf("display: truecolor 16 bpp\n");
      imagenA_buf = (char *) malloc(SIFNTSC_COLUMNS*SIFNTSC_ROWS*2);    
      imagenA = XCreateImage(display,DefaultVisual(display,screen),win_attributes.depth, ZPixmap,0,imagenA_buf,SIFNTSC_COLUMNS, SIFNTSC_ROWS,8,0);
      imagenB_buf = (char *) malloc(SIFNTSC_COLUMNS*SIFNTSC_ROWS*2);    
      imagenB = XCreateImage(display,DefaultVisual(display,screen),win_attributes.depth, ZPixmap,0,imagenB_buf,SIFNTSC_COLUMNS, SIFNTSC_ROWS,8,0);
      return win_attributes.depth;
      }
    else if ((vmode==TrueColor)&&(fl_state[vmode].depth==24)) 
      { printf("display: truecolor 24 bpp\n");
      imagenA_buf = (char *) malloc(SIFNTSC_COLUMNS*SIFNTSC_ROWS*4); 
      imagenA = XCreateImage(display,DefaultVisual(display,screen),24, ZPixmap,0,imagenA_buf,SIFNTSC_COLUMNS, SIFNTSC_ROWS,8,0);
      imagenB_buf = (char *) malloc(SIFNTSC_COLUMNS*SIFNTSC_ROWS*4); 
      imagenB = XCreateImage(display,DefaultVisual(display,screen),24, ZPixmap,0,imagenB_buf,SIFNTSC_COLUMNS, SIFNTSC_ROWS,8,0);
      return win_attributes.depth;
      }
    else if ((vmode==PseudoColor)&&(fl_state[vmode].depth==8)) 
      {
	numCols = 256;
	for (pixelNum=0; pixelNum<numCols; pixelNum++) 
	  {
	    nuevocolor.pixel=0;
	    nuevocolor.red=pixelNum<<8;
	    nuevocolor.green=pixelNum<<8;
	    nuevocolor.blue=pixelNum<<8;
	    nuevocolor.flags=DoRed|DoGreen|DoBlue;
	    
	    /*if (XAllocColor(display,DefaultColormap(display,screen),&nuevocolor)==False) tabla[pixelNum]=tabla[pixelNum-1];*/
	    if (XAllocColor(display,fl_state[vmode].colormap,&nuevocolor)==False) {tabla[pixelNum]=tabla[pixelNum-1]; non_allocated_colors++;}
	    else {tabla[pixelNum]=nuevocolor.pixel;allocated_colors++;}
	  }
	printf("display: depth= %d\n", fl_state[vmode].depth); 
	printf("display: colormap got %d colors, %d non_allocated colors\n",allocated_colors,non_allocated_colors);

	imagenA_buf = (char *) malloc(SIFNTSC_COLUMNS*SIFNTSC_ROWS);    
	imagenA = XCreateImage(display,DefaultVisual(display,screen),8, ZPixmap,0,imagenA_buf,SIFNTSC_COLUMNS, SIFNTSC_ROWS,8,0);
	imagenB_buf = (char *) malloc(SIFNTSC_COLUMNS*SIFNTSC_ROWS);    
	imagenB = XCreateImage(display,DefaultVisual(display,screen),8, ZPixmap,0,imagenB_buf,SIFNTSC_COLUMNS, SIFNTSC_ROWS,8,0);

	pixel8bpp_rojo = fl_get_pixel(FL_RED);
	pixel8bpp_blanco = fl_get_pixel(FL_WHITE);
	pixel8bpp_amarillo = fl_get_pixel(FL_YELLOW);
	return win_attributes.depth;
      }
    else 
      {
	perror("Unsupported color mode in X server");exit(1);
      }
    return win_attributes.depth;
}


void display_iteration() 
     /* Siempre hay una estructura grafica con todo lo que debe estar en pantalla. Permite el pintado incremental para los buffers de puntos, borran el incremento viejo y pintan solo el nuevo */
{
  int i;
	int j;
  Tvoxel kaka;
 
 
  fl_winset(canvas_win); 
  
 if ((track_robot==TRUE)&&
     ((fabs(robot[0]+odometrico[0])>(rango/4.))||
      (fabs(robot[1]+odometrico[1])>(rango/4.))))
   {odometrico[0]=-robot[0];
   odometrico[1]=-robot[1];
   visual_refresh = TRUE;
   if (debug[SCH_GUIXFORMS]) printf("gui: robot tracking, display movement\n"); 
     }

 if (iteracion_display*guixforms_cycle>FORCED_REFRESH) 
   {iteracion_display=0;
   visual_refresh=TRUE;
   }
 else iteracion_display++;


 if (visual_refresh==TRUE)
   {
     if (debug[SCH_GUIXFORMS]) printf(" TOTAL ");
     fl_rectbound(0,0,width,height,FL_WHITE);   
     XFlush(display);
     /*XSync(display,True);*/
   }
 
 
 /* VISUALIZACION de una instantanea ultrasonica */
   if ((((display_state&DISPLAY_SONARS)!=0)&&(visual_refresh==FALSE))
      || (visual_delete_us==TRUE))
     {  
    fl_set_foreground(canvas_gc,FL_WHITE); 
    /* clean last sonars, but only if there wasn't a total refresh. In case of total refresh the white rectangle already cleaned all */
  for(i=0;i<NUM_SONARS*2;i+=2) XDrawLine(display,canvas_win,canvas_gc,us_dpy[i].x,us_dpy[i].y,us_dpy[i+1].x,us_dpy[i+1].y);

     }

  if ((display_state&DISPLAY_SONARS)!=0){
   if (debug[SCH_GUIXFORMS]) printf(" sonars ");
  for(i=0;i<NUM_SONARS;i++)
      {us2xy(i,0.,0.,&kaka); /* Da en el Tvoxel kaka las coordenadas del sensor, pues es distancia 0 */
      xy2canvas(kaka,&us_dpy[2*i]);
      us2xy(i,us[i],0.,&kaka);
      /*us2xy(i,200,0.,&kaka);
	if (i==6) us2xy(i,400,0.,&kaka);*/
      xy2canvas(kaka,&us_dpy[2*i+1]);
      }
  fl_set_foreground(canvas_gc,FL_PALEGREEN);
  for(i=0;i<NUM_SONARS*2;i+=2) XDrawLine(display,canvas_win,canvas_gc,us_dpy[i].x,us_dpy[i].y,us_dpy[i+1].x,us_dpy[i+1].y);
  }

 /* VISUALIZACION de una instantanea laser*/
   if ((((display_state&DISPLAY_LASER)!=0)&&(visual_refresh==FALSE))
      || (visual_delete_laser==TRUE))
     {  
    fl_set_foreground(canvas_gc,FL_WHITE); 
    /* clean last laser, but only if there wasn't a total refresh. In case of total refresh the white rectangle already cleaned all */
    /*for(i=0;i<NUM_LASER;i++) XDrawPoint(display,canvas_win,canvas_gc,laser_dpy[i].x,laser_dpy[i].y);*/
    XDrawPoints(display,canvas_win,canvas_gc,laser_dpy,NUM_LASER,CoordModeOrigin);
     }

  if ((display_state&DISPLAY_LASER)!=0){
   if (debug[SCH_GUIXFORMS]) printf(" laser ");
   for(i=0;i<NUM_LASER;i++)
     {
       laser2xy(i,laser[i],&kaka);
       xy2canvas(kaka,&laser_dpy[i]);
     }
   fl_set_foreground(canvas_gc,FL_BLUE);
   /*for(i=0;i<NUM_LASER;i++) XDrawPoint(display,canvas_win,canvas_gc,laser_dpy[i].x,laser_dpy[i].y);*/
   XDrawPoints(display,canvas_win,canvas_gc,laser_dpy,NUM_LASER,CoordModeOrigin);
  }




  /* VISUALIZACION: pintar o borrar de el PROPIO ROBOT.
     Siempre hay un repintado total. Esta es la ultima estructura que se se pinta, para que ninguna otra se solape encima */

   if ((((display_state&DISPLAY_ROBOT)!=0) &&(visual_refresh==FALSE))
       || (visual_delete_ego==TRUE))
     {  
      fl_set_foreground(canvas_gc,FL_WHITE); 
    /* clean last robot, but only if there wasn't a total refresh. In case of total refresh the white rectangle already cleaned all */
    for(i=0;i<numego;i++) XDrawLine(display,canvas_win,canvas_gc,ego[i].x,ego[i].y,ego[i+1].x,ego[i+1].y);

     }

  if ((display_state&DISPLAY_ROBOT)!=0){
    if (debug[SCH_GUIXFORMS]) printf(" ego ");
    fl_set_foreground(canvas_gc,FL_MAGENTA);
 /* relleno los nuevos */
    us2xy(15,0.,0.,&kaka);
    xy2canvas(kaka,&ego[0]);
    us2xy(3,0.,0.,&kaka);
    xy2canvas(kaka,&ego[1]);
    us2xy(4,0.,0.,&kaka);
    xy2canvas(kaka,&ego[2]);
    us2xy(8,0.,0.,&kaka);
    xy2canvas(kaka,&ego[3]);
    us2xy(15,0.,0.,&kaka);
    xy2canvas(kaka,&ego[EGOMAX-1]);
    for(i=0;i<NUM_SONARS;i++)
      {
      us2xy((15+i)%NUM_SONARS,0.,0.,&kaka); /* Da en el Tvoxel kaka las coordenadas del sensor, pues es distancia 0 */
      xy2canvas(kaka,&ego[i+4]);       
      }
 
    /* pinto los nuevos */
    numego=EGOMAX-1;
    for(i=0;i<numego;i++) XDrawLine(display,canvas_win,canvas_gc,ego[i].x,ego[i].y,ego[i+1].x,ego[i+1].y);
  }

  if (debug[SCH_GUIXFORMS]) printf("\n");



  /* grey imageA display */
  if (((display_state&DISPLAY_GREYIMAGEA)!=0)&&((display_state&DISPLAY_COLORIMAGEA)==0))
     {
       /* Pasa de la imagen capturada (greyA) a la imagen para visualizar (imagenA_buf), "cambiando" de formato adecuadamente */
       if ((vmode==PseudoColor)&&(fl_state[vmode].depth==8))
	 {for(i=0; i<SIFNTSC_ROWS*SIFNTSC_COLUMNS; i++) 
	   {/*shmimage->data[i]= (unsigned char)tabla[(unsigned char)(greyA[i])];*/
	     imagenA_buf[i]= (unsigned char)tabla[(unsigned char)(greyA[i])];}}
       else if ((vmode==TrueColor)&&(fl_state[vmode].depth==16)) 
	 {
	   for(i=0; i<SIFNTSC_ROWS*SIFNTSC_COLUMNS; i++)
	     { imagenA_buf[i*2+1]=(0xf8&(greyA[i]))+((0xe0&(greyA[i]))>>5);
	     imagenA_buf[i*2]=((0xf8&(greyA[i]))>>3)+((0x1c&(greyA[i]))<<3);
	     }
	 }
       else if ((vmode==TrueColor)&&(fl_state[vmode].depth==24)) 
	 {for(i=0; i<SIFNTSC_ROWS*SIFNTSC_COLUMNS; i++) 
	   { imagenA_buf[i*4]=greyA[i]; /* Blue Byte */
	   imagenA_buf[i*4+1]=greyA[i]; /* Green Byte */
	   imagenA_buf[i*4+2]=greyA[i]; /* Red Byte */
	   imagenA_buf[i*4+3]=0; /* dummy byte */  }
	 }
     }


  /* grey imageB display */
  if (((display_state&DISPLAY_GREYIMAGEB)!=0)&&((display_state&DISPLAY_COLORIMAGEB)==0))
     {
       /* Pasa de la imagen capturada (greyA) a la imagen para visualizar (imagenA_buf), "cambiando" de formato adecuadamente */
       if ((vmode==PseudoColor)&&(fl_state[vmode].depth==8))
	 {for(i=0; i<SIFNTSC_ROWS*SIFNTSC_COLUMNS; i++) 
	   {/*shmimage->data[i]= (unsigned char)tabla[(unsigned char)(greyB[i])];*/
	     imagenB_buf[i]= (unsigned char)tabla[(unsigned char)(greyB[i])];}}
       else if ((vmode==TrueColor)&&(fl_state[vmode].depth==16)) 
	 {
	   for(i=0; i<SIFNTSC_ROWS*SIFNTSC_COLUMNS; i++)
	     { imagenB_buf[i*2+1]=(0xf8&(greyB[i]))+((0xe0&(greyB[i]))>>5);
	     imagenB_buf[i*2]=((0xf8&(greyB[i]))>>3)+((0x1c&(greyB[i]))<<3);
	     }
	 }
       else if ((vmode==TrueColor)&&(fl_state[vmode].depth==24)) 
	 {for(i=0; i<SIFNTSC_ROWS*SIFNTSC_COLUMNS; i++) 
	   { imagenB_buf[i*4]=greyB[i]; /* Blue Byte */
	   imagenB_buf[i*4+1]=greyB[i]; /* Green Byte */
	   imagenB_buf[i*4+2]=greyB[i]; /* Red Byte */
	   imagenB_buf[i*4+3]=0; /* dummy byte */  }
	 }
     }


  /* color imageA display */
  if ((display_state&DISPLAY_COLORIMAGEA)!=0)
     {
       /* Pasa de la imagen capturada (greyA) a la imagen para visualizar (imagenA_buf), "cambiando" de formato adecuadamente */
       if ((vmode==PseudoColor)&&(fl_state[vmode].depth==8))
	 {for(i=0; i<SIFNTSC_ROWS*SIFNTSC_COLUMNS; i++) 
	   {/*shmimage->data[i]= (unsigned char)tabla[(unsigned char)(greyA[i])];*/
	     imagenA_buf[i]= (unsigned char)tabla[(unsigned char)(colorA[i*3])];}}
       else if ((vmode==TrueColor)&&(fl_state[vmode].depth==16)) 
	 {
	   for(i=0; i<SIFNTSC_ROWS*SIFNTSC_COLUMNS; i++)
	     { imagenA_buf[i*2+1]=(0xf8&(colorA[i*3+2]))+((0xe0&(colorA[i*3+1]))>>5);
	     imagenA_buf[i*2]=((0xf8&(colorA[i*3]))>>3)+((0x1c&(colorA[i*3+1]))<<3);
	     }
	 }
       else if ((vmode==TrueColor)&&(fl_state[vmode].depth==24)) 
	 {for(i=0; i<SIFNTSC_ROWS*SIFNTSC_COLUMNS; i++) 
	   { imagenA_buf[i*4]=colorA[i*3]; /* Blue Byte */
	   imagenA_buf[i*4+1]=colorA[i*3+1]; /* Green Byte */
	   imagenA_buf[i*4+2]=colorA[i*3+2]; /* Red Byte */
	   imagenA_buf[i*4+3]=0; /* dummy byte */  }
	 }
     }
 

  /* color imageB display */
  if ((display_state&DISPLAY_COLORIMAGEB)!=0)
     {
       /* Pasa de la imagen capturada (greyB) a la imagen para visualizar (imagenB_buf), "cambiando" de formato adecuadamente */
       if ((vmode==PseudoColor)&&(fl_state[vmode].depth==8))
	 {for(i=0; i<SIFNTSC_ROWS*SIFNTSC_COLUMNS; i++) 
	   {/*shmimage->data[i]= (unsigned char)tabla[(unsigned char)(greyB[i])];*/
	     imagenB_buf[i]= (unsigned char)tabla[(unsigned char)(colorB[i*3])];}}
       else if ((vmode==TrueColor)&&(fl_state[vmode].depth==16)) 
	 {
	   for(i=0; i<SIFNTSC_ROWS*SIFNTSC_COLUMNS; i++)
	     { imagenB_buf[i*2+1]=(0xf8&(colorB[i*3+2]))+((0xe0&(colorB[i*3+1]))>>5);
	     imagenB_buf[i*2]=((0xf8&(colorB[i*3]))>>3)+((0x1c&(colorB[i*3+1]))<<3);
	     }
	 }
       else if ((vmode==TrueColor)&&(fl_state[vmode].depth==24)) 
	 {for(i=0; i<SIFNTSC_ROWS*SIFNTSC_COLUMNS; i++) 
	   { imagenB_buf[i*4]=colorB[i*3]; /* Blue Byte */
	   imagenB_buf[i*4+1]=colorB[i*3+1]; /* Green Byte */
	   imagenB_buf[i*4+2]=colorB[i*3+2]; /* Red Byte */
	   imagenB_buf[i*4+3]=0; /* dummy byte */  }
	 }
     }
 


   if (((display_state&DISPLAY_GREYIMAGEA)!=0)||
       ((display_state&DISPLAY_COLORIMAGEA)!=0))
     {    /* Draw screen onto display */
       XPutImage(display,image_win,image_gc,imagenA,0,0,fd_guixforms->ventana0->x+1, fd_guixforms->ventana0->y+1,  SIFNTSC_COLUMNS, SIFNTSC_ROWS);
     }

   if (((display_state&DISPLAY_GREYIMAGEB)!=0)||
       ((display_state&DISPLAY_COLORIMAGEB)!=0))
     {    /* Draw screen onto display */
       XPutImage(display,image_win,image_gc,imagenB,0,0,fd_guixforms->ventana1->x+1, fd_guixforms->ventana1->y+1,  SIFNTSC_COLUMNS, SIFNTSC_ROWS);
     }

   if (state[SCH_IMAGEA]==active)  sprintf(fpstext,"%.1f fps",fpsA);
   else sprintf(fpstext," ");
   fl_set_object_label(fd_guixforms->frame_rateA,fpstext);
   if (state[SCH_IMAGEB]==active)  sprintf(fpstext,"%.1f fps",fpsB);
   else sprintf(fpstext," ");
   fl_set_object_label(fd_guixforms->frame_rateB,fpstext);
   if (state[SCH_SONARS]==active) sprintf(fpstext,"%.1f fps",fpssonars);
   else sprintf(fpstext," ");
   fl_set_object_label(fd_guixforms->fpssonars,fpstext);
   if (state[SCH_LASER]==active) sprintf(fpstext,"%.1f fps",fpslaser);
   else sprintf(fpstext," ");
   fl_set_object_label(fd_guixforms->fpslaser,fpstext);
   if (state[SCH_ENCODERS]==active) sprintf(fpstext,"%.1f fps",fpsencoders);
   else sprintf(fpstext," ");
   fl_set_object_label(fd_guixforms->fpsencoders,fpstext);
   if (state[SCH_PANTILTENCODERS]==active) sprintf(fpstext,"%.1f fps",fpspantiltencoders);
   else sprintf(fpstext," ");
   fl_set_object_label(fd_guixforms->fpspantiltencoders,fpstext);


   if (state[SCH_PANTILTMOTORS]==winner) sprintf(fpstext,"%.1f fps",fpspantiltmotors);
   else sprintf(fpstext," ");
   fl_set_object_label(fd_guixforms->fpspantiltmotors,fpstext);
   if (state[SCH_MOTORS]==winner) sprintf(fpstext,"%.1f fps",fpsmotors);
   else sprintf(fpstext," ");
   fl_set_object_label(fd_guixforms->fpsmotors,fpstext);


   sprintf(fpstext,"%.1f ips",1000./guixforms_cycle);
   fl_set_object_label(fd_guixforms->guifps,fpstext);

   /* clear all flags. If they were set at the beginning, they have been already used in this iteration */
   visual_refresh=FALSE;
   visual_delete_us=FALSE; 
   visual_delete_laser=FALSE; 
   visual_delete_ego=FALSE;

/****************************************************/
/* Llamamos a la funcion para pintar la trayectoria */
/****************************************************/
	for (j=1;j<N_PUNTOS;j++)
		pinta_trayecto_verde(lista[j-1].x,lista[j-1].y,lista[j].x,lista[j].y);
	pinta_trayecto_negro(x_origen,y_origen,x_final,y_final);


}























void buttons_iteration() 
{
  /* Puesto que el control no se cede al form, sino que se hace polling de sus botones pulsados, debemos proveer el enlace para los botones que no tengan callback asociada, en esta rutina de polling. OJO aquellos que tengan callback asociada jamas se veran con fl_check_forms, la libreria llamara automaticamente a su callback sin que fl_check_forms o fl_do_forms se enteren en absoluto.*/
  FL_OBJECT *obj;
  int brothers[NUM_SCHEMAS];
  int i;
  float dpan=0.5,dtilt=0.5, speed_coef;

  obj = fl_check_forms();
  
  if (obj == fd_guixforms->exit) jdeshutdown(0);
  else if (obj == fd_guixforms->escala)  
    {  rango=fl_get_slider_value(fd_guixforms->escala);
    visual_refresh = TRUE; /* activa el flag que limpia el fondo de la pantalla y repinta todo */ 
    escala = width /rango;}
  else if (obj== fd_guixforms->center)
    /* Se mueve 10%un porcentaje del rango */
    {
      odometrico[0]+=rango*(fl_get_positioner_xvalue(fd_guixforms->center)-0.5)*(-2.)*(0.1);
      odometrico[1]+=rango*(fl_get_positioner_yvalue(fd_guixforms->center)-0.5)*(-2.)*(0.1);
      fl_set_positioner_xvalue(fd_guixforms->center,0.5);
      fl_set_positioner_yvalue(fd_guixforms->center,0.5);
      visual_refresh=TRUE;  }
 else if (obj == fd_guixforms->track_robot) 
    {if (fl_get_button(obj)==PUSHED) track_robot=TRUE;
    else track_robot=FALSE;
    }


  else if (obj == fd_guixforms->vissonars)
    {
      if (fl_get_button(fd_guixforms->vissonars)==RELEASED)
	{display_state = display_state & ~DISPLAY_SONARS;
	visual_delete_us=TRUE;
	}
      else 
	display_state=display_state | DISPLAY_SONARS;
    }
  else if (obj == fd_guixforms->sonars)
    {
      if (fl_get_button(fd_guixforms->sonars)==RELEASED)
	sonars_suspend(sonars_cb);
      else 
	sonars_cb=sonars_resume(NULL);
    }
  else if (obj == fd_guixforms->vislaser)
    {
      if (fl_get_button(fd_guixforms->vislaser)==RELEASED)
	{display_state = display_state & ~DISPLAY_LASER;
	visual_delete_laser=TRUE;
	}
      else 
	display_state=display_state | DISPLAY_LASER;
    }
  else if (obj == fd_guixforms->laser)
    {
      if (fl_get_button(fd_guixforms->laser)==RELEASED)
	laser_suspend(laser_cb);
      else 
	laser_cb=laser_resume(NULL);
    }
  else if (obj == fd_guixforms->visrobot)
    {
      if (fl_get_button(fd_guixforms->visrobot)==RELEASED)
	{display_state = display_state & ~DISPLAY_ROBOT;
	visual_delete_ego=TRUE;
	}
      else 
	display_state=display_state | DISPLAY_ROBOT;
    }
  else if (obj == fd_guixforms->robot)
    {
      if (fl_get_button(fd_guixforms->robot)==RELEASED)
	encoders_suspend(encoders_cb);
      else 
	encoders_cb=encoders_resume(NULL);
    }
  else if (obj == fd_guixforms->vispantiltencoders);
  else if (obj == fd_guixforms->pantilt_encoders)
    {
      if (fl_get_button(fd_guixforms->pantilt_encoders)==RELEASED)
	pantiltencoders_suspend(pantiltencoders_cb);
      else pantiltencoders_cb=pantiltencoders_resume(NULL);
    }
  else if (obj == fd_guixforms->visgreyimageA)
    {
      if (fl_get_button(fd_guixforms->visgreyimageA)==RELEASED)
          display_state = display_state & ~DISPLAY_GREYIMAGEA;
      else 
	  display_state=display_state | DISPLAY_GREYIMAGEA;
    }
  else if (obj == fd_guixforms->viscolorimageA)
    {
      if (fl_get_button(fd_guixforms->viscolorimageA)==RELEASED)
	display_state = display_state & ~DISPLAY_COLORIMAGEA;
      else 
	display_state=display_state | DISPLAY_COLORIMAGEA;
    }
  else if (obj == fd_guixforms->colorimageA)
    { 
      if (fl_get_button(fd_guixforms->colorimageA)==PUSHED)
	imageA_cb=imageA_resume(NULL);
      else 
	imageA_suspend(imageA_cb);
      fpsA=0;
    }
  else if (obj == fd_guixforms->visgreyimageB)
    {
      if (fl_get_button(fd_guixforms->visgreyimageB)==RELEASED)
          display_state = display_state & ~DISPLAY_GREYIMAGEB;
      else display_state=display_state | DISPLAY_GREYIMAGEB;
    }
  else if (obj == fd_guixforms->viscolorimageB)
    {
      if (fl_get_button(fd_guixforms->viscolorimageB)==RELEASED)
          display_state = display_state & ~DISPLAY_COLORIMAGEB;
      else 
	  display_state=display_state | DISPLAY_COLORIMAGEB;
    }
  else if (obj == fd_guixforms->colorimageB)
    {
      if (fl_get_button(fd_guixforms->colorimageB)==PUSHED)
	imageB_cb=imageB_resume(NULL);
      else 
	imageB_suspend(imageB_cb);
      fpsB=0;
    }

  else if (obj == fd_guixforms->motors) 
    {
      if (fl_get_button(fd_guixforms->motors)==RELEASED)
	motors_suspend(); /* it makes a safety stop itself before suspending */ 
      else motors_resume(NULL,NULL);
    }
  else if (obj == fd_guixforms->pantiltmotors) 
  {
      if (fl_get_button(fd_guixforms->pantiltmotors)==RELEASED)
	pantiltmotors_suspend();
      else pantiltmotors_resume(NULL,NULL);
    }
 
  else if (obj == fd_guixforms->teleoperator) 
    {   
      if (fl_get_button(fd_guixforms->teleoperator)==RELEASED)
	{v=0.; w=0.; /*safety stop */
	}
    }
  else if (obj == fd_guixforms->joystick) 
    {
      if (fl_get_button(fd_guixforms->teleoperator)==PUSHED)
	{
	  if (fl_get_button(fd_guixforms->back)==RELEASED)
	    joystick_y=0.5+0.5*fl_get_positioner_yvalue(fd_guixforms->joystick);
	  else 
	    joystick_y=0.5-0.5*fl_get_positioner_yvalue(fd_guixforms->joystick);
	  joystick_x=fl_get_positioner_xvalue(fd_guixforms->joystick);
	  fl_redraw_object(fd_guixforms->joystick);
	}
    }    
   else if (obj == fd_guixforms->back) 
    {
      if (fl_get_button(fd_guixforms->back)==RELEASED)
	joystick_y=0.5+0.5*fl_get_positioner_yvalue(fd_guixforms->joystick);
      else 
	joystick_y=0.5-0.5*fl_get_positioner_yvalue(fd_guixforms->joystick);
      joystick_x=fl_get_positioner_xvalue(fd_guixforms->joystick);
      fl_redraw_object(fd_guixforms->joystick);
    }    
  else if (obj == fd_guixforms->stop) 
    {
      fl_set_positioner_xvalue(fd_guixforms->joystick,0.5);
      fl_set_positioner_yvalue(fd_guixforms->joystick,0.);
      joystick_x=0.5;
      joystick_y=0.5;
    }   
 
  else if (obj == fd_guixforms->pantilt_enable) 
    {
      if (fl_get_button(fd_guixforms->pantilt_enable)==PUSHED)
	{
	  /* current pantilt position as initial command, to avoid any movement */
	  if ((MAX_PAN_ANGLE - MIN_PAN_ANGLE) > 0.05) 
	    pt_joystick_x= 1.-(pan_angle-MIN_PAN_ANGLE)/(MAX_PAN_ANGLE-MIN_PAN_ANGLE);
	  if ((MAX_TILT_ANGLE - MIN_TILT_ANGLE) > 0.05) 
	    pt_joystick_y= (tilt_angle-MIN_TILT_ANGLE)/(MAX_TILT_ANGLE-MIN_TILT_ANGLE);   
      } 
    }
  else if (obj == fd_guixforms->pantilt_joystick) 
    {
      if (fl_get_button(fd_guixforms->pantilt_enable)==PUSHED)
	{pt_joystick_y=fl_get_positioner_yvalue(fd_guixforms->pantilt_joystick);
	pt_joystick_x=fl_get_positioner_xvalue(fd_guixforms->pantilt_joystick);
	/*  fl_redraw_object(fd_guixforms->pantilt_joystick);*/
	}
    }    
  else if (obj == fd_guixforms->pantilt_origin) 
    {
      if ((MAX_PAN_ANGLE - MIN_PAN_ANGLE) > 0.05) 
	pt_joystick_x= 1.-(0.-MIN_PAN_ANGLE)/(MAX_PAN_ANGLE-MIN_PAN_ANGLE);
      if ((MAX_TILT_ANGLE - MIN_TILT_ANGLE) > 0.05) 
	pt_joystick_y= (0.-MIN_TILT_ANGLE)/(MAX_TILT_ANGLE-MIN_TILT_ANGLE);   
    }     
  else if (obj == fd_guixforms->pantilt_stop) 
    {
      /* current pantilt position as initial command, to avoid any movement */
      if ((MAX_PAN_ANGLE - MIN_PAN_ANGLE) > 0.05) 
	pt_joystick_x= 1.-(pan_angle-MIN_PAN_ANGLE)/(MAX_PAN_ANGLE-MIN_PAN_ANGLE);
      if ((MAX_TILT_ANGLE - MIN_TILT_ANGLE) > 0.05) 
	pt_joystick_y= (tilt_angle-MIN_TILT_ANGLE)/(MAX_TILT_ANGLE-MIN_TILT_ANGLE);   
    }
  else if (obj == fd_guixforms->ptspeed)
    {
      /*
      speed_coef = fl_get_slider_value(fd_guixforms->ptspeed);
      longitude_speed=(1.-speed_coef)*MAX_SPEED_PANTILT;
      latitude_speed=(1.-speed_coef)*MAX_SPEED_PANTILT;*/
    }
  
  else if (obj == fd_guixforms->myschema) 
    {
      if (fl_get_button(fd_guixforms->myschema)==RELEASED) myschema_suspend();
      else  {
	myschema_cycle=100;
	for(i=0;i<NUM_SCHEMAS;i++) brothers[i]=-1;
	brothers[0]= SCH_MYSCHEMA;
	myschema_resume(brothers,null_arbitration);
      } 
    }
  
  /* modifies pantilt positioner to follow pantilt angles. 
     It tracks the pantilt movement. It should be at
     display_poll, but there it causes a weird display behavior. */
  if ((MAX_PAN_ANGLE - MIN_PAN_ANGLE) > 0.05) 
    dpan= 1.-(pan_angle-MIN_PAN_ANGLE)/(MAX_PAN_ANGLE-MIN_PAN_ANGLE);
  if ((MAX_TILT_ANGLE - MIN_TILT_ANGLE) > 0.05) 
    dtilt= (tilt_angle-MIN_TILT_ANGLE)/(MAX_TILT_ANGLE-MIN_TILT_ANGLE);   
  fl_set_positioner_xvalue(fd_guixforms->pantilt_joystick,dpan);
  fl_set_positioner_yvalue(fd_guixforms->pantilt_joystick,dtilt);
  /*fl_redraw_object(fd_guixforms->pantilt_joystick);*/
  
}



int button_released_on_micanvas(FL_OBJECT *ob, Window win, int win_width, int win_height, XEvent *xev, void *user_data)
{
FL_Coord y,x;
unsigned int whichbutton;
float ygraf, xgraf;

fl_get_win_mouse(win,&x,&y,&whichbutton);
/* win will be always the canvas window, because this callback has been defined only for that canvas */

/* from graphical coordinates to spatial ones */
 ygraf=((float) (height/2-y))/escala;
 xgraf=((float) (x-width/2))/escala;
 mouse_y=(ygraf-odometrico[1])*odometrico[3]+(-xgraf+odometrico[0])*odometrico[4];
 mouse_x=(ygraf-odometrico[1])*odometrico[4]+(xgraf-odometrico[0])*odometrico[3];
 mouse_new=1;
 /*printf("(%d,%d) CLICK on mouse_x=%.2f mouse_y=%.2f robot_x=%.2f robot_y=%.2f robot_theta=%.2f\n",x,y,mouse_x,mouse_y,robot[0],robot[1],robot[2]);*/
 return 0;
}

void guixforms_iteration()
{ 
 double delta, deltapos;
 float speed_coef;

  if (debug[SCH_GUIXFORMS]) printf("guixforms iteration\n");
  buttons_iteration();
  display_iteration();
  
  if (fl_get_button(fd_guixforms->teleoperator)==PUSHED)
    {
      /* ROTACION=ejeX: Ajusta a un % de joystick_maxRotVel. OJO no funcion lineal del desplazamiento visual, sino con el al cuadrado, para aplanarla en el origen y evitar cabeceos, conseguir suavidad en la teleoperacion */
      
      delta = (joystick_x-0.5)*2; /* entre +-1 */
      deltapos = fabs(delta); /* Para que no moleste el signo de delta en el factor de la funcion de control */
      if (delta<0) w = (float) joystick_maxRotVel*deltapos*deltapos*deltapos; 
      else w = (float) -1.*joystick_maxRotVel*deltapos*deltapos*deltapos;
      
 
      /* TRASLACION=ejeY: Ajusta a un % de +-joystick_maxTranVel. OJO no funcion lineal del desplazamiento visual, sino con el a la cuarta, para aplanarla en el origen */

      delta = (joystick_y-0.5)*2; /* entre +-1 */
      deltapos = fabs(delta);/* Para que no moleste el signo de delta en el factor de la funcion de control */
      if (delta<0) v = (float) -1.*joystick_maxTranVel*deltapos*deltapos*deltapos;
      else v = (float) joystick_maxTranVel*deltapos*deltapos*deltapos;
    }

  if (fl_get_button(fd_guixforms->pantilt_enable)==PUSHED)
    {
      /* pt_joystick_x and pt_joystick_y fall in [0,1], the default limits from Xforms */  
      latitude=MIN_TILT_ANGLE+pt_joystick_y*(MAX_TILT_ANGLE-MIN_TILT_ANGLE);
      longitude=MAX_PAN_ANGLE-pt_joystick_x*(MAX_PAN_ANGLE-MIN_PAN_ANGLE);

      speed_coef = fl_get_slider_value(fd_guixforms->ptspeed);
      longitude_speed=(1.-speed_coef)*MAX_SPEED_PANTILT;
      latitude_speed=(1.-speed_coef)*MAX_SPEED_PANTILT;
      /*printf("GUIXFORMS: longitude speed %.2f, latitude speed %.2f\n",longitude_speed,latitude_speed);*/
    }


}

void *guixforms_thread(void *not_used) 
{
  struct timeval a,b;
  long diff, next;

  for(;;)
    {
      pthread_mutex_lock(&mymutex[SCH_GUIXFORMS]);
      if (state[SCH_GUIXFORMS]==slept) 
	{
	  /*printf("guixforms: off\n");*/
	  pthread_cond_wait(&condition[SCH_GUIXFORMS],&mymutex[SCH_GUIXFORMS]);
	  /*printf("guixforms: on\n");*/
	}
      else 
	{
	  gettimeofday(&a,NULL);
	  guixforms_iteration();
	  gettimeofday(&b,NULL);  
	  diff = (b.tv_sec-a.tv_sec)*1000000+b.tv_usec-a.tv_usec;

	  next = guixforms_cycle*1000-diff-10000; 
	  /* discounts 10ms taken by calling usleep itself */
	  if (next>0) usleep(guixforms_cycle*1000-diff);
	  else {printf("display: time interval violated\n"); /*exit(-1);*/}
	}
      pthread_mutex_unlock(&mymutex[SCH_GUIXFORMS]);
    }
}


void guixforms_startup()
{
  int myargc=1;
  char **myargv;
  char *aa;
  char a[]="myjde";

  /* prepara el display */
  aa=a;
  myargv=&aa;
  display= fl_initialize(&myargc,myargv,"JDE",0,0);
  fd_guixforms = create_form_guixformsf();
  fl_set_form_position(fd_guixforms->guixformsf,200,50);
  fl_show_form(fd_guixforms->guixformsf,FL_PLACE_POSITION,FL_FULLBORDER,"Basic JDE");

  image_displaysetup(); /* Tiene que ir despues de la inicializacion de Forms, pues hace uso de informacion que la libreria rellena en tiempo de ejecucion al iniciarse */

  canvas_gc = fl_state[fl_get_vclass()].gc[0];
  canvas_win = FL_ObjWin(fd_guixforms->micanvas);
  width = fd_guixforms->micanvas->w;
  height = fd_guixforms->micanvas->h;
  /* Empiezo con el canvas en blanco */
  fl_winset(canvas_win); 
  fl_rectbound(0,0,width,height,FL_WHITE);


  /* Coord del sistema odometrico respecto del visual */
  odometrico[0]=0.;
  odometrico[1]=0.;
  odometrico[2]=0.;
  odometrico[3]= cos(0.);
  odometrico[4]= sin(0.);

  /*
  display_state=display_state | DISPLAY_ROBOT;
  fl_set_button(fd_guixforms->robot,PUSHED);
  encoders_cb=encoders_resume(NULL);

  display_state=display_state | DISPLAY_LASER;
  fl_set_button(fd_guixforms->laser,PUSHED);
  laser_cb=laser_resume(NULL);

  display_state=display_state | DISPLAY_SONARS;
  fl_set_button(fd_guixforms->sonars,PUSHED);
  sonars_cb=sonars_resume(NULL);
 
  display_state=display_state | DISPLAY_GREYIMAGEA;
  fl_set_button(fd_guixforms->greyimage,PUSHED);

  display_state=display_state | DISPLAY_COLORIMAGEA;
  fl_set_button(fd_guixforms->colorimage,PUSHED);
  */

  track_robot=TRUE;
  fl_set_button(fd_guixforms->track_robot,PUSHED);
  fl_set_button(fd_guixforms->myschema,RELEASED);

  fl_set_slider_bounds(fd_guixforms->escala,RANGO_MAX,RANGO_MIN);
  fl_set_slider_filter(fd_guixforms->escala,rangoenmetros); /* Para poner el valor del slider en metros en pantalla */
  fl_set_slider_value(fd_guixforms->escala,RANGO_INICIAL);
  escala = width/rango;

  fl_set_positioner_xvalue(fd_guixforms->joystick,0.5);
  fl_set_positioner_yvalue(fd_guixforms->joystick,0.);
  joystick_x=0.5;
  joystick_y=0.5;

  fl_set_slider_value(fd_guixforms->ptspeed,(double)(1.-latitude_speed/MAX_SPEED_PANTILT));

  fl_add_canvas_handler(fd_guixforms->micanvas,ButtonRelease,button_released_on_micanvas,NULL);

  pthread_mutex_lock(&mymutex[SCH_GUIXFORMS]);
  printf("guixforms schema started up\n");
  printf("guixforms on\n");
  state[SCH_GUIXFORMS]=active;
  pthread_create(&schema[SCH_GUIXFORMS],NULL,guixforms_thread,NULL);
  pthread_mutex_unlock(&mymutex[SCH_GUIXFORMS]);
}

void guixforms_resume()
{
  pthread_mutex_lock(&mymutex[SCH_GUIXFORMS]);
  if (debug[SCH_GUIXFORMS]) printf("guixforms on\n");
  state[SCH_GUIXFORMS]=active;
  /*  fl_set_form_position(fd_guixforms->guixformsf,200,50);*/
  fl_show_form(fd_guixforms->guixformsf,FL_PLACE_POSITION,FL_FULLBORDER,"Basic JDE-2");
  pthread_cond_signal(&condition[SCH_GUIXFORMS]);
  pthread_mutex_unlock(&mymutex[SCH_GUIXFORMS]);
}

void guixforms_suspend()
{
  pthread_mutex_lock(&mymutex[SCH_GUIXFORMS]);
  if (debug[SCH_GUIXFORMS]) printf("guixforms off\n");
  state[SCH_GUIXFORMS]=slept;
  /*free(imagenA_buf);
    free(imagenB_buf);*/
  fl_hide_form(fd_guixforms->guixformsf);
  pthread_mutex_unlock(&mymutex[SCH_GUIXFORMS]);
}


void guixforms_shutdown()
{
  /*pthread_mutex_lock(&mymutex[SCH_GUIXFORMS]);*/
  state[SCH_GUIXFORMS]=slept;
  /*pthread_mutex_unlock(&mymutex[SCH_GUIXFORMS]);*/
  fl_hide_form(fd_guixforms->guixformsf);
  fl_free_form(fd_guixforms->guixformsf);
}
