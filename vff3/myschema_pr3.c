#include "jde.h"
#include <math.h>
#include "guixforms.h"
void agrupa_sensores(int sensores[]);

int myschema_brothers[NUM_SCHEMAS];
arbitration myschema_callforarbitration;
int myschema_cycle=100; /* ms */

enum myschema_states {init,t1,r1,t2,r2,t3,r3,t4,end};
static int myschema_state;

void agrupa_sensores(int sensores[])
{
	/* Agrupamos todos los sensores en grupos de 20 para dar 9 grupos
		y cogemos el valor más pequeño de cada grupo*/

	int i;
	int sumatorio;
	for (i=0;i<=180;i++)
	{
		if ((i>=0) && (i<20)){
	/*	GRUPO 0*/
			if (i==0)
				sumatorio=0;
			else
				sumatorio+=laser[i];
		}
	/*	GRUPO 1*/
			if ((i>=20) && (i<40)){
				if (i==20){
					/* Metemos el valor medio al grupo anterior	*/
					sumatorio=sumatorio/20;
					sensores[0]=sumatorio;
					sumatorio=0;
				} else 
					sumatorio+=laser[i];
		}
	/*	GRUPO 2*/
			if ((i>=40) && (i<60)){
				if (i==40){
					/* Metemos el valor medio al grupo anterior	*/
					sumatorio=sumatorio/20;
					sensores[1]=sumatorio;
					sumatorio=0;
				} else
					sumatorio+=laser[i];
		}
	/*	GRUPO 3*/
			if ((i>=60) && (i<80)){
				if (i==60){
					/* Metemos el valor medio al grupo anterior	*/
					sumatorio=sumatorio/20;
					sensores[2]=sumatorio;
					sumatorio=0;
				} else
					sumatorio+=laser[i];
		}
	/*	GRUPO 4*/
			if ((i>=80) && (i<100)){
				if (i==80){
					/* Metemos el valor medio al grupo anterior	*/
					sumatorio=sumatorio/20;
					sensores[3]=sumatorio;
					sumatorio=0;
				} else
					sumatorio+=laser[i];
		}
	/*	GRUPO 5*/
			if ((i>=100) && (i<120)){
				if (i==100){
					/* Metemos el valor medio al grupo anterior	*/
					sumatorio=sumatorio/20;
					sensores[4]=sumatorio;
					sumatorio=0;
				} else
					sumatorio+=laser[i];
		}
	/*	GRUPO 6*/
			if ((i>=120) && (i<140)){
				if (i==120){
					/* Metemos el valor medio al grupo anterior	*/
					sumatorio=sumatorio/20;
					sensores[5]=sumatorio;
					sumatorio=0;
				} else
					sumatorio+=laser[i];
		}
	/*	GRUPO 7*/
			if ((i>=140) && (i<160)){
				if (i==140){
					/* Metemos el valor medio al grupo anterior	*/
					sumatorio=sumatorio/20;
					sensores[6]=sumatorio;
					sumatorio=0;
				} else
					sumatorio+=laser[i];
		}
	/*	GRUPO 8*/
			if ((i>=160) && (i<180)){
				if (i==160){
					/* Metemos el valor medio al grupo anterior	*/
					sumatorio=sumatorio/20;
					sensores[7]=sumatorio;
					sumatorio=0;
				} else
					sumatorio+=laser[i];
		}

			if (i==180){
					sumatorio=sumatorio/20;
					sensores[8]=sumatorio;
			}
	}
}

void calcula_fuerza_repulsiva(int sensores[],int grupo,int f_repulsiva[])
{
/* Para un grupo de sensores, te calcula su fuerza repulsiva*/
/* Modulo = Inversamente proporcional a la distancia*/
/* Angulo = 90 + angulo que abarca*/
	f_repulsiva[0] = 7600 / sensores[grupo];
	f_repulsiva[1] = (20 * grupo) + 190;

	if (f_repulsiva[0]<3)
		f_repulsiva[0]=0;
}

void 	suma_fuerzas(int fuerza_pri[],int fuerza_seg[],int fuerza_sum[])
{
/* Dadas dos fuerzas, calcula la suma de ellas*/

	/* Componente de la fuerza en base ortonormal*/
	/* x = Modulo * cos(angulo)		*/
	/* y = Modulo * seno(angulo)	*/
	double f_ortonormal[1];
	double x1; /* X de Fuerza 1	*/
	double y1; /* y de Fuerza 1	*/
	double x2; /* X de Fuerza 2	*/
	double y2; /* Y de Fuerza 2	*/
	double alfa1; /* Angulo en radianes de Fuerza 1	*/
	double alfa2; /* Angulo en radianes de Fuerza 2	*/
	double alfa3; /* Angulo en radianes de Fuerza suma */

	/*Pasamos los vectores a coordenadas*/
	alfa1 = (fuerza_pri[1] * M_PI) / 180;
	x1 = fuerza_pri[0] * cos(alfa1);
	y1 = fuerza_pri[0] * sin(alfa1);

	alfa2 = (fuerza_seg[1] * M_PI) / 180;
	x2 = fuerza_seg[0] * cos(alfa2);
	y2 = fuerza_seg[0] * sin(alfa2);
	
	/*Sumamos coordenadas*/
	f_ortonormal[0] = x1 + x2;
	f_ortonormal[1] = y1 + y2;
	
	/* Modulo por pitagoras a²=x²+y² */
	fuerza_sum[0] = sqrt( (f_ortonormal[0] * f_ortonormal[0]) + (f_ortonormal[1] * f_ortonormal[1]));
	/* Angulo por ecuacion  cos(angulo)= X / Modulo */
	alfa3 = atan2(f_ortonormal[1],f_ortonormal[0]);
	fuerza_sum[1] = (180 * alfa3)/M_PI;
	if (fuerza_sum[1]<0)
		fuerza_sum[1] = 180 + (180 + fuerza_sum[1]);
}

void 	calcula_fuerza_repulsiva_total(int sensores[],int f_repulsiva_total[])
{
	/* Calcula todas las fuerzas repulsiva para el robot, suma todas y te da la
	fuerza repulsiva total*/
	int e;
	int grupo;

	int f_repulsiva[1];

	/* inicializamos la fuerza total */
	f_repulsiva_total[0]=0;
	f_repulsiva_total[1]=0;

	for (e=0;e<=8;e++)
	{
		grupo = e;
		printf("Calculamos fuerza repulsiva para grupo %i",grupo);
		calcula_fuerza_repulsiva(sensores,grupo,f_repulsiva);
		printf(" vale [%i,%i]\n",f_repulsiva[0],f_repulsiva[1]);
		suma_fuerzas(f_repulsiva_total,f_repulsiva, f_repulsiva_total);
	}
	printf("Fuerza repulsiva total: [%i,%i]\n",f_repulsiva_total[0],f_repulsiva_total[1]);
}

void pinta_fuerzas(int f_atractiva[], int f_repulsiva[])
{
	/* Pinta Las fuerzas*/
	PintaX(50,50,100);

}

void 	calcula_fuerza_atractiva(int f_tractiva[])
{
/* No da una fuerza atractiva aleatoria*/
	f_atractiva[0] =30;
	f_atractiva[1] =90;
}

void mueve_robot(f_final[])
{
/* Dada la fuerza final, mueve las ruedas para que se mueva el robot hacia la dirección que
	indica la fuerza*/
	v = f_final[0]/f_final[1];
	w = f_final[1];

}


void myschema_iteration()
{  
	/************************************************
	 *					PROGRAMA PRINCIPAL									*		 
   ************************************************/
	int sensores[8];
	int i;
	int f_repulsiva[1];
	int f_atractiva[1];
	int f_final[1];

	agrupa_sensores(sensores);
	for (i=0;i<=8;i++)
		printf("grupo %i valor medio %i\n",i,sensores[i]);
	calcula_fuerza_repulsiva_total(sensores,f_repulsiva);
	calcula_fuerza_atractiva(f_atractiva);
	pinta_fuerzas(f_atractiva,f_repulsiva_total);
	suma_fuerzas(f_repulsiva_total,f_atractiva,f_final);
	mueve_robot(f_final);
	
}

void myschema_suspend()
{
  pthread_mutex_lock(&mymutex[SCH_MYSCHEMA]);
  state[SCH_MYSCHEMA]=slept;
  printf("myschema: off\n");
  pthread_mutex_unlock(&mymutex[SCH_MYSCHEMA]);
}

void myschema_resume(int *brothers, arbitration fn)
{
  int i;

  pthread_mutex_lock(&mymutex[SCH_MYSCHEMA]);
  for(i=0;i<NUM_SCHEMAS;i++) myschema_brothers[i]=-1;
  i=0;
  while(brothers[i]!=-1) {myschema_brothers[i]=brothers[i];i++;}

  myschema_callforarbitration=fn;
  put_state(SCH_MYSCHEMA,notready);
  printf("myschema: on\n");
  pthread_cond_signal(&condition[SCH_MYSCHEMA]);
  pthread_mutex_unlock(&mymutex[SCH_MYSCHEMA]);
}

void *myschema_thread(void *not_used) 
{
  struct timeval a,b;
  long diff, next;

  for(;;)
    {
      pthread_mutex_lock(&mymutex[SCH_MYSCHEMA]);

      if (state[SCH_MYSCHEMA]==slept) 
	{
	  /*printf("myschema: off\n");*/
	  v=0; w=0;
	  pthread_cond_wait(&condition[SCH_MYSCHEMA],&mymutex[SCH_MYSCHEMA]);
	  /*printf("myschema: on\n");*/

	  myschema_state=init;
	}
      else 
	{
	  gettimeofday(&a,NULL);
	  myschema_iteration();
	  gettimeofday(&b,NULL);  
	  diff = (b.tv_sec-a.tv_sec)*1000000+b.tv_usec-a.tv_usec;
	  
	  next = myschema_cycle*1000-diff-10000; 
	  /* discounts 10ms taken by calling usleep itself */
	  if (next>0) usleep(myschema_cycle*1000-diff);
	  else {printf("time interval violated: myschema\n"); usleep(myschema_cycle*1000);}
	}
      pthread_mutex_unlock(&mymutex[SCH_MYSCHEMA]);
    }
}

void myschema_startup()
{
  pthread_mutex_lock(&mymutex[SCH_MYSCHEMA]);
  printf("myschema schema started up\n");
  state[SCH_MYSCHEMA]=slept;
  pthread_create(&schema[SCH_MYSCHEMA],NULL,myschema_thread,NULL);
  pthread_mutex_unlock(&mymutex[SCH_MYSCHEMA]);
}

