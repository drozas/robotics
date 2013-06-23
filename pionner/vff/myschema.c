#include "jde.h"
#include <math.h>
#include "guixforms.h"
#include <stdlib.h>

/*Cabeceras de nuestras funciones*/
/*****************************************************************************/
void agrupa_sensores(int sensores[]);
int obtener_peso_grupo(int grupo);
void calcula_fuerza_repulsiva(int sensores[],int grupo,int f_repulsiva[]);
void 	suma_fuerzas(int fuerza_pri[],int fuerza_seg[],int fuerza_sum[]);
void 	calcula_fuerza_repulsiva_total(int sensores[],int f_repulsiva_total[]);
double cambia_angular(int f_inicial);
int dame_caso (int angulo);
void mueve_robot(int f_final[]);
void calcula_fuerza_atractiva_total(int f_atractiva[]);
/****************************************************************************/


int myschema_brothers[NUM_SCHEMAS];
arbitration myschema_callforarbitration;
int myschema_cycle=100; /* ms */

enum myschema_states {init,t1,r1,t2,r2,t3,r3,t4,end};
static int myschema_state;


/*Agrupa los sensores en 9 grupos de 20 laseres cada uno.
  Toma el valor mínimo de cada grupo*/
void agrupa_sensores(int sensores[])
{
	int i,j,total,min_actual;

	/*Cargamos con el valor minimo, en lugar de la media */
	min_actual = 7600;
	for(i=1, j=0, total=0; i<=180; i++)
	{
		if(min_actual>laser[(i-1)])
			min_actual = laser[(i-1)];
		
		if (i%20==0)
		{
			sensores[j] = min_actual;
			min_actual = 7600;
			j++;
		}
	}

}


/*Otorga un peso a cada grupo de laseres, para
  regular el factor que controla la velocidad lineal*/
int obtener_peso_grupo(int grupo)
{
	int peso;

			/*Orientación
				0-> + a drcha
				8-> + a izda*/

			/*Pesos :
				+ fuerza a los centrales*/

			switch (grupo) {
			case 0: 
				peso = 0.0;
				break;
			case 1: 
				peso = 1.2;
				break;
			case 2: 
				peso = 1.1;
				break;
			case 3: 
				peso = 1.7;
				break;
			case 4: 
				peso = 2.0;
				break;
			case 5: 
				peso = 1.2;
				break;
			case 6: 
				peso = 1.7;
				break;
			case 7: 
				peso = 1.0;
				break;
			case 8: 
				peso = 0.0;
		}

		return peso;

}


/*Calcula el vector de fuerza repulsiva de un grupo de sensores*/
void calcula_fuerza_repulsiva(int sensores[],int grupo,int f_repulsiva[])
{
/* Modulo = Inversamente proporcional a la distancia*/
/* Angulo = 90 + angulo que abarca*/
	double peso;

	/*Se añade funcion que otorga pesos simetricos al grupo */
	peso = obtener_peso_grupo(grupo);

	f_repulsiva[0] = (peso * 7600) / sensores[grupo];

	f_repulsiva[1] = (20 * grupo) + 190;

	/*Valores optimos, de 3..5 (Anula ese vector)*/
	if (f_repulsiva[0]<5)
		f_repulsiva[0]=0;
}

/*Suma dos vectores, pasandolos previamente a coordenadas.
  Devuelve el vector suma resultante.*/
void 	suma_fuerzas(int fuerza_pri[],int fuerza_seg[],int fuerza_sum[])
{
	/* Componente de la fuerza en base ortonormal*/
	/* x = Modulo * cos(angulo)		*/
	/* y = Modulo * seno(angulo)	*/
	double f_ortonormal[2];
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
	
	/*Obtenemos el vector suma en coordenadas*/
	f_ortonormal[0] = x1 + x2;
	f_ortonormal[1] = y1 + y2;
	
	/* De dichas coordenadas, obtenemos el módulo del vector suma por pitágoras a²=x²+y² */
	fuerza_sum[0] = sqrt( (f_ortonormal[0] * f_ortonormal[0]) + (f_ortonormal[1] * f_ortonormal[1]));
	/* Y el ángulo del vector suma, mediante la función arcotangente*/
	alfa3 = atan2(f_ortonormal[1],f_ortonormal[0]);
	/*Y lo pasamos de radianes a grados*/
	fuerza_sum[1] = (180 * alfa3)/M_PI;

	/*Si es menor que 0, hay que sumarle 360º*/
	if (fuerza_sum[1]<0)
		fuerza_sum[1] = 360 + fuerza_sum[1];
}


/*Devuelve el vector suma repulsivo total, a partir de las medidas de cada grupo de láseres*/
void 	calcula_fuerza_repulsiva_total(int sensores[],int f_repulsiva_total[])
{
	int e;
	int grupo;

	int f_repulsiva[2];

	/* inicializamos la fuerza total */
	f_repulsiva_total[0]=0;
	f_repulsiva_total[1]=0;

	for (e=0;e<=8;e++)
	{
		grupo = e;
		calcula_fuerza_repulsiva(sensores,grupo,f_repulsiva);
		suma_fuerzas(f_repulsiva_total,f_repulsiva, f_repulsiva_total);
	}
}

/*Cambia los angulos a la forma en que los entiende el robot*/
double cambia_angular(int f_inicial)
{
	double angulo;
	double f_final;

	/* Trasladamos los ejes 90º*/
	angulo = f_inicial - 90;

	/*Si el angulo es negativo, hay q sumarle 360*/
	if (angulo<0)
		angulo = angulo + 360;

	/*Si el angulo resultante, esta en la izda, permanece igual.
	 Si está en la derecha, le restamos 360*/

	if (angulo<=180)
		f_final = angulo;
	else
		f_final = angulo - 360;

	printf("Angulo con cambio [m.robot] = %f\n",f_final);

	return f_final;
}

/*Analiza la situación en la que se encuentra el robot
  respecto a su orientación*/		
int dame_caso (int angulo)
{
	int caso;

/*Valores optimos de primer tramo...entre 8..15
	Valores optimos de segundo tramo max(t1)..20-35*/
	if (abs(angulo)<=10)
	{
		caso = 3;
	}else if( abs(angulo>10) && abs(angulo<=35)){
		caso = 2;
	}else{
		caso = 1;
	}

	return caso;

}

/*Realiza el movimiento del robot*/
void mueve_robot(int f_final[])
{
	double fangular = 0.0;
	int factor_lineal = 300;
	int factor_angular = 1;
	int critico = 0;
	int caso = 0;

	fangular = cambia_angular(f_final[1]);

		caso = dame_caso(fangular);

	printf("Factor de v.lineal : ");

		switch (caso) {
			case 3: 
				factor_lineal = 65;
				printf("Factor maximo!\n");
				break;

			case 2:
				factor_lineal = 40;
				printf("Factor / 2!\n");
				break;

			default:
				/*v = 0; */
				factor_lineal = 0;
				v = 0;
				printf("Paramos !\n");
		}

		/* Por si fangular es 0 */
		if (fangular == 0) {
			v = f_final[0] * factor_lineal;
		}
		else{
			/*v = (f_final[0] * factor_lineal) / abs(fangular);*/
			v = f_final[0] * factor_lineal;
		}

	w = fangular / factor_angular;
	printf("Fuerza con cambio [%i,%f]\n",f_final[0],fangular);
	printf("Vel. lineal %f   Vel. Angular %f\n",v,w);

}

/*Devuelve el vector de fuerza atractiva total*/
void calcula_fuerza_atractiva_total(int f_atractiva[])
{
	f_atractiva[0]=30;
	f_atractiva[1]=90;
}


void myschema_iteration()
{  
	/************************************************
	 *					PROGRAMA PRINCIPAL									*		 
   ************************************************/
	int sensores[8];
	int i;
	int f_repulsiva[2];
	int f_atractiva[2];
	int f_final[2];


/* Inicializamos fuerzas */

	f_repulsiva[0] = 0;
	f_repulsiva[1] = 0;

	f_final[0] = 0;
	f_final[1] = 0;

	agrupa_sensores(sensores);

	for (i=0;i<=8;i++)
		printf("Grupo %i . Distancia minima :  %i\n",i,sensores[i]);

	calcula_fuerza_repulsiva_total(sensores,f_repulsiva);

	calcula_fuerza_atractiva_total(f_atractiva);

	printf("F. Atractiva [%i,%i] F. Repulsiva [%i,%i] ",f_atractiva[0],f_atractiva[1],f_repulsiva[0],f_repulsiva[1]);
	/* pinta_fuerzas(f_atractiva,f_repulsiva_total);*/

	suma_fuerzas(f_repulsiva,f_atractiva,f_final);
	printf("F. Final [%i,%i]\n",f_final[0],f_final[1]);

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

