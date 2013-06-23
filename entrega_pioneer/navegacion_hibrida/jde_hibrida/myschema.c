/******************************************************************************/
/* 	PRACTICA 5 DE PIONEER : Navegación híbrida  			*/
/* Sergio Pinilla
/* David Rozas
/******************************************************************************/
#include "jde.h"
#include <math.h>
#include "guixforms.h"
#include <stdlib.h>

/*Cabeceras de nuestras funciones*/
/*****************************************************************************/
void 	agrupa_sensores(int sensores[]);
double 	obtener_peso_grupo(int grupo);
void 	calcula_fuerza_repulsiva(int sensores[],int grupo,double f_repulsiva_aux[]);
void 	suma_fuerzas(double fuerza_pri[],double fuerza_seg[],double fuerza_sum[]);
double cambia_angular(int f_inicial);
int dame_caso(double angulo);

void 	calcula_fuerza_repulsiva_total(int sensores[]);
void 	calcula_fuerza_atractiva_total();
void 	mueve_robot();
void inicializar_lista();
/****************************************************************************/


int myschema_brothers[NUM_SCHEMAS];
arbitration myschema_callforarbitration;
int myschema_cycle=100; /* ms */

enum myschema_states {init,t1,r1,t2,r2,t3,r3,t4,end};
static int myschema_state;


int inicializado = 0;
int pto_actual = 0; /*Esta sera global! con el pto actual */


/*Inicializa la lista de puntos que marca la ruta*/
void inicializar_lista()
{
	
	/*Ruta por los departamentales*/
	lista[0].x = 25400;
	lista[0].y = 12750;

	lista[1].x = 25200;
	lista[1].y = 700;

	lista[2].x = 5500;
	lista[2].y = 700;

	lista[3].x = 5500;
	lista[3].y = 11900;

	lista[4].x = 1300;
	lista[4].y = 11700;
}

/* Guarda en las coordenadas globales el siguiente punto. Devuelve 0
   si todavía quedan puntos, uno en caso de que no*/
int dame_puntos()
{
	double distancia = 0.0;
   //Si me quedan puntos.
	printf("Pto_actual = %i\n",pto_actual);
    
	if (pto_actual<N_PUNTOS)
    	{
        	/*Nos da los puntos de destino y origen */
			distancia = sqrt( ((destino.x-robot[0]) * (destino.x-robot[0])) + ((destino.y-robot[1])*(destino.y-robot[1])));
	      if (distancia<600)
	       {
					printf("******* Cambio de punto *******\n");
					pto_actual++;
					destino = lista[pto_actual];
			}
        	return 0; //Devolvemos 0 si aun nos quedan ptos que calcular       
    	}else{
       	 return 1; //Si ya no hay puntos, devolvemos un 1 al p.ppal.
    	}
}


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
double obtener_peso_grupo(int grupo)
{
	double peso;

			/*Orientación
				0-> + a drcha
				8-> + a izda*/

	switch (grupo) {
			case 0: 
				peso = 0.2;
				break;
			case 1: 
				peso = 3.0;
				break;
			case 2: 
				peso = 4.0;
				break;
			case 3: 
				peso = 5.0;
				break;
			case 4: 
				peso = 1.9;
				break;
			case 5: 
				peso = 5.0;
				break;
			case 6: 
				peso = 4.0;
				break;
			case 7: 
				peso = 3.0;
				break;
			case 8: 
				peso = 0.2;
		}

		return peso;
}


/*Calcula el vector de fuerza repulsiva de un grupo de sensores*/
void calcula_fuerza_repulsiva(int sensores[],int grupo,double f_repulsiva_aux[])
{
/* Modulo = Inversamente proporcional a la distancia*/
/* Angulo = 90 + angulo que abarca*/
	double peso;

	/*Se añade funcion que otorga pesos simetricos al grupo */
	peso = obtener_peso_grupo(grupo);

	f_repulsiva_aux[0] = (peso * 7600) / sensores[grupo];

	f_repulsiva_aux[1] = (20 * grupo) + 190;
}

/*Suma dos vectores, pasandolos previamente a coordenadas.
  Devuelve el vector suma resultante.*/
void 	suma_fuerzas(double fuerza_pri[],double fuerza_seg[],double fuerza_sum[])
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
void 	calcula_fuerza_repulsiva_total(int sensores[])
{
	int e;
	int grupo;

	double f_repulsiva_aux[2];
	double f_repulsiva_total[2],f_repulsiva_sumatoria[2];

	/* inicializamos la fuerza total */
	f_repulsiva_aux[0]=0;
	f_repulsiva_aux[1]=0;
	f_repulsiva_total[0] = 0;
	f_repulsiva_total[1] = 0;

	for (e=0;e<=8;e++)
	{
		grupo = e;
		calcula_fuerza_repulsiva(sensores,grupo,f_repulsiva_aux);
		suma_fuerzas(f_repulsiva_total,f_repulsiva_aux, f_repulsiva_sumatoria);
		f_repulsiva_total[0] = f_repulsiva_sumatoria[0];
		f_repulsiva_total[1] = f_repulsiva_sumatoria[1];

	}
	f_repulsiva[0] = f_repulsiva_total[0]/3.0;
	f_repulsiva[1] = f_repulsiva_total[1];
	
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


	return f_final;
}

/*Analiza la situación en la que se encuentra el robot
  respecto a su orientación*/		
int dame_caso (double angulo)
{
	int caso;

/*Valores optimos de primer tramo...entre 8..15
	Valores optimos de segundo tramo max(t1)..20-35*/

	printf("Angulo de caso : = %f \n", angulo);
	
	/*Paramos v y w si hemos llegado*/ /*OJITO AHORA AQUI, QUE SI NO SE ACHANTA*/
	if((f_final[0]<1) || (f_atractiva[0]<5)){
		caso = 0;
	}else if ( f_repulsiva[0] > 70){
		/*Si la fuerza repulsiva es muy grande, paramos solo la v_lineal, pero seguimos girando*/
		caso = 1;
		printf("caso 1 por f.repulsiva>70\n");
	}else{
		if ((angulo<5) && (angulo>-5))
		{
			caso = 3;
		}else if( (angulo>5 && angulo<=160) || (angulo <-5 && angulo>-160) ){
			caso = 2;
		}else{
			caso = 1;
		printf("caso 1 por angulo\n");
		}
	}

	printf("Devolvemos caso %i \n", caso);

	return caso;

}

/*Realiza el movimiento del robot*/
void mueve_robot()
{
	double fangular = 0.0;
	int factor_lineal = 10;
	int factor_angular = 1;
	int caso = 0;

	fangular = cambia_angular(f_final[1]);


			caso = dame_caso(fangular);

			switch (caso) {
				case 3: 
					factor_lineal = 13;
					break;
	
				case 2:
					factor_lineal = 2.5;
					factor_angular = 1.0;
					break;
	
				case 1:
					factor_lineal = 0;
					factor_angular =1.0;
			}

	v = f_final[0] * factor_lineal;

	if (abs(fangular)<90){
		w = fangular * factor_angular;
	}else {
		if (fangular<0)
			w=-90;
		else
			w=90;
	}

	printf("Vel lineal = %f  Vel Angular = %f\n",v,w);
	printf("Fuerza con cambio [%f,%f]\n",f_final[0],fangular);
	printf("Vel. lineal %f   Vel. Angular %f\n",v,w);

}

/*Devuelve el vector de fuerza atractiva total*/
void calcula_fuerza_atractiva_total()
{
	double alfa,alfa_gr;
	double modulo;

	/* Actualizamos el valor de las x y de la fuerza atractiva con respecto al robot para que lo pinte */
	x_fatrac = destino.x - robot[0];
	y_fatrac = destino.y - robot[1];


	/* Modulo por pitagoras a²=x²+y² */
	modulo = sqrt( (x_fatrac * x_fatrac) + (y_fatrac * y_fatrac));
	modulo = (modulo / 76) * 6.0;
	f_atractiva[0] = modulo;

	/* Angulo primero averiguamos en angulo absoluto con cambio de ejes x->y */
	alfa = atan2(y_fatrac,x_fatrac);

	/* Angulo será alfa - lo que se ha movido el robot*/
	alfa = alfa - robot[2];
	alfa = alfa + (M_PI/2);

	alfa_gr = (180 * alfa)/M_PI;

	/* Cambiamos a grados */
	if (alfa_gr<0)
		alfa_gr = 180 + (180 + alfa_gr);
	f_atractiva[1] = alfa_gr;
}


void myschema_iteration()
{  
		/***********************************/
		/*        PROGRAMA PRINCIPAL       */
		/***********************************/
	int sensores[9];
	int i;
	int fin;
	double distancia = 0.0;
	double f_atractiva_modulada[2];
	
	if (inicializado == 0)
	{
		printf("*******INICIALIZAMOS LAS VARIABLES*********");
		/* Inicializo */
		inicializar_lista();
		destino = lista[0];
		inicializado = 1;
	}

		/* Mientras haya puntos */
		fin = dame_puntos();
		if (fin == 0)
		{
			/*Calculamos la distancia actual respecto a ese punto*/
			distancia = sqrt( ((destino.x-robot[0]) * (destino.x-robot[0])) + ((destino.y-robot[1])*(destino.y-robot[1])));
			printf("Vamos hacia el punto %f %f \n", destino.x, destino.y);
			printf("Estamos a %f del punto destino \n", distancia);

			/*Si es mayor que cien, seguimos nuestro camino. Si no, consideramos que ya hemos llegado y pediremos el sig. pto.*/
			if(distancia>600)
			{

				/* Inicializamos fuerzas */
				f_repulsiva[0] = 0;
				f_repulsiva[1] = 0;

				f_final[0] = 0;
				f_final[1] = 0;

				/*Calculamos el vector total de fuerza repulsiva*/
				agrupa_sensores(sensores);
				for (i=0;i<=8;i++)
					printf("Grupo %i . Distancia minima :  %i\n",i,sensores[i]);
				
				calcula_fuerza_repulsiva_total(sensores);

				/*Calculamos el vector total de fuerza atractiva, para ese par de puntos (globales)*/
				calcula_fuerza_atractiva_total();

				f_atractiva_modulada[0] = f_atractiva[0];
				f_atractiva_modulada[1] = f_atractiva[1];

				/*Hemos aumentado el modulo internamente, pero ahora limitamos a un máximo de 80. Asi conseguimos
				que los valores altos no sean tan altos, y a la vez que los valores minimos nos den una f.repulsiva
				que pueda compensar a la atractiva*/
				if(f_atractiva_modulada[0]>80)
					f_atractiva_modulada[0] = 80;

				printf("F. Atractiva [%f,%f]. F Atractiva_modulada [%f,%f] F. Repulsiva [%f,%f] ",f_atractiva[0],f_atractiva[1], 					f_atractiva_modulada[0], f_atractiva_modulada[1] ,f_repulsiva[0],f_repulsiva[1]);


				suma_fuerzas(f_atractiva_modulada,f_repulsiva,f_final);
				printf("F. Final [%f,%f]\n",f_final[0],f_final[1]);

				mueve_robot();
			}else{

				printf("Ya hemos llegado a %f,%f . A por el siguiente punto!\n", destino.x,destino.y);
				v=0;
			}

		}
		else
		{
				printf("**************** Hemos llegado al final del recorrido **************** \n");
				v=0;
				w=0;
				exit(0);
		}

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

