#include "jde.h"



int inicializado = 0;
int pto_actual = 1; /*Esta sera global! con el pto actual */

void inicializar_lista()
{
	int i;
	
	/*lista[0].x = 1200;
	lista[0].y = 1498;
	lista[1].x = 2300;
	lista[1].y = 5000;
	lista[2].x = 3800;
	lista[2].y = 4500;
	lista[3].x = 2000;
	lista[3].y = 2000;
	lista[4].x = 2310;
	lista[4].y = 5010;
	lista[5].x = 2350;
	lista[5].y = 5020;
	lista[6].x = 2400;
	lista[6].y = 5100;
	lista[7].x = 1200;
	lista[7].y = 1498;*/


	lista[0].x = 1200;
	lista[0].y = 1498;
	lista[1].x = 3500;
	lista[1].y = 6700;
	lista[2].x = 5700;
	lista[2].y = 6700;
	lista[3].x = 5500;
	lista[3].y = 900;
	lista[4].x = 25200;
	lista[4].y = 900;
	lista[5].x = 25300;
	lista[5].y = 5200;
	lista[6].x = 25300;
	lista[6].y = 5200;
	lista[7].x = 22500;
	lista[7].y = 5200;

}

int myschema_brothers[NUM_SCHEMAS];
arbitration myschema_callforarbitration;
int myschema_cycle=100; /* ms */

enum myschema_states {init,t1,r1,t2,r2,t3,r3,t4,end};
static int myschema_state;

int dame_puntos()
{
		double distancia = 0.0;
    //Si me quedan puntos.
		printf("Pto_actual = %i\n",pto_actual);
    if (pto_actual<N_PUNTOS)
    {
        /*Nos da los puntos de destino y origen */
				distancia = sqrt( ((x_final-robot[0]) * (x_final-robot[0])) + ((y_final-robot[1])*(y_final-robot[1])));
        if (distancia<100)
       	{
					/* Comprobamos si hemos llegado al final */
					if (pto_actual == (N_PUNTOS-1)){
						pto_actual++;
						return 1;
					}
					else
					{
	          x_origen = lista[pto_actual].x;
  	        y_origen = lista[pto_actual].y;
  	        /* He llegado y dame el siguiente destino */
  	        pto_actual++;
  	        x_final=lista[pto_actual].x;
  	        y_final=lista[pto_actual].y;
					}
				}
        return 0; //Devolvemos 0 si aun nos quedan ptos que calcular       
    }else{
        return 1; //Si ya no hay puntos, devolvemos un 1 al p.ppal.
    }
}


float dame_lejania()
{
/* Calcula la lejania que está el robot de la linea de trayectoria	*/
/* el signo significa si esta por la derecha o por la izquierda   	*/
	float alfa,beta,gama;
	float x_final_cambio,y_final_cambio;
	float x_origen_cambio,y_origen_cambio;
	float distancia,modulo;
	float x_robot_cambio,y_robot_cambio;

		/* Hacemos un cambio de ejes para el cual el origen sea el punto origen */
		x_final_cambio = x_final - x_origen;
		y_final_cambio = y_final - y_origen;
	
		x_robot_cambio = robot[0] - x_origen;
		y_robot_cambio = robot[1] - y_origen;
	
		/* Calculamos el angulo que tiene la trayectoria */
		alfa = atan2(y_final_cambio,x_final_cambio);
	
		/* Calculamos el angulo que tiene el robot */
		beta = atan2(y_robot_cambio,x_robot_cambio);

		/* Restamos los angulos que nos dirá si estamos a la derecha o a la izquierda */
		gama = beta - alfa;

		/* Calculamos el distancia del orgien al robot */
		modulo = sqrt( (x_robot_cambio * x_robot_cambio) + (y_robot_cambio * y_robot_cambio));

		/* Calculamos la distancia del robot a la trayectoria */
		distancia = modulo * sin(gama);

		if (x_final>x_origen)
		{
				x_final_cambio = x_final + 100;
				x_origen_cambio = x_origen - 100;
		}
		else
		{
				x_final_cambio = x_final - 100;
				x_origen_cambio = x_origen + 100;
		}

		if (y_final>y_origen)
		{
				y_final_cambio = y_final + 100;
				y_origen_cambio = y_origen - 100;
		}
		else
		{
				y_final_cambio = y_final - 100;
				y_origen_cambio = y_origen + 100;
		}
				
					printf("robot[1]=%f, y_final=%f  y_origen=%f\n",robot[1],y_origen_cambio,y_final_cambio);
/* Miramos si esta en el cuadrado que corresponde a los puntos*/
	if (((robot[0]<x_final_cambio) && (robot[0]<x_origen_cambio)) || ((robot[0]>x_final_cambio) && (robot[0]>x_origen_cambio))){
		printf("Nos hemos pasado por las X\n");
		distancia = distancia * (sqrt( ((x_origen-robot[0]) * (x_origen-robot[0])) + ((y_origen-robot[1])*(y_origen-robot[1]))));
	}
	else
	{
		if (((robot[1]<y_final_cambio) && (robot[1]<y_origen_cambio)) || ((robot[1]>y_final_cambio) && (robot[1]>y_origen_cambio))){
			printf("Nos hemos pasado por las Y, robot[1]=%f, y_final=%f  y_origen=%f\n",robot[1],y_origen_cambio,y_final_cambio);			
			distancia = distancia * (sqrt( ((x_origen-robot[0]) * (x_origen-robot[0])) + ((y_origen-robot[1])*(y_origen-robot[1]))));
		}
	}
	return distancia;

}

float cambia_grados(float grados)
{
	float gr_cambio;
	if (grados < (M_PI))
			gr_cambio = grados;
	else
			gr_cambio = -((2*M_PI) - grados);
	return gr_cambio;


}

int dame_caso(float lejania)
{
	int caso;
		if ((lejania<100) && (lejania>-100))
				caso = 1;
		else
				caso = 2;
		return caso;
}

void sigue_trayecto(float direccion)
{
	float alfa_direc,alfa_robot;
	float direc_gr,robot_gr;
	float diferencia;
	alfa_direc = cambia_grados(direccion);
	alfa_robot = cambia_grados(robot[2]);

	/*Y lo pasamos de radianes a grados*/
	direc_gr = (180 * alfa_direc)/M_PI;
	robot_gr = (180 * alfa_robot)/M_PI;

	printf("direc_gr:%f   robot_gr:%f\n",direc_gr,robot_gr);

	/*Si es menor que 0, hay que sumarle 360º*/
	/*if (direc_gr<0)
		direc_gr = 360 + direc_gr;

	if (robot_gr<0)
		robot_gr = 360 + robot_gr;*/


	if (alfa_direc>alfa_robot)
	{
		/* Giramos a Izquierda */
		printf("Giramos a izquierda\n");
		w = +20;
	}
	if (alfa_direc<alfa_robot)
	{
		printf("Giramos a derecha\n");
		/* Giramos a Derecha */
		w = -20;
	}	
	diferencia = abs(direc_gr - robot_gr);
	printf("Resta: %f\n",diferencia);
	if (diferencia<5)
	{
		printf("Vamos recto\n");
		w = 0;
		v = v * 1.5;
	}
	if (diferencia > 45)
	{
		printf("Giramos * 3\n");
		w = w * 3;
	}

}

void busca_ruta(float alejania,float alfa)
{
	if (alejania>0)
		sigue_trayecto(alfa - (M_PI/6));
	else
		sigue_trayecto(alfa + (M_PI/6));
}

void myschema_iteration()
{  
		/***********************************/
		/*        PROGRAMA PRINCIPAL       */
		/***********************************/
	float lejania,x_final_cambio,y_final_cambio,distancia;
	int caso;
	int fin;
	float alfa;
	if (inicializado == 0)
	{
		printf("*******INICIALIZAMOS LAS VARIABLES*********");
		/* Inicializo */
		inicializar_lista();
		x_origen = lista[0].x;
		y_origen = lista[0].y;
		x_final = lista[1].x;
		y_final = lista[1].y;
		inicializado = 1;
	}
		fin = dame_puntos();
		if (fin == 0)
		{
			/* Calculamos el angulo de la trayectoria */
			x_final_cambio = x_final - x_origen;
			y_final_cambio = y_final - y_origen;

			alfa = atan2(y_final_cambio,x_final_cambio);

			lejania = dame_lejania();
			printf("Lejania %f\n",lejania);
			caso = dame_caso(lejania);
			printf("Caso = %i\n",caso);
			distancia = sqrt( ((x_final-robot[0]) * (x_final-robot[0])) + ((y_final-robot[1])*(y_final-robot[1])));
			if (caso==1){
				if (distancia < 200)
					v = 50;
				else
					v = 200;
				sigue_trayecto(alfa);
			}
			else{
				v = 50;
				busca_ruta(lejania,alfa);
			}
			printf("Velocidad Lineal = %f  Velocidad angular = %f\n",v,w);
		}
		else
		{
				printf("HEMOS LLEGADO AL FINAL");
				v=0;
				w=0;
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

