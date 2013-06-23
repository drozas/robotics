/**********************************************************/
/*	Practica 2 : Navegacion con laseres */
/*********************************************************/

#include "jde.h"


int myschema_brothers[NUM_SCHEMAS];
arbitration myschema_callforarbitration;
int myschema_cycle=100; /* ms */

enum myschema_states {init,t1,r1,t2,r2,t3,r3,t4,end};
static int myschema_state;

void myschema_iteration()
{  
  static int d;

	double laseres_agrupados[10];
	int i,j,total;
	float pesos[10];
	float pesoslin[8];
	int min_distancia;
	int min_sensor;

	/*Cargamos con sus medias en 10 grupos de 18 */
	for(i=1, j=0, total=0; i<=180; i++)
	{
		total+= laser[(i-1)];
		if (i%18==0)
		{
			laseres_agrupados[j] = total/18;
			total = 0;
			j++;
		}
	}

	for(i=0; i<10; i++)
	{
		printf("Valor de grupo %i : %f \n", i, laseres_agrupados[i]);
	}

/* Asignación de pesos para velocidad angular*/
	/*Sensores drcha*/
	pesos[0] = 3;
	pesos[1] = 2.5;
	pesos[2] = 2;
	pesos[3] = 1.5;
	pesos[4] = 1;
	/*Sensores izda*/
	pesos[9] = 3;
	pesos[8] = 2.5;
	pesos[7] = 2;
	pesos[6] = 1.5;
	pesos[5] = 1;
/* Pesos para la velocidad lineal*/
	/*Sensores drcha*/
	pesoslin[0] = 0;
	pesoslin[1] = 0;
	pesoslin[2] = 1;
	pesoslin[3] = 2;
	pesoslin[4] = 3;
	/*Sensores izda*/
	pesoslin[5] = 3;
	pesoslin[6] = 2;
	pesoslin[7] = 1;
	pesoslin[8] = 0;
	pesoslin[9] = 0;

/*	v = 250;*/
	
	/*Calculamos la distancia minima de los subgrupos "más centrales"*/
	min_distancia = laseres_agrupados[2];
	min_sensor = 2;
	for (i=2; i<=7; i++)
	{
		if (min_distancia>laseres_agrupados[i])
		{
			min_distancia = laseres_agrupados[i];
			min_sensor = i;
		}
	}

	printf("La distancia minima es %i \n", min_distancia);



	/*Modulamos la velocidad en funcion de esta distancia minima */
	/*Ademas si la distancia minima es muy amplia, "centramos"*/
	if(min_distancia>=2500)
	{
		v = MAX_VEL/pesoslin[min_sensor];
		w=0;
	}else if(min_distancia<2500 && min_distancia>=2000){
		v = MAX_VEL/pesoslin[min_sensor];
		w=0;
	}else if(min_distancia<2000 && min_distancia>=1500){
		v = 800/pesoslin[min_sensor];
		w=0;
	}else if(min_distancia<1500 && min_distancia>=1000){
		v = 400/pesoslin[min_sensor];
		w=0;
	}else if(min_distancia<1000 && min_distancia>=500){
		v = 200/pesoslin[min_sensor];
	}else if (min_distancia>500 && min_distancia>=100){
		v = 100/pesoslin[min_sensor];
	}else{
		printf("frenamos!\n");
		v = 0;
	}


	/* Modificamos la velocidad angular en función del grupo 
		afectado */
	for(i=0;i<=9;i++)
	{

		printf("grupo de laseres %i valor %f ",i,laseres_agrupados[i]);

		/*Grupo de sonares del lado derecho */
		if (i<=4)
		{
				if (laseres_agrupados[i]<1000)
				{
					if (w<20)
					{
						w+=pesos[i]*1;
						printf("hace saltar (LADO DERECHO CRITICAMENTE)");
					}	
				}
				else if(laseres_agrupados[i]<1500)
				{
					if (w<20)
					{
						w+=pesos[i]*0.8;
						printf("hace saltar (LADO DERECHO)");	
					}
				}
		}

		/* Sonares del lado izquierdo*/
		if ((i>4) && (i<=9))
		{
					
			if (laseres_agrupados[i]<1000)
			{
				/*Controlamos una velocidad angular max, para que no pegue giros bruscos*/
				if(w>-20)
				{
					w-=pesos[i]*1;
					printf("hace saltar(LADO IZQUIERDO CRITICAMENTE)");	
				}
			}
			else if(laseres_agrupados[i]<1500)
			{
				if(w>-20)
				{
					w-=pesos[i]*0.8;
					printf("hace saltar(LADO IZQUIERDO)");	
				}
			}

		}

		printf("\n");
	}

	printf("Velocidad angular = %f\n",w);	
	printf("La velocidad lineal es :  %f \n", v);

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

