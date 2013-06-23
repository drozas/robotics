#include "jde.h"


int myschema_brothers[NUM_SCHEMAS];
arbitration myschema_callforarbitration;
int myschema_cycle=100; /* ms */

enum myschema_states {init,t1,r1,t2,r2,t3,r3,t4,end};
static int myschema_state;

void myschema_iteration()
{  
  static int d;

	int i;
	int pesos[8];
	int pesoslin[8];
	int min_distancia;
	int min_sensor;

	/*Sensores izda*/
	pesos[0] = 4;
	pesos[1] = 3;
	pesos[2] = 2;
	pesos[3] = 1;
	/*Sensores drcha*/
	pesos[4] = 3;
	pesos[5] = 2;
	pesos[6] = 3;
	pesos[7] = 4;

// Pesos para la velocidad lineal
	/*Sensores izda*/
	pesoslin[1] = 1;
	pesoslin[2] = 2;
	pesoslin[3] = 3;
	/*Sensores drcha*/
	pesoslin[4] = 3;
	pesoslin[5] = 2;
	pesoslin[6] = 1;


	v = 250;
	i = 0;
	
	
	/*Calculamos la distancia minima*/
	min_distancia = us[1];
	min_sensor = 1;
	for (i=1; i<7; i++)
	{
		if (min_distancia>us[i]){
			min_distancia = us[i];
			min_sensor = i;
			}
	}

	printf("La distancia minima es %i \n", min_distancia);

	/*Modulamos la velocidad en funcion de esta distancia minima */
	/*Ademas si la distancia minima es muy amplia, "centramos"*/
	if(min_distancia<=3000 && min_distancia>=2500)
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
		v = 200/pesoslin[min_sensor];
	}else if(min_distancia<1000 && min_distancia>=500){
		v = 200/pesoslin[min_sensor];
	}else{
		v = 100/pesoslin[min_sensor];
	}

	for(i=0;i<=7;i++)
	{

		printf("sonar %i valor %f ",i,us[i]);
		// Miramos si ese sonar esta "pitando"
		// Sonares del lado izquierdo
		if (i<4)
		{
				if (us[i]<1000)
				{
					if (w>-30)
					{
						w-=pesos[i]*1;
						printf("hace saltar (LADO IZDO CRITICAMENTE) w=%f",w);
					}	
				}
				else if(us[i]<1500)
				{
					if (w>-30)
					{
						w-=pesos[i]*0.8;
						printf("hace saltar (LADO IZDO) w=%f",w);	
					}
				}

		}
	// Miramos si ese sonar esta "pitando"
	// Sonares del lado derecho
		if ((i>=4) && (i<=7))
		{
				
				if (us[i]<1000)
				{
					/*Controlamos una velocidad angular max, para que no pegue giros bruscos*/
					if(w<30)
					{
						w+=pesos[i]*1;
						printf("hace saltar(LADO DRCHO CRITICAMENTE) w=%f",w);	
					}
				}
				else if(us[i]<1500)
				{
					if(w<30)
					{
						w+=pesos[i]*0.8;
						printf("hace saltar(LADO DRCHO) w=%f",w);	
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

