/* ********************************
   ROBÓTICA : PRÁCTICA 2 
   "Robot que coge latas"
***********************************
* David Rozas Domingo
* Sergio Pinilla Melo
* Caja 23
**********************************/

#include <config.h>
#if defined(CONF_DSENSOR) && defined(CONF_DMOTOR)

#include <conio.h>
#include <unistd.h>
#include <tm.h>


#include <dsensor.h>
#include <dmotor.h>

#define NORMAL_SPEED (100)
#define BUSQUEDA_SPEED (200)
#define PINZAS_SPEED (255)
#define DARK_THRESH     0x40
#define BRIGHT_THRESH   0x48
#define LIGHTSENS       SENSOR_2

int dir = 0;
int MAHOU = 0;
int final=0;

static wakeup_t sensor_pulsado(wakeup_t data) {
	return TOUCH_3;
}
static void abrir_pinzas()
{
  	motor_b_speed(PINZAS_SPEED);
	motor_b_dir(rev);
}
static void cerrar_pinzas()
{
  	motor_b_speed(PINZAS_SPEED);
	motor_b_dir(fwd);
}

int buscar_mahou()
{
	abrir_pinzas();
	/*Bloqueo hebra mientras no encuentre lata*/
   wait_event(sensor_pulsado,0);
	
   cerrar_pinzas();
	MAHOU = 1; 
   exit(0);
}



static wakeup_t bright_found(wakeup_t data) 
{

	return ((LIGHT_2 > 36) || (MAHOU ==1));

}
static wakeup_t black_found(wakeup_t data) 
{

	return (LIGHT_2 < 36);

}
static void locate_line() {
  	int grad=5;
  	int i=0;
	int cambioDir =0;
  	lcd_refresh();
  	cputs("locate");
  	motor_a_dir(brake);
	motor_c_dir(brake);
			if (dir==0)
			{
				/*Empiezo mirando hacia la derecha*/
				lcd_refresh();
				cputs("der");
  				motor_a_speed(BUSQUEDA_SPEED);
  				motor_c_speed(BUSQUEDA_SPEED);
				motor_a_dir(rev);
				motor_c_dir(rev);
				dir = 1;
			}else{
				/*Empiezo mirando hacia la izda*/
				lcd_refresh();
				cputs("izq");
			  	motor_a_speed(BUSQUEDA_SPEED);
  				motor_c_speed(BUSQUEDA_SPEED);
  				motor_c_dir(fwd);
				motor_a_dir(fwd);
				dir = 0;
			}
	
	while ((LIGHT_2 > 36) && (final!=1)){
   	msleep(20);
		/*Comprobamos si hay que cambiar la direccion*/
		if (i==grad)
		{
			cambioDir++;
			if ((cambioDir==5) && (MAHOU=2)){
  				motor_a_dir(off);
				motor_c_dir(off);
				final=1;
			}else{
				/*Si hay que cambiar la direccion*/
				if (dir==0)
				{
				/*Cambio a derecha*/
				lcd_refresh();
				cputs("der");
  				motor_a_dir(brake);
				motor_c_dir(brake);
  				motor_a_speed(BUSQUEDA_SPEED);
  				motor_c_speed(BUSQUEDA_SPEED);
				motor_a_dir(rev);
				motor_c_dir(rev);
				dir = 1;
				}else{

				/*Cambio a izda*/
				lcd_refresh();
				cputs("izq");
				motor_a_dir(brake);
				motor_c_dir(brake);
			  	motor_a_speed(BUSQUEDA_SPEED);
  				motor_c_speed(BUSQUEDA_SPEED);
  				motor_c_dir(fwd);
				motor_a_dir(fwd);
				dir = 0;
				}
			}
			/*Reinicializamos ind de busqueda, y aumentamos grados*/
			i = 0;
			grad = grad * 2.0;

		}else{
		
		/*Si no hay q cambiar direccion, nos limitamos a aumentar indice*/
		i++;

		}

	  }	
}


static void follow_line() {


  while ((!shutdown_requested()) && (final==0)) {
  		lcd_refresh();
  		cputs("seguir");
   	motor_a_speed(NORMAL_SPEED);
   	motor_c_speed(NORMAL_SPEED);
   	motor_a_dir(rev);
    	motor_c_dir(fwd);
    
		wait_event(bright_found,BRIGHT_THRESH);

		/*Si he encontrado la lata*/
		if (MAHOU==1) {
			/*Giramos hasta volver a encontrar linea negra*/
			motor_a_speed(BUSQUEDA_SPEED);
  			motor_c_speed(BUSQUEDA_SPEED);
			motor_a_dir(rev);
			motor_c_dir(rev);
			MAHOU=2;
			msleep(1000);
			wait_event(black_found,BRIGHT_THRESH);

		}else{
			locate_line();
		}
   }
	
}

int main(int argc, char *argv[]) {

	int tm;

	ds_active(&LIGHTSENS);
	tm = execi(&buscar_mahou,0,NULL,10,DEFAULT_STACK_SIZE);
   tm_start();
  	follow_line();
  	lcd_refresh();
  	cputs("salir");
	abrir_pinzas();
  	motor_b_dir(off);
  	ds_passive(&LIGHTSENS);	
  	kill(tm);

  	return 0;
}
#else
#warning linetrack.c requires CONF_DSENSOR and CONF_DMOTOR
#warning linetrack demo will do nothing
int main(int argc, char *argv[]) {
  return 0;
}
#endif 

