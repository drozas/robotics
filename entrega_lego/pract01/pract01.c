/* ********************************
   ROBÓTICA : PRÁCTICA 1 
   "Robot que sigue líneas negras
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
#define DARK_THRESH     0x40
#define BRIGHT_THRESH   0x48
#define LIGHTSENS       SENSOR_2

// #define STRAIGHT_LINE
int dir = 0;

static wakeup_t bright_found(wakeup_t data) {
return LIGHT_2 > 36;
}

static void locate_line() {
  	int grad=1000;
  	int i=0;
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
	
	while (LIGHT_2 > 36){
   	
		/*Comprobamos si hay que cambiar la direccion*/
		if (i==grad)
		{
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


  while (!shutdown_requested()) {
  		lcd_refresh();
  		cputs("seguir");
   	motor_a_speed(NORMAL_SPEED);
   	motor_c_speed(NORMAL_SPEED);
   	motor_a_dir(rev);
    	motor_c_dir(fwd);
    
		wait_event(bright_found,BRIGHT_THRESH);
		locate_line();
		motor_a_dir(brake);
		motor_c_dir(brake);
  		lcd_refresh();
		if (dir==0)
		{
			cputs("D");
		}else{
			cputs("I");
		}
    }
}

int main(int argc, char *argv[]) {
  ds_active(&LIGHTSENS);
  follow_line();
  
  return 0;
}
#else
#warning linetrack.c requires CONF_DSENSOR and CONF_DMOTOR
#warning linetrack demo will do nothing
int main(int argc, char *argv[]) {
  return 0;
}
#endif 
