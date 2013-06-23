INC-DIR = -I. -I../include -I/usr/X11R6/include -I/usr/local/include -I/usr/X11R6/include/X11
LIB-DIR = -L. -L/usr/lib -L -L/usr/local/lib -L/usr/X11R6/lib 
GCC = gcc
CFLAGS = -g -pedantic -Wall $(INC-DIR)
#-ansi
#LIBS = -lpthread -lm -lforms -lX11 -ldc1394_control -lraw1394
LIBS = -lpthread -lm -lforms -lX11
SCHEMAS = guixforms.o guixformsf.o myschema.o
CFGDIR="/etc/jde-robot"

ALL: jde

jde: jde.o $(SCHEMAS)
	$(GCC) $(CFLAGS) $(LIB-DIR) -o jde jde.o $(SCHEMAS) $(LIBS)

jde.o: jde.c
	$(GCC) $(CFLAGS) -D CONFIGDIR='$(CFGDIR)' -D JDEVIDEO4LINUX -c jde.c	
#	$(GCC) $(CFLAGS) -D CONFIGDIR='$(CFGDIR)' -D JDEVIDEO4LINUX -D JDEFIREWIRE -c jde.c
#	$(GCC) $(CFLAGS) -D CONFIGDIR='$(CFGDIR)' -D JDEFIREWIRE -c jde.c


guixforms.o: guixforms.c guixforms.h
	$(GCC) $(CFLAGS) -c guixforms.c

guixformsf.o: guixformsf.c guixformsf.h
	$(GCC) $(CFLAGS) -c guixformsf.c

myschema.o: myschema.c myschema.h
	$(GCC) $(CFLAGS) -c myschema.c

clean:
	rm -f *.o 
release:
	tar -zcvf newrelease.tgz *h *c *fd *.conf *ppm *pnm README makefile 
