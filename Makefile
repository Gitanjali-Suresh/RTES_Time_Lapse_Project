INCLUDE_DIRS = 
LIB_DIRS = 
CC=gcc

CDEFS=
CFLAGS= -O0 -g $(INCLUDE_DIRS) $(CDEFS)
LIBS= -lrt -lpthread

HFILES= 
CFILES= image_sharpen.c

SRCS= ${HFILES} ${CFILES}
OBJS= ${CFILES:.c=.o}

all:	image_sharpen

clean:
	-rm -f *.o *.d
	-rm -f *.ppm
	-rm -f *.mp4
	-rm -f image_sharpen

distclean:
	-rm -f *.o *.d

image_sharpen: ${OBJS}
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o $(LIBS)

depend:

.c.o:
	$(CC) $(CFLAGS) -c $<