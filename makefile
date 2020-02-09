TARGET=thor
OBJS=thor.o world.o msg_com.o logger.o buffer.o normal.o robot.o sim.o simcom.o\
     replay.o global.o \
     vm.o vm-buffer.o natives.o \
     thymio.o thymio_desc.o thymio_natives.o thysim.o 

$(TARGET) : $(OBJS)
	gcc -g -Wall -o $(TARGET) $(OBJS) -lm

SUFFIXES: .c .o
.c.o :
	gcc -g -Wall -c $<

%.o : %.c
	gcc -g -Wall -c $<


vm.o : ../../vm/vm.c
	gcc -g -Wall -c $<

natives.o : ../../vm/natives.c
	gcc -g -Wall -c $<

vm-buffer.o : ../../transport/buffer/vm-buffer.c
	gcc -g -Wall -c $<

clean:
	rm *.o

# thor.o : thor.c
# thymio.o : thymio.c
# thymio_natives.o : thymio_natives.c
# gcc -g -Wall -o thor thor.c thymio.c thymio_natives.c ../../vm/vm.c ../../transport/buffer/vm-buffer.c ../../vm/natives.c
