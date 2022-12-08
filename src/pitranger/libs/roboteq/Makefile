CC=g++


all: sample.o

sample.o: RoboteqDevice.o
		$(CC) RoboteqDevice.o src/sample.cpp -o sample.o

RoboteqDevice.o: 
	$(CC) -c src/RoboteqDevice.cpp


clean:
		rm *.o

