SRC=./src/
INC=./inc/
OBJ=./obj/
CFLAGSP= -Wall -pedantic -g
CFLAGS= -Wall -g

run: sputnik
	./sputnik

sputnik: ${OBJ}SputnikSDK.o ${OBJ}DrRobotSDK.o ${OBJ}Sputnik.o ${OBJ}uyvy2rgb.o ${OBJ}JpegDecodeHuffMan.o ${OBJ}DctQuant.o
	g++ -g -Wall -lcv -lhighgui -lpthread `allegro-config --libs`  ${OBJ}SputnikSDK.o ${OBJ}DrRobotSDK.o ${OBJ}Sputnik.o ${OBJ}uyvy2rgb.o ${OBJ}JpegDecodeHuffMan.o ${OBJ}DctQuant.o -o sputnik

${OBJ}Sputnik.o: ${SRC}Sputnik.cpp ${INC}SputnikSDK.hh ${INC}uyvy2rgb.hh
	g++ -c $(CFLAGS) -I/usr/include/opencv -Iinc ${SRC}Sputnik.cpp -o ${OBJ}Sputnik.o

${OBJ}SputnikSDK.o: ${SRC}SputnikSDK.cpp ${INC}SputnikSDK.hh ${INC}DrRobotSDK.hh
	g++ -c $(CFLAGS) -c  -I/usr/include/opencv -Iinc ${SRC}SputnikSDK.cpp -o ${OBJ}SputnikSDK.o

${OBJ}DrRobotSDK.o: ${SRC}DrRobotSDK.cpp ${INC}DrRobotSDK.hh
	g++ -c $(CFLAGS) -I/usr/include/opencv -Iinc ${SRC}DrRobotSDK.cpp -o ${OBJ}DrRobotSDK.o

${OBJ}uyvy2rgb.o: ${SRC}uyvy2rgb.cpp ${INC}uyvy2rgb.hh
	g++ -c $(CFLAGSP) -Iinc  ${SRC}uyvy2rgb.cpp -o ${OBJ}uyvy2rgb.o

${OBJ}JpegDecodeHuffMan.o: ${SRC}JpegDecodeHuffMan.cpp ${INC}JpegDecodeHuffMan.h
	g++ -c $(CFLAGSP) -Iinc ${SRC}JpegDecodeHuffMan.cpp -o ${OBJ}JpegDecodeHuffMan.o

${OBJ}DctQuant.o: ${SRC}DctQuant.cpp ${INC}DctQuant.h
	g++ -c $(CFLAGSP) -Iinc ${SRC}DctQuant.cpp -o ${OBJ}DctQuant.o

clean:
	rm ${OBJ}*.o
	rm sputnik


doc:
	doxygen sputnik_dox

