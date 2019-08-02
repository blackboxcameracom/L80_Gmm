CC=gcc

CFLAGS= 

gps_example: gps_example.o nmea.o gps_functions.o
	gcc gps_example.c nmea.c gps_functions.c -o gps_example -lwiringPi -lm -lpthread  