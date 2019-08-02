CC=gcc

CFLAGS= 

combined_example: main.o ../GPS_EXAMPLE/nmea.o ../GPS_EXAMPLE/gps_functions.o ../BNO055/bno055.o ../BNO055/bno055_funcs.o ../MS5637/ms5637_funcs.o
	gcc main.c ../GPS_EXAMPLE/nmea.o ../GPS_EXAMPLE/gps_functions.o ../BNO055/bno055.o ../BNO055/bno055_funcs.o ../MS5637/ms5637_funcs.o -o combined_example -lm -lwiringPi -lpthread