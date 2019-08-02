/** These functions are taken from Walter Dal Mut's GPS Library.
 *  We have added parsing of the date and time from the GPRMC
 * data sentence as well as parsing the fix quality indicator 
 * so as to enable the reporting of data valid in a variable. 
 * For details of the NMEA sentence structure please see
 * www.gpsinformation.org/dale/nmea.htm
 */


#ifndef _NMEA_H_
#define _NMEA_H_

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#define _EMPTY 0x00
#define NMEA_GPRMC 0x01
#define NMEA_GPRMC_STR "$GPRMC"
#define NMEA_GPGGA 0x02
#define NMEA_GPGGA_STR "$GPGGA"
#define NMEA_UNKNOWN 0x00
#define _COMPLETED 0x03

#define NMEA_CHECKSUM_ERR 0x80
#define NMEA_MESSAGE_ERR 0xC0

struct gpgga {
    // Latitude eg: 4124.8963 (XXYY.ZZKK.. DEG, MIN, SEC.SS)
    double latitude;
    // Latitude eg: N
    char lat;
    // Longitude eg: 08151.6838 (XXXYY.ZZKK.. DEG, MIN, SEC.SS)
    double longitude;
    // Longitude eg: W
    char lon;
    // Quality 0, 1, 2
    uint8_t quality;
    // Number of satellites: 1,2,3,4,5...
    uint8_t satellites;
    // Altitude eg: 280.2 (Meters above mean sea level)
    double altitude;
    
    int dataValid;
};
typedef struct gpgga gpgga_t;

struct gprmc {
    double latitude;
    char lat;
    double longitude;
    char lon;
    double speed;
    double course;
    
    char	time[7];
    char	date[7];
    
    int dataValid;
};
typedef struct gprmc gprmc_t;

extern uint8_t nmea_get_message_type(const char *);
extern uint8_t nmea_valid_checksum(const char *);
extern void nmea_parse_gpgga(char *, gpgga_t *);
extern void nmea_parse_gprmc(char *, gprmc_t *);

// From gps.h
// convert deg to decimal deg latitude, (N/S), longitude, (W/E)
extern void gps_convert_deg_to_dec(double *, char, double *, char);
extern double gps_deg_dec(double);

#endif

