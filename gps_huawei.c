/*
**
** Copyright 2006, The Android Open Source Project
** Copyright 2009, Michael Trimarchi <michael@panicking.kicks-ass.org>
** Copyright 2011, Eduardo José Tagle <ejtagle@tutopia.com>
**
** This program is free software; you can redistribute it and/or modify it under
** the terms of the GNU General Public License as published by the Free
** Software Foundation; either version 2, or (at your option) any later
** version.
**
** This program is distributed in the hope that it will be useful, but WITHOUT
** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
** FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
** more details.
**
** You should have received a copy of the GNU General Public License along with
** this program; if not, write to the Free Software Foundation, Inc., 59
** Temple Place - Suite 330, Boston, MA 02111-1307, USA.
**/

#include <errno.h>
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <math.h>
#include <time.h>
#include <semaphore.h>
#include <signal.h>
#include <unistd.h>
#include <linux/socket.h>
#include <sys/socket.h>

#define  LOG_TAG  "gps_huawei"

#include <cutils/log.h>
#include <cutils/sockets.h>
#include <cutils/properties.h>
#include <hardware/gps.h>

#define  GPS_DEBUG  0

#if GPS_DEBUG
#  define  D(...)   LOGD(__VA_ARGS__)
#else
#  define  D(...)   ((void)0)
#endif

typedef void (*start_t)(void*);

/* Nmea Parser stuff */
#define  NMEA_MAX_SIZE  200

enum {
  STATE_QUIT  = 0,
  STATE_INIT  = 1,
  STATE_START = 2
};

typedef struct {
    int     pos;
    int     overflow;
    int     utc_year;
    int     utc_mon;
    int     utc_day;
    int     utc_diff;
    GpsLocation  fix;
    GpsSvStatus  sv_status;
    int     sv_status_changed;
    char    in[ NMEA_MAX_SIZE+1 ];
} NmeaReader;


typedef struct {
    volatile int            init;
    int                     fd;
    int                     ctrl_fd;
    GpsCallbacks            callbacks;
    AGpsCallbacks           a_callbacks;
    GpsXtraCallbacks        xtra_callbacks;
    GpsStatus               gps_status;
    char                    nmea_buf[512];
    int                     nmea_len;
    pthread_t               thread;
    sem_t                   fix_sem;
    pthread_t               tmr_thread;
    int                     control[2];
    int                     min_interval; // in ms
    NmeaReader              reader;

} GpsState;


/* Since NMEA parser requires lcoks */
#define GPS_STATE_LOCK_FIX(_s)           \
{                                        \
    int ret;                             \
    do {                                 \
        ret = sem_wait(&(_s)->fix_sem);  \
    } while (ret < 0 && errno == EINTR); \
}

#define GPS_STATE_UNLOCK_FIX(_s)         \
    sem_post(&(_s)->fix_sem)

static GpsState  _gps_state[1];
static GpsState *gps_state = _gps_state;

#define GPS_POWER_IF "/sys/bus/platform/devices/shuttle-pm-gps/power_on"

#define GPS_DEV_SLOW_UPDATE_RATE (10)
#define GPS_DEV_HIGH_UPDATE_RATE (1)

static void dev_start(int fd);
static void dev_stop(int fd);
static void *gps_timer_thread( void*  arg );


static void serial_write(int fd, char *msg)
{
  int i, n, ret;

  i = strlen(msg);

  n = 0;

  do {

    ret = write(fd, msg + n, i - n);

    if (ret < 0 && errno == EINTR) {
      continue;
    }

    n += ret;

  } while (n < i);

  return;
}

static unsigned char calc_nmea_csum(char *msg)
{
  unsigned char csum = 0;
  int i;

  for (i = 1; msg[i] != '*'; ++i) {
    csum ^= msg[i];
  }

  return csum;
}


/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       D E V I C E                                     *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

static void dev_power(int state)
{
    char   prop[PROPERTY_VALUE_MAX];
    int fd;
    char cmd = '0';
    int ret;
    return ; //RvdB

    if (property_get("gps.power_on",prop,GPS_POWER_IF) == 0) {
        LOGE("no gps power interface name");
        return;
    }

    do {
        fd = open( prop, O_RDWR );
    } while (fd < 0 && errno == EINTR);

    if (fd < 0) {
        LOGE("could not open gps power interface: %s", prop );
        return;
    }

    if (state) {
      cmd = '1';
    } else {
      cmd = '0';
    }

    do {
        ret = write( fd, &cmd, 1 );
    } while (ret < 0 && errno == EINTR);

    close(fd);

    D("gps power state = %c", cmd);

    if (state)
        usleep(500*1000);

    return;

}

static void dev_set_nmea_message_rate(int fd,const char *msg, int rate)
{

  char buff[50];
  int i;

  sprintf(buff, "$PUBX,40,%s,%d,%d,%d,0*", msg, rate, rate, rate);

  i = strlen(buff);

  sprintf((buff + i), "%02x\r\n", calc_nmea_csum(buff));

  serial_write(fd, buff);

  D("gps sent to device: %s", buff);

  return;

}

static void dev_set_message_rate(int fd, int rate)
{

  unsigned int i;
  static const char *msg[] = {
                 "GGA", "GLL", "ZDA",
                 "VTG", "GSA", "GSV",
                 "RMC"
                };

  for (i = 0; i < sizeof(msg)/sizeof(msg[0]); ++i) {
        dev_set_nmea_message_rate(fd, msg[i], rate);
  }

  return;
}


static void dev_start(int fd)
{
    D("gps dev start initiated");
    // Set full message rate
    //RvdB dev_set_message_rate(fd, GPS_DEV_HIGH_UPDATE_RATE);
}

static void dev_stop(int fd)
{
    D("gps dev stop");

    // Set slow message rate
    //RvdB dev_set_message_rate(fd, GPS_DEV_SLOW_UPDATE_RATE);
}

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       N M E A   T O K E N I Z E R                     *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

typedef struct {
    const char*  p;
    const char*  end;
} Token;

#define  MAX_NMEA_TOKENS  32

typedef struct {
    int     count;
    Token   tokens[ MAX_NMEA_TOKENS ];
} NmeaTokenizer;

static int nmea_tokenizer_init( NmeaTokenizer*  t, const char*  p, const char*  end )
{
    int    count = 0;
    char*  q;

    // the initial '$' is optional
    if (p < end && p[0] == '$')
        p += 1;

    // remove trailing newline
    if (end > p && end[-1] == '\n') {
        end -= 1;
        if (end > p && end[-1] == '\r')
            end -= 1;
    }

    // get rid of checksum at the end of the sentecne
    if (end >= p+3 && end[-3] == '*') {
        end -= 3;
    }

    while (p < end) {
        const char*  q = p;

        q = memchr(p, ',', end-p);
        if (q == NULL)
            q = end;

        if (count < MAX_NMEA_TOKENS) {
            t->tokens[count].p   = p;
            t->tokens[count].end = q;
            count += 1;
        }

        if (q < end)
            q += 1;

        p = q;
    }

    t->count = count;
    return count;
}

static Token nmea_tokenizer_get( NmeaTokenizer*  t, int  index )
{
    Token  tok;
    static const char*  dummy = "";

    if (index < 0 || index >= t->count) {
        tok.p = tok.end = dummy;
    } else
        tok = t->tokens[index];

    return tok;
}


static int str2int( const char*  p, const char*  end )
{
    int   result = 0;
    int   len    = end - p;

    if (len == 0) {
      return -1;
    }

    for ( ; len > 0; len--, p++ )
    {
        int  c;

        if (p >= end)
            goto Fail;

        c = *p - '0';
        if ((unsigned)c >= 10)
            goto Fail;

        result = result*10 + c;
    }
    return  result;

Fail:
    return -1;
}

static double str2float( const char*  p, const char*  end )
{
    int   result = 0;
    int   len    = end - p;
    char  temp[16];

    if (len == 0) {
      return -1.0;
    }

    if (len >= (int)sizeof(temp))
        return 0.;

    memcpy( temp, p, len );
    temp[len] = 0;
    return strtod( temp, NULL );
}

/** @desc Convert struct tm to time_t (time zone neutral).
 *
 * The one missing function in libc: It works basically like mktime, with the main difference that
 * it does no time zone-related processing but interprets the members of the struct tm as UTC.
 * Unlike mktime, it will not modify any fields of the tm structure; if you need this behavior, call
 * mktime before this function.
 *
 * @param t Pointer to a struct tm containing date and time. Only the tm_year, tm_mon, tm_mday,
 * tm_hour, tm_min and tm_sec members will be evaluated, all others will be ignored.
 *
 * @return The epoch time (seconds since 1970-01-01 00:00:00 UTC) which corresponds to t.
 *
 * @author Originally written by Philippe De Muyter <phdm@macqel.be> for Lynx.
 * http://lynx.isc.org/current/lynx2-8-8/src/mktime.c
 */

static time_t mkgmtime(struct tm *t)
{
    short month, year;
    time_t result;
    static const int m_to_d[12] =
        {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

    month = t->tm_mon;
    year = t->tm_year + month / 12 + 1900;
    month %= 12;
    if (month < 0) {
        year -= 1;
        month += 12;
    }
    result = (year - 1970) * 365 + m_to_d[month];
    if (month <= 1)
        year -= 1;
    result += (year - 1968) / 4;
    result -= (year - 1900) / 100;
    result += (year - 1600) / 400;
    result += t->tm_mday;
    result -= 1;
    result *= 24;
    result += t->tm_hour;
    result *= 60;
    result += t->tm_min;
    result *= 60;
    result += t->tm_sec;
    return (result);
}

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       N M E A   P A R S E R                           *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

static void nmea_reader_update_utc_diff( NmeaReader*  r )
{
    time_t         now = time(NULL);
    struct tm      tm_local;
    struct tm      tm_utc;
    long           time_local, time_utc;

    gmtime_r( &now, &tm_utc );
    localtime_r( &now, &tm_local );

    time_local = mktime(&tm_local);
    time_utc = mktime(&tm_utc);

    r->utc_diff = time_local - time_utc;
}


static void nmea_reader_init( NmeaReader*  r )
{
    int i;
    memset( r, 0, sizeof(*r) );

    // Initialize the sizes of all the structs we use
    r->fix.size = sizeof(GpsLocation);
    r->sv_status.size = sizeof(GpsSvStatus);
    for (i = 0; i < GPS_MAX_SVS; i++) {
        r->sv_status.sv_list[i].size = sizeof(GpsSvInfo);
    }

    r->pos      = 0;
    r->overflow = 0;
    r->utc_year = -1;
    r->utc_mon  = -1;
    r->utc_day  = -1;

    // not sure if we still need this (this module doesn't use utc_diff)
    nmea_reader_update_utc_diff( r );
}

static int nmea_reader_update_time( NmeaReader*  r, Token  tok )
{
    int        hour, minute, seconds, milliseconds;
    struct tm  tm;
    time_t     fix_time;

    if (tok.p + 6 > tok.end)
        return -1;

    if (r->utc_year < 0) {
        // no date, can't return valid timestamp (never ever make up a date, this could wreak havoc)
        return -1;
    }
    else
    {
        tm.tm_year = r->utc_year - 1900;
        tm.tm_mon  = r->utc_mon - 1;
        tm.tm_mday = r->utc_day;
    }

    hour    = str2int(tok.p,   tok.p+2);
    minute  = str2int(tok.p+2, tok.p+4);
    seconds = str2int(tok.p+4, tok.p+6);

    // parse also milliseconds (if present) for better precision
    milliseconds = 0;
    if (tok.end - (tok.p+7) == 2) {
        milliseconds = str2int(tok.p+7, tok.end) * 10;
    }
    else if (tok.end - (tok.p+7) == 1) {
        milliseconds = str2int(tok.p+7, tok.end) * 100;
    }
    else if (tok.end - (tok.p+7) >= 3) {
        milliseconds = str2int(tok.p+7, tok.p+10);
    }

    // the following is only guaranteed to work if we have previously set a correct date, so be sure
    // to always do that before

    tm.tm_hour = hour;
    tm.tm_min  = minute;
    tm.tm_sec  = seconds;

    fix_time = mkgmtime( &tm );

    r->fix.timestamp = (long long)fix_time * 1000 + milliseconds;
    return 0;
}

static int nmea_reader_update_cdate( NmeaReader*  r, Token  tok_d, Token tok_m, Token tok_y )
{

    if ( (tok_d.p + 2 > tok_d.end) ||
         (tok_m.p + 2 > tok_m.end) ||
         (tok_y.p + 4 > tok_y.end) )
        return -1;

    r->utc_day = str2int(tok_d.p,   tok_d.p+2);
    r->utc_mon = str2int(tok_m.p, tok_m.p+2);
    r->utc_year = str2int(tok_y.p, tok_y.p+4);

    return 0;
}

static int nmea_reader_update_date( NmeaReader*  r, Token  date, Token  time )
{
    Token  tok = date;
    int    day, mon, year;

    if (tok.p + 6 != tok.end) {
        D("date not properly formatted: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }
    day  = str2int(tok.p, tok.p+2);
    mon  = str2int(tok.p+2, tok.p+4);
    year = str2int(tok.p+4, tok.p+6) + 2000;

    if ((day|mon|year) < 0) {
        D("date not properly formatted: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }

    r->utc_year  = year;
    r->utc_mon   = mon;
    r->utc_day   = day;

    return nmea_reader_update_time( r, time );
}


static double convert_from_hhmm( Token  tok )
{
    double  val     = str2float(tok.p, tok.end);
    int     degrees = (int)(floor(val) / 100);
    double  minutes = val - degrees*100.;
    double  dcoord  = degrees + minutes / 60.0;
    return dcoord;
}


static int nmea_reader_update_latlong( NmeaReader*  r,
                            Token        latitude,
                            char         latitudeHemi,
                            Token        longitude,
                            char         longitudeHemi )
{
    double   lat, lon;
    Token    tok;

    tok = latitude;
    if (tok.p + 6 > tok.end) {
        D("latitude is too short: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }
    lat = convert_from_hhmm(tok);
    if (latitudeHemi == 'S')
        lat = -lat;

    tok = longitude;
    if (tok.p + 6 > tok.end) {
        D("longitude is too short: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }
    lon = convert_from_hhmm(tok);
    if (longitudeHemi == 'W')
        lon = -lon;

    r->fix.flags    |= GPS_LOCATION_HAS_LAT_LONG;
    r->fix.latitude  = lat;
    r->fix.longitude = lon;
    return 0;
}


static int nmea_reader_update_altitude( NmeaReader*  r,
                             Token        altitude,
                             Token        units )
{
    double  alt;
    Token   tok = altitude;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_ALTITUDE;
    r->fix.altitude = str2float(tok.p, tok.end);
    return 0;
}

static int nmea_reader_update_accuracy( NmeaReader*  r,
                             Token        accuracy )
{
    double  acc;
    Token   tok = accuracy;

    if (tok.p >= tok.end)
        return -1;

    r->fix.accuracy = str2float(tok.p, tok.end);

    if (r->fix.accuracy == 99.99){
      return 0;
    }

    r->fix.flags   |= GPS_LOCATION_HAS_ACCURACY;
    return 0;
}

static int nmea_reader_update_bearing( NmeaReader*  r,
                            Token        bearing )
{
    double  alt;
    Token   tok = bearing;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_BEARING;
    r->fix.bearing  = str2float(tok.p, tok.end);
    return 0;
}


static int nmea_reader_update_speed( NmeaReader*  r,
                          Token        speed )
{
    double  alt;
    Token   tok = speed;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_SPEED;
    // convert knots into m/sec (1 knot equals 1.852 km/h, 1 km/h equals 3.6 m/s)
    // since 1.852 / 3.6 is an odd value (periodic), we're calculating the quotient on the fly
    // to obtain maximum precision (we don't want 1.9999 instead of 2)
    r->fix.speed    = str2float(tok.p, tok.end) * 1.852 / 3.6;
    return 0;
}


static void nmea_reader_parse( NmeaReader*  r )
{
   /* we received a complete sentence, now parse it to generate
    * a new GPS fix...
    */
    NmeaTokenizer  tzer[1];
    Token          tok;

    D("Received: '%.*s'", r->pos, r->in);

    if (r->pos < 9) {
        D("Too short. discarded.");
        return;
    }

    nmea_tokenizer_init(tzer, r->in, r->in + r->pos);
#if GPS_DEBUG
    {
        int  n;
        D("Found %d tokens", tzer->count);
        for (n = 0; n < tzer->count; n++) {
            Token  tok = nmea_tokenizer_get(tzer,n);
            D("%2d: '%.*s'", n, tok.end-tok.p, tok.p);
        }
    }
#endif

    tok = nmea_tokenizer_get(tzer, 0);

    if (tok.p + 5 > tok.end) {
        D("sentence id '%.*s' too short, ignored.", tok.end-tok.p, tok.p);
        return;
    }

    // ignore first two characters.
    tok.p += 2;

    if ( !memcmp(tok.p, "GGA", 3) ) {
        // GPS fix
        Token  tok_fixstaus      = nmea_tokenizer_get(tzer,6);

        if ((tok_fixstaus.p[0] > '0') && (r->utc_year >= 0)) {
          // ignore this until we have a valid timestamp

          Token  tok_time          = nmea_tokenizer_get(tzer,1);
          Token  tok_latitude      = nmea_tokenizer_get(tzer,2);
          Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,3);
          Token  tok_longitude     = nmea_tokenizer_get(tzer,4);
          Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,5);
          Token  tok_altitude      = nmea_tokenizer_get(tzer,9);
          Token  tok_altitudeUnits = nmea_tokenizer_get(tzer,10);

          // don't use this as we have no fractional seconds and no date; there are better ways to
          // get a good timestamp from GPS
          //nmea_reader_update_time(r, tok_time);
          nmea_reader_update_latlong(r, tok_latitude,
                                        tok_latitudeHemi.p[0],
                                        tok_longitude,
                                        tok_longitudeHemi.p[0]);
          nmea_reader_update_altitude(r, tok_altitude, tok_altitudeUnits);
        }

    } else if ( !memcmp(tok.p, "GLL", 3) ) {

        Token  tok_fixstaus      = nmea_tokenizer_get(tzer,6);

        if ((tok_fixstaus.p[0] == 'A') && (r->utc_year >= 0)) {
          // ignore this until we have a valid timestamp

          Token  tok_latitude      = nmea_tokenizer_get(tzer,1);
          Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,2);
          Token  tok_longitude     = nmea_tokenizer_get(tzer,3);
          Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,4);
          Token  tok_time          = nmea_tokenizer_get(tzer,5);

          // don't use this as we have no fractional seconds and no date; there are better ways to
          // get a good timestamp from GPS
          //nmea_reader_update_time(r, tok_time);
          nmea_reader_update_latlong(r, tok_latitude,
                                        tok_latitudeHemi.p[0],
                                        tok_longitude,
                                        tok_longitudeHemi.p[0]);
        }

    } else if ( !memcmp(tok.p, "GSA", 3) ) {

        Token  tok_fixStatus   = nmea_tokenizer_get(tzer, 2);
        int i;

        if (tok_fixStatus.p[0] != '\0' && tok_fixStatus.p[0] != '1') {

          Token  tok_accuracy      = nmea_tokenizer_get(tzer, 15);

          nmea_reader_update_accuracy(r, tok_accuracy);

          r->sv_status.used_in_fix_mask = 0ul;

          for (i = 3; i <= 14; ++i){

            Token  tok_prn  = nmea_tokenizer_get(tzer, i);
            int prn = str2int(tok_prn.p, tok_prn.end);

            if (prn > 0){
              r->sv_status.used_in_fix_mask |= (1ul << (32 - prn));
              r->sv_status_changed = 1;
              D("%s: fix mask is %d", __FUNCTION__, r->sv_status.used_in_fix_mask);
            }

          }

        }

    } else if ( !memcmp(tok.p, "GSV", 3) ) {

        Token  tok_noSatellites  = nmea_tokenizer_get(tzer, 3);
        int    noSatellites = str2int(tok_noSatellites.p, tok_noSatellites.end);

        if (noSatellites > 0) {

          Token  tok_noSentences   = nmea_tokenizer_get(tzer, 1);
          Token  tok_sentence      = nmea_tokenizer_get(tzer, 2);

          int sentence = str2int(tok_sentence.p, tok_sentence.end);
          int totalSentences = str2int(tok_noSentences.p, tok_noSentences.end);
          int curr;
          int i;

          if (sentence == 1) {
              r->sv_status_changed = 0;
              r->sv_status.num_svs = 0;
          }

          curr = r->sv_status.num_svs;

          i = 0;

          while (i < 4 && r->sv_status.num_svs < noSatellites){

                 Token  tok_prn = nmea_tokenizer_get(tzer, i * 4 + 4);
                 Token  tok_elevation = nmea_tokenizer_get(tzer, i * 4 + 5);
                 Token  tok_azimuth = nmea_tokenizer_get(tzer, i * 4 + 6);
                 Token  tok_snr = nmea_tokenizer_get(tzer, i * 4 + 7);

                 r->sv_status.sv_list[curr].prn = str2int(tok_prn.p, tok_prn.end);
                 r->sv_status.sv_list[curr].elevation = str2float(tok_elevation.p, tok_elevation.end);
                 r->sv_status.sv_list[curr].azimuth = str2float(tok_azimuth.p, tok_azimuth.end);
                 r->sv_status.sv_list[curr].snr = str2float(tok_snr.p, tok_snr.end);

                 r->sv_status.num_svs += 1;

                 curr += 1;

                 i += 1;
          }

          if (sentence == totalSentences) {
              r->sv_status_changed = 1;
          }

          D("%s: GSV message with total satellites %d", __FUNCTION__, noSatellites);

        }

    } else if ( !memcmp(tok.p, "RMC", 3) ) {

        Token  tok_fixStatus     = nmea_tokenizer_get(tzer,2);

        if (tok_fixStatus.p[0] == 'A')
        {
          Token  tok_time          = nmea_tokenizer_get(tzer,1);
          Token  tok_latitude      = nmea_tokenizer_get(tzer,3);
          Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,4);
          Token  tok_longitude     = nmea_tokenizer_get(tzer,5);
          Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,6);
          Token  tok_speed         = nmea_tokenizer_get(tzer,7);
          Token  tok_bearing       = nmea_tokenizer_get(tzer,8);
          Token  tok_date          = nmea_tokenizer_get(tzer,9);

            nmea_reader_update_date( r, tok_date, tok_time );

            nmea_reader_update_latlong( r, tok_latitude,
                                           tok_latitudeHemi.p[0],
                                           tok_longitude,
                                           tok_longitudeHemi.p[0] );

            nmea_reader_update_bearing( r, tok_bearing );
            nmea_reader_update_speed  ( r, tok_speed );
        }

    } else if ( !memcmp(tok.p, "VTG", 3) ) {

        Token  tok_fixStatus     = nmea_tokenizer_get(tzer,9);

        if (tok_fixStatus.p[0] != '\0' && tok_fixStatus.p[0] != 'N')
        {
            Token  tok_bearing       = nmea_tokenizer_get(tzer,1);
            Token  tok_speed         = nmea_tokenizer_get(tzer,5);

            nmea_reader_update_bearing( r, tok_bearing );
            nmea_reader_update_speed  ( r, tok_speed );
        }

    } else if ( !memcmp(tok.p, "ZDA", 3) ) {

        Token  tok_time;
        Token  tok_year  = nmea_tokenizer_get(tzer,4);
        tok_time  = nmea_tokenizer_get(tzer,1);

        if ((tok_year.p[0] != '\0') && (tok_time.p[0] != '\0')) {

          // make sure to always set date and time together, lest bad things happen
          Token  tok_day   = nmea_tokenizer_get(tzer,2);
          Token  tok_mon   = nmea_tokenizer_get(tzer,3);

          nmea_reader_update_cdate( r, tok_day, tok_mon, tok_year );
          nmea_reader_update_time(r, tok_time);
        }


    } else {
        tok.p -= 2;
        D("unknown sentence '%.*s", tok.end-tok.p, tok.p);
    }

#if GPS_DEBUG
    if (r->fix.flags != 0) {

        char   temp[256];
        char*  p   = temp;
        char*  end = p + sizeof(temp);
        struct tm   utc;

        p += snprintf( p, end-p, "sending fix" );
        if (r->fix.flags & GPS_LOCATION_HAS_LAT_LONG) {
            p += snprintf(p, end-p, " lat=%g lon=%g", r->fix.latitude, r->fix.longitude);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_ALTITUDE) {
            p += snprintf(p, end-p, " altitude=%g", r->fix.altitude);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_SPEED) {
            p += snprintf(p, end-p, " speed=%g", r->fix.speed);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_BEARING) {
            p += snprintf(p, end-p, " bearing=%g", r->fix.bearing);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_ACCURACY) {
            p += snprintf(p,end-p, " accuracy=%g", r->fix.accuracy);
        }
        gmtime_r( (time_t*) &r->fix.timestamp, &utc );
        p += snprintf(p, end-p, " time=%s", asctime( &utc ) );
        LOGE("%s\n",temp);
    }
#endif
}


static void nmea_reader_addc( NmeaReader*  r, int  c )
{
    if (r->overflow) {
        r->overflow = (c != '\n');
        return;
    }

    if (r->pos >= (int) sizeof(r->in)-1 ) {
        r->overflow = 1;
        r->pos      = 0;
        return;
    }

    r->in[r->pos] = (char)c;
    r->pos       += 1;

    if (c == '\n') {
        nmea_reader_parse( r );
        r->pos = 0;
    }
}

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       C O N N E C T I O N   S T A T E                 *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

/* commands sent to the gps thread */
enum {
    CMD_QUIT  = 0,
    CMD_START = 1,
    CMD_STOP  = 2
};


static void gps_state_start( GpsState*  s )
{
    char  cmd = CMD_START;
    int   ret;

    do { ret=write( s->control[0], &cmd, 1 ); }
    while (ret < 0 && errno == EINTR);

    if (ret != 1)
        D("%s: could not send CMD_START command: ret=%d: %s",
          __FUNCTION__, ret, strerror(errno));
}


static void gps_state_stop( GpsState*  s )
{
    char  cmd = CMD_STOP;
    int   ret;

    do { ret=write( s->control[0], &cmd, 1 ); }
    while (ret < 0 && errno == EINTR);

    if (ret != 1)
        D("%s: could not send CMD_STOP command: ret=%d: %s",
          __FUNCTION__, ret, strerror(errno));
}


static int epoll_register( int  epoll_fd, int  fd )
{
    struct epoll_event  ev;
    int                 ret, flags;

    /* important: make the fd non-blocking */
    flags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    ev.events  = EPOLLIN;
    ev.data.fd = fd;
    do {
        ret = epoll_ctl( epoll_fd, EPOLL_CTL_ADD, fd, &ev );
    } while (ret < 0 && errno == EINTR);
    return ret;
}


static int epoll_deregister( int  epoll_fd, int  fd )
{
    int  ret;
    do {
        ret = epoll_ctl( epoll_fd, EPOLL_CTL_DEL, fd, NULL );
    } while (ret < 0 && errno == EINTR);
    return ret;
}

static void gps_nmea_thread_cb( GpsState* state )
{
    D("%s()", __FUNCTION__ );
    state->callbacks.nmea_cb(state->reader.fix.timestamp,&state->nmea_buf[0],state->nmea_len);
    GPS_STATE_UNLOCK_FIX(state);
}

static void gps_nmea_cb( GpsState* state , const char* buf, int len)
{
    D("%s()", __FUNCTION__ );
    // Forward NMEA sentences ....
    if (state->callbacks.nmea_cb) {

        GPS_STATE_LOCK_FIX(state);
        memcpy(&state->nmea_buf[0],buf,len);
        state->nmea_buf[len] = 0;
        state->nmea_len = len;
        state->callbacks.create_thread_cb("nmea",(start_t)gps_nmea_thread_cb,(void*)state);
    }
}

static void gps_status_thread_cb( GpsState* state )
{
    D("%s()", __FUNCTION__ );
    state->callbacks.status_cb(&state->gps_status);
    GPS_STATE_UNLOCK_FIX(state);
}

static void gps_status_cb( GpsState* state , GpsStatusValue status)
{
    D("%s()", __FUNCTION__ );
    if (state->callbacks.status_cb) {
        GPS_STATE_LOCK_FIX(state);

        state->gps_status.size = sizeof(GpsStatus);
        state->gps_status.status = status;
        state->callbacks.create_thread_cb("status",(start_t)gps_status_thread_cb,(void*)state);

        D("gps status callback: 0x%x", status);
    }
}

static void gps_set_capabilities_cb( GpsState* state , uint32_t caps)
{
    D("%s()", __FUNCTION__ );
    if (state->callbacks.set_capabilities_cb) {
        state->callbacks.create_thread_cb("caps",(start_t)state->callbacks.set_capabilities_cb,(void*)caps);
    }
}

static void gps_location_thread_cb( GpsState* state )
{
    D("%s()", __FUNCTION__ );
    state->callbacks.location_cb( &state->reader.fix );
    state->reader.fix.flags = 0;
    GPS_STATE_UNLOCK_FIX(state);
}


static void gps_location_cb( GpsState* state )
{
    D("%s()", __FUNCTION__ );
    if (state->callbacks.location_cb) {
        GPS_STATE_LOCK_FIX(state);
        state->callbacks.create_thread_cb("fix",(start_t)gps_location_thread_cb,(void*)state);
    }
}

static void gps_sv_status_thread_cb( GpsState* state )
{
    D("%s()", __FUNCTION__ );
    state->callbacks.sv_status_cb( &state->reader.sv_status );
    state->reader.sv_status_changed = 0;
    GPS_STATE_UNLOCK_FIX(state);
}


static void gps_sv_status_cb( GpsState* state )
{
    D("%s()", __FUNCTION__ );
    if (state->callbacks.sv_status_cb) {
        GPS_STATE_LOCK_FIX(state);
        state->callbacks.create_thread_cb("sv-status",(start_t)gps_sv_status_thread_cb,(void*)state);
    }
}


/* this is the main thread, it waits for commands from gps_state_start/stop and,
 * when started, messages from the QEMU GPS daemon. these are simple NMEA sentences
 * that must be parsed to be converted into GPS fixes sent to the framework
 */
static void* gps_state_thread( void* arg )
{
    GpsState*   state = (GpsState*) arg;
    NmeaReader  *reader;
    int         epoll_fd   = epoll_create(2);
    int         started    = 0;
    int         gps_fd     = state->fd;
    int         control_fd = state->control[1];

    reader = &state->reader;

    nmea_reader_init( reader );

    // register control file descriptors for polling
    epoll_register( epoll_fd, control_fd );
    epoll_register( epoll_fd, gps_fd );

    D("gps thread running");

    gps_set_capabilities_cb( state , GPS_CAPABILITY_MSA | GPS_CAPABILITY_MSB );

    D("after set capabilities");

    gps_status_cb( state , GPS_STATUS_ENGINE_ON);

    D("after set status");

    // now loop
    for (;;) {
        struct epoll_event   events[2];
        int                  ne, nevents;

        nevents = epoll_wait( epoll_fd, events, 2, -1 );
        if (nevents < 0) {
            if (errno != EINTR)
                LOGE("epoll_wait() unexpected error: %s", strerror(errno));
            continue;
        }
        D("gps thread received %d events", nevents);
        for (ne = 0; ne < nevents; ne++) {
            if ((events[ne].events & (EPOLLERR|EPOLLHUP)) != 0) {
                LOGE("EPOLLERR or EPOLLHUP after epoll_wait() !?");
                goto Exit;
            }
            if ((events[ne].events & EPOLLIN) != 0) {
                int  fd = events[ne].data.fd;

                if (fd == control_fd)
                {
                    char  cmd = 255;
                    int   ret;
                    D("gps control fd event");
                    do {
                        ret = read( fd, &cmd, 1 );
                    } while (ret < 0 && errno == EINTR);

                    if (cmd == CMD_QUIT) {
                        D("gps thread quitting on demand");
                        goto Exit;
                    }
                    else if (cmd == CMD_START) {
                        if (!started) {
                            D("gps thread starting  location_cb=%p", state->callbacks.location_cb);
                            started = 1;

                            dev_start(gps_fd);

                            gps_status_cb( state , GPS_STATUS_SESSION_BEGIN);

                            state->init = STATE_START;

                            if ( pthread_create( &state->tmr_thread, NULL, gps_timer_thread, state ) != 0 ) {
                                LOGE("could not create gps timer thread: %s", strerror(errno));
                                started = 0;
                                state->init = STATE_INIT;
                                goto Exit;
                            }

                        }
                    }
                    else if (cmd == CMD_STOP) {
                        if (started) {
                            void *dummy;
                            D("gps thread stopping");
                            started = 0;

                            dev_stop(gps_fd);

                            state->init = STATE_INIT;

                            pthread_join(state->tmr_thread, &dummy);

                            gps_status_cb( state , GPS_STATUS_SESSION_END);

                        }
                    }
                }
                else if (fd == gps_fd)
                {
                    char buf[512];
                    int  nn, ret;

                    do {
                        ret = read( fd, buf, sizeof(buf) );
                    } while (ret < 0 && errno == EINTR);

                    if (ret > 0) {

                        gps_nmea_cb( state , &buf[0], ret);

                        GPS_STATE_LOCK_FIX(state);
                        for (nn = 0; nn < ret; nn++)
                            nmea_reader_addc( reader, buf[nn] );
                        GPS_STATE_UNLOCK_FIX(state);

                    }
                    D("gps fd event end");
                }
                else
                {
                    LOGE("epoll_wait() returned unkown fd %d ?", fd);
                }
            }
        }
    }
Exit:

    gps_status_cb( state , GPS_STATUS_ENGINE_OFF);

    return NULL;
}

static void* gps_timer_thread( void*  arg )
{

  GpsState *state = (GpsState *)arg;

  D("gps entered timer thread");

  do {

    D ("gps timer exp");

    if (state->reader.fix.flags != 0) {

      D("gps fix cb: 0x%x", state->reader.fix.flags);

      gps_location_cb( state );
    }

    if (state->reader.sv_status_changed != 0) {

      D("gps sv status callback");

      gps_sv_status_cb( state );

    }

    if (state->min_interval == 0) {
        state->min_interval = 1000;
    }

    usleep(state->min_interval*1000);

  } while(state->init == STATE_START);

  D("gps timer thread destroyed");

  return NULL;

}

static int open_serialport( const char* name )
{
    int fd = -1;
    do {
        fd = open( name, O_RDWR );
    } while (fd < 0 && errno == EINTR);

    if (fd < 0) {
        LOGE("could not open serial device %s: %s", name, strerror(errno) );
        return fd;
    }

    // disable echo on serial lines
    if ( isatty( fd ) ) {
        struct termios  ios;
        tcgetattr( fd, &ios );
        ios.c_lflag = 0;  /* disable ECHO, ICANON, etc... */
        ios.c_oflag &= (~ONLCR); /* Stop \n -> \r\n translation on output */
        ios.c_iflag &= (~(ICRNL | INLCR)); /* Stop \r -> \n & \n -> \r translation on input */
        ios.c_iflag |= (IGNCR | IXOFF);  /* Ignore \r & XON/XOFF on input */
        tcsetattr( fd, TCSANOW, &ios );
    }

    return fd;
}

static void gps_state_done( GpsState*  s )
{
    // tell the thread to quit, and wait for it
    char   cmd = CMD_QUIT;
    void*  dummy;
    int ret;

    D("gps send quit command");

    do { ret=write( s->control[0], &cmd, 1 ); }
    while (ret < 0 && errno == EINTR);

    D("gps waiting for command thread to stop");

    pthread_join(s->thread, &dummy);

    /* Timer thread depends on this state check */
    s->init = STATE_QUIT;
    s->min_interval = 1000;

    // close the control socket pair
    close( s->control[0] ); s->control[0] = -1;
    close( s->control[1] ); s->control[1] = -1;

    // Turn off GPS function
    serial_write(s->ctrl_fd,"AT^WPEND\r\n");

    // close connection to the GPS
    close( s->fd ); s->fd = -1;
    close( s->ctrl_fd ); s->ctrl_fd = -1;

    // Power down the GPS interface
    dev_power(0);

    sem_destroy(&s->fix_sem);

    memset(s, 0, sizeof(*s));

    D("gps deinit complete");

}


static void gps_state_init( GpsState*  state )
{
    char   gps_data[PROPERTY_VALUE_MAX];
    char   gps_ctrl[PROPERTY_VALUE_MAX];
    char   gps_supl[PROPERTY_VALUE_MAX];
    char   buf[PROPERTY_VALUE_MAX+20];
    int    ret;
    int    done = 0;

    struct sigevent tmr_event;

    state->init       = STATE_INIT;
    state->control[0] = -1;
    state->control[1] = -1;
    state->fd         = -1;
    state->ctrl_fd    = -1;
    state->min_interval   = 1000;

    if (sem_init(&state->fix_sem, 0, 1) != 0) {
      D("gps semaphore initialization failed! errno = %d", errno);
      return;
    }

    // Power up the GPS interface
    dev_power(1);

    // look for a kernel-provided gps_data port
    strcpy(gps_data,"/dev/ttyUSB3") ; //RvdB
    /* RvdB
    if (property_get("ro.kernel.gps.data",gps_data,"/dev/ttyUSB3") == 0) {
        D("no kernel-provided gps device data name");
        dev_power(0);
        return;
    } */

    state->fd = open_serialport( gps_data );
    if (state->fd < 0) {
        LOGE("could not open gps serial device %s: %s", gps_data, strerror(errno) );
        dev_power(0);
        return;
    }

    D("gps will read from %s", gps_data);

    // look for a kernel-provided gps_ctrl port
    strcpy(gps_ctrl,"/dev/ttyUSB4")  ;//RvdB

    /*
    if (property_get("ro.kernel.gps.ctl",gps_ctrl,"/dev/ttyUSB4") == 0) {
        D("no kernel-provided gps device ctrl name");
        dev_power(0);
        return;
    } */

    state->ctrl_fd = open_serialport( gps_ctrl );
    if (state->ctrl_fd < 0) {
        LOGE("could not open gps serial device %s: %s", gps_ctrl, strerror(errno) );
        close(state->fd);
        state->fd = -1;
        dev_power(0);
        return;
    }

    D("gps will be controlled by %s", gps_ctrl);

    // Turn on GPS function
    serial_write(state->ctrl_fd,"AT^WPEND\r\n");

    // look for a kernel-provided supl
    strcpy(gps_supl,"http://supl.nokia.com") ; //RvdB
    /*
    if (property_get("ro.kernel.gps.supl",gps_supl,"http://supl.nokia.com") == 0) {
        D("no kernel-provided supl");
        close(state->fd);
        state->fd = -1;
        close(state->ctrl_fd);
        state->ctrl_fd = -1;
        dev_power(0);
        return;
    } */

    sprintf(buf,"AT^WPURL=%s\r\n",gps_supl);
    serial_write(state->ctrl_fd,buf);

    // configure AGPS to work speed optimal
    serial_write(state->ctrl_fd,"AT^WPDOM=2\r\n");

    // Power up GPS
    serial_write(state->ctrl_fd,"AT^WPDGP\r\n");

    if ( socketpair( AF_LOCAL, SOCK_STREAM, 0, state->control ) < 0 ) {
        LOGE("could not create thread control socket pair: %s", strerror(errno));
        goto Fail;
    }


    if ( pthread_create( &state->thread, NULL, gps_state_thread, state ) != 0 ) {
        LOGE("could not create gps thread: %s", strerror(errno));
        goto Fail;
    }


    D("gps state initialized");

    return;

Fail:
    gps_state_done( state );
}



/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       I N T E R F A C E                               *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

static int gps_init(GpsCallbacks* callbacks)
{
    GpsState*  s = _gps_state;

    if (!s->init)
        gps_state_init(s);

    if (s->fd < 0)
        return -1;

    s->callbacks = *callbacks;

    return 0;
}

static void gps_cleanup(void)
{
    GpsState*  s = _gps_state;

    if (s->init)
        gps_state_done(s);
}


static int gps_start(void)
{
    GpsState*  s = _gps_state;

    if (!s->init) {
        D("%s: called with uninitialized state !!", __FUNCTION__);
        return -1;
    }

    D("%s: called", __FUNCTION__);
    gps_state_start(s);
    return 0;
}


static int gps_stop(void)
{
    GpsState*  s = _gps_state;

    if (!s->init) {
        D("%s: called with uninitialized state !!", __FUNCTION__);
        return -1;
    }

    D("%s: called", __FUNCTION__);
    gps_state_stop(s);
    return 0;
}


static int gps_inject_time(GpsUtcTime time, int64_t timeReference, int uncertainty)
{
    return 0;
}

/** Injects current location from another location provider
 *  (typically cell ID).
 *  latitude and longitude are measured in degrees
 *  expected accuracy is measured in meters
 */
static int gps_inject_location(double latitude, double longitude, float accuracy)
{
    return 0;
}

static void gps_delete_aiding_data(GpsAidingData flags)
{
}

static int gps_set_position_mode(GpsPositionMode mode,  GpsPositionRecurrence recurrence,
         uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time)
{
    GpsState*  s = _gps_state;

    if (!s->init) {
        D("%s: called with uninitialized state !!", __FUNCTION__);
        return -1;
    }

    // only standalone supported for now.
    if (mode == GPS_POSITION_MODE_STANDALONE) {
    } else
    if (mode == GPS_POSITION_MODE_MS_BASED ||
        mode == GPS_POSITION_MODE_MS_ASSISTED) {
    }


    s->min_interval = min_interval;

    D("gps fix frquency set to %d secs", min_interval);

    return 0;
}

/***** AGpsInterface *****/

/**
 * Opens the AGPS interface and provides the callback routines
 * to the implemenation of this interface.
 */
static void agps_init( AGpsCallbacks* callbacks )
{
    D("%s() is called", __FUNCTION__);

    GpsState*  s = _gps_state;

    if (!s->init) {
        D("%s: called with uninitialized state !!", __FUNCTION__);
        return;
    }

    s->a_callbacks = *callbacks;
}

/**
 * Notifies that a data connection is available and sets
 * the name of the APN to be used for SUPL.
 */
static int agps_conn_open( const char* apn )
{
    D("%s() is called", __FUNCTION__);
    D("apn=%s", apn);

    GpsState*  s = _gps_state;

    if (!s->init) {
        D("%s: called with uninitialized state !!", __FUNCTION__);
        return 0;
    }

    return 0;
}

/**
 * Notifies that the AGPS data connection has been closed.
 */
static int agps_conn_closed( void )
{
    D("%s() is called", __FUNCTION__);

    GpsState*  s = _gps_state;

    if (!s->init) {
        D("%s: called with uninitialized state !!", __FUNCTION__);
        return 0;
    }

    return 0;
}

/**
 * Notifies that a data connection is not available for AGPS.
 */
static int agps_conn_failed( void )
{
    D("%s() is called", __FUNCTION__);

    GpsState*  s = _gps_state;

    if (!s->init) {
        D("%s: called with uninitialized state !!", __FUNCTION__);
        return 0;
    }

    return 0;
}

/**
 * Sets the hostname and port for the AGPS server.
 */
static int agps_set_server( AGpsType type, const char* hostname, int port )
{
    char buf[512];

    D("%s() is called", __FUNCTION__);
    D("type=%d, hostname=%s, port=%d", type, hostname, port);

    GpsState*  s = _gps_state;

    if (!s->init) {
        D("%s: called with uninitialized state !!", __FUNCTION__);
        return 0;
    }

#if 0
    sprintf(buf,"AT^WPURL=%s\r\n",hostname);
    serial_write(s->ctrl_fd,buf);
#endif

    return 0;
}

static const AGpsInterface  sAGpsInterface = {
    sizeof(AGpsInterface),
    agps_init,
    agps_conn_open,
    agps_conn_closed,
    agps_conn_failed,
    agps_set_server
};


/***** GpsXtraInterface *****/
static int xtra_init(GpsXtraCallbacks* callbacks)
{
    D("%s() is called", __FUNCTION__);
    GpsState*  s = _gps_state;

    s->xtra_callbacks = *callbacks;

    return 0;
}

static int xtra_inject_xtra_data(char* data, int length)
{
    D("%s() is called", __FUNCTION__);
    D("xtra size = %d, data ptr = 0x%x\n", length, (int) data);

    GpsState*  s = _gps_state;
    if (!s->init)
        return 0;

    return 0;
}

static const GpsXtraInterface sGpsXtraInterface = {
    sizeof(GpsXtraInterface),
    xtra_init,
    xtra_inject_xtra_data,
};

static const void* gps_get_extension(const char* name)
{
    D("%s('%s') is called", __FUNCTION__, name);

    if (!strcmp(name, GPS_XTRA_INTERFACE)) {
        return &sGpsXtraInterface;
    } else if (!strcmp(name, AGPS_INTERFACE)) {
        return &sAGpsInterface;
    }
    return NULL;
}

static const GpsInterface sGpsInterface = {
    sizeof(GpsInterface),
    gps_init,
    gps_start,
    gps_stop,
    gps_cleanup,
    gps_inject_time,
    gps_inject_location,
    gps_delete_aiding_data,
    gps_set_position_mode,
    gps_get_extension,
};

// As under Android, there are no exceptions in C++ base system, we will hide all exported symbols
// except the required ones. This will generate better code!

#if __GNUC__ >= 4
    #define DLL_PUBLIC __attribute__ ((visibility ("default")))
    #define DLL_LOCAL  __attribute__ ((visibility ("hidden")))
#else
    #define DLL_PUBLIC
    #define DLL_LOCAL
#endif


static const GpsInterface* gps_get_hardware_interface(struct gps_device_t* dev)
{
    LOGV("get_interface was called");
    return &sGpsInterface;
}

static int open_gps(const struct hw_module_t* module, char const* name,
        struct hw_device_t** device)
{
    struct gps_device_t *dev = malloc(sizeof(struct gps_device_t));
    memset(dev, 0, sizeof(*dev));

    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module = (struct hw_module_t*)module;
    dev->get_gps_interface = gps_get_hardware_interface;

    *device = (struct hw_device_t*)dev;
    return 0;
}


static struct hw_module_methods_t gps_module_methods = {
    .open = open_gps
};

DLL_PUBLIC const struct hw_module_t HAL_MODULE_INFO_SYM = {
    .tag = HARDWARE_MODULE_TAG,
    .version_major = 1,
    .version_minor = 0,
    .id = GPS_HARDWARE_MODULE_ID,
    .name = "GPS Module",
    .author = "Eduardo José Tagle",
    .methods = &gps_module_methods,
};
