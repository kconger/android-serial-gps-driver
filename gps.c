/* //hardware/hw/gps/gps_freerunner.c
**
** Copyright 2006, The Android Open Source Project
** Copyright 2009, Michael Trimarchi <michael@panicking.kicks-ass.org>
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
#include <signal.h>
#include <unistd.h>

#define  LOG_TAG  "gps_gj"

#include <cutils/log.h>
#include <cutils/sockets.h>
#include <cutils/properties.h>
#include <hardware/gps.h>

/* this is the state of our connection to the qemu_gpsd daemon */
typedef struct {
    int                     init;
    int                     fd;
    GpsCallbacks            *callbacks;
    GpsStatus status;
    pthread_t               thread;
    int                     control[2];
} GpsState;

static GpsState  _gps_state[1];
static int    id_in_fixed[12];
#define  GPS_DEBUG  0

#define  DFR(...)   ALOGD(__VA_ARGS__)

#if GPS_DEBUG
#  define  D(...)   ALOGD(__VA_ARGS__)
#else
#  define  D(...)   ((void)0)
#endif

#define GPS_DEV_SLOW_UPDATE_RATE (10)
#define GPS_DEV_HIGH_UPDATE_RATE (1)

#define GPS_DEV_LOW_BAUD  (B9600)
#define GPS_DEV_HIGH_BAUD (B19200)
  static void gps_dev_init(int fd);
  static void gps_dev_deinit(int fd);
  static void gps_dev_start(int fd);
  static void gps_dev_stop(int fd);

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

#define  MAX_NMEA_TOKENS  16

typedef struct {
    int     count;
    Token   tokens[ MAX_NMEA_TOKENS ];
} NmeaTokenizer;

static int
nmea_tokenizer_init( NmeaTokenizer*  t, const char*  p, const char*  end )
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

        if (q > p) {
            if (count < MAX_NMEA_TOKENS) {
                t->tokens[count].p   = p;
                t->tokens[count].end = q;
                count += 1;
            }
        }
        if (q < end)
            q += 1;

        p = q;
    }

    t->count = count;
    return count;
}

static Token
nmea_tokenizer_get( NmeaTokenizer*  t, int  index )
{
    Token  tok;

    if (index < 0 || index >= t->count || index >= MAX_NMEA_TOKENS)
        tok.p = tok.end = "";
    else
        tok = t->tokens[index];

    return tok;
}


static int
str2int( const char*  p, const char*  end )
{
    int   result = 0;
    int   len    = end - p;

    for ( ; len > 0; len--, p++ ) {
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

static double
str2float( const char*  p, const char*  end )
{
    size_t len = end - p;
    char   temp[16];

    if (len >= sizeof(temp))
        return 0.;

    memcpy( temp, p, len );
    temp[len] = '\0';
    return strtod( temp, NULL );
}

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       N M E A   P A R S E R                           *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

//#define  NMEA_MAX_SIZE  83
#define  NMEA_MAX_SIZE  255

typedef struct {
    int     pos;
    int     overflow;
    int     utc_year;
    int     utc_mon;
    int     utc_day;
    int     utc_diff;
    GpsLocation  fix;
    GpsSvStatus sv_status;
    gps_location_callback  callback;
    char    in[ NMEA_MAX_SIZE+1 ];
} NmeaReader;

void update_gps_status(GpsStatusValue val)
{
    GpsState*  state = _gps_state;
    //Should be made thread safe...
    state->status.status=val;
    if (state->callbacks->status_cb)
        state->callbacks->status_cb(&state->status);
}

void update_gps_svstatus(GpsSvStatus *val)
{
    GpsState*  state = _gps_state;
    //Should be made thread safe...
    if (state->callbacks->sv_status_cb)
        state->callbacks->sv_status_cb(val);
}

void update_gps_location(GpsLocation *fix)
{
    GpsState*  state = _gps_state;
    //Should be made thread safe...
    if (state->callbacks->location_cb)
        state->callbacks->location_cb(fix);
}

static void
nmea_reader_update_utc_diff( NmeaReader*  r )
{
    time_t         now = time(NULL);
    struct tm      tm_local;
    struct tm      tm_utc;
    long           time_local, time_utc;

    gmtime_r( &now, &tm_utc );
    localtime_r( &now, &tm_local );

    time_local = tm_local.tm_sec +
                 60*(tm_local.tm_min +
                 60*(tm_local.tm_hour +
                 24*(tm_local.tm_yday +
                 365*tm_local.tm_year)));

    time_utc = tm_utc.tm_sec +
               60*(tm_utc.tm_min +
               60*(tm_utc.tm_hour +
               24*(tm_utc.tm_yday +
               365*tm_utc.tm_year)));

    r->utc_diff = time_utc - time_local;
}


static void
nmea_reader_init( NmeaReader*  r )
{
    memset( r, 0, sizeof(*r) );

    r->pos      = 0;
    r->overflow = 0;
    r->utc_year = -1;
    r->utc_mon  = -1;
    r->utc_day  = -1;
    r->callback = NULL;
    r->fix.size = sizeof(r->fix);

    nmea_reader_update_utc_diff( r );
}


static void
nmea_reader_set_callback( NmeaReader*  r, gps_location_callback  cb )
{
    r->callback = cb;
    if (cb != NULL && r->fix.flags != 0) {
        D("%s: sending latest fix to new callback", __FUNCTION__);
        r->callback( &r->fix );
        r->fix.flags = 0;
    }
}


static int
nmea_reader_update_time( NmeaReader*  r, Token  tok )
{
    int        hour, minute;
    double     seconds;
    struct tm  tm;
    time_t     fix_time;

    if (tok.p + 6 > tok.end)
        return -1;

    if (r->utc_year < 0) {
        // no date yet, get current one
        time_t  now = time(NULL);
        gmtime_r( &now, &tm );
        r->utc_year = tm.tm_year + 1900;
        r->utc_mon  = tm.tm_mon + 1;
        r->utc_day  = tm.tm_mday;
    }

    hour    = str2int(tok.p,   tok.p+2);
    minute  = str2int(tok.p+2, tok.p+4);
    seconds = str2float(tok.p+4, tok.end);

    tm.tm_hour  = hour;
    tm.tm_min   = minute;
    tm.tm_sec   = (int) seconds;
    tm.tm_year  = r->utc_year - 1900;
    tm.tm_mon   = r->utc_mon - 1;
    tm.tm_mday  = r->utc_day;
    tm.tm_isdst = -1;

    fix_time = mktime( &tm ) + r->utc_diff;
    r->fix.timestamp = (long long)fix_time * 1000;
    return 0;
}

static int
nmea_reader_update_date( NmeaReader*  r, Token  date, Token  time )
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


static double
convert_from_hhmm( Token  tok )
{
    double  val     = str2float(tok.p, tok.end);
    int     degrees = (int)(floor(val) / 100);
    double  minutes = val - degrees*100.;
    double  dcoord  = degrees + minutes / 60.0;
    return dcoord;
}


static int
nmea_reader_update_latlong( NmeaReader*  r,
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


static int
nmea_reader_update_altitude( NmeaReader*  r,
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


static int
nmea_reader_update_bearing( NmeaReader*  r,
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


static int
nmea_reader_update_speed( NmeaReader*  r,
                          Token        speed )
{
    double  alt;
    Token   tok = speed;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_SPEED;
    r->fix.speed    = str2float(tok.p, tok.end);
    return 0;
}

static int
nmea_reader_update_svs( NmeaReader*  r, int inview, int num, int i, Token prn, Token elevation, Token azimuth, Token snr )
{
    int o;
    i = (num - 1)*4 + i;
    if (i < inview) {
        r->sv_status.sv_list[i].prn=str2int(prn.p,prn.end);
        r->sv_status.sv_list[i].elevation=str2int(elevation.p,elevation.end);
        r->sv_status.sv_list[i].azimuth=str2int(azimuth.p,azimuth.end);
        r->sv_status.sv_list[i].snr=str2int(snr.p,snr.end);
        for (o=0;o<12;o++)
            if (id_in_fixed[o]==str2int(prn.p,prn.end))
                r->sv_status.used_in_fix_mask |= 1<i ;
    }
    return 0;
}


static void
nmea_reader_parse( NmeaReader*  r )
{
   /* we received a complete sentence, now parse it to generate
    * a new GPS fix...
    */
    NmeaTokenizer  tzer[1];
    Token          tok;
    struct timeval tv;

    D("Received: '%.*s'", r->pos, r->in);
    if (r->pos < 9) {
        D("Too short. discarded.");
        return;
    }

    gettimeofday(&tv, NULL);
    if (_gps_state->init)
        _gps_state->callbacks->nmea_cb(tv.tv_sec*1000+tv.tv_usec/1000, r->in, r->pos);

    nmea_tokenizer_init(tzer, r->in, r->in + r->pos);
//#if GPS_DEBUG
#if 0
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
/*
$GPRMC,084537.345,V,,,,,,,270811,,,N*4F
$GPGGA,084538.353,,,,,0,00,,,M,0.0,M,,0000*51
$GPGSA,A,1,,,,,,,,,,,,,,,*1E
$GPGSV,3,1,11,09,80,303,,12,41,229,,15,40,178,,17,39,070,*70
$GPGSV,3,2,11,22,20,295,,18,18,256,,14,13,317,,26,10,154,*72
$GPGSV,3,3,11,25,08,234,,28,08,056,,24,04,011,*40
*/
    // ignore first two characters.
    tok.p += 2;
    if ( !memcmp(tok.p, "GGA", 3) ) {
        // GPS fix
        Token  tok_time          = nmea_tokenizer_get(tzer,1);
        Token  tok_latitude      = nmea_tokenizer_get(tzer,2);
        Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,3);
        Token  tok_longitude     = nmea_tokenizer_get(tzer,4);
        Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,5);
        Token  tok_altitude      = nmea_tokenizer_get(tzer,9);
        Token  tok_altitudeUnits = nmea_tokenizer_get(tzer,10);

        nmea_reader_update_time(r, tok_time);
        nmea_reader_update_latlong(r, tok_latitude,
                                      tok_latitudeHemi.p[0],
                                      tok_longitude,
                                      tok_longitudeHemi.p[0]);
        nmea_reader_update_altitude(r, tok_altitude, tok_altitudeUnits);

    } else if ( !memcmp(tok.p, "GSA", 3) ) {
          //Satellites are handled by RPC-side code.
          /*
          1    = Mode:
                 M=Manual, forced to operate in 2D or 3D
                 A=Automatic, 3D/2D
          2    = Mode:
                 1=Fix not available
                 2=2D
                 3=3D
          3-14 = IDs of SVs used in position fix (null for unused fields)
          15   = PDOP
          16   = HDOP
          17   = VDOP
          */
        Token tok_mode = nmea_tokenizer_get(tzer,1);
        Token tok_fix  = nmea_tokenizer_get(tzer,2);
        Token tok_id  = nmea_tokenizer_get(tzer,3);
        Token tok_pdop = nmea_tokenizer_get(tzer,15);
        Token tok_hdop = nmea_tokenizer_get(tzer,16);
        Token tok_vdop = nmea_tokenizer_get(tzer,17);
        int i;
        for ( i=0; i<12; i++ ) {
            Token tok_id  = nmea_tokenizer_get(tzer,3+i);
            if ( tok_id.end > tok_id.p )
                id_in_fixed[i]=str2int(tok_id.end, tok_id.p);
        }
    } else if ( !memcmp(tok.p, "GSV", 3) ) {
        //Satellites are handled by RPC-side code.
        Token tok_num_messages   = nmea_tokenizer_get(tzer,1);
        Token tok_msg_number     = nmea_tokenizer_get(tzer,2);
        Token tok_svs_inview     = nmea_tokenizer_get(tzer,3);
        Token tok_sv1_prn_num    = nmea_tokenizer_get(tzer,4);
        Token tok_sv1_elevation  = nmea_tokenizer_get(tzer,5);
        Token tok_sv1_azimuth    = nmea_tokenizer_get(tzer,6);
        Token tok_sv1_snr        = nmea_tokenizer_get(tzer,7);
        Token tok_sv2_prn_num    = nmea_tokenizer_get(tzer,8);
        Token tok_sv2_elevation  = nmea_tokenizer_get(tzer,9);
        Token tok_sv2_azimuth    = nmea_tokenizer_get(tzer,10);
        Token tok_sv2_snr        = nmea_tokenizer_get(tzer,11);
        Token tok_sv3_prn_num    = nmea_tokenizer_get(tzer,12);
        Token tok_sv3_elevation  = nmea_tokenizer_get(tzer,13);
        Token tok_sv3_azimuth    = nmea_tokenizer_get(tzer,14);
        Token tok_sv3_snr        = nmea_tokenizer_get(tzer,15);
        Token tok_sv4_prn_num    = nmea_tokenizer_get(tzer,16);
        Token tok_sv4_elevation  = nmea_tokenizer_get(tzer,17);
        Token tok_sv4_azimuth    = nmea_tokenizer_get(tzer,18);
        Token tok_sv4_snr        = nmea_tokenizer_get(tzer,19);
        int num_messages = str2int(tok_num_messages.p,tok_num_messages.end);
        int msg_number = str2int(tok_msg_number.p,tok_msg_number.end);
        int svs_inview = str2int(tok_svs_inview.p,tok_svs_inview.end);
        D("GSV %d %d %d", num_messages, msg_number, svs_inview );
        if (msg_number==1)
            r->sv_status.used_in_fix_mask=0;
        nmea_reader_update_svs( r, svs_inview, msg_number, 0, tok_sv1_prn_num, tok_sv1_elevation, tok_sv1_azimuth, tok_sv1_snr );
        nmea_reader_update_svs( r, svs_inview, msg_number, 1, tok_sv2_prn_num, tok_sv2_elevation, tok_sv2_azimuth, tok_sv2_snr );
        nmea_reader_update_svs( r, svs_inview, msg_number, 2, tok_sv3_prn_num, tok_sv3_elevation, tok_sv3_azimuth, tok_sv3_snr );
        nmea_reader_update_svs( r, svs_inview, msg_number, 3, tok_sv4_prn_num, tok_sv4_elevation, tok_sv4_azimuth, tok_sv4_snr );
        r->sv_status.num_svs=svs_inview;

        if (num_messages==msg_number)
            update_gps_svstatus(&r->sv_status);
          /*
          1    = Total number of messages of this type in this cycle
          2    = Message number
          3    = Total number of SVs in view
          4    = SV PRN number
          5    = Elevation in degrees, 90 maximum
          6    = Azimuth, degrees from true north, 000 to 359
          7    = SNR, 00-99 dB (null when not tracking)
          8-11 = Information about second SV, same as field 4-7
          12-15= Information about third SV, same as field 4-7
          16-19= Information about fourth SV, same as field 4-7
          */
/*
          int i;
          _gps_state.sv_status.num_svs=ntohl(data[82]) & 0x1F;
          for(i=0;i<ret.num_svs;++i) {
              _gps_state.sv_status.sv_list[i].prn=ntohl(data[83+3*i]);
              _gps_state.sv_status.sv_list[i].elevation=ntohl(data[83+3*i+1]);
              _gps_state.sv_status.sv_list[i].azimuth=ntohl(data[83+3*i+2])/100;
              _gps_state.sv_status.sv_list[i].snr=ntohl(data[83+3*i+2])%100;
          }
          _gps_state.sv_status.used_in_fix_mask=ntohl(data[77]);
          update_gps_svstatus(&_gps_state.

*/
    } else if ( !memcmp(tok.p, "RMC", 3) ) {
        Token  tok_time          = nmea_tokenizer_get(tzer,1);
        Token  tok_fixStatus     = nmea_tokenizer_get(tzer,2);
        Token  tok_latitude      = nmea_tokenizer_get(tzer,3);
        Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,4);
        Token  tok_longitude     = nmea_tokenizer_get(tzer,5);
        Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,6);
        Token  tok_speed         = nmea_tokenizer_get(tzer,7);
        Token  tok_bearing       = nmea_tokenizer_get(tzer,8);
        Token  tok_date          = nmea_tokenizer_get(tzer,9);

        D("in RMC, fixStatus=%c", tok_fixStatus.p[0]);
        if (tok_fixStatus.p[0] == 'A') {
            nmea_reader_update_date( r, tok_date, tok_time );

            nmea_reader_update_latlong( r, tok_latitude,
                                           tok_latitudeHemi.p[0],
                                           tok_longitude,
                                           tok_longitudeHemi.p[0] );

            nmea_reader_update_bearing( r, tok_bearing );
            nmea_reader_update_speed  ( r, tok_speed );
        }
    } else {
        tok.p -= 2;
        D("unknown sentence '%.*s", tok.end-tok.p, tok.p);
    }
    if (r->fix.flags != 0) {
#if GPS_DEBUG
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
#endif
       if (_gps_state->callbacks->location_cb) {
           _gps_state->callbacks->location_cb( &r->fix );
           r->fix.flags = 0;
       } else {
           D("no callback, keeping data until needed !");
       }

/*
        if (r->callback) {
            r->callback( &r->fix );
            r->fix.flags = 0;
        } else {
            D("no callback, keeping data until needed !");
        }
*/
    }
}


static void
nmea_reader_addc( NmeaReader*  r, int  c )
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





static void
gps_state_done( GpsState*  s )
{
    // tell the thread to quit, and wait for it
    char   cmd = CMD_QUIT;
    void*  dummy;
    write( s->control[0], &cmd, 1 );
    pthread_join(s->thread, &dummy);

    // close the control socket pair
    close( s->control[0] ); s->control[0] = -1;
    close( s->control[1] ); s->control[1] = -1;

    // close connection to the QEMU GPS daemon
    close( s->fd ); s->fd = -1;
    s->init = 0;
}


static void
gps_state_start( GpsState*  s )
{
    char  cmd = CMD_START;
    int   ret;

    do {
        ret = write( s->control[0], &cmd, 1 );
    } while (ret < 0 && errno == EINTR);

    if (ret != 1)
        D("%s: could not send CMD_START command: ret=%d: %s",
                __FUNCTION__, ret, strerror(errno));
}


static void
gps_state_stop( GpsState*  s )
{
    char  cmd = CMD_STOP;
    int   ret;

    do { ret=write( s->control[0], &cmd, 1 ); }
    while (ret < 0 && errno == EINTR);

    if (ret != 1)
        D("%s: could not send CMD_STOP command: ret=%d: %s",
          __FUNCTION__, ret, strerror(errno));
}


static int
epoll_register( int  epoll_fd, int  fd )
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


static int
epoll_deregister( int  epoll_fd, int  fd )
{
    int  ret;
    do {
        ret = epoll_ctl( epoll_fd, EPOLL_CTL_DEL, fd, NULL );
    } while (ret < 0 && errno == EINTR);
    return ret;
}

/* this is the main thread, it waits for commands from gps_state_start/stop and,
 * when started, messages from the QEMU GPS daemon. these are simple NMEA sentences
 * that must be parsed to be converted into GPS fixes sent to the framework
 */
static void
gps_state_thread( void*  arg )
{
    GpsState*   state = (GpsState*) arg;
    NmeaReader  reader[1];
    int         epoll_fd   = epoll_create(2);
    int         started    = 0;
    int         gps_fd     = state->fd;
    int         control_fd = state->control[1];

    nmea_reader_init( reader );

    // register control file descriptors for polling
    epoll_register( epoll_fd, control_fd );
    epoll_register( epoll_fd, gps_fd );

    D("gps thread running");

    // now loop
    for (;;) {
        struct epoll_event   events[2];
        int                  ne, nevents;

        nevents = epoll_wait( epoll_fd, events, 2, -1 );
        if (nevents < 0) {
            if (errno != EINTR)
                ALOGE("epoll_wait() unexpected error: %s", strerror(errno));
            continue;
        }
//      D("gps thread received %d events", nevents);
        for (ne = 0; ne < nevents; ne++) {
            if ((events[ne].events & (EPOLLERR|EPOLLHUP)) != 0) {
                ALOGE("EPOLLERR or EPOLLHUP after epoll_wait() !?");
                return;
            }
            if ((events[ne].events & EPOLLIN) != 0) {
                int  fd = events[ne].data.fd;

                if (fd == control_fd) {
                    char  cmd = 255;
                    int   ret;
                    D("gps control fd event");
                    do {
                        ret = read( fd, &cmd, 1 );
                    } while (ret < 0 && errno == EINTR);

                    if (cmd == CMD_QUIT) {
                        D("gps thread quitting on demand");
                        return;
                    } else if (cmd == CMD_START) {
                        if (!started) {
                            D("gps thread starting  location_cb=%p", state->callbacks->location_cb);
                            started = 1;
                            //nmea_reader_set_callback( reader, state->callbacks.location_cb );
                            update_gps_status(GPS_STATUS_SESSION_BEGIN);
                        }
                    } else if (cmd == CMD_STOP) {
                        if (started) {
                            D("gps thread stopping");
                            started = 0;
                            //nmea_reader_set_callback( reader, NULL );
                            update_gps_status(GPS_STATUS_SESSION_END);
                        }
                    }
                } else if (fd == gps_fd) {
                    char  buff[32];
//                  D("gps fd event");
                    for (;;) {
                        int  nn, ret;

                        ret = read( fd, buff, sizeof(buff) );
                        if (ret < 0) {
                            if (errno == EINTR)
                                continue;
                            if (errno != EWOULDBLOCK)
                                ALOGE("error while reading from gps daemon socket: %s:", strerror(errno));
                            break;
                        }
//                      D("received %d bytes: %.*s", ret, ret, buff);
                        for (nn = 0; nn < ret; nn++)
                            nmea_reader_addc( reader, buff[nn] );
                    }
//                  D("gps fd event end");
                } else {
                    ALOGE("epoll_wait() returned unkown fd %d ?", fd);
                }
            }
        }
    }
}


static void
gps_state_init( GpsState*  state, GpsCallbacks* callbacks )
{
    char   prop[PROPERTY_VALUE_MAX];
    char   device[256];
    int    ret;
    int    done = 0;

    struct sigevent tmr_event;

    state->init       = 1;
    state->control[0] = -1;
    state->control[1] = -1;
    state->fd         = -1;
    state->callbacks  = callbacks;
    D("gps_state_init");

    // look for a kernel-provided device name

    if (property_get("ro.kernel.android.gps",prop,"") == 0) {
        D("no kernel-provided gps device name");
        return;
    }

    //if ( snprintf(device, sizeof(device), "/dev/%s", prop) >= (int)sizeof(device) ) {
    //    ALOGE("gps serial device name too long: '%s'", prop);
    //    return;
    //}
    snprintf(device, sizeof(device), "/dev/%s",prop);
    do {
        state->fd = open( device, O_RDWR );
    } while (state->fd < 0 && errno == EINTR);

    if (state->fd < 0) {
        ALOGE("could not open gps serial device %s: %s", device, strerror(errno) );
        return;
    }

    D("gps will read from %s", device);

    // disable echo on serial lines
    if ( isatty( state->fd ) ) {
        struct termios  ios;
        tcgetattr( state->fd, &ios );
        ios.c_lflag = 0;  /* disable ECHO, ICANON, etc... */
        ios.c_oflag &= (~ONLCR); /* Stop \n -> \r\n translation on output */
        ios.c_iflag &= (~(ICRNL | INLCR)); /* Stop \r -> \n & \n -> \r translation on input */
        ios.c_iflag |= (IGNCR | IXOFF);  /* Ignore \r & XON/XOFF on input */
        ios.c_cflag = B115200 | CRTSCTS | CS8 | CLOCAL | CREAD;	 // baud 115200, cs8
//GJ
        tcsetattr( state->fd, TCSANOW, &ios );
//EGJ
    }

    if ( socketpair( AF_LOCAL, SOCK_STREAM, 0, state->control ) < 0 ) {
        ALOGE("could not create thread control socket pair: %s", strerror(errno));
        goto Fail;
    }

    state->thread = callbacks->create_thread_cb( "gps_state_thread", gps_state_thread, state );

    if ( !state->thread ) {
        ALOGE("could not create gps thread: %s", strerror(errno));
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


static int
freerunner_gps_init(GpsCallbacks* callbacks)
{
    D("freerunner_gps_init");
    GpsState*  s = _gps_state;

    if (!s->init)
        gps_state_init(s, callbacks);

    if (s->fd < 0)
        return -1;



    return 0;
}

static void
freerunner_gps_cleanup(void)
{
    GpsState*  s = _gps_state;

    if (s->init)
        gps_state_done(s);
}


static int
freerunner_gps_start()
{
    GpsState*  s = _gps_state;

    if (!s->init) {
        DFR("%s: called with uninitialized state !!", __FUNCTION__);
        return -1;
    }

    D("%s: called", __FUNCTION__);
    gps_state_start(s);
    return 0;
}


static int
freerunner_gps_stop()
{
    GpsState*  s = _gps_state;

    if (!s->init) {
        DFR("%s: called with uninitialized state !!", __FUNCTION__);
        return -1;
    }

    D("%s: called", __FUNCTION__);
    gps_state_stop(s);
    return 0;
}


static int
freerunner_gps_inject_time(GpsUtcTime time, int64_t timeReference, int uncertainty)
{
    return 0;
}


static int
freerunner_gps_inject_location(double latitude, double longitude, float accuracy)
{
    return 0;
}

static void
freerunner_gps_delete_aiding_data(GpsAidingData flags)
{
}

static int freerunner_gps_set_position_mode(GpsPositionMode mode, GpsPositionRecurrence recurrence,
        uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time)
{
    GpsState*  s = _gps_state;

    // only standalone supported for now.
//  if (mode != GPS_POSITION_MODE_STANDALONE)
//      return -1;

    if (!s->init) {
        D("%s: called with uninitialized state !!", __FUNCTION__);
        return -1;
    }

    D("set_position_mode: mode=%d recurrence=%d min_interval=%d preferred_accuracy=%d preferred_time=%d",
            mode, recurrence, min_interval, preferred_accuracy, preferred_time);

    return 0;
}

static const void*
freerunner_gps_get_extension(const char* name)
{
    return NULL;
}

static const GpsInterface  freerunnerGpsInterface = {
    sizeof(GpsInterface),
    freerunner_gps_init,
    freerunner_gps_start,
    freerunner_gps_stop,
    freerunner_gps_cleanup,
    freerunner_gps_inject_time,
    freerunner_gps_inject_location,
    freerunner_gps_delete_aiding_data,
    freerunner_gps_set_position_mode,
    freerunner_gps_get_extension,
};

const GpsInterface* gps_get_hardware_interface()
{
    D("gps dev get_hardware_interface");
    return &freerunnerGpsInterface;
}



/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       D E V I C E                                     *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

static void gps_dev_power(int state)
{
    char   prop[PROPERTY_VALUE_MAX];
    int fd;
    char cmd = '0';
    int ret;
#if 0
    if (property_get("gps.power_on",prop,GPS_POWER_IF) == 0) {
        ALOGE("no gps power interface name");
        return;
    }

    do {
        fd = open( prop, O_RDWR );
    } while (fd < 0 && errno == EINTR);

    if (fd < 0) {
        ALOGE("could not open gps power interfcae: %s", prop );
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

    DFR("gps power state = %c", cmd);
#endif
    return;
}

static void gps_dev_send(int fd, char *msg)
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
}

static unsigned char gps_dev_calc_nmea_csum(char *msg)
{
    unsigned char csum = 0;
    int i;

    for (i = 1; msg[i] != '*'; ++i) {
        csum ^= msg[i];
    }

    return csum;
}

static void gps_dev_set_nmea_message_rate(int fd, char *msg, int rate)
{
    char buff[50];
    int i;

    sprintf(buff, "$PUBX,40,%s,%d,%d,%d,0*", msg, rate, rate, rate);

    i = strlen(buff);

    sprintf((buff + i), "%02x\r\n", gps_dev_calc_nmea_csum(buff));

    gps_dev_send(fd, buff);

    D("gps sent to device: %s", buff);
}

static void gps_dev_set_baud_rate(int fd, int baud)
{
    char buff[50];
    int i, u;

    for (u = 0; u < 3; ++u) {

        sprintf(buff, "$PUBX,41,%d,0003,0003,%d,0*", u, baud);

        i = strlen(buff);

        sprintf((buff + i), "%02x\r\n", gps_dev_calc_nmea_csum(buff));

        gps_dev_send(fd, buff);

        D("gps sent to device: %s", buff);

    }
}

static void gps_dev_set_message_rate(int fd, int rate)
{

    unsigned int i;

    char *msg[] = {
                     "GGA", "GLL", "ZDA",
                     "VTG", "GSA", "GSV",
                     "RMC"
                  };

    for (i = 0; i < sizeof(msg)/sizeof(msg[0]); ++i) {
        gps_dev_set_nmea_message_rate(fd, msg[i], rate);
    }

    return;
}

static void gps_dev_init(int fd)
{
    gps_dev_power(1);

    //usleep(1000*1000);

    // To set to STOP state
    //gps_dev_stop(fd);

    return;
}

static void gps_dev_deinit(int fd)
{
    gps_dev_power(0);
}

static void gps_dev_start(int fd)
{
    // Set full message rate
    gps_dev_set_message_rate(fd, GPS_DEV_HIGH_UPDATE_RATE);

    D("gps dev start initiated");
}

static void gps_dev_stop(int fd)
{
    // Set slow message rate
    gps_dev_set_message_rate(fd, GPS_DEV_SLOW_UPDATE_RATE);

    D("gps dev stop initiated");
}

static int open_gps(const struct hw_module_t* module, char const* name, struct hw_device_t** device)
{
    D("gps dev open_gps");
    struct gps_device_t *dev = malloc(sizeof(struct gps_device_t));
    memset(dev, 0, sizeof(*dev));

    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = 0;
    dev->common.module = (struct hw_module_t*)module;
    //    dev->common.close = (int (*)(struct hw_device_t*))close_lights;
    dev->get_gps_interface = gps_get_hardware_interface;

    *device = &dev->common;
    return 0;
}


static struct hw_module_methods_t gps_module_methods = {
    .open = open_gps
};

struct hw_module_t HAL_MODULE_INFO_SYM = {
    .tag = HARDWARE_MODULE_TAG,
    .version_major = 1,
    .version_minor = 0,
    .id = GPS_HARDWARE_MODULE_ID,
    .name = "USB GPS Module",
    .author = "The Android-x86 Open Source Project",
    .methods = &gps_module_methods,
};
