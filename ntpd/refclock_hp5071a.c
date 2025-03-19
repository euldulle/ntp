/*
 * hp5071a - clock driver for HP 5071A Cs standard 
 */

#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#if defined(REFCLOCK) && defined(CLOCK_HP5071A)

#include "ntpd.h"
#include "ntp_io.h"
#include "ntp_refclock.h"
#include "ntp_stdlib.h"

#include <stdio.h>
#include <ctype.h>

/* Version 0.1 April  1, 1995  
 *         0.2 April 25, 1995
 *         0.3 July 23,  2014
 *             tolerant of missing timecode response prompt and sends
 *             clear status if prompt indicates error;
 *             can use either local time or UTC from receiver;
 *             can get receiver status screen via flag4
 *
 * WARNING!: This driver is UNDER CONSTRUCTION
 * Everything in here should be treated with suspicion.
 * If it looks wrong, it probably is.
 *
 * Comments and/or questions to: 
 *                      original HP58503A GPS refclock :
 *                               Dave Vitanye
 *                               Hewlett Packard Company
 *                               dave@scd.hp.com
 *                               (408) 553-2856
 *                               
 *                      specific HP5071A refclock :
 *                               François Meyer    Tel : (+33) 3 81 66 69 27
 *                               Observatoire de Besancon 
 *                               Institut UTINAM * Universite de Franche-Comte * CNRS UMR 6213 ***
 *                               fm@ltfb.fr
 *
 * Thanks to the author of the PST driver, which was the starting point for
 * the 58503A driver and to the author of the 58530A driver which was the
 * starting point for this one (HP5071A)
 *
 * This driver supports the HP 5071A Cs frequency standard.
 *
 * The driver uses the poll sequence 
 *
 *  :PTIME:MJD?;TIME?;LEAP?;LEAP:MJD?;DUR?
 *
 * the answer comes as follows : 
 *
 *  +56849;+9,+41,+10;0;+56849;+61 
 *
 * Field (semi-colon-wise) 4 (MJD of the next leap second) and 5 (duration of the last minute
 * of that leap-second day) are only relevant if field 3 is non zero.
 * 
 * WARNING : this answer is asynchronous, so is not synchronised in any way with the 5071a 1PPS signal ;
 * this produces a 1s ambiguity that will disqualify the associated peer ;
 * this can be avoided by sync-ing the poll sequence with the 5071 1PPS using a 
 * small device that will act as a buffer on the serial line to the 5071.
 * There is still a delay associated but this delay is constant at the 1ms level
 * and can be accounted for in the peer configuration. FIXME

 * to get a response from the device. The device responds with a timecode string of ASCII
 * printing characters, followed by a <cr><lf>, followed by a prompt string
 * issued by the device, in the following format:

 * FIXME to match HP5071A output
 * FIXME suppress echo at poll

 *
 */

/*
 * Fudge Factors
 *
 * Fudge time1 is used to accomodate the timecode serial interface adjustment.
 * FIXME Fudge flag4 can be set to request a receiver status screen summary, which
 *       is recorded in the clockstats file.
 */

/*
 * Interface definitions
 */
#define	DEVICE		"/dev/hp5071a%d" /* device name and unit */
#define	SPEED232	B9600	/* uart speed (9600 baud) */
#define	SPEED232Z	B19200	/* uart speed (19200 baud) */
#define	PRECISION	(-10)	/* precision assumed (about 1 ms) */
#define	REFID		"5071\0"	/*  reference ID */
#define	DESCRIPTION	"HP 5071A Cesium frequency standard" 

#define SMAX            23*80+1 /* for :SYSTEM:PRINT? status screen response */

#define MTZONE          2       /* number of fields in timezone reply */
#define MTCODET2        12      /* number of fields in timecode format T2 */
#define NTCODET2        21      /* number of chars to checksum in format T2 */


/*
 * Unit control structure
 */
struct hp5071aunit {
	int	pollcnt;	/* poll message counter */
	int     linecnt;        /* set for expected multiple line responses */
	char	*lastptr;	/* pointer to receiver response data */
	char    statscrn[SMAX]; /* receiver status screen buffer */
};

/*
 * Function prototypes
 */
static	int	hp5071a_start	(int, struct peer *);
static	void	hp5071a_shutdown	(int, struct peer *);
static	void	hp5071a_receive	(struct recvbuf *);
static	void	hp5071a_poll	(int, struct peer *);

/*
 * Transfer vector
 */
struct	refclock refclock_hp5071a = {
	hp5071a_start,		/* start up driver */
	hp5071a_shutdown,		/* shut down driver */
	hp5071a_poll,		/* transmit poll message */
	noentry,		/* not used (old hp5071a_control) */
	noentry,		/* initialize driver */
	noentry,		/* not used (old hp5071a_buginfo) */
	NOFLAGS			/* not used */
};


#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#define 	sgn(x)   ((x) < 0 ? -1: 1)
#include <unistd.h>

/* Table to get from month to day of the year */
   const int days_of_year [12] = {
       0,  31,  59,  90, 120, 151, 181, 212, 243, 273, 304, 334
   };

//extern const int days_of_year[12];

int date2doy (int year, int mon, int day);
void mjdtocal(int mjd, int *y, int*m, int*d);

/* 
 * unpack_date - get day and year from date
 */
int date2doy (int year, int mon, int day){
    int doy;

	/* Check month is inside array bounds */
	if ((mon < 1) || (mon > 12)) 
		return -1;

	doy = day + days_of_year[mon - 1];

	if ( !(year % 4) && ((year % 100) || 
			     (!(year % 100) && !(year%400)))
	     &&(mon > 2))
		doy ++; /* leap year and March or later */

	return doy;
}

/*
 * mjdtocal(int mjd, int *y, int*m, int*d) 
 *  computes year month day for given mjd
 */
void mjdtocal(int mjd, int *y, int*m, int*d){
	int n,i,j;
	int l;
		
	l = mjd + 2468569 +1;
	n = ( 4 * l ) / 146097;
	l = l - ( 146097 * n + 3 ) / 4;
	i = ( 4000 * ( l + 1 ) ) / 1461001;// (that's 1,461,001)
	l = l - ( 1461 * i ) / 4 + 31;
	j = ( 80 * l ) / 2447;
	*d = l - ( 2447 * j ) / 80;
	l = j / 11;
	*m = j + 2 - ( 12 * l );
	*y = 100 * ( n - 49 ) + i + l;
    }
/*
 * hp5071a_start - open the devices and initialize data for processing
 */
static int
hp5071a_start(
	int unit,
	struct peer *peer
	)
{
	register struct hp5071aunit *up;
	struct refclockproc *pp;
	int fd;
	char device[20];

	/*
	 * Open serial port. Use CLK line discipline, if available.
	 */
	snprintf(device, sizeof(device), DEVICE, unit);
	/* mode parameter to server config line shares ttl slot */
    if (!(fd = refclock_open(&peer->srcadr,device, SPEED232, LDISC_PPS)))
        return (0);
	/*
	 * Allocate and initialize unit structure
	 */
	up = emalloc(sizeof(*up));
	memset(up, 0, sizeof(*up));
	pp = peer->procptr;
	pp->io.clock_recv = hp5071a_receive;
	pp->io.srcclock = (caddr_t)peer;
	pp->io.datalen = 0;
	pp->io.fd = fd;
	if (!io_addclock(&pp->io)) {
		close(fd);
		pp->io.fd = -1;
		free(up);
		return (0);
	}
	pp->unitptr = (caddr_t)up;

	/*
	 * Initialize miscellaneous variables
	 */
	peer->precision = PRECISION;
	pp->clockdesc = DESCRIPTION;
	memcpy((char *)&pp->refid, REFID, 4);

	*up->statscrn = '\0';
	up->lastptr = up->statscrn;
	up->pollcnt = 2;

	/*
	 * Get the identifier string, which is logged but otherwise ignored,
	 */
	up->linecnt = 1;
	if (write(pp->io.fd, "*IDN?\r", 6) != 6)
	    refclock_report(peer, CEVNT_FAULT);

	return (1);
}


/*
 * hp5071a_shutdown - shut down the clock
 */
static void
hp5071a_shutdown(
	int unit,
	struct peer *peer
	)
{
	register struct hp5071aunit *up;
	struct refclockproc *pp;

	pp = peer->procptr;
	up = (struct hp5071aunit *)pp->unitptr;
	if (-1 != pp->io.fd)
		io_closeclock(&pp->io);
	if (NULL != up)
		free(up);
}


/*
 * hp5071a_receive - receive data from the serial interface
 */
static void
hp5071a_receive(
	struct recvbuf *rbufp
	)
{
	register struct hp5071aunit *up;
	struct refclockproc *pp;
	struct peer *peer;
	l_fp trtmp;
	int m;
	int month, day;
	char *tcp;              /* timecode pointer (skips over the prompt) */
	char prompt[BMAX];      /* prompt in response from receiver */

    int mjd;
    int leap, mjdleap, yearleap, monthleap, dayleap, leapdur;     

	/*
	 * Initialize pointers and read the receiver response
	 */
	peer = (struct peer *)rbufp->recv_srcclock;
	pp = peer->procptr;
	up = (struct hp5071aunit *)pp->unitptr;
	*pp->a_lastcode = '\0';
	pp->lencode = refclock_gtlin(rbufp, pp->a_lastcode, BMAX, &trtmp);

#ifdef DEBUG
	if (debug)
	    printf("hp5071a: lencode: %d timecode:%s\n",
		   pp->lencode, pp->a_lastcode);
#endif

	/*
	 * If there's no characters in the reply, we can quit now
	 */
	if (pp->lencode == 0)
	    return;

	/*
	 * If linecnt is greater than zero, we are getting information only,
	 * such as the receiver identification string or the receiver status
	 * screen, so put the receiver response at the end of the status
	 * screen buffer. When we have the last line, write the buffer to
	 * the clockstats file and return without further processing.
	 *
	 * If linecnt is zero, we are expecting either the timezone
	 * or a timecode. At this point, also write the response
	 * to the clockstats file, and go on to process the prompt (if any),
	 * timezone, or timecode and timestamp.
	 */


	if (up->linecnt-- > 0) {
		if ((int)(pp->lencode + 2) <= (SMAX - (up->lastptr - up->statscrn))) {
			*up->lastptr++ = '\n';
			(void)strcpy(up->lastptr, pp->a_lastcode);
			up->lastptr += pp->lencode;
		}
		if (up->linecnt == 0) 
		    record_clock_stats(&peer->srcadr, up->statscrn);
               
		return;
	}

	record_clock_stats(&peer->srcadr, pp->a_lastcode);
	pp->lastrec = trtmp;
            
	up->lastptr = up->statscrn;
	*up->lastptr = '\0';
	up->pollcnt = 2;

	/*
	 * We get down to business: get a prompt if one is there, issue
	 * a clear status command if it contains an error indication.
	 * Next, check for either the timezone reply or the timecode reply
	 * and decode it.  If we don't recognize the reply, or don't get the
	 * proper number of decoded fields, or get an out of range timezone,
	 * or if the timecode checksum is bad, then we declare bad format
	 * and exit.
	 *
	 * Timezone format (including nominal prompt):
	 * scpi > -H,-M<cr><lf>
	 *
	 * Timecode format (including nominal prompt):
     *
     * scpi > +56849;+9,+41,+10;0;+56849;+61 
     *
	 */

	(void)strcpy(prompt,pp->a_lastcode);
	tcp = strrchr(pp->a_lastcode,'>');
	if (tcp == NULL)
	    tcp = pp->a_lastcode; 
	else
	    tcp++;
	prompt[tcp - pp->a_lastcode] = '\0';
	while ((*tcp == ' ') || (*tcp == '\t')) tcp++;

	/*
	 * deal with an error indication in the prompt here
	 */
	if (strrchr(prompt,'E') > strrchr(prompt,'s')){
#ifdef DEBUG
		if (debug)
		    printf("hp5071a: error indicated in prompt: %s\n", prompt);
#endif
		if (write(pp->io.fd, "*CLS\r\r", 6) != 6)
		    refclock_report(peer, CEVNT_FAULT);
	}

	m=sscanf(tcp,"%d;%d,%d,%d;%d;%d;%d", &mjd, &pp->hour, &pp->minute, &pp->second, &leap, &mjdleap, &leapdur);

    if (m<7) {
#ifdef DEBUG
			if (debug)
			    printf("hp5071a: only %d fields recognized in message %s\n", m, tcp);
#endif
			refclock_report(peer, CEVNT_BADREPLY);
			return;
		}
    else{
#ifdef DEBUG
			if (debug)
			    printf("hp5071a: %d fields recognized in message %s\n", m, tcp);
#endif
        }


	/* 
	 * Compute day/month/year from MJD
	 */
    mjdtocal(mjd, &pp->year, &month, &day);     
	/* 
	 * Compute day of year from day/month/year
	 */
    pp->day=date2doy(pp->year, month, day);

	/*
     * WARNING 
     *
     * The driver assumes that the device is a primary source for leap seconds,
     * and then, that people maintaining the device take care of its proper 
     * scheduling of leap seconds via the interface provided by the device.
	 */
    pp->leap = LEAP_NOWARNING;

    if (leap != 0){
    /*
     *  get the date of the next scheduled leap second
     *  after the MJD provided by the device
     */
        mjdtocal(mjdleap, &yearleap, &monthleap, &dayleap);     
        
    /*
     *  take care of possible long term notice of leap seconds
     *  by checking correct month and year
     */
        if (monthleap==month && yearleap==pp->year){
            if (leapdur==59){
                pp->leap = LEAP_DELSECOND;
                }
            if (leapdur==61){
                pp->leap = LEAP_ADDSECOND;
                }
        }
    } 

	/*
	 * Process the new sample in the median filter and determine the
	 * reference clock offset and dispersion. We use lastrec as both
	 * the reference time and receive time in order to avoid being
	 * cute, like setting the reference time later than the receive
	 * time, which may cause a paranoid protocol module to chuck out
	 * the data.
	 */
	if (!refclock_process(pp)) {
		refclock_report(peer, CEVNT_BADTIME);
		return;
	}
	pp->lastref = pp->lastrec;
	refclock_receive(peer);

	/*
	 * If CLK_FLAG4 is set, ask for the status screen response.
	 */
	if (pp->sloppyclockflag & CLK_FLAG4){
		up->linecnt = 22; 
		if (write(pp->io.fd, ":SYSTEM:PRINT?\r", 15) != 15)
		    refclock_report(peer, CEVNT_FAULT);
	}
}


/*
 * hp5071a_poll - called by the transmit procedure
 */
static void
hp5071a_poll(
	int unit,
	struct peer *peer
	)
{
	register struct hp5071aunit *up;
	struct refclockproc *pp;

	/*
	 * Time to poll the clock. FIXME HP5071 The HP 58503A responds to a
	 * poll with the appropriate query 
	 *  
     *  :PTIME:MJD?;TIME?;LEAP?;LEAP:MJD?;DUR?
     *
     * by returning a timecode in the format specified
	 * above. If nothing is heard from the clock for two polls,
	 * declare a timeout and keep going.
	 */
#ifdef DEBUG
    if (debug)
        printf("hp5071a: polling\n");
#endif
 	pp = peer->procptr;
	up = (struct hp5071aunit *)pp->unitptr;
	if (up->pollcnt == 0)
	    refclock_report(peer, CEVNT_TIMEOUT);
	else
	    up->pollcnt--;

    if (write(pp->io.fd, ":PTIME:MJD?;TIME?;LEAP?;LEAP:MJD?;DUR?\r", 39) != 39) {
		refclock_report(peer, CEVNT_FAULT);
	}
	else
	    pp->polls++;
}

#else
int refclock_hp5071a_bs;
#endif /* REFCLOCK */
