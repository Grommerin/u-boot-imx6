/*
 * (C) Copyright 2002 SIXNET, dge@sixnetio.com.
 *
 * (C) Copyright 2004, Li-Pro.Net <www.li-pro.net>
 * Stephan Linz <linz@li-pro.net>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * Date & Time support for DS1306 RTC using SPI:
 *
 *    - SXNI855T:    it uses its own soft SPI here in this file
 *    - all other:   use the external spi_xfer() function
 *                   (see include/spi.h)
 */

#include <common.h>
#include <command.h>
#include <rtc.h>
#include <spi.h>

#if defined(CONFIG_CMD_DATE)

#define	RTC_SECONDS		        (0x01)
#define	RTC_MINUTES		        (0x02)
#define	RTC_HOURS		        (0x03)
#define	RTC_DAY_OF_WEEK		    (0x04)
#define	RTC_DATE_OF_MONTH	    (0x05)
#define	RTC_MONTH		        (0x06)
#define	RTC_YEAR		        (0x07)

#define RTC_HUND_SECONDS_ALARM  (0x08)
#define	RTC_SECONDS_ALARM	    (0x09)
#define	RTC_MINUTES_ALARM	    (0x0A)
#define	RTC_HOURS_ALARM	        (0x0B)
#define	RTC_DAY_OF_WEEK_ALARM	(0x0C)

#define	RTC_CONTROL		        (0x0D)
#define	RTC_STATUS		        (0x0E)
#define	RTC_TRCHARGER	        (0x0F)

#define RTC_CONTROL_EOSC_BIT    (0x80)
#define RTC_CONTROL_BBSQI_BIT   (0x20)
#define RTC_CONTROL_RS2_BIT     (0x10)
#define RTC_CONTROL_RS1_BIT     (0x08)
#define RTC_CONTROL_INTCN_BIT   (0x04)
#define RTC_CONTROL_AIE_BIT     (0x01)

#define RTC_STATUS_OSF_BIT      (0x80)
#define RTC_STATUS_AF_BIT       (0x01)

#define RTC_TRCHARGER_TCS3_BIT  (0x80)
#define RTC_TRCHARGER_TCS2_BIT  (0x40)
#define RTC_TRCHARGER_TCS1_BIT  (0x20)
#define RTC_TRCHARGER_TCS0_BIT  (0x10)
#define RTC_TRCHARGER_DS1_BIT   (0x08)
#define RTC_TRCHARGER_DS0_BIT   (0x04)
#define RTC_TRCHARGER_ROUT1_BIT (0x02)
#define RTC_TRCHARGER_ROUT0_BIT (0x01)
#define RTC_TRCHARGER_ENABLE    (RTC_TRCHARGER_TCS3_BIT | RTC_TRCHARGER_TCS1_BIT)

/* ************************************************************************* */

static unsigned char rtc_read (unsigned char reg);
static void rtc_write (unsigned char reg, unsigned char val);

static struct spi_slave *slave;

/* read clock time from DS1390 and return it in *tmp */
int rtc_get (struct rtc_time *tmp)
{
	unsigned char sec, min, hour, mday, wday, mon, year;
	unsigned char control, status, tcharger;

	if (!slave) {
		slave = spi_setup_slave(CONFIG_SYS_SPI_RTC_BUS, CONFIG_SYS_SPI_RTC_DEVID,
		                        CONFIG_SYS_SPI_RTC_SPEED, SPI_MODE_3);
		if (!slave) { return (-1); }
	}

	if (spi_claim_bus(slave)) { return (-1); }

	control  = rtc_read(RTC_CONTROL);
	status   = rtc_read(RTC_STATUS);
	tcharger = rtc_read(RTC_TRCHARGER);

	sec  = rtc_read(RTC_SECONDS);
	min  = rtc_read(RTC_MINUTES);
	hour = rtc_read(RTC_HOURS);
	mday = rtc_read(RTC_DATE_OF_MONTH);
	wday = rtc_read(RTC_DAY_OF_WEEK);
	mon  = rtc_read(RTC_MONTH);
	year = rtc_read(RTC_YEAR);

	spi_release_bus(slave);

	printf ("Get regs: control 0x%02x, status 0x%02x, trickle-charger 0x%02x\n",
				   control, status, tcharger);
	printf ("Get BDC: year 0x%02x mon 0x%02x mday 0x%02x wday 0x%02x "
					  "hour 0x%02x min 0x%02x sec 0x%02x\n",
					  year, mon, mday, wday, hour, min, sec);
	printf ("Get alarms: wday: 0x%02x hour: 0x%02x min: 0x%02x sec: 0x%02x\n",
					  rtc_read(RTC_DAY_OF_WEEK_ALARM), rtc_read(RTC_HOURS_ALARM),
					  rtc_read(RTC_MINUTES_ALARM), rtc_read(RTC_SECONDS_ALARM));

	tmp->tm_sec = bcd2bin(sec & 0x7F);  /* convert Seconds */
	tmp->tm_min = bcd2bin(min & 0x7F);  /* convert Minutes */

	/* convert Hours */
	tmp->tm_hour = (hour & 0x40)
		? ((hour & 0x20)			/* 12 hour mode */
		   ? bcd2bin(hour & 0x1F) + 11		/* PM */
		   : bcd2bin(hour & 0x1F) - 1		/* AM */
		)
		: bcd2bin (hour & 0x3F);	/* 24 hour mode */

	tmp->tm_mday  = bcd2bin(mday & 0x3F);		/* convert Day of the Month */
	tmp->tm_mon   = bcd2bin(mon & 0x1F);		/* convert Month */
	tmp->tm_year  = bcd2bin(year) + ((mon & 0x80) ? 2000 : 1900);	/* convert Year */
	tmp->tm_wday  = bcd2bin(wday & 0x07) - 1;	/* convert Day of the Week */
	tmp->tm_yday  = 0;
	tmp->tm_isdst = 0;

	printf ("Get date: %4d-%02d-%02d (wday=%d) %2d:%02d:%02d\n",
		   tmp->tm_year, tmp->tm_mon, tmp->tm_mday, tmp->tm_wday,
		   tmp->tm_hour, tmp->tm_min, tmp->tm_sec);

	return (0);
}

/* ------------------------------------------------------------------------- */

/* set clock time from *tmp in DS1390 RTC */
int rtc_set (struct rtc_time *tmp)
{
	unsigned char sec, min, hour, mday, wday, mon, year;

	if (!slave) {
		slave = spi_setup_slave(CONFIG_SYS_SPI_RTC_BUS, CONFIG_SYS_SPI_RTC_DEVID,
		                        CONFIG_SYS_SPI_RTC_SPEED, SPI_MODE_3/* | SPI_CS_HIGH*/);
		if (!slave) { return (-1); }
	}

	if (spi_claim_bus(slave)) { return (-1); }

	printf ("Set date: %4d-%02d-%02d (wday=%d) %2d:%02d:%02d\n",
	           tmp->tm_year, tmp->tm_mon, tmp->tm_mday, tmp->tm_wday,
	           tmp->tm_hour, tmp->tm_min, tmp->tm_sec);

	sec  = bin2bcd(tmp->tm_sec);
	min  = bin2bcd(tmp->tm_min);
	hour = bin2bcd(tmp->tm_hour);
	wday = bin2bcd(tmp->tm_wday + 1);
	mday = bin2bcd(tmp->tm_mday);
	/* Set century bit if year > 2000 */
	mon  = bin2bcd(tmp->tm_mon) | ((tmp->tm_year >= 2000) ? 0x80 : 0x00);
	year = bin2bcd(tmp->tm_year - ((tmp->tm_year >= 2000) ? 2000 : 1900));

	rtc_write(RTC_SECONDS, sec);
	rtc_write(RTC_MINUTES, min);
	rtc_write(RTC_HOURS, hour);
	rtc_write(RTC_DAY_OF_WEEK, wday);
	rtc_write(RTC_DATE_OF_MONTH, mday);
	rtc_write(RTC_MONTH, mon);
	rtc_write(RTC_YEAR, year);

	printf ("Set BDC: year 0x%02x mon 0x%02x mday 0x%02x wday 0x%02x "
	                  "hour 0x%02x min 0x%02x sec 0x%02x\n",
	                  year, mon, mday, wday, hour, min, sec);

	spi_release_bus(slave);

	return (0);
}

/* ------------------------------------------------------------------------- */

/* reset the DS1390 */
void rtc_reset (void)
{
	/* Assuming Vcc = 2.0V (lowest speed) */
	if (!slave) {
		slave = spi_setup_slave(CONFIG_SYS_SPI_RTC_BUS, CONFIG_SYS_SPI_RTC_DEVID,
		                        CONFIG_SYS_SPI_RTC_SPEED, SPI_MODE_3/* | SPI_CS_HIGH*/);
		if (!slave) { return; }
	}

	if (spi_claim_bus(slave)) { return; }

	/* clear the control register */
	rtc_write(RTC_CONTROL, 0x00);	/* 1st step: reset WP */
	rtc_write(RTC_CONTROL, 0x00);	/* 2nd step: reset 1Hz, AIE1, AIE0 */

	/* reset all alarms */
	rtc_write(RTC_HUND_SECONDS_ALARM, 0x00);
	rtc_write(RTC_SECONDS_ALARM, 0x00);
	rtc_write(RTC_MINUTES_ALARM, 0x00);
	rtc_write(RTC_HOURS_ALARM, 0x00);
	rtc_write(RTC_DAY_OF_WEEK_ALARM, 0x00);

	spi_release_bus(slave);
}

int rtc_run (void)
{
    unsigned char control;

    if (!slave) {
        slave = spi_setup_slave(CONFIG_SYS_SPI_RTC_BUS, CONFIG_SYS_SPI_RTC_DEVID,
                                CONFIG_SYS_SPI_RTC_SPEED, SPI_MODE_3/* | SPI_CS_HIGH*/);
        if (!slave) { return (-1); }
    }

    puts("Run external RTC DS1390\n");

    if (spi_claim_bus(slave)) { return (-1); }

    control = rtc_read(RTC_CONTROL);
    /* set EOSC in control register */
    rtc_write(RTC_CONTROL, (control & ~RTC_CONTROL_EOSC_BIT));

    /* set Vbackup charge mode: charge ON, No diode, 250 Ohm (~13mA) */
    rtc_write(RTC_TRCHARGER, (RTC_TRCHARGER_ENABLE | RTC_TRCHARGER_DS0_BIT  | RTC_TRCHARGER_ROUT0_BIT));

    spi_release_bus(slave);

    return (0);
}

/* ------------------------------------------------------------------------- */

static unsigned char rtc_read (unsigned char reg)
{
	int ret;

	ret = spi_w8r8(slave, reg);
	return ((ret < 0) ? (0) : (ret));
}

/* ------------------------------------------------------------------------- */

static void rtc_write (unsigned char reg, unsigned char val)
{
	unsigned char dout[2];	/* SPI Output Data Bytes */
	unsigned char din[2];	/* SPI Input Data Bytes */

	dout[0] = 0x80 | reg;
	dout[1] = val;

	spi_xfer (slave, 16, dout, din, SPI_XFER_BEGIN | SPI_XFER_END);
}

#endif /* end of code exclusion (see #ifdef CONFIG_SXNI855T above) */
