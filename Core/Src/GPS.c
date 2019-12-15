#include "GPS.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx.h"

GPS_t GPS;
//##################################################################################################################
double convertDegMinToDecDeg (float degMin)
{
	double min = 0.0;
	double decDeg = 0.0;

	//get the minutes, fmod() requires double
	min = fmod((double)degMin, 100.0);

	//rebuild coordinates in decimal degrees
	degMin = (int) ( degMin / 100 );
	decDeg = degMin + ( min / 60 );

	return decDeg;
}
//##################################################################################################################
void	GPS_Init(void)
{
	GPS.rxIndex=0;
	HAL_UART_Receive_IT(&huart2,&GPS.rxTmp,1);
}
//##################################################################################################################
void	GPS_CallBack(void)
{
	GPS.LastTime=HAL_GetTick();
	if(GPS.rxIndex < sizeof(GPS.rxBuffer)-2)
	{
		GPS.rxBuffer[GPS.rxIndex] = GPS.rxTmp;
		GPS.rxIndex++;
	}
	HAL_UART_Receive_IT(&huart2,&GPS.rxTmp,1);
}
//##################################################################################################################
void	GPS_Process(void)
{
	if( (HAL_GetTick()-GPS.LastTime>50) && (GPS.rxIndex>0))
	{
		char *ggaStr, *rmcStr, *gsaStr, *gsvStr, *vtgStr;
		ggaStr=strstr((char*)GPS.rxBuffer,"$GPGGA,");// checking whether
		rmcStr=strstr((char*)GPS.rxBuffer,"$GPRMC,");// the incoming NMEA
		gsaStr=strstr((char*)GPS.rxBuffer,"$GPGSA,");// data is a GGA, RMC,
		gsvStr=strstr((char*)GPS.rxBuffer,"$GPGSV,");//	GSA, GSV or GTV sentence
		vtgStr=strstr((char*)GPS.rxBuffer,"$GPVTG,");//

		if(ggaStr!=NULL) //$GPGGA sentence found!
		{
			memset(&GPS.GPGGA,0,sizeof(GPS.GPGGA));
			sscanf(ggaStr,"$GPGGA,%02d%02d%02d.%03hd,%f,%c,%f,%c,%2d,%2d,%f,%f,%c,%hd,%s,*%2s\r\n",&GPS.GPGGA.UTC_Hour,&GPS.GPGGA.UTC_Min,&GPS.GPGGA.UTC_Sec,&GPS.GPGGA.UTC_MicroSec,&GPS.GPGGA.Latitude,&GPS.GPGGA.NS_Indicator,&GPS.GPGGA.Longitude,&GPS.GPGGA.EW_Indicator,&GPS.GPGGA.PositionFixIndicator,&GPS.GPGGA.SatellitesUsed,&GPS.GPGGA.HDOP,&GPS.GPGGA.MSL_Altitude,&GPS.GPGGA.MSL_Units,&GPS.GPGGA.AgeofDiffCorr,GPS.GPGGA.DiffRefStationID,GPS.GPGGA.CheckSum);
			if(GPS.GPGGA.NS_Indicator==0)
				GPS.GPGGA.NS_Indicator='-';
			if(GPS.GPGGA.EW_Indicator==0)
				GPS.GPGGA.EW_Indicator='-';
			if(GPS.GPGGA.Geoid_Units==0)
				GPS.GPGGA.Geoid_Units='-';
			if(GPS.GPGGA.MSL_Units==0)
				GPS.GPGGA.MSL_Units='-';
			GPS.GPGGA.LatitudeDecimal=convertDegMinToDecDeg(GPS.GPGGA.Latitude);
			GPS.GPGGA.LongitudeDecimal=convertDegMinToDecDeg(GPS.GPGGA.Longitude);
		}

		if(rmcStr!=NULL) //$GPRMC sentence found!
		{
			memset(&GPS.GPRMC,0,sizeof(GPS.GPRMC));
			sscanf(rmcStr,"$GPRMC,%02d%02d%02d.%03hd,%c,%f,%c,%f,%c,%f,%f,%02d%02d%02d,%f,%c*%2s\r\n",&GPS.GPRMC.UTC_Hour,&GPS.GPRMC.UTC_Min,&GPS.GPRMC.UTC_Sec,&GPS.GPRMC.UTC_MicroSec,&GPS.GPRMC.Status,&GPS.GPRMC.Latitude,&GPS.GPRMC.NS_Indicator,&GPS.GPRMC.Longitude,&GPS.GPRMC.EW_Indicator,&GPS.GPRMC.GroundSpeed,&GPS.GPRMC.GroundCourse,&GPS.GPRMC.Date,&GPS.GPRMC.Month,&GPS.GPRMC.Year,&GPS.GPRMC.MagVariationDeg,&GPS.GPRMC.MagVariationEW_Indicator,&GPS.GPRMC.Mode,GPS.GPRMC.CheckSum);
			if(GPS.GPRMC.NS_Indicator==0)
				GPS.GPRMC.NS_Indicator='-';
			if(GPS.GPRMC.EW_Indicator==0)
				GPS.GPRMC.EW_Indicator='-';
			GPS.GPRMC.LatitudeDecimal=convertDegMinToDecDeg(GPS.GPRMC.Latitude);
			GPS.GPRMC.LongitudeDecimal=convertDegMinToDecDeg(GPS.GPRMC.Longitude);
		}

		if(gsaStr!=NULL) //$GPGSA sentence found!
		{
			memset(&GPS.GPGSA,0,sizeof(GPS.GPGSA));
			sscanf(gsaStr,"$GPGSA,%c,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f*%2s\r\n",&GPS.GPGSA.Mode_1,&GPS.GPGSA.Mode_2,&GPS.GPGSA.SatsUsedOnCh[0],&GPS.GPGSA.SatsUsedOnCh[1],&GPS.GPGSA.SatsUsedOnCh[2],&GPS.GPGSA.SatsUsedOnCh[3],&GPS.GPGSA.SatsUsedOnCh[4],&GPS.GPGSA.SatsUsedOnCh[5],&GPS.GPGSA.SatsUsedOnCh[6],&GPS.GPGSA.SatsUsedOnCh[7],&GPS.GPGSA.SatsUsedOnCh[8],&GPS.GPGSA.SatsUsedOnCh[9],&GPS.GPGSA.SatsUsedOnCh[10],&GPS.GPGSA.SatsUsedOnCh[11],&GPS.GPGSA.PDOP,&GPS.GPGSA.HDOP,&GPS.GPGSA.VDOP,GPS.GPGSA.CheckSum);
		}

		if(gsvStr!=NULL) //$GPGSV sentence found!
		{
			memset(&GPS.GPGSV,0,sizeof(GPS.GPGSV));
			sscanf(gsvStr,"$GPGSV,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d*%2s\r\n",&GPS.GPGSV.NoOfMessages,&GPS.GPGSV.MessageNo,&GPS.GPGSV.SatsInView,&GPS.GPGSV.GPSSAT[0].SatteliteID,&GPS.GPGSV.GPSSAT[0].Elevation,&GPS.GPGSV.GPSSAT[0].Azimuth,&GPS.GPGSV.GPSSAT[0].SNR,&GPS.GPGSV.GPSSAT[1].SatteliteID,&GPS.GPGSV.GPSSAT[1].Elevation,&GPS.GPGSV.GPSSAT[1].Azimuth,&GPS.GPGSV.GPSSAT[1].SNR,&GPS.GPGSV.GPSSAT[2].SatteliteID,&GPS.GPGSV.GPSSAT[2].Elevation,&GPS.GPGSV.GPSSAT[2].Azimuth,&GPS.GPGSV.GPSSAT[2].SNR,&GPS.GPGSV.GPSSAT[3].SatteliteID,&GPS.GPGSV.GPSSAT[3].Elevation,&GPS.GPGSV.GPSSAT[3].Azimuth,&GPS.GPGSV.GPSSAT[3].SNR,GPS.GPGSV.CheckSum);
		}

		if(vtgStr!=NULL) //$GPVTG sentence found!
		{
			memset(&GPS.GPVTG,0,sizeof(GPS.GPVTG));
			sscanf(vtgStr,"$GPVTG,%f,%c,%f,%c,%f,%c,%f,%c,%c,%2s\r\n",&GPS.GPVTG.Course_1,&GPS.GPVTG.Reference_1,&GPS.GPVTG.Course_2,&GPS.GPVTG.Reference_2,&GPS.GPVTG.SpeedInKnots,&GPS.GPVTG.KnotsIndicator,&GPS.GPVTG.SpeedInKmh,&GPS.GPVTG.KmhIndicator,&GPS.GPVTG.Mode,GPS.GPVTG.CheckSum);
		}
		memset(GPS.rxBuffer,0,sizeof(GPS.rxBuffer));
		GPS.rxIndex=0;
	}
	HAL_UART_Receive_IT(&huart2,&GPS.rxTmp,1);
}
//##################################################################################################################
