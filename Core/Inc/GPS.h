#ifndef _GPS_H_
#define _GPS_H_

#include <stdint.h>
#include "main.h"
#include "stm32f4xx.h"

extern UART_HandleTypeDef huart2;

//##################################################################################################################

typedef struct
{
	uint8_t			SatteliteID;
	uint8_t			Elevation;
	uint8_t			Azimuth;
	uint8_t			SNR;
}GPSSAT_t;

typedef struct
{
	uint8_t			UTC_Hour;
	uint8_t			UTC_Min;
	uint8_t			UTC_Sec;
	uint16_t		UTC_MicroSec;

	float			Latitude;
	double			LatitudeDecimal;
	char			NS_Indicator;
	float			Longitude;
	double			LongitudeDecimal;
	char			EW_Indicator;

	uint8_t			PositionFixIndicator;
	uint8_t			SatellitesUsed;
	float			HDOP;
	float			MSL_Altitude;
	char			MSL_Units;
	float			Geoid_Separation;
	char			Geoid_Units;

	uint16_t		AgeofDiffCorr;
	char			DiffRefStationID[4];
	char			CheckSum[2];
}GPGGA_t;

typedef struct
{
	uint8_t			UTC_Hour;
	uint8_t			UTC_Min;
	uint8_t			UTC_Sec;
	uint16_t		UTC_MicroSec;

	char			Status;

	float			Latitude;
	double			LatitudeDecimal;
	char			NS_Indicator;
	float			Longitude;
	double			LongitudeDecimal;
	char			EW_Indicator;

	float			GroundSpeed;
	float			GroundCourse;

	uint8_t			Date;
	uint8_t			Month;
	uint8_t			Year;

	float			MagVariationDeg;
	char			MagVariationEW_Indicator;
	char			Mode;
	char			CheckSum[2];
}GPRMC_t;

typedef struct
{
	char			Mode_1;
	uint8_t			Mode_2;

	uint8_t			SatsUsedOnCh[4];

	float			PDOP;
	float			HDOP;
	float			VDOP;

	char			CheckSum[2];
}GPGSA_t;

typedef struct
{
	uint8_t			NoOfMessages;
	uint8_t			MessageNo;

	uint8_t			SatsInView;
	GPSSAT_t		GPSSAT[12];

	char			CheckSum[2];
}GPGSV_t;

typedef struct
{
	float			Course_1;
	char			Reference_1;

	float			Course_2;
	char			Reference_2;

	float			SpeedInKnots;
	char			KnotsIndicator;

	float			SpeedInKmh;
	char			KmhIndicator;

	char			Mode;

	char			CheckSum[2];
}GPVTG_t;



typedef struct
{
	uint8_t			rxBuffer[512];
	uint16_t		rxIndex;
	uint8_t			rxTmp;
	uint32_t		LastTime;

	GPGGA_t			GPGGA;
	GPRMC_t			GPRMC;
	GPGSA_t			GPGSA;
	GPGSV_t			GPGSV;
	GPVTG_t			GPVTG;

}GPS_t;



extern GPS_t GPS;
//##################################################################################################################
void	GPS_Init(void);
void	GPS_CallBack(void);
void	GPS_Process(void);
//##################################################################################################################

#endif
