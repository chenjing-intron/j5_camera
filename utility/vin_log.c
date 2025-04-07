/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include "vin_log.h"

static uint64_t vin_gettime_us(void) {
	struct timespec tp;
	uint64_t timeTmp;

	(void)clock_gettime(CLOCK_MONOTONIC, &tp);
	timeTmp = (((uint64_t)tp.tv_sec * (uint64_t)HB_TIME_CALCULATE_UNIT_1000000)
		+ ((uint64_t)tp.tv_nsec / (uint64_t)HB_TIME_CALCULATE_UNIT_1000));

	return timeTmp;
}
static inline int32_t vin_get_loglevel(void)
{
	const char *loglevel_env = NULL;
	int32_t loglevel_value = ALOG_INFO_LEVEL;

	loglevel_env = getenv(LOGLEVEL_ENV);

	if (loglevel_env != NULL) {
		loglevel_value = atoi(loglevel_env);	/*PRQA S 5125*/
		/* loglevel value should in the configuration area */
		if (((loglevel_value >= ALOG_ERROR_LEVEL) && (loglevel_value <= ALOG_DEBUG_LEVEL)))
			return loglevel_value;
	}

	/* default log level */
	loglevel_value = ALOG_INFO_LEVEL;

	return loglevel_value;
}

static inline int32_t loglevel_check(vin_log_level_e level)
{
	int32_t loglevel = vin_get_loglevel();

	//print only in alog rang
	if (((int32_t)level <= loglevel) &&
		(loglevel >= (int32_t)VIN_ERR) &&
		(loglevel <= (int32_t)VIN_DEBUG))
	{
		return 0;
	} else {
		return -1;
	}
}

void hb_vin_log_warpper(vin_log_level_e level, const char *format, ...) {
	va_list ptr;
	char logBuf[MAX_VIN_LARGE_PRINT_LENGTH] = {0};
	char secPrefix[MAX_VIN_PRINT_LENGTH] = {0};

	if (level >= MAX_VIN_LOG_LEVEL || (loglevel_check(level) == -1))
		return;

	(void)snprintf(secPrefix, MAX_VIN_PRINT_LENGTH, "[%.6lf]",
		(float)vin_gettime_us()/HB_TIME_CALCULATE_UNIT_FLOAT_1000000);/*PRQA S 5209*/

	va_start(ptr, format);/*PRQA S 0315,5140*/
	(void)vsnprintf(logBuf, MAX_VIN_LARGE_PRINT_LENGTH, format, ptr);
	va_end(ptr);

	switch (level) {
	case VIN_INFO:
		(void)android_printLog_info("%s%s", secPrefix, logBuf);
		break;
	case VIN_ERR:
		(void)android_printLog_err("%s%s", secPrefix, logBuf);
		break;
	case VIN_DEBUG:
		(void)android_printLog_dbg("%s%s", secPrefix, logBuf);
		break;
	case VIN_WARN:
		(void)android_printLog_warn("%s%s", secPrefix, logBuf);
		break;
	default:
		(void)android_printLog_dbg("%s%s", secPrefix, logBuf);
		break;
	}
}
