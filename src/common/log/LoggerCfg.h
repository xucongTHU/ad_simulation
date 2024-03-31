/****************************************************************
 * Namespace: caic_logger
 * File: LoggerCfg.h
 *
 * Function: It is an head file for this library.
 *
 * Thie file is config of the Logger Library.
 * Copyright (c) 2023 CAIC Corporation. All rights reserved.
 *****************************************************************/
#ifndef H_CAIC_LOGGER_CFG_H
#define H_CAIC_LOGGER_CFG_H

namespace caic_logger {
/* max length of each log */
#define MAX_LOG_LEN 1024
/* max length of file path */
#define MAX_FILE_PATH_LEN 256
/* max length of process or thread id */
#define MAX_THREAD_ID_LEN 32

/* Logger file log plugin's using file max size */
#define LOG_FILE_MAX_SIZE 500 * 1024 * 1024

/* Logger file log plugin's using max rotate file count */
#define LOG_FILE_MAX_ROTATE 5

/* enable tag output in the log */
// #define LOG_TAG_OUTPUT_ENABLE

/* enable log color */
#define LOG_COLOR_ENABLE

#ifdef LOG_COLOR_ENABLE
/**
 * CSI(Control Sequence Introducer/Initiator) sign
 * more information on https://en.wikipedia.org/wiki/ANSI_escape_code
 */
#define CSI_START "\033["
#define CSI_END "\033[0m"
/* output log front color */
#define F_BLACK "30;"
#define F_RED "31;"
#define F_GREEN "32;"
#define F_YELLOW "33;"
#define F_BLUE "34;"
#define F_MAGENTA "35;"
#define F_CYAN "36;"
#define F_WHITE "37;"
/* output log background color */
#define B_NULL
#define B_BLACK "40;"
#define B_RED "41;"
#define B_GREEN "42;"
#define B_YELLOW "43;"
#define B_BLUE "44;"
#define B_MAGENTA "45;"
#define B_CYAN "46;"
#define B_WHITE "47;"
/* output log fonts style */
#define S_F_BOLD "1m"
#define S_F_UNDERLINE "4m"
#define S_F_BLINK "5m"
#define S_F_NORMAL "22m"

/* output log default color definition: [front color] + [background color] +
 * [show style] */
#define LOG_COLOR_ERROR (F_RED B_NULL S_F_NORMAL)
#define LOG_COLOR_WARN (F_YELLOW B_NULL S_F_NORMAL)
#define LOG_COLOR_INFO (F_BLACK B_NULL S_F_NORMAL)
#define LOG_COLOR_DEBUG (F_GREEN B_NULL S_F_NORMAL)
#endif /* LOG_COLOR_ENABLE */

}  // namespace caic_logger

#endif
