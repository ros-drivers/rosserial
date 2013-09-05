/******************************************************************************
*This file is for isnprintf()
*******************************************************************************/
#ifndef ISNPRINTF_H
#define ISNPRINTF_H
#include <inttypes.h>
#include <cstddef>

int isnprintf(char *s, int buf_len, const char *pszFmt,...);

#endif
/* [] END OF FILE */
