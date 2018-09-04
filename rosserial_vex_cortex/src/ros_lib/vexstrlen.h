#ifndef _ROSSERIAL_VEX_CORTEX_VEXSTRLEN_H_
#define _ROSSERIAL_VEX_CORTEX_VEXSTRLEN_H_

#define MAX_VEXSTRLEN 2048

/* substitute string length function.
 * if the length is too long or if there is no null terminator, behavior is undefined.
 */
inline uint32_t vexstrlen(const char* st) {
  for(int i = 0; i < MAX_VEXSTRLEN; i++) {
    if(st[i] == '\0') return i; 
  }
  return MAX_VEXSTRLEN;
}

#endif
