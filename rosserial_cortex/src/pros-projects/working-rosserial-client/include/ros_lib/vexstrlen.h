#ifndef __VEXSTRLEN_H_
#define __VEXSTRLEN_H_

#define MAX_VEXSTRLEN 2048

/* substitute string length function.
 * if the length is too long or if there is no null terminator, behavior is undefined.
 */
inline size_t vexstrlen(const char* st) {
  for(int i = 0; i < MAX_VEXSTRLEN; i++) {
    if(st[i] == '\0') return i; 
  }
  return MAX_VEXSTRLEN;
}

#endif
