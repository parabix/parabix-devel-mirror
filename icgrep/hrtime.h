#ifndef __HRTIME_H__
#define __HRTIME_H__

//Downloaded from code.Google.com

#include <stdio.h>
#include <string.h>
#include <assert.h>

// get the number of CPU cycles per microsecond from Linux /proc filesystem
// return < 0 on error
inline double getMHZ(void) {
  double mhz = -1;
  char line[1024], *s, search_str[] = "cpu MHz";
  FILE* fp;

  // open proc/cpuinfo
  if ((fp = fopen("/proc/cpuinfo", "r")) == NULL)
    return -1;

  // ignore all lines until we reach MHz information
  while (fgets(line, 1024, fp) != NULL) {
    if (strstr(line, search_str) != NULL) {
      // ignore all characters in line up to :
      for (s = line; *s && (*s != ':'); ++s)
        ;
      // get MHz number
      if (*s && (sscanf(s+1, "%lf", &mhz) == 1))
        break;
    }
  }

  if (fp != NULL)
    fclose(fp);
  return mhz;
}

// get the number of CPU cycles since startup using rdtsc instruction
inline unsigned long long get_hrcycles() {
  unsigned int tmp[2];
  asm ("rdtsc" : "=a" (tmp[1]), "=d" (tmp[0]));
  return (((unsigned long long)tmp[0] << 32 | tmp[1]));
}

// get the elapsed time (in milliseconds) since startup
inline double getElapsedTime() {
  static double CPU_HZ = 0;
  if (CPU_HZ == 0)
    CPU_HZ = getMHZ() * 1000000;
  return (get_hrcycles() / CPU_HZ);
}

#endif // __HRTIME_H__

