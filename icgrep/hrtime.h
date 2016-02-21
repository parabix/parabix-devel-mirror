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
  if ((fp = fopen("/proc/cpuinfo", "r")) == nullptr)
    return -1;

  // ignore all lines until we reach MHz information
  while (fgets(line, 1024, fp) != nullptr) {
    if (strstr(line, search_str) != nullptr) {
      // ignore all characters in line up to :
      for (s = line; *s && (*s != ':'); ++s)
        ;
      // get MHz number
      if (*s && (sscanf(s+1, "%lf", &mhz) == 1))
        break;
    }
  }

  if (fp != nullptr)
    fclose(fp);
  return mhz;
}

typedef uint64_t timestamp_t;

// get the number of CPU cycles since startup using rdtsc instruction
static inline timestamp_t read_cycle_counter() {
#ifdef __GNUC__
timestamp_t ts;
#ifdef __x86_64__
  unsigned int eax, edx;
  asm volatile("rdtsc" : "=a" (eax), "=d" (edx));
  ts = ((timestamp_t) eax) | (((timestamp_t) edx) << 32);
#else
  asm volatile("rdtsc\n" : "=A" (ts));
#endif
  return(ts);
#endif
#ifdef _MSC_VER
  return __rdtsc();
#endif
}

// get the elapsed time (in milliseconds) since startup
inline double getElapsedTime() {
  static double CPU_HZ = 0;
  if (CPU_HZ == 0)
    CPU_HZ = getMHZ() * 1000000;
  return (read_cycle_counter() / CPU_HZ);
}

#endif // __HRTIME_H__

