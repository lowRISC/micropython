#include <string.h>
#include <stdint.h>

char* strcpy(char* dst, const char* src)
{
  char* dst0 = dst;
  char ch;
  do
  {
    ch = *src;
    src++;
    dst++;
    *(dst-1) = ch;
  } while(ch);

  return dst0;
}
