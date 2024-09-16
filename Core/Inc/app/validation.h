#ifndef VALIDATION_H_
#define VALIDATION_H_



#include <stdint.h>

#define KEY "ACELAB"

extern bool Comparehash(uint64_t signature);
extern volatile uint8_t bootid;

extern const uint8_t version_M;
extern const uint8_t version_m;
extern const uint8_t version_P;


uint64_t HashString(char* str);
#endif
