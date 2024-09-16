#include "app/validation.h"

uint8_t const version_M __attribute__((section(".version_M_tag")))= VERSION_MAJOR;
uint8_t const version_m __attribute__((section(".version_m_tag")))= VERSION_MINOR;
uint8_t const version_P __attribute__((section(".version_P_tag")))= VERSION_PATCH;

volatile uint8_t bootid = *(unsigned char *)0x08002fff;


bool Comparehash(uint64_t t_signature){

	uint64_t temp_hash = HashString(KEY);
	uint8_t a[6];

	// To overcome the Endian difference
	a[0] =t_signature;
	a[1] =t_signature >>8;
	a[2] =t_signature >>16;
	a[3] =t_signature >>24;
	a[4] =t_signature >>32;
	a[5] =t_signature >>40;

	t_signature = a[0];
	t_signature =t_signature<<8;
	t_signature = t_signature + a[1];
	t_signature =t_signature<<8;
	t_signature = t_signature + a[2];
	t_signature =t_signature<<8;
	t_signature = t_signature + a[3];
	t_signature =t_signature<<8;
	t_signature = t_signature + a[4];
	t_signature =t_signature<<8;
	t_signature = t_signature + a[5];

	return !(t_signature ^ temp_hash);
}

uint64_t HashString(char* str) {

    uint64_t hash = 5381;
    int c;

    while ((c = *str++))
        hash = ((hash << 5) + hash) + c; /* hash * 33 + c */

    return hash;
}




