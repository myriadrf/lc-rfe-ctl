#include <stdio.h>
#include <stdint.h>

void _binp8(uint8_t val){
    printf("0b%d%d%d%d%d%d%d%d",
           (val >> 7) & 1, (val >> 6) & 1, (val >> 5) & 1, (val >> 4) & 1,
           (val >> 3) & 1, (val >> 2) & 1, (val >> 1) & 1, (val >> 0) & 1);
}

void _float_to_string(float val, char *buf)
{
    int val_int = (int)val;
    int val_dec = (int)((val - val_int) * 100); // 2 decimal places
    // negative values
    if (val < 0) {
        *buf++ = '-';
        val = -val;
    }
    sprintf(buf, "%d.%02d", val_int, val_dec);
}
