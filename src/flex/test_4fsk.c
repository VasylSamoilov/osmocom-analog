#include <stdio.h>
#include <stdint.h>

int main() {
    uint8_t interleaved[16] = {0};
    uint8_t phase_bytes[4][1] = {
        {0x80}, // Phase A: 10000000
        {0x40}, // Phase B: 01000000
        {0x20}, // Phase C: 00100000
        {0x10}  // Phase D: 00010000
    };

    printf("6400 (4FSK, 4 phases)\n");
    for (int bi = 0; bi < 8; bi++) {
        uint8_t bits[4];
        for (int p = 0; p < 4; p++)
            bits[p] = (phase_bytes[p][0] >> (7 - bi)) & 1;
        
        for (int p = 0; p < 4; p++) {
            int out_pos = bi * 4 + p;
            if (bits[p])
                interleaved[out_pos / 8] |= (1 << (7 - (out_pos % 8)));
        }
    }
    
    printf("Interleaved output: ");
    for(int i=0; i<4; i++) printf("%02x ", interleaved[i]);
    printf("\n");
    return 0;
}
