#include <stdio.h>
#include <stdint.h>
#include <string.h>

void flex_interleave_block(uint32_t *frame_words)
{
uint32_t src[8];
uint8_t  dst[32];
uint32_t i;

memcpy(src, frame_words, sizeof(src));

for (i = 0; i < 32; i++) {
dst[i] =
((src[0] >> (31 - i)) & 1) << 7 |
((src[1] >> (31 - i)) & 1) << 6 |
((src[2] >> (31 - i)) & 1) << 5 |
((src[3] >> (31 - i)) & 1) << 4 |
((src[4] >> (31 - i)) & 1) << 3 |
((src[5] >> (31 - i)) & 1) << 2 |
((src[6] >> (31 - i)) & 1) << 1 |
((src[7] >> (31 - i)) & 1) << 0;
}

memcpy(frame_words, dst, sizeof(dst));
}

int main() {
    uint32_t words[8] = {
        0x127bc998, 0xc988cc9d, 0x9cc88d8d, 0x99d99ddd,
        0x9ddd9dc9, 0x989c8888, 0xc9c8899c, 0x88dc8c8d
    };
    
    printf("Original word 0: %08x\n", words[0]);
    flex_interleave_block(words);
    
    uint8_t *bytes = (uint8_t*)words;
    printf("Interleaved bytes: ");
    for(int i=0; i<8; i++) printf("%02x ", bytes[i]);
    printf("\n");
    return 0;
}
