#include <stdio.h>
#include <stdint.h>

int main() {
    printf("multimon-ng 4FSK decode:\n");
    for (int sym=0; sym<4; sym++) {
        int a = (sym > 1);
        int b = (sym == 1) || (sym == 2);
        printf("Sym %d: A=%d B=%d -> %d\n", sym, a, b, (a<<1)|b);
    }
    
    printf("\nmultimon-ng Phase Mapping for 3200/2FSK:\n");
    printf("Even bits -> phase_toggle=0 (Phase A)\n");
    printf("Odd bits  -> phase_toggle=1 (Phase C)\n");
    return 0;
}
