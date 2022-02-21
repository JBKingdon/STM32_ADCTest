#include <stdio.h>
#include "../lib/1AUDfilter/1AUDfilterInt.h"
#include "../lib/1AUDfilter/1AUDfilter.h"

float minCutoff =  50;  // Hz
float maxCutoff = 150;  // Hz
float beta = 0.01f;
// slewLimit is in steps per second and gets auto converted when sample rate changes
float slewLimit = 100000.0f;

float sampleRate = 10000;    // Hz

float dCutoff = 50.0f;      // Hz


OneAUDfilter audFloat(minCutoff, maxCutoff, beta, sampleRate, dCutoff, slewLimit);
OneAUDfilterInt audInt(minCutoff, maxCutoff, 1.0f/beta, sampleRate, dCutoff, slewLimit);

int main()
{
    printf("1AUD test\n");
    
    // update the filters with a synthetic data pulse and log the results
    
    for(int i=0; i<1000; i++) {
        int input = i<500 ? 1 : 1000;
        float rf = audFloat.update(float(input));
        int ri = audInt.update(input);
        printf("%d %f %f %d %f\n", i, audFloat.getCutoff(), rf, ri, audInt.getCutoff());
    }
    
    return 0;
}
