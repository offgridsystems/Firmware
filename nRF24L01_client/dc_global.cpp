#include "dc_global.h"

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif  

uint8_t getBestPaLevelResult(int32_t* goodResults, int32_t* fallResults, uint8_t resultCount) {
    uint8_t pa[4] = { 0, 1, 2, 3 };
    for (int i = 0; i < resultCount; ++i) {
        for (int j = i + 1; j < resultCount; ++j) {
            if (goodResults[j] > goodResults[i])
            {
                int32_t tmp = goodResults[i];
                goodResults[i] = goodResults[j];
                goodResults[j] = tmp;
                tmp = fallResults[i];
                fallResults[i] = fallResults[j];
                fallResults[j] = tmp;
                tmp = pa[i];
                pa[i] = pa[j];
                pa[j] = tmp;
            }
        }
    }

    for (int i = 0; i < resultCount; ++i)
        if (fallResults[i] == 0)
            return pa[i];

    return pa[0];
}

void myDelay(int32_t mSec)
{
    auto startTime = millis();
    while ((millis() - startTime) < mSec)
        yield();
}
