#ifndef DCGLOBAL_H
#define DCGLOBAL_H

#include <cstdint>

#define DC_START_SESSION_TAG_STR "S:"
#define DC_END_SESSION_TAG_STR "E:"
#define DC_DEFAULT_SESSION_TIMEOUT 3000
#define DC_DEFAULT_KEEPALIVE_TIMEOUT 100
#define DC_CHANNEL_COUNT 126
#define DC_DEFAULT_OFFLINE_TIMEOUT 6000
#define DC_TUNNIG_PA_TIMEOUT 1000
#define DC_MAX_SIZE_OF_RF_PACKET 32

uint8_t getBestPaLevelResult(int32_t* goodResults, int32_t* fallResults, uint8_t resultCount);

#endif // !DCGLOBAL_H

