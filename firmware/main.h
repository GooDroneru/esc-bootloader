#ifndef MAIN_H
#define MAIN_H

// #define SYSCLK_FREQ_96MHz_HSI  96000000

#pragma pack (push, 1)
typedef struct version_s {
    uint8_t major;
    uint8_t minor;
}version_t;

typedef struct hardwareVersion_s {
    char deviceId[16];
} hardwareVersion_t;

#pragma pack (pop)


#endif /* MAIN_H */
