#ifndef BALANCING_OBSERVATION_TIME_H
#define BALANCING_OBSERVATION_TIME_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 同じ単調増加時計に対応する有効な観測時刻 [us]。 */
typedef struct {
    bool valid;
    uint64_t stamp_us;
} BSampleStamp;

/** @brief 各測定の最終新規観測時刻。受信時刻の偽装をしない。 */
typedef struct {
    BSampleStamp imu;
    BSampleStamp rs[4];
    BSampleStamp wheel[2];
    BSampleStamp power;
} BObservedStamps;

#ifdef __cplusplus
}
#endif
#endif /* BALANCING_OBSERVATION_TIME_H */
