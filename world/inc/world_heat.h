/** world_heat.h - A board's heat as the truth its observer is judged by: thermal.c's network. */
#ifndef WORLD_HEAT_H
#define WORLD_HEAT_H

#include "thermal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  thermal_t th;
  thermal_loss_t loss;
  thermal_power_t power;
} world_heat_t;

/** The board at `ambient`, C, every node there: thermal.c's defaults and loss table. */
void world_heat_init(world_heat_t *h, float ambient);

/** `dt` s on `load`; what the three thermometers read into `seen`: the NTC's element, each die
    its node plus its watts through R_th,JC. */
void world_heat_step(world_heat_t *h, const thermal_load_t *load, float dt,
                     thermal_sense_t *seen);

#ifdef __cplusplus
}
#endif

#endif /* WORLD_HEAT_H */
