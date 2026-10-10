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
  thermal_app_t app;            /**< what the board is mounted in */
  float air, capacity;          /**< the room's scales on that */
  /** The network in still air its rooms are laid on: the core's, or a stand-in's record. */
  thermal_cfg_t base;
} world_heat_t;

/** The board at `ambient`, C, every node there: thermal.c's defaults and loss table. */
void world_heat_init(world_heat_t *h, float ambient);

/** The board's room: `ambient`, C, and its air path and laminate capacity scaled as a
    situation lays them on - thermal_ident's rule, the one the observer finds them by - on the
    network its application has. */
void world_heat_room(world_heat_t *h, float ambient, float air, float capacity);

/** What the board is mounted in (thermal_application): its room laid again on that. */
void world_heat_application(world_heat_t *h, thermal_app_t app);

/** The network in still air its rooms are laid on, `base`: its room laid again on that. */
void world_heat_base(world_heat_t *h, const thermal_cfg_t *base);

/** `dt` s on `load`; what the three thermometers read into `seen`: the NTC's element, each die
    its node plus its watts through R_th,JC. */
void world_heat_step(world_heat_t *h, const thermal_load_t *load, float dt,
                     thermal_sense_t *seen);

#ifdef __cplusplus
}
#endif

#endif /* WORLD_HEAT_H */
