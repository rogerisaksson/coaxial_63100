/** world_heat.c - A board's heat: thermal.c's network stepped as the truth, the stand-in's way. */
#include "world_heat.h"

#include <math.h>
#include <string.h>

void world_heat_init(world_heat_t *h, float ambient)
{
  thermal_cfg_t cfg;

  thermal_defaults(&cfg);
  thermal_init(&h->th, &cfg, ambient);
  thermal_losses(&h->loss);
  memset(&h->power, 0, sizeof(h->power));
}

void world_heat_step(world_heat_t *h, const thermal_load_t *load, float dt,
                     thermal_sense_t *seen)
{
  const thermal_sense_t none = { NAN, NAN, NAN };
  const float phase_c[3] = { h->th.t[THERMAL_DRIVER(0)], h->th.t[THERMAL_DRIVER(1)],
                             h->th.t[THERMAL_DRIVER(2)] };

  thermal_power_estimate(&h->power, load, &h->loss, phase_c);
  thermal_step(&h->th, &h->power, &none, load, dt);
  seen->ntc_c = thermal_expected_ntc(&h->th);
  seen->mcu_c = h->th.t[THERMAL_MCU]
                + h->power.watt[THERMAL_MCU] * h->th.cfg.node[THERMAL_MCU].rth_die;
  seen->afe_c = h->th.t[THERMAL_AFE]
                + h->power.watt[THERMAL_AFE] * h->th.cfg.node[THERMAL_AFE].rth_die;
}
