/** The robot's libm's coefficients and tables: written by host/tools/cores/libm.py, not by hand. */
#ifndef WBC_LIBM_TABLES_H
#define WBC_LIBM_TABLES_H

#include <stdint.h>

static const double lm_sin[7] = {
  -0x1.5555555555555p-3, 0x1.1111111111110p-7, -0x1.a01a01a019938p-13, 0x1.71de3a546095bp-19,
  -0x1.ae645412c560cp-26, 0x1.61217f0b800d5p-33, -0x1.ab17d404de5b3p-41,
};
static const double lm_cos[6] = {
  0x1.5555555555555p-5, -0x1.6c16c16c16967p-10, 0x1.a01a019f4eb01p-16, -0x1.27e4fa17da09ep-22,
  0x1.1eeb68e93b64cp-29, -0x1.907da367a37cbp-37,
};
static const double lm_exp[11] = {
  0x1.0000000000000p-1, 0x1.5555555555557p-3, 0x1.5555555555556p-5, 0x1.11111111100dfp-7,
  0x1.6c16c16c162d6p-10, 0x1.a01a01abe62ddp-13, 0x1.a01a01a6d7808p-16, 0x1.71de0237564cap-19,
  0x1.27e4db67b42a6p-22, 0x1.af4ddd849d43dp-26, 0x1.1f72fc73175a9p-29,
};
static const double lm_log[7] = {
  0x1.5555555555558p-1, 0x1.99999999952e2p-2, 0x1.2492492df148dp-2, 0x1.c71c62e5800a1p-3,
  0x1.7462b4ab2ef6bp-3, 0x1.39fe606542ddep-3, 0x1.2b584aae78a57p-3,
};
static const double lm_atan[12] = {
  -0x1.5555555555555p-2, 0x1.99999999998c5p-3, -0x1.2492492485503p-3, 0x1.c71c71bd2b8bcp-4,
  -0x1.745d154c84f7ap-4, 0x1.3b1375ce5bdc6p-4, -0x1.110c9ce7b0572p-4, 0x1.e170800a46210p-5,
  -0x1.ab59b417b2d3fp-5, 0x1.7001816fd063fp-5, -0x1.0f62bba6a2558p-5, 0x1.e4167464d3de8p-7,
};
static const double lm_asin[14] = {
  0x1.5555555555555p-3, 0x1.3333333333388p-4, 0x1.6db6db6dac1e0p-5, 0x1.f1c71c7a52ba3p-6,
  0x1.6e8ba123e494cp-6, 0x1.1c4efce23019fp-6, 0x1.c990ad3d8fdcap-7, 0x1.7b027ee1dd585p-7,
  0x1.3b49de7121487p-7, 0x1.31622469ce5adp-7, 0x1.8f193743418ffp-9, 0x1.406192d124629p-6,
  -0x1.3b416bb7d9257p-6, 0x1.e529c6fce9bb4p-6,
};
static const double lm_pio2[3] = {
  0x1.921fb54400000p+0, 0x1.0b4611a800000p-34, -0x1.d9cceba3f91f2p-66,
};
static const double lm_two_over_pi[1] = {
  0x1.45f306dc9c883p-1,
};
static const double lm_pi_2[2] = {
  0x1.921fb54442d18p+0, 0x1.1a62633145c07p-54,
};
static const double lm_pi_4[2] = {
  0x1.921fb54442d18p-1, 0x1.1a62633145c07p-55,
};
static const double lm_pi[2] = {
  0x1.921fb54442d18p+1, 0x1.1a62633145c07p-53,
};
static const double lm_pi_3_4[1] = {
  0x1.2d97c7f3321d2p+1,
};
static const double lm_grid_pi_2[2] = {
  0x1.921fb54442d18p+0, 0x1.1a62633145c07p-54,
};
static const double lm_atan_c[4] = {
  0x1.921fb54442d18p-1, 0x1.1a62633145c07p-55, 0x1.dac670561bb50p-2, -0x1.2ea406ee84d0fp-55,
};
static const double lm_inv_ln2[1] = {
  0x1.71547652b82fep+0,
};
static const double lm_ln2[2] = {
  0x1.62e42ff000000p-1, -0x1.718432a1b0e26p-35,
};
static const double lm_ln1_5[2] = {
  0x1.9f323ecbf984cp-2, -0x1.a92e513217f5cp-59,
};
static const double lm_third[2] = {
  0x1.5555555555555p-2, 0x1.5555555555555p-56,
};
static const double lm_pow_log[9] = {
  0x1.999999999999ap-3, 0x1.2492492492492p-3, 0x1.c71c71c71c71cp-4, 0x1.745d1745d1746p-4,
  0x1.3b13b13b13b14p-4, 0x1.1111111111111p-4, 0x1.e1e1e1e1e1e1ep-5, 0x1.af286bca1af28p-5,
  0x1.8618618618618p-5,
};
static const double lm_pow_cut[2] = {
  0x1.3988e1409212ep+0, 0x1.bb67ae8584caap+0,
};
static const uint32_t lm_two_over_pi_words[38] = {
  0xa2f9836eU, 0x4e441529U, 0xfc2757d1U, 0xf534ddc0U,
  0xdb629599U, 0x3c439041U, 0xfe5163abU, 0xdebbc561U,
  0xb7246e3aU, 0x424dd2e0U, 0x6492eeaU, 0x9d1921cU,
  0xfe1deb1cU, 0xb129a73eU, 0xe88235f5U, 0x2ebb4484U,
  0xe99c7026U, 0xb45f7e41U, 0x3991d639U, 0x835339f4U,
  0x9c845f8bU, 0xbdf9283bU, 0x1ff897ffU, 0xde05980fU,
  0xef2f118bU, 0x5a0a6d1fU, 0x6d367ecfU, 0x27cb09b7U,
  0x4f463f66U, 0x9e5fea2dU, 0x7527bac7U, 0xebe5f17bU,
  0x3d0739f7U, 0x8a5292eaU, 0x6bfb5fb1U, 0x1f8d5d08U,
  0x56033046U, 0xfc7b6babU,
};

#endif
