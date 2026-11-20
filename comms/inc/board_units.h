/** board_units.h - Shared unit scales, mathematical constants and physical constants. */
#ifndef BOARD_UNITS_H
#define BOARD_UNITS_H

#define MILLI_PER_UNIT      1000.0f
#define CENTI_PER_UNIT      100.0f
#define MICRO_PER_UNIT      1.0e6f     /**< micro-volts, -amps, -ohms, -radians */
#define NANO_PER_UNIT       1.0e9f     /**< nanohenries; nano kg m2 for the model */
#define PPM_PER_UNIT        1.0e6f     /**< a ratio carried as parts per million */
#define PPM_WHOLE           1000000U   /**< the same, as a record field's bound */
#define US_PER_S            1000000U   /**< cycles a microsecond: the clock over this */
#define MS_PER_S            1000U

#define PI_F                3.1415927f
#define TWO_PI_F            6.2831853f
#define HALF_PI_F           1.5707963f
#define INV_TWO_PI_F        0.15915494f
#define EULER_E_F           2.7182817f
#define SQRT2_F             1.4142135f
#define INV_SQRT2_F         0.70710677f
#define SQRT3_F             1.7320508f
#define HALF_SQRT3_F        0.8660254f
#define INV_SQRT3_F         0.57735027f
#define ONE_THIRD_F         0.33333334f
#define DEG_TO_RAD_F        0.01745329252f
#define RAD_TO_DEG_F        57.2957795f

#define PI_D                3.14159265358979323846
#define TAU_D               6.28318530717958647692
#define HALF_PI_D           1.57079632679489661923
#define INV_TAU_D           0.15915494309189533577
#define EULER_E_D           2.71828182845904523536
#define LN2_D               0.69314718055994530942
#define SQRT2_D             1.41421356237309504880
#define INV_SQRT2_D         0.70710678118654752440
#define SQRT3_D             1.73205080756887729353
#define HALF_SQRT3_D        0.86602540378443864676
#define INV_SQRT3_D         0.57735026918962576451
#define DEG_TO_RAD_D        (3.14159265358979323846 / 180.0)
#define RAD_TO_DEG_D        (180.0 / 3.14159265358979323846)

#define KELVIN_AT_ZERO_C    273.15f
#define KELVIN_AT_ZERO_C_D  273.15
#define STANDARD_GRAVITY_D  9.80665
#define GAS_CONSTANT_D      8.314462618
#define AIR_GAS_CONSTANT_D  287.05
#define AIR_GAMMA_D         1.4
#define COPPER_TEMPCO_PER_K 0.00393

#endif /* BOARD_UNITS_H */
