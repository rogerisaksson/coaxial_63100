/**
  ******************************************************************************
  * @file    board_units.h
  * @brief   What the cooked readings and the record count in: milli- and
  *          micro-units of a volt, an amp, an ohm, a radian, a second or a
  *          watt, centi-units of a degree, ratios in parts per million.
  *          Each factor once, not 1000.0f at every conversion. Shared by
  *          the board layer and the command handlers, so it sits beside
  *          board.h.
  ******************************************************************************
  */
#ifndef BOARD_UNITS_H
#define BOARD_UNITS_H

#define MILLI_PER_UNIT   1000.0f
#define CENTI_PER_UNIT   100.0f
#define MICRO_PER_UNIT   1.0e6f     /**< micro-volts, -amps, -ohms, -radians */
#define NANO_PER_UNIT    1.0e9f     /**< nanohenries; nano kg m2 for the model */
#define PPM_PER_UNIT     1.0e6f     /**< a ratio carried as parts per million */
#define PPM_WHOLE        1000000U   /**< the same, as a record field's bound */
#define MICRO_PER_MILLI  1000UL
#define US_PER_S         1000000U   /**< cycles a microsecond: the clock over this */
#define MS_PER_S         1000U
#define KELVIN_AT_ZERO_C 273.15f

#endif /* BOARD_UNITS_H */
