/** wbc_upright.h - machine.law.motion.control's own arithmetic between its parts in C (wbc_upright.c): the
    share its seat takes sat upright on what bears it (borne's contacts), its arms' hold and its law's four
    weights up its body - Python's floats, min and max as CPython takes them, numpy's sum and max where its
    arrays are summed. Portable C11, the host's alone. */
#ifndef WBC_UPRIGHT_H
#define WBC_UPRIGHT_H

#include "wbc_borne.h"

/** Where control reads what it weighs, laid once: each flat index into the robot's latent state `st`, its
    latent's out, its rest, its ask `a`, what is told `told` and its table's columns; the constants it weighs by. */
typedef struct
{
  int    st_to_pal, st_to_zmp, st_to_gain, st_now_gain, st_to_drive; /* st's TO and NOW rows' entries */
  int    out_slid, out_held, out_ks[3], rest_k[3];                   /* out's SLID, HELD, KS's arms; rest's K's */
  int    a_strides, a_borne, told_isch, told_palmar, told_n;         /* a's STRIDES, BORNE; told's seat, palms */
  int    tab_cols, tab_pal, pelvis, spare;                           /* the table's width, its palms' column */
  double touch_n, tiny, prop_on, stirs;                              /* leap.TOUCH_N, observer.TINY, ... */
} wbc_upright_t;

/** control's weighing at a law pass: got[0] its seat's share sat upright (isch_w borne_w upright), got[1] its
    arms' hold, got[2..5] its law's weights up its body (w_up: neck, base, hop's twice), got[6] 1 where its pelvis
    bears on a contact of borne's read `b`; `rows` of its table `tab`, its soles' `loads`, its `weight`, its
    trunk `reclined`, its surety's weight `sw` and its hop's `w_hop`. */
void wbc_upright(const wbc_upright_t *u, const wbc_borne_t *b, const double *st, const double *out, const double *a,
                 const double *rest, const double *told, const double *tab, int rows, double load0, double load1,
                 double weight, double reclined, double sw, double w_hop, double got[7]);

#endif
