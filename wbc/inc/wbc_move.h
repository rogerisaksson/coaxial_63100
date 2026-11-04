/** wbc_move.h - her moves in C (wbc_bedexit.c, wbc_scoot.c): machine.moves' programs whose phases her senses tell,
    numpy's arithmetic operation for operation (wbc_np.h). Portable C11, the host's alone. */
#ifndef WBC_MOVE_H
#define WBC_MOVE_H

/** The bed exit's phases, the parts it moves (hips, knees, ankles, shoulders, elbows, spine) and their joints. */
#define WBC_BEDEXIT_PHASES 10
#define WBC_BEDEXIT_PARTS  6
#define WBC_BEDEXIT_MOVED  11

/** Its state's slots: the phase, when it began, the strokes, past the edge (0/1), the pose the phase began from. */
enum
{
  WBC_BEDEXIT_PHASE,
  WBC_BEDEXIT_AT,
  WBC_BEDEXIT_COUNT,
  WBC_BEDEXIT_OVER,
  WBC_BEDEXIT_FROM
};

/** machine.moves.bedexit's tables, filled once from it: the joints it moves (figure.JOINTS) and each one's part
    (the knees part 1), her joints, the push's phase (it counts the strokes, ending once she sits), the stretch's
    (it tells her past the edge), the settle's (where she goes seated), the last; each phase's length (s), the phase after it, the trunk's
    pitch her hips hold (deg) and whether they do; each part's goal (deg), how it is reached (0 held, 1 blended over
    the phase from where it began, 2 at its rate, deg/s, 3 her sensed angle and the goal past it, 4 drawn toward from
    her sensed angle by its rate, deg, at most, yielding past it) and its rate, on the bed and past its edge; her
    hips' gain on her trunk (deg/s a deg) and range (deg); her shanks' share past the edge, her soles' share and her
    knees' fold (deg) seated, the strokes at most. */
typedef struct
{
  int    joint[WBC_BEDEXIT_MOVED], part[WBC_BEDEXIT_MOVED], joints, pushed, stretched, seated, done;
  double length[WBC_BEDEXIT_PHASES], next[WBC_BEDEXIT_PHASES], trunk[WBC_BEDEXIT_PHASES], holds[WBC_BEDEXIT_PHASES];
  double goal[WBC_BEDEXIT_PHASES][WBC_BEDEXIT_PARTS], how[WBC_BEDEXIT_PHASES][WBC_BEDEXIT_PARTS];
  double rate[WBC_BEDEXIT_PHASES][WBC_BEDEXIT_PARTS], over[WBC_BEDEXIT_PHASES][WBC_BEDEXIT_PARTS];
  double how_over[WBC_BEDEXIT_PHASES][WBC_BEDEXIT_PARTS], rate_over[WBC_BEDEXIT_PHASES][WBC_BEDEXIT_PARTS];
  double trunk_gain, hip_lo, hip_hi, shin_share, sole_share, knee_seat, strokes;
} wbc_bedexit_t;

/** The move's `state` from her pose `q_deg` (deg, her joints) at `t` s: its first phase, nothing done. */
void wbc_bedexit_laid(const wbc_bedexit_t *b, const double *q_deg, double t, double *state);

/** A pass of the move at `t`, `dt` on: `cmd` (deg, her joints) laid over from `state`, her pose `q_deg` (deg), her
    soles' and shanks' share of her weight and her trunk's pitch (deg) sensed; 1.0 once she sits on the edge. */
double wbc_bedexit_step(const wbc_bedexit_t *b, double *state, double *cmd, const double *q_deg, double t, double dt,
                        double soles, double shins, double pitch);

/** The floor scoot's phases, the parts it moves (hips, knees, shoulders, elbows, the legs' rolls and yaws a side each,
    the trunk's roll and twist) and their joints. */
#define WBC_SCOOT_PHASES 6
#define WBC_SCOOT_PARTS  9
#define WBC_SCOOT_MOVED  16

/** Its state's slots: the phase, each side's share of the stroke (left, right), the heading aimed at, whether aimed. */
enum
{
  WBC_SCOOT_PHASE,
  WBC_SCOOT_WAY,
  WBC_SCOOT_AIM = WBC_SCOOT_WAY + 2,
  WBC_SCOOT_AIMED,
  WBC_SCOOT_SLOTS
};

/** machine.moves.scoot's tables, filled once from it: the joints it moves (figure.JOINTS), each one's part (hips 0,
    knees 1) and side (left 1, right -1, the trunk 0), the phase a stroke begins at and its stroke's; when each phase is over (0 its goals
    reached, 1 braced, 2 set, 3 stroked, 4 asked); each part's how (0 held, 1 its goal at its rate, 2 the stroke's end
    at the twist's rate, 3 the stroke's start, 4 a stretched knee settled, 5 its goal while its foot bears none), goal
    (deg) and rate (deg/s) a phase, the next phase asked and idle; the knees' fold, stretch and settle and the hips'
    lift (deg), her trunk's lean (deg), its hold's gain (deg/s a deg), range and band (deg), a stroke's rate (deg/s),
    the shares a limb bears and touches at, a twist asks past, a goal is reached within, a stroke's share of a turn,
    a whole stroke's turn (deg), the aim's lead turning (deg) and her trunk's pitch over a stroke back (deg). */
typedef struct
{
  int    joint[WBC_SCOOT_MOVED], part[WBC_SCOOT_MOVED], cycle, stroking, over[WBC_SCOOT_PHASES];
  double side[WBC_SCOOT_MOVED];
  double how[WBC_SCOOT_PHASES][WBC_SCOOT_PARTS], goal[WBC_SCOOT_PHASES][WBC_SCOOT_PARTS];
  double rate[WBC_SCOOT_PHASES][WBC_SCOOT_PARTS], asked[WBC_SCOOT_PHASES], idle[WBC_SCOOT_PHASES];
  double fold, stretch, settle, folded, trunk, gain, hip_lo, hip_hi, leant, stroke_rate, bear, skim, ask, tol, turn;
  double stroke_turn, lead, upright;
} wbc_scoot_t;

/** The move's `state` as it begins: bracing, no stroke and no aim yet. */
void wbc_scoot_laid(double *state);

/** A pass of the move `dt` s on: `cmd` (deg, her joints) laid over from `state`, her trunk's pitch (deg), her fists'
    and feet's shares of her weight, her twist back and turning (shares of her keys') and her heading (deg, left +)
    sensed; 0.0, never over. */
double wbc_scoot_step(const wbc_scoot_t *s, double *state, double *cmd, double dt, double pitch, double hands,
                      double left, double right, double back, double turn, double heading);

#endif
