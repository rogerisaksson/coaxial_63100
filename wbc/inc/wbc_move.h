/** wbc_move.h - her moves in C (wbc_bedexit.c, wbc_scoot.c, wbc_bedentry.c): machine.moves' programs whose phases her
    senses tell,
    numpy's arithmetic operation for operation (wbc_np.h). Portable C11, the host's alone. */
#ifndef WBC_MOVE_H
#define WBC_MOVE_H

/** Her moves' anatomy (wbc_anatomy.c, machine.moves.anatomy): how a part goes - held where it is, blended over its
    phase from where it began, to its goal at its rate (deg/s), her sensed angle and its goal past it, drawn toward
    its goal from her sensed angle by its rate (deg) at most, yielding past it, to its goal at its rate but no
    straighter than it is. */
enum
{
  WBC_ANATOMY_HELD,
  WBC_ANATOMY_BLEND,
  WBC_ANATOMY_RATE,
  WBC_ANATOMY_FOLLOW,
  WBC_ANATOMY_TOWARD,
  WBC_ANATOMY_LEAST
};

/** A joint's goal (deg) as `how` it goes: WBC_ANATOMY_LEAST's no straighter than its command `x`, the larger. */
double wbc_anatomy_aimed(double how, double goal, double x);

/** A joint's command (deg) a pass `dt` s on, as `how` it goes (WBC_ANATOMY_*): its `goal` and `rate`, its command
    `x`, her sensed angle `q`, where its phase began `start`, `a` of its phase's way. */
double wbc_anatomy_driven(double how, double goal, double rate, double x, double q, double start, double a, double dt);

/** `x` (deg), her hips holding her trunk's `pitch` at `trunk` (deg): moved `gain` deg/s a deg of its error, within
    [`lo`, `hi`]. */
double wbc_anatomy_held(double x, double gain, double pitch, double trunk, double dt, double lo, double hi);

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
    pitch her hips hold (deg) and whether they do; each part's goal (deg), how it is reached (WBC_ANATOMY_*) and its
    rate, on the bed and past its edge; her
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

/** The floor scoot's phases, the parts it moves (hips, knees, shoulders, elbows, the legs' rolls and yaws and the
    ankles a side each, the trunk's roll and twist), their joints and its heels' steps. */
#define WBC_SCOOT_PHASES 9
#define WBC_SCOOT_PARTS  10
#define WBC_SCOOT_MOVED  18
#define WBC_SCOOT_STEPS  2

/** Its state's slots: the phase, the step, each side's way this cycle, the heading aimed at, whether aimed. */
enum
{
  WBC_SCOOT_PHASE,
  WBC_SCOOT_STEP,
  WBC_SCOOT_WAY,
  WBC_SCOOT_AIM = WBC_SCOOT_WAY + 2,
  WBC_SCOOT_AIMED,
  WBC_SCOOT_SLOTS
};

/** machine.moves.scoot's tables, filled once from it: the joints it moves (figure.JOINTS), each one's part (hips 0,
    knees 1), side (0 left and the trunk, 1 right) and pair (its other side's, the trunk's its own); each step's side;
    when each phase is over (0 its goals reached, 1 braced, 2 its heel touching, 3 stroked, 4 asked, 5 vaulted, 6
    leant); the phases she sits, sets a heel, reaches, pushes and holds in; each step's joints (1 its heel's); each
    part's how (0 held, 1 its goal at its rate, 2 the stroke's end at the way's rate, 3 the stroke's start, 4 its goal
    while its foot touches none, 5 its pair's mean, a hip holding the trunk as well, 6 its goal while the trunk is short
    of its vault), goal (deg) and rate (deg/s) a phase, the next phase asked and idle, whether a step's heel alone moves,
    whether the twist's ways are taken as it ends and whether the hips hold the trunk; the knees' fold and stretch and
    the hips' lift (deg), her trunk's lean (deg), its hold's gain (deg/s a deg), range and band (deg), a stroke's rate
    (deg/s), the shares a foot bears and touches at, a twist asks past, a goal is reached within, a side's share of a
    turn, a heading's error a whole share (deg), the aim's lead turning (deg), the inner side's least share back. */
typedef struct
{
  int    joint[WBC_SCOOT_MOVED], part[WBC_SCOOT_MOVED], sided[WBC_SCOOT_MOVED], partner[WBC_SCOOT_MOVED];
  int    order[WBC_SCOOT_STEPS], over[WBC_SCOOT_PHASES];
  int    sitting, setting_down, reaching, pushing, holding;
  double steps[WBC_SCOOT_STEPS][WBC_SCOOT_MOVED];
  double how[WBC_SCOOT_PHASES][WBC_SCOOT_PARTS], goal[WBC_SCOOT_PHASES][WBC_SCOOT_PARTS];
  double rate[WBC_SCOOT_PHASES][WBC_SCOOT_PARTS], asked[WBC_SCOOT_PHASES], idle[WBC_SCOOT_PHASES];
  double masked[WBC_SCOOT_PHASES], cycles[WBC_SCOOT_PHASES], trunked[WBC_SCOOT_PHASES];
  double fold, stretch, folded, trunk, gain, hip_lo, hip_hi, leant, stroke_rate, bear, skim, ask, tol, turn;
  double stroke_turn, lead, half;
} wbc_scoot_t;

/** The move's `state` as it begins: bracing, the first step next, no way and no aim yet. */
void wbc_scoot_laid(double *state);

/** A pass of the move `dt` s on: `cmd` (deg, her joints) laid over from `state`, her trunk's pitch (deg), her fists'
    and feet's shares of her weight, her twist back and turning (shares of her keys') and her heading (deg, left +)
    sensed; 0.0, never over. */
double wbc_scoot_step(const wbc_scoot_t *s, double *state, double *cmd, double dt, double pitch, double hands,
                      double left, double right, double back, double turn, double heading);

/** Her way into a bed over its end to its middle: its phases, the parts it moves (shoulders, elbows, hips, knees,
    ankles, the legs' rolls and yaws a side each, the spine, waist and neck) and their joints. */
#define WBC_BEDENTRY_PHASES 15
#define WBC_BEDENTRY_PARTS  10
#define WBC_BEDENTRY_MOVED  17

/** Its state's slots: the phase, the strokes, her side of the bed's middle (0 till her first pass). */
enum
{
  WBC_BEDENTRY_PHASE,
  WBC_BEDENTRY_COUNT,
  WBC_BEDENTRY_WAY,
  WBC_BEDENTRY_SLOTS
};

/** machine.moves.bedentry's tables, filled once from it: the joints it moves (figure.JOINTS) and each one's part, how
    each phase ends (0 its pose reached, 1 her feet planted or that, 2 her pelvis pulled in or that and she still, 3
    that and she lying, 4 that and she still, 5 at the middle or that), the last phase; each phase's next, its next
    while a stroke is due and whether it counts a stroke; each part's goal (deg), how it goes (WBC_ANATOMY_HELD, _RATE,
    _LEAST) and its rate (deg/s); her feet's share planted, her pelvis's pull
    in (m), still under her speed (m/s), lying past her pitch (deg), a stroke again while this short of the middle
    (m), the strokes at most and a goal reached within (deg). */
typedef struct
{
  int    joint[WBC_BEDENTRY_MOVED], part[WBC_BEDENTRY_MOVED], over[WBC_BEDENTRY_PHASES], done;
  double next[WBC_BEDENTRY_PHASES], again[WBC_BEDENTRY_PHASES], counts[WBC_BEDENTRY_PHASES];
  double goal[WBC_BEDENTRY_PHASES][WBC_BEDENTRY_PARTS], how[WBC_BEDENTRY_PHASES][WBC_BEDENTRY_PARTS];
  double rate[WBC_BEDENTRY_PHASES][WBC_BEDENTRY_PARTS];
  double plant, pull_in, still, lain, near, strokes, tol;
} wbc_bedentry_t;

/** The move's `state` as it begins: her feet first, no stroke, no side yet. */
void wbc_bedentry_laid(double *state);

/** A pass of the move `dt` s on: `cmd` (deg, her joints) laid over from `state`, her soles' share of her weight, her
    pelvis from the bed's middle along its length and that half length (m), her trunk's pitch (deg) and her speed
    (m/s) sensed; 1.0 once she lies straight. */
double wbc_bedentry_step(const wbc_bedentry_t *b, double *state, double *cmd, double dt, double soles, double along,
                         double half, double pitch, double speed);

#endif
