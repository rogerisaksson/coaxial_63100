/** wbc_model.h - her figure as static arrays: written by host/tools/cores/model.py, not by hand. */
#ifndef WBC_MODEL_H
#define WBC_MODEL_H

/** Links, link 0 the floating pelvis; her degrees of freedom, 6 of them the pelvis's:
    link k > 0 is u[5 + k]; the drives and the held joints, as links. */
#define WBC_LINKS 34
#define WBC_N 39
#define WBC_DRIVEN 21
#define WBC_HELD 4

/** A link's kind (wbc_kind). */
#define WBC_FREE 0
#define WBC_IS_DRIVEN 1
#define WBC_IS_HELD 2

extern const int    wbc_parent[WBC_LINKS];   /**< a link's parent; -1, the world, under the pelvis */
extern const double wbc_x[WBC_LINKS][12];    /**< its frame at rest in the parent's: R by rows, then p */
extern const double wbc_s[WBC_LINKS][6];     /**< its screw in its own frame, (w, v) */
extern const double wbc_g[WBC_LINKS][36];    /**< the spatial inertia it carries, by rows, over (w, v) */
extern const double wbc_mass[WBC_LINKS];     /**< the mass it carries, kg */
extern const double wbc_com[WBC_LINKS][3];   /**< its centre, in its frame */
extern const double wbc_armature[WBC_LINKS]; /**< the rotor's through the box, kg m^2 */
extern const double wbc_stop[WBC_LINKS][2];  /**< its stops, rad; lo > hi: none */
extern const double wbc_passive[WBC_LINKS][3]; /**< its spring N m/rad, damper N m s/rad, rest rad */
extern const int    wbc_kind[WBC_LINKS];     /**< free, driven or held */
extern const double wbc_clamp[WBC_LINKS];    /**< its drive's clamp, N m; 0 undriven */
extern const double wbc_peak[WBC_LINKS];     /**< its drive's peak at its board's amps, N m */
extern const double wbc_kt[WBC_LINKS];       /**< its drive's N m an ampere at the joint */
extern const int    wbc_pair[WBC_LINKS];     /**< the link sharing its rods + 1, negative on the second; 0 alone */
extern const int    wbc_driven[WBC_DRIVEN];  /**< the drives' links, in the drives' order */
extern const int    wbc_held[WBC_HELD];      /**< the held joints' links */
extern const int    wbc_sole[2];             /**< the feet's links, left and right */
extern const int    wbc_knee[2];             /**< the knees' links */
extern const int    wbc_trunk;               /**< the trunk's link */
extern const double wbc_sole_at[3];          /**< a sole's middle, in its foot's frame */
extern const double wbc_sole_corner[4][3];   /**< its corners */
extern const double wbc_total_mass;          /**< kg */
extern const double wbc_gravity[3];          /**< m/s^2, world frame */
extern const char *const wbc_name[WBC_LINKS]; /**< the joint's name */

#endif
