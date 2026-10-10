/** wbc_model.h - her figure as static arrays: written by host/tools/cores/model.py, not by hand. */
#ifndef WBC_MODEL_H
#define WBC_MODEL_H

/** Links, link 0 the floating pelvis; her degrees of freedom, 6 of them the pelvis's:
    link k > 0 is u[5 + k]. */
#define WBC_LINKS 34
#define WBC_N 39

extern const int    wbc_parent[WBC_LINKS];   /**< a link's parent; -1, the world, under the pelvis */
extern const double wbc_x[WBC_LINKS][12];    /**< its frame at rest in the parent's: R by rows, then p */
extern const double wbc_s[WBC_LINKS][6];     /**< its screw in its own frame, (w, v) */
extern const double wbc_g[WBC_LINKS][36];    /**< the spatial inertia it carries, by rows, over (w, v) */
extern const double wbc_armature[WBC_LINKS]; /**< the rotor's through the box, kg m^2 */
extern const double wbc_stop[WBC_LINKS][2];  /**< its stops, rad; lo > hi: none */
extern const double wbc_gravity[3];          /**< m/s^2, world frame */
extern const char *const wbc_name[WBC_LINKS]; /**< the joint's name */

#endif
