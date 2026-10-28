"""The helicopter's form, a T-REX 700's, measured off a model 1.74 units across its rotor.

The model's axes: y up from the ground under its skids, z aft from the mast, x toward the tail
rotor's side. The page draws it (`coaxial.graphics.helicopter`) at the boom's length, on the
skids the sky stands it on.
"""

#: The main rotor's plane and its radius; the tail rotor's hub (x, y, z) and radius; the mast's
#: swashplate (y, radius); the head's hub (y, radius) and its grips along the blades (from, to,
#: radius).
DISC_Y, ROTOR, TAIL, TAIL_ROTOR = 0.475, 0.872, (0.064, 0.2435, 1.005), 0.1325
SWASH, HUB, GRIPS = (0.37, 0.03), (0.465, 0.03), (0.028, 0.066, 0.016)

#: A main blade from its root out (root, chord, thickness); a tail blade's chord.
BLADE, TAIL_CHORD = (0.07, 0.046, 0.010), 0.035

#: The canopy, nose to back: (z, bottom, top, half-width); open below from z -0.24 on, its
#: bottom there its sides' lower edge. Its window ahead of WINDOW's z (z, y, slope) over the line
#: from there falling to the nose.
CANOPY = (
    (-0.426, 0.158, 0.158, 0.0), (-0.42, 0.147, 0.169, 0.019), (-0.40, 0.126, 0.191, 0.036),
    (-0.37, 0.113, 0.218, 0.046), (-0.34, 0.101, 0.240, 0.053), (-0.30, 0.089, 0.266, 0.060),
    (-0.26, 0.080, 0.287, 0.064), (-0.22, 0.085, 0.304, 0.067), (-0.18, 0.128, 0.318, 0.071),
    (-0.14, 0.160, 0.329, 0.074), (-0.10, 0.188, 0.336, 0.076), (-0.06, 0.214, 0.333, 0.076),
    (-0.02, 0.229, 0.331, 0.074), (0.02, 0.246, 0.329, 0.072), (0.05, 0.263, 0.326, 0.068),
    (0.065, 0.282, 0.319, 0.062), (0.07, 0.300, 0.300, 0.0))
WINDOW = (-0.16, 0.26, 0.385)

#: The frames' side plates, nose to back: (z, bottom, top), HALF_FRAME out and FRAME_T thick;
#: the battery's tray (z from, to, y from, to, half-width) in their front.
FRAME = ((-0.31, 0.15, 0.20), (-0.25, 0.15, 0.23), (-0.22, 0.075, 0.25), (-0.16, 0.075, 0.30),
         (-0.06, 0.075, 0.30), (-0.04, 0.075, 0.36), (0.02, 0.075, 0.36), (0.05, 0.075, 0.28),
         (0.11, 0.11, 0.265), (0.14, 0.215, 0.27))
HALF_FRAME, FRAME_T, TRAY = 0.045, 0.006, (-0.30, -0.10, 0.09, 0.15, 0.03)

#: The boom (z from, to, y, radius); its two supports from the frame (x, y, z) to it; the
#: stabiliser on it, (x, leading edge z, trailing edge z) at its root and tip, at STAB_Y.
BOOM, SUPPORT = (0.14, 1.0, 0.2437, 0.0131), ((0.040, 0.122, 0.12), (0.014, 0.236, 0.76))
STAB, STAB_Y = ((0.013, 0.717, 0.7765), (0.069, 0.776, 0.7835)), 0.258

#: The fin under the boom on the tail case's far side at FIN_X: (y, leading edge z, trailing
#: edge z) down to its tip, FIN_T of its chord thick; the tail case (z from, to, y from, to).
FIN = ((0.27, 0.933, 0.988), (0.20, 0.930, 0.988), (0.15, 0.962, 1.008), (0.10, 0.994, 1.027),
       (0.07, 1.013, 1.039))
FIN_X, FIN_T, CASE = -0.021, 0.06, (0.988, 1.022, 0.20, 0.29)

#: The skids: x out, the tube's middle y and radius, z from the front's turn up to the back;
#: the front turned up to (y, z); the gear's two bows at z, each from under the frame (x, y)
#: down to its skid.
SKID, NOSE_UP = (0.1033, 0.0093, 0.0055, -0.25, 0.079), (0.04, -0.285)
BOWS, BOW_TOP = (-0.2, 0.05), (0.055, 0.075)
