"""Catalog of hot-swappable motor engrams and per-stunt bundles modeled on mocap takes."""
import collections

from machine.moves import anatomy

#: Runtime hot-swappable sensorimotor module bound to a controller slot (`sensed`, `wire`, or `program`).
Engram = collections.namedtuple('Engram', 'name slot kind task stunt_ref description spec')

#: Prepared catalog of motor engrams modeled on natural human mocap takes in `stunts/`.
CATALOG = {
    'chair_mount_stock': Engram(
        'chair_mount_stock', 'chair_mount', 'sensed', 'chair', 'stand_to_sit',
        'Stock chair-mount sensed move table',
        {'base': 'chair-mount'},
    ),
    'chair_mount_calm': Engram(
        'chair_mount_calm', 'chair_mount', 'sensed', 'chair', 'stand_to_sit',
        'Slower vault and lap transition aligned with Stand To Sit cadence',
        {'base': 'chair-mount', 'rate_scale': 0.82, 'constants': {'pull-in': 0.20}},
    ),
    'chair_mount_deep_pull': Engram(
        'chair_mount_deep_pull', 'chair_mount', 'sensed', 'chair', 'stand_to_sit',
        'Deeper posterior pull-in before arm opening for secure seat placement',
        {'base': 'chair-mount', 'rate_scale': 0.90, 'constants': {'pull-in': 0.22}},
    ),
    'chair_mount_soft_arms': Engram(
        'chair_mount_soft_arms', 'chair_mount', 'sensed', 'chair', 'stand_to_sit',
        'Reduced upper-limb slew during open and lap phases to cut hand jerk',
        {'base': 'chair-mount', 'phase_rate_scales': {'open': 0.72, 'lap': 0.75},
         'constants': {'pull-in': 0.20}},
    ),
    'chair_mount_brisk': Engram(
        'chair_mount_brisk', 'chair_mount', 'sensed', 'chair', 'stand_to_sit',
        'Faster arm vault and pull-in rates for rapid docking',
        {'base': 'chair-mount', 'rate_scale': 1.15},
    ),
    'chair_dismount_stock': Engram(
        'chair_dismount_stock', 'chair_dismount', 'sensed', 'chair', 'sit_to_stand',
        'Stock chair-dismount sensed move table',
        {'base': 'chair-dismount'},
    ),
    'chair_dismount_forward': Engram(
        'chair_dismount_forward', 'chair_dismount', 'sensed', 'chair', 'sit_to_stand',
        'Enhanced forward trunk lean and dorsiflexion aligned with Sit To Stand',
        {'base': 'chair-dismount', 'rate_scale': 0.95,
         'phase_goals': {'lean': {'hip': (-120.0, 72.0, anatomy.RATE),
                                  'ankle': (-14.0, 44.0, anatomy.RATE)}}},
    ),
    'chair_dismount_calm': Engram(
        'chair_dismount_calm', 'chair_dismount', 'sensed', 'chair', 'sit_to_stand',
        'Reduced dismount slew to limit chair pitch recoil during weight transfer',
        {'base': 'chair-dismount', 'rate_scale': 0.82},
    ),
    'chair_dismount_brisk': Engram(
        'chair_dismount_brisk', 'chair_dismount', 'sensed', 'chair', 'sit_to_stand',
        'Brisk foot plant and press rates matching faster sit-to-stand transfer',
        {'base': 'chair-dismount', 'rate_scale': 1.15},
    ),
    'bed_entry_stock': Engram(
        'bed_entry_stock', 'bed_entry', 'sensed', 'bed', 'lying_down_from_sitting',
        'Stock bed-entry sensed move table',
        {'base': 'bed-entry'},
    ),
    'bed_entry_gentle': Engram(
        'bed_entry_gentle', 'bed_entry', 'sensed', 'bed', 'lying_down_from_sitting',
        'Smoother recline and leg fold matching Lying Down from Sitting',
        {'base': 'bed-entry', 'rate_scale': 0.85, 'phase_rate_scales': {'lie': 0.78, 'fold': 0.85}},
    ),
    'bed_entry_deep': Engram(
        'bed_entry_deep', 'bed_entry', 'sensed', 'bed', 'lying_down_from_sitting',
        'Deeper initial bed pull-in before supine transition',
        {'base': 'bed-entry', 'rate_scale': 0.92, 'constants': {'pull-in': 0.33}},
    ),
    'bed_entry_compact': Engram(
        'bed_entry_compact', 'bed_entry', 'sensed', 'bed', 'lying_down_from_sitting',
        'Brisker bridging cadence for compact bed clearance',
        {'base': 'bed-entry', 'rate_scale': 1.05, 'constants': {'near': 0.04}},
    ),
    'crouch_down_mocap': Engram(
        'crouch_down_mocap', 'crouch_down', 'wire', 'crouch', 'crouching_down',
        'Mocap-derived crouching-down latent curve from Crouching Down take',
        {'take': 'crouching_down', 'tempo_scale': 1.0, 'amp_scale': 1.0, 'rest_tail': False},
    ),
    'crouch_down_calm': Engram(
        'crouch_down_calm', 'crouch_down', 'wire', 'crouch', 'crouching_down',
        'Time-stretched, damped crouch descent from Crouching Down take',
        {'take': 'crouching_down', 'tempo_scale': 1.25, 'amp_scale': 0.92, 'rest_tail': False},
    ),
    'crouch_down_brisk': Engram(
        'crouch_down_brisk', 'crouch_down', 'wire', 'crouch', 'crouching_down',
        'Faster crouch descent from Crouching Down take',
        {'take': 'crouching_down', 'tempo_scale': 0.85, 'amp_scale': 0.95, 'rest_tail': False},
    ),
    'crouch_up_mocap': Engram(
        'crouch_up_mocap', 'crouch_up', 'wire', 'crouch', 'crouch_up',
        'Mocap-derived crouch recovery curve from Crouch Up take',
        {'take': 'crouch_up', 'tempo_scale': 1.0, 'amp_scale': 1.0, 'rest_tail': True},
    ),
    'crouch_up_calm': Engram(
        'crouch_up_calm', 'crouch_up', 'wire', 'crouch', 'crouch_up',
        'Calm crouch recovery curve returning cleanly to rest',
        {'take': 'crouch_up', 'tempo_scale': 1.25, 'amp_scale': 1.0, 'rest_tail': True},
    ),
    'kneel_down_mocap': Engram(
        'kneel_down_mocap', 'kneel_down', 'wire', 'kneel', 'kneeling_down',
        'Mocap-derived kneeling descent from Kneeling Down take',
        {'take': 'kneeling_down', 'tempo_scale': 1.0, 'amp_scale': 0.90, 'rest_tail': False},
    ),
    'kneel_down_calm': Engram(
        'kneel_down_calm', 'kneel_down', 'wire', 'kneel', 'kneeling_down',
        'Calm kneeling descent scaled from Kneeling Down take',
        {'take': 'kneeling_down', 'tempo_scale': 1.25, 'amp_scale': 0.85, 'rest_tail': False},
    ),
    'turn_left_stock': Engram(
        'turn_left_stock', 'turn_left', 'wire', 'turn', 'left_turn',
        'Stock turn_left orthokinematic table',
        {'base_entry': 'turn_left', 'tempo_scale': 1.0},
    ),
    'turn_left_calm': Engram(
        'turn_left_calm', 'turn_left', 'wire', 'turn', 'left_turn',
        'Extended blend turn_left matching softer Left Turn yaw onset',
        {'base_entry': 'turn_left', 'tempo_scale': 1.25},
    ),
    'turn_left_brisk': Engram(
        'turn_left_brisk', 'turn_left', 'wire', 'turn', 'left_turn',
        'Compressed blend turn_left for snappier Left Turn pivot',
        {'base_entry': 'turn_left', 'tempo_scale': 0.82},
    ),
    'walk_stock': Engram(
        'walk_stock', 'walk', 'wire', 'gait', 'walking_arc_left',
        'Stock walk gait row',
        {'base_entry': 'walk', 'tempo_scale': 1.0},
    ),
    'walk_arc_pattern': Engram(
        'walk_arc_pattern', 'walk', 'wire', 'gait', 'walking_arc_left',
        'Walk gait modified by Walking Arc Left spectral CPG modifiers',
        {'pattern': 'walking_arc_left', 'tempo_scale': 1.0},
    ),
    'stand_to_sit_chair_poised': Engram(
        'stand_to_sit_chair_poised', 'stand_to_sit', 'wire', 'chair', 'stand_to_sit',
        'Sit-chair wire with ischial manifold and calibrated seating depth for stable landing',
        {'base_entry': 'sit_chair', 'tempo_scale': 0.95,
         'voice_scales': {'axial_compression': 1.02, 'sagittal_shift': 1.12}},
    ),
    'bed_entry_poised': Engram(
        'bed_entry_poised', 'bed_entry', 'sensed', 'bed', 'lying_down_from_sitting',
        'Calibrated bed-entry rates and 0.32 m pull-in matching Lying Down from Sitting',
        {'base': 'bed-entry', 'rate_scale': 0.94,
         'phase_rate_scales': {'lie': 0.90, 'fold': 0.94}, 'constants': {'pull-in': 0.32}},
    ),
    'crouch_down_paced': Engram(
        'crouch_down_paced', 'crouching_down', 'wire', 'crouch', 'crouching_down',
        'Time-stretched crouching_down take matching human descent duration and depth',
        {'take': 'crouching_down', 'tempo_scale': 1.48, 'amp_scale': 1.12, 'rest_tail': False},
    ),
    'kneel_down_paced': Engram(
        'kneel_down_paced', 'kneeling_down', 'wire', 'kneel', 'kneeling_down',
        'Gently paced kneeling_down take cutting hand jerk and full-body distance',
        {'take': 'kneeling_down', 'tempo_scale': 1.25, 'amp_scale': 1.02, 'rest_tail': False},
    ),
    'kneel_up_paced': Engram(
        'kneel_up_paced', 'kneeling_up', 'wire', 'kneel', 'kneeling_up',
        'Paced kneeling_up ascent extending rise duration toward human cadence',
        {'take': 'kneeling_up', 'tempo_scale': 1.19, 'amp_scale': 1.02, 'rest_tail': False},
    ),
    'hip_hop_headroom': Engram(
        'hip_hop_headroom', 'hip_hop', 'wire', 'dance', 'hip_hop',
        'Damped upper-limb reach/fold voices eliminating late elbow torque clipping',
        {'take': 'hip_hop', 'tempo_scale': 1.05, 'amp_scale': 0.88,
         'voice_scales': {'arm_reach': 0.55, 'arm_fold': 0.55}, 'rest_tail': True},
    ),
    'moonwalk_glide': Engram(
        'moonwalk_glide', 'moonwalk', 'wire', 'dance', 'moonwalk',
        'Moonwalk gait modifier boosting backward stride and pace to match human travel',
        {'pattern': 'moonwalk', 'modifiers': {'stride': 0.15, 'pace': 0.24}},
    ),
    'jog_forward_stable': Engram(
        'jog_forward_stable', 'jogging_forward', 'wire', 'run', 'jogging_forward',
        'Scaled crouch/lean/swing with boosted stride and pace for Jogging Forward',
        {'pattern': 'jogging_forward', 'amount_scales': {'crouch': 0.95, 'lean': 0.85, 'swing': 0.9},
         'modifiers': {'stride': 0.16, 'pace': 0.18}, 'blend_s': 1.1},
    ),
    'jog_backward_stable': Engram(
        'jog_backward_stable', 'jogging_backwards', 'wire', 'run', 'jogging_backwards',
        'Reduced crouch and boosted backward stride/pace matching Jogging Backwards',
        {'pattern': 'jogging_backwards', 'amount_scales': {'crouch': 0.25, 'lean': 0.3, 'swing': 0.5},
         'modifiers': {'stride': 0.42, 'pace': 0.12}},
    ),
    'walk_crouch_prep': Engram(
        'walk_crouch_prep', 'walk', 'wire', 'run', 'running_forward',
        'Gentle axial compression on walk entry eliminating pre-run torque clipping',
        {'base_entry': 'walk', 'voice_offsets': {'axial_compression': 0.022}},
    ),
    'run_forward_stable': Engram(
        'run_forward_stable', 'running_forward', 'wire', 'run', 'running_forward',
        'Damped crouch/lean/stride/pace envelope preventing forward pitch-over fall',
        {'pattern': 'running_forward',
         'amount_scales': {'crouch': 0.65, 'lean': 0.35, 'swing': 0.7, 'stride': 0.3, 'pace': 0.45},
         'blend_s': 1.1},
    ),
    'run_backward_stable': Engram(
        'run_backward_stable', 'running_backward', 'wire', 'run', 'running_backward',
        'Fast-blend forward-lean-stabilized backward sprint preventing posterior fall',
        {'pattern': 'running_backward', 'amount_scales': {'crouch': 0.1, 'swing': 0.25, 'pace': 0.75},
         'modifiers': {'lean': 0.04, 'stride': 0.35}, 'blend_s': 0.65},
    ),
    'turn_left_arc_walk': Engram(
        'turn_left_arc_walk', 'turn_left', 'wire', 'turn', 'walking_arc_left',
        'Compact stance width with crouch and forward pitch on turn_left for Walking Arc Left',
        {'base_entry': 'turn_left', 'tempo_scale': 1.0,
         'voice_offsets': {'axial_compression': 0.05, 'pitch_attitude': 0.04, 'stance_width': -0.1}},
    ),
    'turn_left_arc_run': Engram(
        'turn_left_arc_run', 'turn_left', 'wire', 'turn', 'run_arc_left',
        'Moderate crouch/pitch and narrowed stance width on turn_left for Run Arc Left',
        {'base_entry': 'turn_left', 'tempo_scale': 1.0,
         'voice_offsets': {'axial_compression': 0.04, 'pitch_attitude': 0.03, 'stance_width': -0.08}},
    ),
    'turn_right_arc_run': Engram(
        'turn_right_arc_run', 'turn_right', 'wire', 'turn', 'running_right_turn',
        'Scaled blend with crouch/pitch and narrowed stance on turn_right for Running Right Turn',
        {'base_entry': 'turn_right', 'tempo_scale': 1.2,
         'voice_offsets': {'axial_compression': 0.04, 'pitch_attitude': 0.03, 'stance_width': -0.08}},
    ),
    'walk_stairs_climb': Engram(
        'walk_stairs_climb', 'walk', 'wire', 'stairs', 'ascending_stairs',
        'Stair-climb gait with crouch, forward pitch, and sagittal shift for ascent',
        {'base_entry': 'stairs_walk', 'tempo_scale': 1.0,
         'voice_offsets': {'axial_compression': 0.04, 'pitch_attitude': 0.06, 'sagittal_shift': 0.03}},
    ),
    'hop_down_chair_balanced': Engram(
        'hop_down_chair_balanced', 'hop_down_chair', 'wire', 'jump', 'jump_down',
        'Balanced hop_down_chair eliminating deep-squat knee torque clipping',
        {'base_entry': 'hop_down_chair', 'tempo_scale': 1.0,
         'voice_scales': {'axial_compression': 0.85, 'sagittal_drive': 1.6, 'stance_width': 1.1}},
    ),
    'hop_up_chair_clean': Engram(
        'hop_up_chair_clean', 'hop_up_chair', 'wire', 'jump', 'jumping_up',
        'Scaled hop_up_chair eliminating knee torque saturation and halving hand jerk',
        {'base_entry': 'hop_up_chair', 'tempo_scale': 1.0,
         'voice_scales': {'axial_compression': 0.65, 'sagittal_drive': 1.25, 'stance_width': 0.85}},
    ),
    'walk_back_edge_recover': Engram(
        'walk_back_edge_recover', 'walk_back', 'wire', 'balance', 'edge_tripping_backwards',
        'Crouched walk_back widening stance to recover from rear step-off edge',
        {'base_entry': 'walk_back', 'tempo_scale': 1.0,
         'voice_offsets': {'axial_compression': 0.04, 'sagittal_drive': 0.02, 'stance_width': 0.07}},
    ),
    'jog_running_jump_lean': Engram(
        'jog_running_jump_lean', 'jog', 'wire', 'jump', 'running_jump',
        'Deeper crouch and wider stance in jog approach for running_jump',
        {'base_entry': 'jog', 'tempo_scale': 1.0,
         'voice_scales': {'axial_compression': 1.2, 'stance_width': 3.0}},
    ),
}

#: Prepared candidate engram lists grouped by controller slot for combinatorial Monte Carlo sweeps.
SUITES = {
    'chair': {
        'mount': ('chair_mount_stock', 'chair_mount_calm', 'chair_mount_deep_pull',
                  'chair_mount_soft_arms', 'chair_mount_brisk'),
        'dismount': ('chair_dismount_stock', 'chair_dismount_forward', 'chair_dismount_calm',
                     'chair_dismount_brisk'),
    },
    'bed': {
        'entry': ('bed_entry_stock', 'bed_entry_gentle', 'bed_entry_deep', 'bed_entry_compact',
                  'bed_entry_poised'),
    },
    'crouch': {
        'down': ('crouch_down_mocap', 'crouch_down_calm', 'crouch_down_brisk'),
        'up': ('crouch_up_mocap', 'crouch_up_calm'),
    },
    'kneel': {
        'down': ('kneel_down_mocap', 'kneel_down_calm'),
    },
    'turn': {
        'turn': ('turn_left_stock', 'turn_left_calm', 'turn_left_brisk'),
    },
}

#: Verified per-stunt default engram bundles improving anatomical fidelity across `stunts/`.
STUNT_BUNDLES = {
    'stand_to_sit': ('stand_to_sit_chair_poised',),
    'sit_to_stand': ('stand_to_sit_chair_poised',),
    'lying_down_from_sitting': ('bed_entry_poised',),
    'crouching_down': ('crouch_down_paced',),
    'kneeling_down': ('kneel_down_paced',),
    'kneeling_up': ('kneel_up_paced',),
    'hip_hop': ('hip_hop_headroom',),
    'moonwalk': ('moonwalk_glide',),
    'walking_arc_left': ('turn_left_arc_walk',),
    'run_arc_left': ('turn_left_arc_run',),
    'running_right_turn': ('turn_right_arc_run',),
    'jogging_forward': ('jog_forward_stable',),
    'jogging_backwards': ('jog_backward_stable',),
    'running_forward': ('run_forward_stable', 'walk_crouch_prep'),
    'running_backward': ('run_backward_stable',),
    'ascending_stairs': ('walk_stairs_climb',),
    'walking_up_the_stairs': ('walk_stairs_climb',),
    'jump_down': ('hop_down_chair_balanced',),
    'jumping_down': ('hop_down_chair_balanced',),
    'jumping_up': ('hop_up_chair_clean',),
    'edge_tripping_backwards': ('walk_back_edge_recover',),
    'running_jump': ('jog_running_jump_lean',),
}
