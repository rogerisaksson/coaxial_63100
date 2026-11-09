"""The takes staged as scenarios (`scenarios.STUNTS`): a take's scene, its commands, its signature.

Each is judged against its take by `tools/sim/stunts.py`; `scenarios.ran` plays it.
"""
from machine.world import floor as fl
from machine.process import sprung
from machine.texts import takes
from tools.sim.player import FORCES_N, KEYS, MOMENTS, SHOVE_S, WALK

#: A stairs take's window opens as her first sole lands on the first tread, its first frame's.
STAIRS_TAKE_S = 1.5
#: The takes as scenarios, each judged against its take (`tools/sim/stunts.py`): key: (the take
#: under stunts/, scene, commands, from when her rows are measured - the telling, or MOMENTS[0]
#: with a gait under way -, the measures of its signature). A posture's take is told as its own
#: rows ({'take': key}, `stunts.wire`), a gait's as a player's twist, a jump's as the hop's text,
#: a fall's as a shove; a verb she has no text for is refused by name: backflip. The falls'
#: signature is SAFE alone: a shove of the polar's most, the slick patch and the ice, a step's
#: edge behind her.
_BACK, _RUN, _SHOVED = KEYS['s'], KEYS['W'], FORCES_N[1]
#: A gait take's pattern laid over her walk said, as the walk's last entry (`sprung.WALK`) has
#: entered: forward over the walk, backward over the walk back (on within 1.5 s of its saying).
PATTERN_S = sprung.WALK[-1][1] + takes.ENTER_S
_PATTERN = lambda key: [(1.0, {'say': 'walk'}), (PATTERN_S, {'pattern': key})]
_BACK_PATTERN = lambda key: [(1.0, {'say': 'walk_back'}), (PATTERN_S, {'pattern': key})]
#: The stairs takes' flight told at 0 s where the scene stands, the walk by twist: a climb's pattern
#: (its tempo, drive and lean over the walk) hopped her onto the first tread and toppled her
#: backward at 4.0 s where the walk climbs the flight (sequencer.md).
_FLIGHT = [(0.0, {'floor': fl.told_stairs(fl.TAKE_RISE_M, fl.TAKE_RUN_M)}), (1.0, {'twist': WALK})]
#: s a joint program her text binds to a phase may wait for it: her sit and her plant at the bed's edge.
PLAYED_S = 8.0
STUNTS = {
    'hip_hop': ('dance/Hip Hop Dancing', 'flat', [(1.0, {'take': 'hip_hop'})], 1.0, ('travel', 'drop', 'arms')),
    'moonwalk': ('dance/Moonwalk', 'flat', _BACK_PATTERN('moonwalk'), PATTERN_S, ('travel', 'speed', 'flight', 'upright')),
    'falling_on_butt': ('falling/Falling on Butt', 'flat',
                        [(1.0, {'push': (0.0, 0.0, -_SHOVED), 'seconds': SHOVE_S})], 1.0, ()),
    'slipping_on_a_patch': ('falling/Slipping Backwards', 'slip', [(1.0, {'twist': WALK})], 1.0, ()),
    'tripping_forward': ('falling/Tripping Forward', 'flat',
                         [(0.5, {'twist': WALK}), (MOMENTS[0], {'push': (0.0, 0.0, _SHOVED), 'seconds': SHOVE_S})],
                         MOMENTS[0], ()),
    'backflip': ('flips/Backflip', 'flat', [(1.0, {'say': 'backflip'})], 1.0, ('rise', 'flight', 'pitched')),
    'run_to_flip': ('flips/Run To Flip', 'flat', [(0.5, {'twist': _RUN}), (MOMENTS[0], {'say': 'backflip'})],
                    MOMENTS[0], ('rise', 'flight', 'pitched')),
    'jump_down': ('jumping/Jump Down', 'on_chair', [(1.0, {'say': 'hop_down_chair'})], 1.0, ('ended', 'travel')),
    'jumping_down': ('jumping/Jumping Down', 'on_chair', [(1.0, {'say': 'hop_down_chair'})], 1.0, ('ended', 'travel')),
    'jumping_up': ('jumping/Jumping Up', 'chair_ahead', [(1.0, {'say': 'hop_up_chair'})], 1.0, ('ended', 'travel')),
    'running_jump': ('jumping/Running Jump', 'flat', [(0.5, {'twist': _RUN}), (MOMENTS[0], {'charge': 1.0, 'say': 'hop'})],
                     MOMENTS[0], ('flight', 'travel', 'speed')),
    'crouch_up': ('kneel/Crouch Up', 'flat', [(1.0, {'take': 'crouching_down'}), (6.0, {'take': 'crouch_up'})], 6.0,
                  ('ended', 'lasted')),
    'crouching_down': ('kneel/Crouching Down', 'flat', [(1.0, {'take': 'crouching_down'})], 1.0, ('drop', 'ended', 'knee', 'lasted')),
    'kneeling_down': ('kneel/Kneeling Down', 'flat', [(1.0, {'take': 'kneeling_down'})], 1.0, ('drop', 'ended', 'knee', 'lasted')),
    'kneeling_up': ('kneel/Kneeling Up', 'flat', [(1.0, {'take': 'kneeling_down'}), (6.0, {'take': 'kneeling_up'})], 6.0,
                    ('ended', 'lasted')),
    'lying_down_from_sitting': ('lying/Lying Down from Sitting', 'bed_foot_behind', [(1.0, {'say': 'lie_bed'})], None,
                                ('upright', 'hips back', 'ended', 'lasted')),
    'sitting_on_bed': ('lying/Sitting on Bed', 'bed_foot_behind', [(1.0, {'say': 'lie_bed'})], 0.5,
                       ('drop', 'hips back', 'upright', 'knee', 'lifted', 'lasted')),
    'standing_yawn': ('lying/Standing Yawn', 'flat', [(1.0, {'take': 'standing_yawn'})], 1.0, ('arms', 'back', 'travel')),
    'turning_in_bed': ('lying/Turning in Bed', 'mattress_under',
                       [(1.0, {'say': 'lie_floor'}), (9.0, {'take': 'turning_in_bed'})], 9.0, ('rolled', 'turned', 'upright')),
    'jogging_backwards': ('run/Jogging Backwards', 'flat', _BACK_PATTERN('jogging_backwards'), PATTERN_S,
                          ('travel', 'speed', 'upright')),
    'jogging_forward': ('run/Jogging Forward', 'flat', _PATTERN('jogging_forward'), PATTERN_S, ('travel', 'speed', 'flight')),
    'running_backward': ('run/Running Backward', 'flat', _BACK_PATTERN('running_backward'), PATTERN_S, ('travel', 'speed')),
    'running_forward': ('run/Running Forward', 'flat', _PATTERN('running_forward'), PATTERN_S, ('travel', 'speed', 'flight')),
    'sit_to_stand': ('sitting/Sit To Stand', 'chair_behind', [(1.0, {'take': 'stand_to_sit'}), (8.0, {'take': 'sit_to_stand'})], 8.0,
                     ('ended', 'forward', 'lasted')),
    'situp_to_stand': ('sitting/Situp To Stand', 'mattress_under',
                       [(1.0, {'say': 'lie_floor'}), (9.0, {'take': 'situp_to_stand'})], 9.0, ('ended', 'upright', 'lasted')),
    'stand_to_sit': ('sitting/Stand To Sit', 'chair_behind', [(1.0, {'take': 'stand_to_sit'})], 1.0,
                     ('drop', 'hips back', 'forward', 'knee', 'lifted', 'lasted')),
    'edge_tripping_backwards': ('slipping/Edge Tripping Backwards', 'edge_behind', [(1.0, {'twist': _BACK})], 1.0, ('ended',)),
    'slipping_backwards': ('slipping/Slipping Backwards', 'ice', [(1.0, {'twist': WALK})], MOMENTS[0], ()),
    'ascending_stairs': ('stairs/Ascending Stairs', 'stairs_take', _FLIGHT, STAIRS_TAKE_S, ('ended', 'travel')),
    'walking_up_the_stairs': ('stairs/Walking Up The Stairs', 'stairs_take', _FLIGHT, STAIRS_TAKE_S, ('ended', 'travel')),
    'standing_jump': ('standing/Standing Jump', 'flat', [(1.0, {'charge': 1.0, 'say': 'hop'})], 1.0, ('rise', 'flight', 'travel')),
    'crawl_backwards': ('crawl/Crawl Backwards', 'flat', [(1.0, {'say': 'scoot_floor'}), (8.0, {'twist': _BACK})], 8.0,
                        ('travel', 'back', 'knee', 'hips back')),
}
