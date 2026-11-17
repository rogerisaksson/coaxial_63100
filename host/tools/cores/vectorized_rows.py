"""The cores' vectorised loops: written by host/tools/cores/vectorized.py, not by hand."""
# A section a compiler, its major version and its machine: its targets, then each source's functions'
# vectorised loops a target. A loop lost fails test_vectorized; the tool's --write lays a host's section.
ROWS = {
    'gcc 13 aarch64': (('sve2',), {
        'wbc_body.c': {
            'bracket_t': (1,),
            'compose': (2,),
            'out_of': (1,),
            'twist_exp': (1,),
            'wbc_body_bias': (4,),
            'wbc_body_com': (1,),
            'wbc_body_drift': (1,),
            'wbc_body_jacobian': (2,),
            'wbc_body_mass': (2,),
            'wbc_body_momentum': (2,),
            'wbc_body_point': (1,),
        },
        'wbc_borne.c': {
            'normal_of': (2,),
            'room_of': (2,),
            'wbc_borne_read': (2,),
            'wbc_borne_seated': (4,),
            'wbc_borne_supports': (2,),
        },
        'wbc_chain.c': {
            'wbc_chain': (3,),
        },
        'wbc_chain_qp.c': {
            'affine': (1,),
            'keyed': (1,),
            'unknowns': (1,),
        },
        'wbc_chain_rows.c': {
            'wbc_chain_levels': (2,),
            'wbc_chain_rows': (1,),
        },
        'wbc_chain_tasks.c': {
            'laid': (1,),
            'wbc_chain_tasks': (1,),
        },
        'wbc_cone.c': {
            'faced': (2,),
            'kinetic': (5,),
            'taken': (1,),
            'wbc_cone_bounded': (2,),
        },
        'wbc_floor.c': {
            'wbc_imp_floor_at': (1,),
            'wbc_imp_landings': (2,),
        },
        'wbc_gmo.c': {
            'accelerations': (2,),
            'added': (1,),
            'closest': (1,),
            'expect': (1,),
            'lessened': (1,),
            'pushes': (1,),
            'shared': (1,),
            'split': (1,),
            'stage': (1,),
            'touches': (4,),
            'wbc_gmo_felt': (1,),
        },
        'wbc_imp.c': {
            'wbc_imp_ask': (11,),
        },
        'wbc_law.c': {
            'wbc_law_couple': (2,),
            'wbc_law_latent': (3,),
        },
        'wbc_maps.c': {
            'inv6': (1,),
            'wbc_step_maps': (6,),
        },
        'wbc_pass.c': {
            'wbc_pass_contacts': (2,),
            'wbc_pass_tick': (1,),
        },
        'wbc_patch.c': {
            'wbc_patch_rows': (2,),
        },
        'wbc_poise.c': {
            'spanned_of': (1,),
        },
        'wbc_poses.c': {
            'wbc_imp_poses': (1,),
        },
        'wbc_qp.c': {
            'dual': (1,),
            'wbc_qp_solve': (1,),
        },
        'wbc_registry.c': {
            'seen_of': (1,),
            'w2': (1,),
            'wbc_registry_seen': (1,),
        },
        'wbc_rounds.c': {
            'taken': (2,),
            'wbc_rounds': (2,),
        },
        'wbc_scoot.c': {
            'wbc_scoot_step': (1,),
        },
        'wbc_shrunk.c': {
            'wbc_wrench_motion': (1,),
            'wbc_wrench_out': (7,),
        },
        'wbc_sil.c': {
            'wbc_sil_drive': (1,),
        },
        'wbc_solve.c': {
            'dot': (1,),
            'wbc_cholesky': (2,),
            'wbc_level_solve': (17,),
        },
        'wbc_stack.c': {
            'lay': (1,),
            'lay_bounds': (2,),
            'lay_level': (1,),
            'total_of': (1,),
            'wbc_stack_step': (1,),
        },
        'wbc_step.c': {
            'stepped': (4,),
        },
        'wbc_stride.c': {
            'wbc_imp_reading': (1,),
            'wbc_imp_under': (1,),
            'wbc_imp_vaults': (1,),
        },
        'wbc_surety.c': {
            'wbc_imp_surety': (1,),
        },
        'wbc_swing.c': {
            'wbc_imp_rested': (2,),
            'wbc_imp_swings': (4,),
        },
        'wbc_touch.c': {
            'wbc_touch_step': (4,),
        },
        'wbc_wait.c': {
            'wbc_wait_latched': (1,),
            'wbc_wait_risen': (1,),
            'wbc_wait_sunk': (1,),
            'wbc_wait_woken': (1,),
        },
        'wbc_world.c': {
            'wbc_world_contacted': (1,),
            'wbc_world_held': (2,),
        },
        'wbc_wrench.c': {
            'maps': (6,),
            'over': (1,),
            'passive_of': (6,),
            'solved': (4,),
            'wbc_wrench_step': (5,),
        },
    }),
    'gcc 15 aarch64': (('sve2', 'sme2'), {
        'wbc_body.c': {
            'bracket_t': (1, 1),
            'compose': (2, 2),
            'out_of': (1, 1),
            'wbc_body_bias': (4, 4),
            'wbc_body_com': (1, 1),
            'wbc_body_drift': (1, 1),
            'wbc_body_jacobian': (2, 2),
            'wbc_body_mass': (2, 2),
            'wbc_body_momentum': (3, 3),
            'wbc_body_point': (1, 1),
        },
        'wbc_borne.c': {
            'normal_of': (2, 2),
            'room_of': (2, 2),
            'wbc_borne_read': (2, 2),
            'wbc_borne_seated': (2, 2),
            'wbc_borne_supports': (1, 1),
        },
        'wbc_chain_qp.c': {
            'affine': (1, 1),
            'unknowns': (1, 1),
        },
        'wbc_chain_rows.c': {
            'wbc_chain_levels': (1, 1),
            'wbc_chain_rows': (1, 1),
        },
        'wbc_chain_tasks.c': {
            'wbc_chain_tasks': (1, 1),
        },
        'wbc_classes.c': {
            'laid': (1, 1),
        },
        'wbc_cone.c': {
            'faced': (1, 1),
            'kinetic': (5, 5),
            'taken': (2, 2),
            'wbc_cone_bounded': (2, 2),
        },
        'wbc_floor.c': {
            'wbc_imp_floor_at': (1, 1),
        },
        'wbc_gmo.c': {
            'accelerations': (2, 2),
            'added': (1, 1),
            'closest': (1, 1),
            'columns': (1, 1),
            'expect': (1, 1),
            'lessened': (1, 1),
            'shared': (1, 1),
            'split': (1, 1),
            'stage': (1, 1),
            'touches': (4, 4),
            'wbc_gmo_felt': (1, 1),
            'wbc_gmo_step': (1, 1),
        },
        'wbc_imp.c': {
            'wbc_imp_ask': (6, 6),
        },
        'wbc_law.c': {
            'wbc_law_couple': (2, 2),
            'wbc_law_latent': (3, 3),
        },
        'wbc_libm.c': {
            'lm_reduce_run': (1, 1),
            'lm_run_arccos': (1, 1),
            'lm_run_arcsin': (1, 1),
            'lm_run_arctan': (1, 1),
            'lm_run_arctan2': (1, 1),
            'lm_run_cos': (1, 1),
            'lm_run_exp': (1, 1),
            'lm_run_expm1': (1, 1),
            'lm_run_hypot': (1, 1),
            'lm_run_log': (1, 1),
            'lm_run_sin': (1, 1),
            'lm_run_sincos': (1, 1),
            'lm_run_tan': (1, 1),
        },
        'wbc_maps.c': {
            'wbc_step_maps': (3, 3),
        },
        'wbc_pass.c': {
            'wbc_pass_contacts': (1, 1),
        },
        'wbc_poise.c': {
            'spanned_of': (1, 1),
        },
        'wbc_poses.c': {
            'wbc_imp_poses': (1, 1),
        },
        'wbc_qp.c': {
            'wbc_qp_solve': (1, 1),
        },
        'wbc_registry.c': {
            'seen_of': (1, 1),
            'w2': (1, 1),
        },
        'wbc_rounds.c': {
            'taken': (2, 2),
            'wbc_within': (1, 1),
        },
        'wbc_shrunk.c': {
            'wbc_wrench_motion': (2, 2),
            'wbc_wrench_out': (3, 3),
            'wbc_wrench_torques': (1, 1),
        },
        'wbc_solve.c': {
            'reduced': (1, 1),
            'wbc_level_solve': (12, 12),
        },
        'wbc_stack.c': {
            'lay': (1, 1),
            'lay_bounds': (2, 2),
            'lay_level': (4, 4),
            'total_of': (2, 2),
            'wbc_stack_init': (2, 2),
            'wbc_stack_step': (2, 2),
        },
        'wbc_step.c': {
            'stepped': (2, 2),
        },
        'wbc_stride.c': {
            'wbc_imp_reading': (1, 1),
            'wbc_imp_vaults': (1, 1),
        },
        'wbc_swing.c': {
            'wbc_imp_rested': (1, 1),
            'wbc_imp_swings': (2, 2),
        },
        'wbc_touch.c': {
            'wbc_touch_step': (4, 4),
        },
        'wbc_wait.c': {
            'wbc_wait_risen': (1, 1),
            'wbc_wait_woken': (1, 1),
        },
        'wbc_wholes.c': {
            'laid': (1, 1),
        },
        'wbc_world.c': {
            'wbc_world_held': (1, 1),
            'wbc_world_stuck': (1, 1),
        },
        'wbc_wrench.c': {
            'maps': (5, 5),
            'over': (1, 1),
            'passive_of': (4, 4),
            'solved': (4, 4),
            'wbc_wrench_step': (5, 5),
        },
    }),
    'gcc 15 x86_64': (('avx', 'avx2', 'avx512'), {
        'wbc_body.c': {
            'bracket_t': (1, 1, 1),
            'compose': (2, 2, 2),
            'out_of': (1, 1, 1),
            'twist_exp': (0, 0, 1),
            'wbc_body_bias': (4, 4, 4),
            'wbc_body_com': (1, 1, 1),
            'wbc_body_drift': (1, 1, 1),
            'wbc_body_jacobian': (2, 2, 2),
            'wbc_body_mass': (2, 2, 2),
            'wbc_body_momentum': (3, 3, 3),
            'wbc_body_point': (1, 1, 1),
        },
        'wbc_borne.c': {
            'normal_of': (2, 2, 2),
            'room_of': (2, 2, 2),
            'wbc_borne_read': (2, 2, 2),
            'wbc_borne_seated': (2, 2, 3),
            'wbc_borne_supports': (1, 1, 2),
        },
        'wbc_chain.c': {
            'wbc_chain': (2, 2, 3),
        },
        'wbc_chain_qp.c': {
            'affine': (1, 1, 1),
            'keyed': (0, 1, 1),
            'unknowns': (1, 1, 1),
        },
        'wbc_chain_rows.c': {
            'wbc_chain_levels': (2, 2, 2),
            'wbc_chain_rows': (1, 1, 1),
        },
        'wbc_chain_tasks.c': {
            'laid': (1, 1, 1),
            'wbc_chain_tasks': (1, 1, 1),
        },
        'wbc_classes.c': {
            'laid': (1, 1, 1),
        },
        'wbc_cone.c': {
            'faced': (1, 1, 1),
            'kinetic': (5, 5, 5),
            'taken': (2, 2, 3),
            'wbc_cone_bounded': (2, 2, 2),
        },
        'wbc_floor.c': {
            'wbc_imp_floor_at': (0, 1, 1),
            'wbc_imp_landings': (2, 2, 2),
        },
        'wbc_gmo.c': {
            'accelerations': (2, 2, 2),
            'added': (1, 1, 1),
            'closest': (1, 1, 1),
            'columns': (1, 1, 1),
            'expect': (1, 1, 1),
            'lessened': (1, 1, 1),
            'pushes': (0, 0, 1),
            'shared': (1, 1, 1),
            'split': (1, 1, 1),
            'stage': (1, 1, 1),
            'touches': (3, 3, 3),
            'wbc_gmo_felt': (1, 1, 1),
            'wbc_gmo_step': (1, 1, 1),
        },
        'wbc_imp.c': {
            'wbc_imp_ask': (7, 7, 12),
        },
        'wbc_law.c': {
            'wbc_law_couple': (1, 1, 2),
            'wbc_law_latent': (3, 3, 3),
        },
        'wbc_libm.c': {
            'lm_reduce_run': (1, 1, 1),
            'lm_run_arccos': (1, 1, 1),
            'lm_run_arcsin': (1, 1, 1),
            'lm_run_arctan': (1, 1, 1),
            'lm_run_arctan2': (1, 1, 1),
            'lm_run_cos': (1, 1, 1),
            'lm_run_exp': (1, 1, 1),
            'lm_run_expm1': (1, 1, 1),
            'lm_run_hypot': (1, 1, 1),
            'lm_run_log': (1, 1, 1),
            'lm_run_sin': (1, 1, 1),
            'lm_run_sincos': (1, 1, 1),
            'lm_run_tan': (1, 1, 1),
        },
        'wbc_maps.c': {
            'wbc_step_maps': (2, 2, 4),
        },
        'wbc_pass.c': {
            'wbc_pass_contacts': (1, 1, 1),
        },
        'wbc_patch.c': {
            'wbc_patch_rows': (2, 2, 3),
        },
        'wbc_poise.c': {
            'spanned_of': (1, 1, 1),
        },
        'wbc_poses.c': {
            'wbc_imp_poses': (1, 1, 1),
        },
        'wbc_qp.c': {
            'dual': (0, 0, 1),
            'wbc_qp_solve': (1, 1, 2),
        },
        'wbc_registry.c': {
            'seen_of': (1, 2, 2),
            'w2': (1, 1, 1),
            'wbc_registry_seen': (0, 0, 1),
        },
        'wbc_rounds.c': {
            'taken': (2, 2, 2),
            'wbc_within': (1, 1, 2),
        },
        'wbc_shrunk.c': {
            'wbc_wrench_motion': (3, 3, 3),
            'wbc_wrench_out': (4, 4, 4),
            'wbc_wrench_torques': (2, 2, 2),
        },
        'wbc_sil.c': {
            'wbc_sil_drive': (0, 0, 1),
        },
        'wbc_solve.c': {
            'dot': (1, 1, 1),
            'reduced': (2, 2, 2),
            'wbc_cholesky': (2, 2, 2),
            'wbc_level_solve': (16, 16, 21),
        },
        'wbc_stack.c': {
            'bound': (1, 1, 1),
            'lay': (1, 1, 1),
            'lay_bounds': (3, 3, 3),
            'lay_level': (5, 5, 5),
            'prepare': (1, 1, 1),
            'total_of': (2, 2, 2),
            'wbc_stack_init': (2, 2, 2),
            'wbc_stack_step': (3, 3, 3),
        },
        'wbc_step.c': {
            'stepped': (3, 4, 5),
        },
        'wbc_stride.c': {
            'wbc_imp_reading': (1, 1, 1),
            'wbc_imp_under': (1, 1, 1),
            'wbc_imp_vaults': (1, 1, 1),
        },
        'wbc_swing.c': {
            'wbc_imp_rested': (1, 1, 1),
            'wbc_imp_swings': (4, 4, 4),
        },
        'wbc_touch.c': {
            'wbc_touch_step': (4, 4, 4),
        },
        'wbc_wait.c': {
            'wbc_wait_risen': (1, 1, 1),
            'wbc_wait_woken': (0, 0, 1),
        },
        'wbc_wholes.c': {
            'laid': (1, 1, 1),
        },
        'wbc_world.c': {
            'wbc_world_contacted': (0, 0, 1),
            'wbc_world_held': (1, 1, 2),
            'wbc_world_slipped': (1, 1, 1),
            'wbc_world_stuck': (1, 1, 1),
        },
        'wbc_wrench.c': {
            'maps': (6, 6, 7),
            'over': (1, 1, 1),
            'passive_of': (4, 4, 6),
            'solved': (4, 4, 4),
            'wbc_wrench_step': (5, 5, 5),
        },
    }),
    'gcc 16 x86_64': (('avx', 'avx2', 'avx512'), {
        'wbc_body.c': {
            'bracket_t': (1, 1, 1),
            'compose': (2, 2, 2),
            'out_of': (1, 1, 1),
            'twist_exp': (0, 0, 1),
            'wbc_body_bias': (4, 4, 4),
            'wbc_body_com': (1, 1, 1),
            'wbc_body_drift': (1, 1, 1),
            'wbc_body_jacobian': (2, 2, 2),
            'wbc_body_mass': (2, 2, 2),
            'wbc_body_momentum': (3, 3, 3),
            'wbc_body_point': (1, 1, 1),
        },
        'wbc_borne.c': {
            'normal_of': (2, 2, 2),
            'room_of': (2, 2, 2),
            'wbc_borne_read': (1, 1, 1),
            'wbc_borne_seated': (2, 2, 3),
            'wbc_borne_supports': (1, 1, 2),
        },
        'wbc_chain.c': {
            'wbc_chain': (2, 2, 3),
        },
        'wbc_chain_qp.c': {
            'affine': (2, 2, 2),
            'keyed': (0, 1, 1),
            'unknowns': (1, 1, 1),
            'wbc_chain_qp': (1, 1, 1),
        },
        'wbc_chain_rows.c': {
            'wbc_chain_levels': (2, 2, 2),
            'wbc_chain_rows': (1, 1, 1),
        },
        'wbc_chain_tasks.c': {
            'laid': (1, 1, 1),
            'wbc_chain_tasks': (1, 1, 1),
        },
        'wbc_classes.c': {
            'laid': (1, 1, 1),
        },
        'wbc_cone.c': {
            'faced': (1, 1, 1),
            'kinetic': (5, 5, 5),
            'taken': (2, 2, 3),
            'wbc_cone_bounded': (2, 2, 2),
        },
        'wbc_floor.c': {
            'wbc_imp_floor_at': (0, 1, 1),
            'wbc_imp_landings': (2, 2, 2),
        },
        'wbc_gmo.c': {
            'accelerations': (2, 2, 2),
            'added': (1, 1, 1),
            'closest': (1, 1, 1),
            'columns': (1, 1, 1),
            'expect': (1, 1, 1),
            'left': (1, 1, 1),
            'lessened': (1, 1, 1),
            'pushes': (0, 0, 1),
            'shared': (1, 1, 1),
            'split': (1, 1, 1),
            'stage': (1, 1, 1),
            'touches': (3, 3, 3),
            'wbc_gmo_felt': (1, 1, 1),
            'wbc_gmo_step': (1, 1, 1),
        },
        'wbc_imp.c': {
            'wbc_imp_ask': (6, 6, 9),
        },
        'wbc_law.c': {
            'wbc_law_couple': (1, 1, 1),
            'wbc_law_latent': (2, 2, 2),
        },
        'wbc_libm.c': {
            'lm_reduce_run': (1, 1, 1),
            'lm_run_arccos': (1, 1, 1),
            'lm_run_arcsin': (1, 1, 1),
            'lm_run_arctan': (1, 1, 1),
            'lm_run_arctan2': (1, 1, 1),
            'lm_run_cos': (1, 1, 1),
            'lm_run_exp': (1, 1, 1),
            'lm_run_expm1': (1, 1, 1),
            'lm_run_hypot': (1, 1, 1),
            'lm_run_log': (1, 1, 1),
            'lm_run_sin': (1, 1, 1),
            'lm_run_sincos': (1, 1, 1),
            'lm_run_tan': (1, 1, 1),
        },
        'wbc_maps.c': {
            'wbc_step_maps': (2, 2, 4),
        },
        'wbc_pass.c': {
            'wbc_pass_contacts': (1, 1, 1),
        },
        'wbc_patch.c': {
            'wbc_patch_rows': (2, 2, 3),
        },
        'wbc_poses.c': {
            'wbc_imp_poses': (1, 1, 1),
        },
        'wbc_qp.c': {
            'dual': (0, 0, 1),
            'wbc_qp_solve': (1, 1, 2),
        },
        'wbc_registry.c': {
            'seen_of': (2, 2, 2),
            'w2': (1, 1, 1),
            'wbc_registry_seen': (0, 0, 1),
        },
        'wbc_rounds.c': {
            'taken': (1, 1, 1),
        },
        'wbc_shrunk.c': {
            'wbc_wrench_motion': (3, 3, 3),
            'wbc_wrench_out': (4, 4, 4),
            'wbc_wrench_torques': (2, 2, 2),
        },
        'wbc_sil.c': {
            'wbc_sil_drive': (0, 0, 1),
        },
        'wbc_solve.c': {
            'dot': (1, 1, 1),
            'reduced': (2, 2, 2),
            'wbc_cholesky': (2, 2, 2),
            'wbc_level_solve': (18, 18, 23),
        },
        'wbc_stack.c': {
            'bound': (1, 1, 1),
            'lay': (1, 1, 1),
            'lay_bounds': (3, 3, 3),
            'lay_level': (5, 5, 5),
            'prepare': (1, 1, 1),
            'total_of': (2, 2, 2),
            'wbc_stack_init': (2, 2, 2),
            'wbc_stack_step': (3, 3, 3),
        },
        'wbc_step.c': {
            'stepped': (3, 4, 5),
        },
        'wbc_stride.c': {
            'wbc_imp_reading': (1, 1, 1),
            'wbc_imp_under': (1, 1, 1),
            'wbc_imp_vaults': (1, 1, 1),
        },
        'wbc_swing.c': {
            'wbc_imp_rested': (1, 1, 1),
            'wbc_imp_swings': (4, 4, 4),
        },
        'wbc_touch.c': {
            'wbc_touch_step': (3, 3, 3),
        },
        'wbc_wait.c': {
            'wbc_wait_risen': (1, 1, 1),
        },
        'wbc_wholes.c': {
            'laid': (1, 1, 1),
        },
        'wbc_world.c': {
            'wbc_world_contacted': (0, 0, 1),
            'wbc_world_held': (1, 1, 2),
            'wbc_world_slipped': (1, 1, 1),
            'wbc_world_stuck': (1, 1, 1),
        },
        'wbc_wrench.c': {
            'maps': (6, 6, 7),
            'over': (1, 1, 1),
            'passive_of': (4, 4, 6),
            'solved': (4, 4, 4),
            'wbc_wrench_step': (5, 5, 5),
        },
    }),
}
