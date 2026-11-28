"""The suites run right after the structural ones, what CI left red at their head.

Then those that regressed before: a regression stops a run at once."""

#: Suites with prior CI or local regressions, scheduled right after structural checks so a regression stops the run at once:
#: what GitHub's last host run left red is laid at its head at every commit (tools/dev/ci.py, the pre-commit).
FAILED_FIRST = ('test_world_core.py', 'test_errands.py', 'test_sysid_live.py', 'test_scenarios.py',
                'test_chair_seated.py', 'test_leap.py', 'test_ortho.py', 'test_docks.py',
                'test_ortho_texts.py', 'test_views_terminal.py', 'test_views_flying.py',
                'test_vectorized.py', 'test_camera.py', 'test_daq_api.py', 'test_views_quad.py',
                'test_quick.py', 'test_tick.py', 'test_kernel.py', 'test_engrams.py',
                'test_move_handover.py', 'test_native_heat.py', 'test_ci.py', 'test_smart.py',
                'test_wayfinder.py', 'test_views_built.py', 'test_ported.py',
                'test_quad_streams.py', 'test_situation.py', 'test_thermal_app.py', 'test_wear.py',
                'test_wasd.py', 'test_wrench.py', 'test_views_apps.py', 'test_stiction.py',
                'test_introspect.py', 'test_floor.py', 'test_wbc_core.py', 'test_ollama_runner.py',
                'test_se3_core.py', 'test_native.py', 'test_mirrors.py', 'test_sysid.py',
                'test_dsp.py', 'test_views_grove.py', 'test_sight.py', 'test_gaze.py',
                'test_replays.py', 'test_bedside.py', 'test_substrate.py', 'test_vault.py',
                'test_emulator.py')
