# Architecture: Views

The terminal and what it draws with: its pages and live views, the 3D
renderers, the 2D drawings. Findings: [host](../findings/host.md).

```text
terminal/           python -m terminal: loader, menu, readout
terminal/pages/     the front page's entries, one module each
terminal/views/     the live views (show_*.py), each runnable on its own
terminal/ui/        what they draw with: stage, chrome (the house HUD: CRT snow, lock,
                    clock, kana tags), screen, console, bindings (every page's
                    keys, one table: Esc's start menu, Tab's column, ?'s keys),
                    taskbar (the key bar, its help, the start menu, the folded
                    column's rail), scroll, recorder (R: a page's rows to a
                    CSV, its counter on the page), ..
coaxial/graphics/   board renderer (wireframe pipeline + one module per concern);
                    gpu (the raster and a lit mesh on a card, wgpu; the crew
                    where none), gynoid (the figure's body, posed and lit),
                    shapes and lit (a lit mesh's parts, its light and its
                    braille), callouts (framed rows, leaders), quadcopter and
                    scenery (the quad of machine.quad and its course, drawn),
                    inset (a picture in the viewport's corner, the
                    instruments' frame: humanoid robot eyes' view; the quad's carried
                    camera's, onboard)
coaxial/draw/       2D drawings: dials, gauges, cross_section, thermal map
tools/render/       renderer checks against the exporter; ansi2png; attitude (the
                    view at a pose and a moment, to a PNG - no window); page (any
                    page's last frame, simulated, to a PNG)
```

- The repository's `render/`: the board's STL models and `render_demo.ps1`;
  its `terminal/`: a launcher a view (`adc.ps1`, `angle.ps1`, ..).
- Suites: `test_render.py` the 3D engine stage by stage; the pages' in
  `VIEWS` (`tools/dev/suites.py`).
