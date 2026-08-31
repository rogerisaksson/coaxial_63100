

## The chooser opened BOARD CHAT on every start, and the first fix was a guess

2026-08-31. ESC from a view under MOTOR CONTROLLER or BOARD CHAT was to
reopen the front page on that question (`menu.py --open`), so the chooser
got a list of those views, `$Asked`. From then on every start went
straight into the chat page. First answer: a PowerShell script reads the
caller's variables when its own are unset, so a `$view` left in the
session leaked in - demonstrated (`$view='chat'; & { $from = $view }`
reads `chat`), an initialiser added, and the symptom stayed.

What found it: the front page alone in a hidden console, every key it saw
logged - 60 frames, 9.39 s, no key, returned 0. So not input. The chooser
itself in a hidden console, its child processes listed: the first child
was the chat page. `$asked = $null; $Asked = @(...)` then printed `$asked`
as the list - **PowerShell variable names are case-insensitive**, `$Asked`
and `$asked` (the `-Name` parameter) are one variable, `$view` became the
list, `while (-not $view)` skipped the front page, and `$Views[$view].Chat`
on an array is truthy. Renamed `$SubViews`; the same run then showed
`python -X utf8 tools/menu.py --port COM4` as the first child, no `--open`.
Two lessons kept: the trap is in CLAUDE.md, and a plausible mechanism
with a demonstration is still not the cause until the symptom is
reproduced and gone.
