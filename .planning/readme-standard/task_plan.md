# Standard deployment README

Goal: replace the root README with a Chinese deployment and maintenance guide organized from environment through build, run and debugging; remove session-specific change logs and duplicate procedures.

1. Read existing deployment content and verify source defaults — complete.
2. Write standard environment, configuration, build, service/runtime and diagnostic sections — complete.
3. Check commands, document links, anchors and Markdown; review final diff — complete.

Keep source code, runtime data and user/device-specific settings unchanged. Retain the existing service-section anchor for links from package docs.

Validation notes: inline WSL Bash check lost shell variables during native argument quoting; use a saved check script instead. A guessed RPP YAML filename was absent; located rpp_local_planner_params.yaml and confirmed /odom_wheel directly.
Extracted Bash snippets initially used Windows CRLF, invalidating continuation checks; normalized the temporary snippets to LF. All 32 snippets then passed bash -n. Root README uses LF. Link/anchor/fence and documented entrypoint checks passed; no deployment commands executed.
