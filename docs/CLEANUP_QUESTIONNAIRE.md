# Repository cleanup questionnaire

Fill in the **Answer** fields or table cells and send this Markdown back.
Short answers are enough. Write `recommended` to accept a suggestion, or
`you decide` for implementation choices. Blank answers remain undecided.
Do not include passwords, private keys, or tokens.

## Requirements already given

- Normal access to the drone should use Wi-Fi.
- The Pi may create its own Wi-Fi hotspot only through a direct command.
  Losing a connection or finding no saved network must not trigger a hotspot.
- Prefer concise, functional code.
- This document prepares the cleanup; it does not implement it.

## What I would do

1. Remove features and platforms you no longer need, together with their setup
   paths, tests, configuration, and maintained documentation.
2. Give Pi networking one clear policy: saved client networks normally, hotspot
   activation explicitly requested. Prefer NetworkManager's existing connection
   management over a custom retry service where it meets your requirements.
3. Separate decisions from effects: small functions transform data and choose
   actions; a small execution layer owns SSH, camera, serial, threads, and GPIO.
4. Reduce the large recording, flight-controller, and tag/servo modules around
   those boundaries. Consolidate shared logic where its behavior really agrees.
5. Keep tests for observable behavior, failure handling, and hardware safeguards;
   allow internal structure and tests tied to it to change.

This is based on local source inspection, not a new live check of the Pi.
The current code has automatic hotspot paths in the network selector, hotspot
setup, and dual-network setup. It also carries Pi USB Ethernet, three laptop
platforms, and compatibility imports from earlier refactors. The largest cleanup
candidates include [recording](../ai_drone/cli/record.py),
[flight control](../ai_drone/flight/controller.py),
[tag/servo recording](../ai_drone/cli/tag_servo_record.py), and
[connection/deployment](../ai_drone/link/).

## A. What the project should contain

### Q1. What is the drone's intended job?

Describe the desired end-to-end use in a few sentences. Distinguish what you
need now from future work such as autonomous room search and payload delivery.

**Answer:**

- Needed now: The drone should be able to take off, land, hold altitude, enable loiter mode, detect apriltags, activate a servo based on the april tag, certain obstacle detection, make good use of the ai camera...
- the exact requirements are these:
Task 1: Familiarization with the FPV drone as well as its hardware and software components.

Learn the capabilities and limitations of the hardware components. These include:

Frame, flight controller with ESC (electronic speed controller), video transmitter (VTX), ELRS receiver, motors, GPS receiver with compass, FPV camera, Li-ion batteries, radio controller, Raspberry Pi Zero 2 WH, Raspberry Pi AI camera module, FPV goggles, A/V video grabber, etc.

Learn the capabilities and limitations of the software components. These include:

Flight controller firmware (Betaflight, INAV, ArduPilot), ground control station / mission planning software (e.g., QGroundControl for ArduPilot, INAV Configurator for INAV), operating systems (e.g., Raspberry Pi OS) for the single-board computer, as well as AI software (e.g., TensorFlow Lite, YOLO), etc.

Task 2: Drone expansion aimed at enabling automated delivery of objects (including indoors).

This task comprises several subtasks (see Tasks 3–6).

Task 3: Integration of an autopilot feature.

Preferably using ArduPilot, alternatively using INAV.

Task 4: Integration of Position Hold and Altitude Hold using distance measurement (LiDAR) and optical flow.

Use the MicroAir MTF-01P sensor.

Task 5: Research and integrate implementation options for delivery/payload systems.

Design and test a simple drop mechanism using a micro servo (e.g., [1], [2], [3], [4], [5]). 3D printing facilities are available.

Task 6: Design an improved frame or an extension for the existing frame to accommodate the distance measurement and optical flow sensors as well as the delivery/payload mechanism.

Use suitable software solutions such as Tinkercad or UltiMaker Cura for this purpose. 3D printing facilities are available.

Task 7: Documentation and presentation of results from Tasks 1–6.

Create documentation and tutorials that enable students, researchers, and instructors to replicate the AI drone scenarios and use them for their own courses and research projects.

No slide decks or traditional PDF project reports will be prepared. Instead, each team will develop comprehensive, accessible online documentation (e.g., via GitHub Pages) and present their results as a poster along with a live demonstration.


- Later:
  An extensible autonomous drone, including AprilTag-based payload delivery and
  potentially obstacle detection using the forward-facing LiDAR.
- Your three most common workflows: connect to the drone via ssh then run the drone disarmed and check sensor readings from all sensors and maybe test the servo motor. then: testing the loiter mode that currently doesnt work because it always reports a too high magnetic disturbance
- Biggest current annoyance: the code is just way to big and as way too many markdown files in it, way too much documentation, comments, docstrings etc. the code just isnt concise enough

### Q2. Which existing capabilities should survive?

Use `keep`, `simplify`, `remove`, or `unsure`. For `simplify`, say what part you use.

| Capability | Decision / essential part |
| --- | --- |
| Disarmed camera and sensor recording (`drone-inspect`) | keep — read and record all sensor data while also detecting AprilTags and estimating their poses; video saving and servo activation must each be optional. |
| Timed recordings that survive SSH disconnect (`drone-walk`) | keep recording independently of SSH — video and log saving must continue locally after disconnect; recording duration is a runtime option. |
| Recording during a pilot-controlled flight | keep — recording and AprilTag detection must also work while armed and flying, both under manual radio control and autonomous control, with optional tag-triggered servo activation. |
| AprilTag detection and pose estimation | keep both — they must work even when saving camera video is disabled. |
| Live camera preview in a browser | keep — a desirable optional capability. |
| CSV exports and offline HTML/video reports | keep — retain reports and exports, local video saving when enabled, and all generated logs. Use JSON Lines for structured events alongside native logs; generate CSV/HTML reports afterward, preferably on a laptop. |
| Sensor/firmware checks and configuration snapshots | keep — retain configuration snapshots and all kinds of diagnostics the Pi/drone can produce. |
| Automatic takeoff, Loiter hold, and landing (`drone-control hover`) | keep — takeoff, landing, altitude hold, and Loiter are required capabilities toward the eventual autonomous drone. |
| Tag-triggered payload-servo operation during recording | keep — disabled unless explicitly enabled for that run; support disarmed recording and manual or autonomous flight. Easily select one tag ID, multiple IDs, or an ID range. |
| Direct servo and motor bench-test commands | keep — directly trigger the servo and motors, with selectable motor speed. |
| Connection/power status and controlled shutdown helpers | keep — retain diagnostics and maintenance helpers, with helper entry points under `scripts/`. |
| Pi OS upgrade and power-resilience setup scripts | keep — retain upgrade, Pi OS setup, and resilience tooling under `scripts/`. |
| Custom firmware verification and simulator support | keep — custom firmware, its diagnostics/verification, and simulator support are essential. |

**Answer details:**

- Prefer small Python scripts for individual jobs that can be composed together.
  The codebase must be easy to edit and extend with additional capabilities.
- Place maintenance/helper entry points under `scripts/`, including upgrades,
  USB fallback access, rotor/motor tests, servo tests, resilience, and manual
  hotspot control. Retain their capabilities while simplifying the code.
- Sensor recording and AprilTag detection/pose estimation must remain usable
  with video saving or servo activation disabled. Control these options
  independently for the same workflow.
- AprilTags will always lie flat on the ground. The drone will usually fly at
  1–3 metres, and the current tags are printed on DIN A4 pages.
  Accept the recommendation to retain `tag36h11` and test larger A3 prints
  around 22.4 cm across the measured black square first. Set focus and calibrate
  for the actual camera setup; evaluate detection and pose reliability at the
  working heights before increasing analysis resolution or changing hardware.
- When video recording is enabled, save the videos on the drone. All generated
  logs must also be saveable locally, including detections and pose estimates
  when video saving is disabled. Saving must work offline and continue after
  SSH disconnect, without requiring video streaming or overloading the connection.
- Check free storage before and during recording, warn early, and show an
  estimate of recording time remaining. Reserve space for logs and stop video
  cleanly before using that reserve. Keep thresholds configurable and choose
  defaults during implementation and testing.
- Provide an explicit transfer helper under `scripts/` to send videos, logs,
  and reports to a laptop or another selected machine. Verify copied files
  before deleting originals from the Pi; deletion is an explicit operation.
  Bulk transfer must not be required for recording to continue offline.
- Eventually support autonomous flight with AprilTag detection and potentially
  use the forward-facing LiDAR for obstacle detection.
- If possible, record actual motor/propeller rotational speed during both manual
  and autonomous flight.

### Q3. What must run together, and what data do you actually use?

Examples: recording + preview; recording + tags + servo; flight control +
recording; status checks during recording. These affect ownership of the camera
and flight-controller connection.

**Answer:**

- Required simultaneous combinations:
  All-sensor recording + AprilTag detection + pose estimation, with independent
  options for local video saving and tag-triggered servo activation. Support
  disarmed operation, manual radio-controlled flight, and autonomous flight.
  Browser preview is desirable and optional; recording must not depend on it.
- Required outputs: video / MAVLink logs / telemetry JSONL / tag detections and
  poses / CSV / offline HTML / other:
  Save video locally when enabled, and make all generated logs saveable,
  including sensor readings, AprilTag detections and poses, and actual
  motor/propeller RPM if available. Accept JSON Lines (`.jsonl`) for structured
  telemetry, detections, poses, servo events, and diagnostics, alongside native
  MAVLink `.tlog` and available flight-controller logs. Use JSON for recording
  metadata/configuration summaries while preserving native parameter snapshots.
  Generate CSV exports and HTML reports afterward, preferably on the laptop.
  Detection and pose logging must also work without saving video.
- During ordinary inspection, if a camera or sensor is missing: record available
  sources with a clear incomplete status, or stop? Which sources are mandatory?
  Report missing sources clearly and continue with the available sources.
  A missing camera or sensor must not stop ordinary inspection/recording.
- Any required AprilTag backend: native / OpenCV / no preference:
  Native AprilTag 3 on the Pi. Keep OpenCV for calibration and pose estimation,
  with its detector available as a diagnostic fallback. A fallback lacking
  quality information required by servo checks must not bypass those checks.

### Q4. Which machines and versions must we support?

Recommend supporting the hardware and operating systems you actually use.
The repository currently declares Python 3.11–3.13 support.

| Target | Required support |
| --- | --- |
| Current Pi Zero 2 W and FlywooF745 only, or other aircraft too? | Current drone only; no other aircraft required. |
| Developer laptops: Linux, macOS, Windows? | Support Linux, macOS, and Windows. |
| Current Pi OS with NetworkManager only, or older networking setups too? | Current Pi OS only; no older/alternative Pi OS versions required. |
| Python 3.11–3.13, or a narrower range? | No fixed requirement to preserve this range; Python must remain upgradeable, with versions selected according to tested dependency support. |
| Hardware-free development and ArduPilot simulation? | Simulator support is essential; all code needs simulation testing, alongside initial disarmed tests on the Pi. |

### Q5. Which maintenance connections remain?

These are separate connections: laptop → Pi USB Ethernet, laptop → flight
controller USB serial, and the onboard Pi → flight controller UART.
The onboard UART is independent of how you reach the Pi over Wi-Fi.

**Recommendation:** remove Pi USB networking from the active application; retain
onboard UART and direct flight-controller USB maintenance. Keep simulator
network endpoints if simulation stays in scope.

**Answer:**

- Pi USB networking: remove entirely / retain a manual recovery procedure only:
  Retain as a fallback helper under `scripts/`.
- Direct flight-controller USB maintenance: keep / remove project helpers:
  Keep the helpers under `scripts/`.
- Any other connection requirement:
  Keep the current onboard Pi–flight-controller connection and simulator
  support. Maintenance helpers generally belong under `scripts/`, including
  upgrades, rotor/motor and servo tests, and resilience tooling.

## B. Exact Wi-Fi behavior

### Q6. Which saved networks should the Pi use, and how should it reconnect?

The current selector prefers `eduroam`, then other saved client profiles.

**Recommendation:** configurable saved-network priorities, automatic reconnect,
and continued attempts when nothing is reachable, with the hotspot off. Keep a
working connection until it drops rather than interrupting it to chase priority.

**Answer:**

- Networks to support, in priority order; names/types only:
  `eduroam` first, followed by saved Wi-Fi fallbacks using NetworkManager's
  configured profile priorities.
- Join any saved auto-connect profile, or only an explicit allowlist?
  Use saved Wi-Fi networks as fallbacks; no separate allowlist requested.
- Keep a working connection, or switch when a preferred network appears?
  Keep a working connection; prefer `eduroam` when choosing a network. Try
  alternatives while grounded after actual network loss or loss of the
  monitored operator connection. Use an explicit operator heartbeat with a
  configurable timeout, independent of individual SSH terminal sessions.
  Deliberately closing SSH, or having no open SSH terminal, is not a failure
  and must not trigger failover. Configure the operator endpoint(s) locally;
  a working direct local route remains valid if Tailscale is unavailable.
  Never switch networks during flight, including to chase a preferred network.
  Select and validate heartbeat/timeout defaults during implementation.
- If none is reachable: keep retrying / stop after a specified time:
  Accept the recommendation to keep trying saved client networks, with the
  hotspot off, subject to the no-switching-during-flight rule. Starting the
  Pi's own hotspot requires an explicit manual helper command.

### Q7. How should a laptop find and reach the Pi?

Tailscale is an access route over the Pi's network connection; retaining it is
compatible with Wi-Fi-only access. Some shared Wi-Fi networks prevent devices
from talking directly to each other.

Choose: local Wi-Fi only / local Wi-Fi plus optional Tailscale / Tailscale as
the usual route with explicit local access.

**Recommendation:** support explicit local hostname/IP access without internet;
retain Tailscale if teammates use it. Internet failure alone should not cause
the Pi to leave a working Wi-Fi network.

**Answer:**

- Access routes and preferred order:
  Prefer Tailscale and aim to keep the drone reachable through it whenever
  possible; provide direct local access when Tailscale is unavailable. Normal
  access remains over Wi-Fi, with `ssh seb@seb-is-pm` as the current SSH command.
- What must work without internet?
  Onboard sensor recording, AprilTag detection/pose estimation, and local
  video/log saving; direct local access must work when Tailscale is unavailable.
  Deployment should work without internet on the Pi wherever feasible.
  Transfer recordings over a working local connection without requiring cloud
  access; the destination can be a laptop or another selected machine.
- Must teammates connect from outside the drone's local network?
  Yes — both the project owner and teammates need remote access via Tailscale.

### Q8. How should first setup and recovery work when no saved Wi-Fi is reachable?

An unreachable Pi cannot receive a remote command to start its hotspot.
Choose a route you will actually have available.

**Recommendation:** provision a known Wi-Fi network before use and save a phone
hotspot as a recovery network; the Pi joins that network as a client.

**Answer:**

- First setup: preconfigured SD image / local setup / other:
  Provision a known client Wi-Fi network before use. Use NetworkManager on
  the Pi to save credentials during local/USB setup; image provisioning can
  reuse the same settings. No credentials belong in Git.
- Recovery: known phone hotspot / local console / physical command button
  (new hardware work) / manual USB recovery from Q5 / other:
  Try saved fallback networks while grounded; retain manual Pi USB networking
  as a recovery route. Accept the recommendation to provision a phone hotspot
  as a saved fallback where available, with the Pi joining as a client. The
  Pi hotspot is available only through an explicit helper command, issued
  through an existing access route such as USB. No automatic hotspot creation.

### Q9. Where should the explicit hotspot command be issued?

Choose: on the Pi through an existing SSH session / a laptop command that sends
the request to the Pi / both / another direct control.

**Recommendation:** one Pi implementation with a thin laptop wrapper. A network
switch must finish even if it disconnects the initiating SSH session.
Any command names in your answer can be proposed names; no new interface exists yet.

**Answer:**

Accept the recommendation: one Pi implementation with a thin laptop wrapper,
provided as helper tooling under `scripts/`. Switching must finish even if it
disconnects the initiating SSH session. Refuse network switching during flight.

### Q10. How long should an explicitly started hotspot stay on?

**Recommendation:** until an explicit stop or reboot. Stopping it returns to
saved client networks; reboot returns to client mode and requires a fresh
command before another hotspot starts. Setup should save the hotspot profile
without activating it.

**Answer:** recommended / timeout of ___ / another explicitly commanded lifetime:

Recommended: stay active until explicitly stopped or rebooted. Stop/reboot
returns to saved client networks, and starting another hotspot requires a
fresh explicit command. Setup saves the profile without activating it.

### Q11. Do you need hotspot and client Wi-Fi at the same time?

The documented setup uses one onboard radio and switches between those roles.
The repository also contains a setup path for a second Wi-Fi adapter.

**Recommendation:** keep one-radio switching and remove dual-network setup
unless simultaneous operation is a real requirement. Any retained hotspot path
must still obey explicit activation.

**Answer:** one radio / simultaneous operation with a second adapter / other:

One radio with switching between client Wi-Fi and the manually started
hotspot. Accept the recommendation to remove dual-network setup; simultaneous
client/hotspot operation is not required.

### Q12. May a connection command change the laptop's Wi-Fi?

The current helper can join the Pi hotspot and later restore the previous
laptop network.

**Recommendation:** ordinary connection commands use the laptop's current
network. Switching laptop Wi-Fi requires an explicit action.

**Answer:**

- Manual laptop network selection / explicit project switch command /
  automatic selection of an already active Pi hotspot:
  Manual laptop network selection only. Project commands must never switch
  the laptop's Wi-Fi, including when using the Pi's manually started hotspot.
- If the tool switches networks, should it restore the previous network after SSH exits?
  Not applicable: the tool must not switch laptop networks.

### Q13. How much Wi-Fi setup should this repo own?

**Recommendation:** NetworkManager stores credentials on the Pi; project setup
configures the desired policy without saving secrets in Git. Keep the documented
hotspot name `AI-Drone-Zero` and Pi address `192.168.4.1` unless you want them changed.

**Answer:**

- Configure client credentials using OS tools / provide a project setup command:
  Use NetworkManager/OS tools to store credentials on the Pi. Project setup
  configures the agreed network policy without storing secrets in Git.
- Keep hotspot name/address / desired non-secret replacements:
  Keep `AI-Drone-Zero` and `192.168.4.1`.
- Any specific enterprise-network or team-access requirements:
  `eduroam` is the preferred client network; teammates need Tailscale access
  from outside the local network. Retain direct local access as a fallback.

## C. Commands and runtime behavior

### Q14. What should start at boot and survive a lost laptop connection?

**Recommendation:** only network/access services start at boot. Explicit timed
recordings continue locally until completion. Preserve the existing flight
safeguards and flight-controller failsafes during cleanup; Wi-Fi loss and loss
of the onboard Pi–controller connection are distinct events.

**Answer:**

- Tasks, if any, that must start automatically:
  Only network/access services. Recording and flight jobs start explicitly.
- Recording behavior after Wi-Fi/SSH disconnect:
  Continue the explicitly started recording on the drone, including local video
  saving when enabled and log saving, during disarmed use and manual or
  autonomous flight.
- Required behavior during flight after Wi-Fi/SSH disconnect, or `preserve existing`:
  Safely land during autonomous flight on actual Wi-Fi/control-connection loss,
  monitored with an explicit operator heartbeat and configurable timeout.
  Deliberately closing an SSH session is not a failure: the heartbeat must
  operate independently of terminal sessions, and recording continues locally.
  Track human versus autonomous piloting through explicit control ownership
  and flight mode; receiver connectivity alone does not establish human control.
  Do not initiate landing solely for network loss while a human is piloting
  via the radio controller. Choose and test timing and ownership details in
  simulation before any flight use.
  Existing safeguards and installed Pi/flight-controller settings need to be
  checked against this policy before implementing it.
- Should explicit network switching be refused while recording or flight work is active?
  Refuse all network switching during flight. Allow it when only recording;
  recording must continue despite the resulting connection loss.

### Q15. What should the command interface look like?

Choose: keep separate commands such as `drone-connect` and `drone-inspect` /
one command with subcommands such as `drone connect` and `drone record` /
mostly ordinary SSH and OS tools with only necessary project commands.

**Recommendation:** choose one consistent interface; remove obsolete aliases
unless an actual user or script needs a transition period.

**Answer:**

Prefer small Python scripts for individual jobs that compose together and are
easy to extend. Provide independent per-run options for video saving and servo
activation, plus easy selection of one AprilTag ID, multiple IDs, or a range.
Use one consistent interface, with maintenance/helper entry points under
`scripts/`. Accept a unified main command with subcommands, invoked through
`uv run`, while keeping helper entry points under `scripts/`. Exact command
spelling is an implementation detail; obsolete aliases need not be retained.

### Q16. What must remain compatible?

The tests currently preserve some internal imports from earlier refactors.
Knowing which interfaces are actually used lets us remove unnecessary compatibility code.

**Recommendation:** reorganize private Python internals freely, document command
changes, and preserve access to existing recordings.

| Interface | Must remain compatible / may change / unused |
| --- | --- |
| Command names, flags, exit codes, and scripted output | May change; remove code kept solely for backward compatibility. |
| Environment variables and SSH aliases | May change; no historical compatibility requirement. |
| Python imports used by scripts outside this repo; list them if any | No backward-compatibility requirement; compatibility-only imports/shims may be removed. |
| Recording files, manifest fields, and output-directory layout | May change for new recordings. Preserve old recordings, but they need not remain on the Pi. |
| Reading old datasets with the report tools | No obligation to retain compatibility-only readers in the new code; preserve the datasets themselves. |

Preserve suitable reports, snapshots, and diagnostic/recording material in
GitHub through explicit publication rather than automatic commits/pushes.
Store videos and large datasets on a laptop or another selected machine using
the transfer helper. Verify the destination copy before explicit deletion of
Pi originals; retaining old recordings does not require keeping them on the Pi.

### Q17. How should deployment and configuration capture behave?

Current tools include optional post-deployment tasks, deployment before some
configuration captures, and optional Git publication of captures.

**Recommendation:** make deployment, running a task, capturing configuration,
and publishing to Git explicit operations. Use one deployment method where the
chosen laptop platforms allow it.

**Answer:**

- Desired normal workflow, in a few steps:
  Keep deployment, running a task, capturing configuration, and publishing to
  Git explicit operations. Before restructuring, save the current Pi/drone
  configuration so it can be restored if needed.
  After recording, explicitly transfer the session's videos/logs/reports to
  a chosen machine, verify the copy, and remove Pi originals when requested.
- Combined operations you actually want to keep:
  Prefer the recommended explicit operations; no automatic combinations are
  required.
- Does anyone need automatic Git commits or pushes from a project command?
  No. Publish suitable recording/diagnostic material to GitHub explicitly.
- Must deployment work without internet on the Pi?
  Yes, wherever feasible; support deployment without Pi internet access when
  the required source and dependencies can be supplied locally.

## D. Code style and configuration

### Q18. Does this capture your preferred functional style?

Proposed default:

- Keep Python; use small typed functions with explicit inputs and return values.
- Prefer immutable records for configuration, observations, and decisions.
- Keep necessary mutation local to device handling and execution loops.
- Use small classes or context managers where they make resource ownership clear.
- Use ordinary expressions, comprehensions, and loops; avoid inheritance-heavy
  designs, generic frameworks, and clever one-liners.
- Share code when its meaning agrees; avoid abstractions that only hide a few lines.
- Keep comments that explain hardware constraints and reasoning; remove narration
  of obvious code.
- Preserve meaningful error handling, cleanup, and safeguards while simplifying them.

**Answer:** accept / changes:

Accept the proposed style for now. Prefer concise, functional Python and small
composable jobs, with less excess documentation, comments, and docstrings.
A fuller functional Python style specification will be provided later; it is
not a prerequisite for the initial cleanup.

Optional example of code whose style you like:

### Q19. Where should project configuration live?

The connection code currently embeds a Pi username, Tailscale hostname, and
addresses, with environment-variable overrides.

**Recommendation:** one documented local configuration for project settings,
explicit command overrides, normal SSH configuration for login details, and
NetworkManager for Wi-Fi credentials. Avoid a new configuration framework.

**Answer:**

- One shared drone / multiple named drones:
  One shared drone only; no multiple-drone configuration support needed.
- Preferred configuration: local file / environment variables / command flags /
  SSH configuration where applicable / you decide:
  Per-run options must control video saving, servo activation, and selected tag
  IDs. Accept one documented local configuration file for project settings,
  explicit command overrides, normal SSH configuration for login details, and
  NetworkManager for Wi-Fi credentials. Avoid a new configuration framework.
  Keep transfer destinations and configurable operating thresholds here.
- Dependencies or development tools you specifically want removed or retained:
  Keep Python. Use `uv` for everything it can manage, including dependencies,
  environments, and execution. Add Python project dependencies with `uv add`,
  never `uv pip install` or `pip`. Run programs, tests, and tools through
  `uv run`, never by directly invoking `.venv` executables or activating an
  environment for execution. `ty` and `ruff` are mandatory tooling.

## E. Repository contents and documentation

### Q20. What historical and non-runtime material belongs in the checkout?

Use `keep`, `remove from checkout; retain in Git history`, `move elsewhere`, or
`unsure`. This does not propose rewriting Git history.

| Material | Decision |
| --- | --- |
| Retired person-detection/following code in `attic/` | Remove from the local checkout if already preserved in Git/GitHub or if it contains no useful unique data. |
| Dated observations in `state/` and old `notes/` | Remove low-value or already archived material; preserve useful unique data, especially logs and snapshots. |
| Flight-controller parameter snapshots and firmware manifests | Preserve; local copies may be removed once retained in Git history/GitHub. Save a restorable current configuration before restructuring. |
| Historical test logs and archived incident evidence | Preserve; they need not all remain in the local checkout if retained in Git history/GitHub. |
| Project website and poster | keep — retain the required online documentation and poster, consolidating documentation according to Q21. Archived duplicates may be removed locally. |
| CAD files and 3D-print assets | General archival rule applies: local copies may be removed if retained in Git history/GitHub; no separate asset-specific removal request. |
| Startup-tone media and recovery references | Retain the startup-tone media and keep the tone working. The old `TONE_HANDOVER.md` is not required; preserve useful recovery/configuration evidence. |

Accept the recommendation to remove unused executable code and consolidate
current instructions while preserving useful provenance. Material already in
Git history or retained on GitHub may be removed locally, subject to the
startup-tone requirement. The refactor primarily concerns code and Markdown;
test logs, parameter snapshots, and useful diagnostic evidence must persist.

**Recommendation:** remove unused executable code from the active tree; retain
useful hardware/configuration provenance. Consolidate current instructions while
keeping dated evidence distinguishable from current procedures.

### Q21. Which documents should people actually use?

**Recommendation:** a short README, one networking/setup guide, one operator
guide, and concise development/architecture notes, with specialist procedures
linked where needed. Generate the site from those sources if the site remains.

**Answer:**

- Main audience: you / project teammates / future students / public readers:
  The project owner and teammates now; future students and the professor for
  the maintained documentation and explanations.
- Documentation language: English / German / both:
  English as the implementation default; no requirement to maintain duplicate
  documentation in both languages. Existing presentation material can retain
  its language.
- Documents or site pages that must remain available:
  Accept the recommended short README, networking/setup guide, operator guide,
  and concise development/architecture notes, with specialist procedures linked
  as needed and the site generated from those sources where retained. Keep
  accessible online documentation and the project poster; no additional
  individual pages have been named as mandatory.

### Q22. Are the current local deletions intentional?

At inspection, Git already showed deletions of root `AGENTS.md`, `CLAUDE.md`,
`TONE_HANDOVER.md`, startup-tone media/checksum material, and `round5/` files.
There was also an untracked `state/2026-09-15/` directory. These changes were
present before this questionnaire was created.

**Answer:** deletions intentional / some accidental, specify / handle separately:

Intentional. `CLAUDE.md` is no longer needed. The former `AGENTS.md` was too
large and has been replaced by a smaller version focused on the Astra model.
The deletions of `TONE_HANDOVER.md` and `round5/` are also intentional. Retaining
the startup tone remains a requirement even though the handover document is
removed.

Also name any files or directories that cleanup must not change:

Preserve the startup-tone media/functionality, test logs, parameter snapshots,
and useful diagnostic/configuration evidence, whether in the checkout or the
agreed archive as appropriate. No further protected paths have been named.

## F. Scope and acceptance

### Q23. How far should the eventual cleanup go?

**Recommendation:** implement in reviewable stages: agreed feature removals,
network policy, runtime refactoring, then documentation and evidence checks.
Prepare any installed Pi network migration as a separate step: copying source
alone does not update installed services or saved hotspot autoconnect settings.

**Answer:**

- Priority order: fewer features / clearer code / fewer files / simpler commands /
  fewer dependencies / faster startup or runtime:
  First reduce code size and excess Markdown/documentation while making a
  later substantial restructuring easier. Preserve the selected capabilities,
  diagnostics, and useful data; aim for editable, composable code with low
  compute and network overhead.
- Small incremental changes / a substantial internal restructure is welcome:
  Small incremental changes now, leading to a substantial refactor eventually.
- Desired deliverable: repository changes only / also prepare Pi migration /
  later supervised deployment and verification:
  The current task remains questionnaire updates only, with no code edits.
  The later cleanup includes simulation testing of all code and testing on
  the Pi, initially only in disarmed mode. Save the current configuration
  before changes so it can be restored if restructuring breaks the setup.
  Later flight testing is outside the initial disarmed verification stage.
- Deadline or effort limit, if any:
  No deadline specified.

### Q24. What would make you say the cleanup is finished?

Give two or three concrete scenarios. For example: power on near a saved
network, connect over Wi-Fi, start a recording, disconnect the laptop, and
retrieve a complete report afterward. Also include explicit hotspot activation
and recovery without automatic hotspot creation.

**Recommendation:** keep meaningful offline behavior and failure tests, update
tests that pin removed internals, and run the relevant existing checks. Retained
flight behavior needs its simulator checks; live verification is a separate,
supervised step. Do not trade away cleanup or failure handling for shorter code.

**Answer:**

- Acceptance scenario 1:
  Disarmed inspection records sensor data and AprilTag detections/poses, with
  video saving and servo activation independently enabled or disabled. When
  servo activation is enabled, select one tag ID, multiple IDs, or an ID range.
  Report a missing camera or sensor and continue recording available sources.
- Acceptance scenario 2:
  During manual radio-controlled or autonomous flight, record sensor data,
  detections/poses, and optional video locally, with optional tag-triggered
  servo activation. Recording continues after SSH disconnect; record actual
  motor/propeller RPM if the hardware and firmware make it available.
  Deliberately close an SSH terminal: recording and the independent operator
  heartbeat continue, with no landing or network switch triggered by terminal
  closure. Simulate actual Wi-Fi/operator-heartbeat loss during autonomous
  flight and verify safe landing; during explicit human radio control, network
  loss alone must not initiate landing. Never switch networks during flight.
  Validate these flight cases in simulation first; initial Pi verification
  remains disarmed only.
- Acceptance scenario 3:
  Save video and logs offline on the drone without requiring a video stream.
  Storage checks warn when offloading is needed, report estimated remaining
  recording time, and stop video cleanly before using the log reserve. Transfer
  videos/logs/reports to a laptop or another machine; verify the copy before
  explicit deletion of originals. Check offline local transfer and retention
  of source data after a failed transfer.
- Performance limits that matter: startup time / Pi memory / camera frame rate /
  detection latency / recording duration / no specific targets:
  Avoid excessive compute and connection overhead. Tags lie flat on the ground
  and the usual flying height is 1–3 metres. Choose numeric resource, frame-rate,
  latency, storage, and timeout defaults through measurement and simulation;
  these implementation choices are delegated rather than awaiting user input.
- Other requirements or things you definitely do not want:
  Custom firmware and simulation testing of all code are essential. Capture
  the current configuration for restoration before restructuring, and initially
  test on the Pi only while disarmed. Preserve diagnostics, reports, test logs,
  snapshots, old recordings, and the startup tone.
  Network behavior must prefer `eduroam`, allow saved-network fallback while
  grounded under Q6's loss conditions, provide Tailscale and direct local
  access, and retain USB recovery. Hotspot activation requires a direct helper
  command; stop/reboot returns to client mode. No automatic hotspot creation,
  concurrent client/hotspot requirement, or project-driven laptop Wi-Fi changes.
  Accept the recommended logging formats, native AprilTag backend, explicit
  operator heartbeat/control ownership, storage reserve with clean video stop,
  and verified offloading to another machine. Deliberate SSH closure is not a
  failure. Apply the recommended defaults to remaining implementation choices
  while preserving the user's explicit decisions elsewhere in this document.
  The later functional Python style specification remains intentionally deferred.
