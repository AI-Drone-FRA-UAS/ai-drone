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
- Your three most common workflows: connect to the drone via ssh then run the drone disarmed and check sensor readings from all sensors and maybe test the servo motor. then: testing the loiter mode that currently doesnt work because it always reports a too high magnetic disturbance
- Biggest current annoyance: the code is just way to big and as way too many markdown files in it, way too much documentation, comments, docstrings etc. the code just isnt concise enough

### Q2. Which existing capabilities should survive?

Use `keep`, `simplify`, `remove`, or `unsure`. For `simplify`, say what part you use.

| Capability | Decision / essential part |
| --- | --- |
| Disarmed camera and sensor recording (`drone-inspect`) | |
| Timed recordings that survive SSH disconnect (`drone-walk`) | | 
| Recording during a pilot-controlled flight | |
| AprilTag detection and pose estimation | |
| Live camera preview in a browser | |
| CSV exports and offline HTML/video reports | |
| Sensor/firmware checks and configuration snapshots | |
| Automatic takeoff, Loiter hold, and landing (`drone-control hover`) | |
| Tag-triggered payload-servo operation during recording | |
| Direct servo and motor bench-test commands | |
| Connection/power status and controlled shutdown helpers | |
| Pi OS upgrade and power-resilience setup scripts | |
| Custom firmware verification and simulator support | |

### Q3. What must run together, and what data do you actually use?

Examples: recording + preview; recording + tags + servo; flight control +
recording; status checks during recording. These affect ownership of the camera
and flight-controller connection.

**Answer:**

- Required simultaneous combinations:
- Required outputs: video / MAVLink logs / telemetry JSONL / tag detections and
  poses / CSV / offline HTML / other:
- During ordinary inspection, if a camera or sensor is missing: record available
  sources with a clear incomplete status, or stop? Which sources are mandatory?
- Any required AprilTag backend: native / OpenCV / no preference:

### Q4. Which machines and versions must we support?

Recommend supporting the hardware and operating systems you actually use.
The repository currently declares Python 3.11–3.13 support.

| Target | Required support |
| --- | --- |
| Current Pi Zero 2 W and FlywooF745 only, or other aircraft too? | |
| Developer laptops: Linux, macOS, Windows? | |
| Current Pi OS with NetworkManager only, or older networking setups too? | |
| Python 3.11–3.13, or a narrower range? | |
| Hardware-free development and ArduPilot simulation? | |

### Q5. Which maintenance connections remain?

These are separate connections: laptop → Pi USB Ethernet, laptop → flight
controller USB serial, and the onboard Pi → flight controller UART.
The onboard UART is independent of how you reach the Pi over Wi-Fi.

**Recommendation:** remove Pi USB networking from the active application; retain
onboard UART and direct flight-controller USB maintenance. Keep simulator
network endpoints if simulation stays in scope.

**Answer:**

- Pi USB networking: remove entirely / retain a manual recovery procedure only:
- Direct flight-controller USB maintenance: keep / remove project helpers:
- Any other connection requirement:

## B. Exact Wi-Fi behavior

### Q6. Which saved networks should the Pi use, and how should it reconnect?

The current selector prefers `eduroam`, then other saved client profiles.

**Recommendation:** configurable saved-network priorities, automatic reconnect,
and continued attempts when nothing is reachable, with the hotspot off. Keep a
working connection until it drops rather than interrupting it to chase priority.

**Answer:**

- Networks to support, in priority order; names/types only:
- Join any saved auto-connect profile, or only an explicit allowlist?
- Keep a working connection, or switch when a preferred network appears?
- If none is reachable: keep retrying / stop after a specified time:

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
- What must work without internet?
- Must teammates connect from outside the drone's local network?

### Q8. How should first setup and recovery work when no saved Wi-Fi is reachable?

An unreachable Pi cannot receive a remote command to start its hotspot.
Choose a route you will actually have available.

**Recommendation:** provision a known Wi-Fi network before use and save a phone
hotspot as a recovery network; the Pi joins that network as a client.

**Answer:**

- First setup: preconfigured SD image / local setup / other:
- Recovery: known phone hotspot / local console / physical command button
  (new hardware work) / manual USB recovery from Q5 / other:

### Q9. Where should the explicit hotspot command be issued?

Choose: on the Pi through an existing SSH session / a laptop command that sends
the request to the Pi / both / another direct control.

**Recommendation:** one Pi implementation with a thin laptop wrapper. A network
switch must finish even if it disconnects the initiating SSH session.
Any command names in your answer can be proposed names; no new interface exists yet.

**Answer:**

### Q10. How long should an explicitly started hotspot stay on?

**Recommendation:** until an explicit stop or reboot. Stopping it returns to
saved client networks; reboot returns to client mode and requires a fresh
command before another hotspot starts. Setup should save the hotspot profile
without activating it.

**Answer:** recommended / timeout of ___ / another explicitly commanded lifetime:

### Q11. Do you need hotspot and client Wi-Fi at the same time?

The documented setup uses one onboard radio and switches between those roles.
The repository also contains a setup path for a second Wi-Fi adapter.

**Recommendation:** keep one-radio switching and remove dual-network setup
unless simultaneous operation is a real requirement. Any retained hotspot path
must still obey explicit activation.

**Answer:** one radio / simultaneous operation with a second adapter / other:

### Q12. May a connection command change the laptop's Wi-Fi?

The current helper can join the Pi hotspot and later restore the previous
laptop network.

**Recommendation:** ordinary connection commands use the laptop's current
network. Switching laptop Wi-Fi requires an explicit action.

**Answer:**

- Manual laptop network selection / explicit project switch command /
  automatic selection of an already active Pi hotspot:
- If the tool switches networks, should it restore the previous network after SSH exits?

### Q13. How much Wi-Fi setup should this repo own?

**Recommendation:** NetworkManager stores credentials on the Pi; project setup
configures the desired policy without saving secrets in Git. Keep the documented
hotspot name `AI-Drone-Zero` and Pi address `192.168.4.1` unless you want them changed.

**Answer:**

- Configure client credentials using OS tools / provide a project setup command:
- Keep hotspot name/address / desired non-secret replacements:
- Any specific enterprise-network or team-access requirements:

## C. Commands and runtime behavior

### Q14. What should start at boot and survive a lost laptop connection?

**Recommendation:** only network/access services start at boot. Explicit timed
recordings continue locally until completion. Preserve the existing flight
safeguards and flight-controller failsafes during cleanup; Wi-Fi loss and loss
of the onboard Pi–controller connection are distinct events.

**Answer:**

- Tasks, if any, that must start automatically:
- Recording behavior after Wi-Fi/SSH disconnect:
- Required behavior during flight after Wi-Fi/SSH disconnect, or `preserve existing`:
- Should explicit network switching be refused while recording or flight work is active?

### Q15. What should the command interface look like?

Choose: keep separate commands such as `drone-connect` and `drone-inspect` /
one command with subcommands such as `drone connect` and `drone record` /
mostly ordinary SSH and OS tools with only necessary project commands.

**Recommendation:** choose one consistent interface; remove obsolete aliases
unless an actual user or script needs a transition period.

**Answer:**

### Q16. What must remain compatible?

The tests currently preserve some internal imports from earlier refactors.
Knowing which interfaces are actually used lets us remove unnecessary compatibility code.

**Recommendation:** reorganize private Python internals freely, document command
changes, and preserve access to existing recordings.

| Interface | Must remain compatible / may change / unused |
| --- | --- |
| Command names, flags, exit codes, and scripted output | |
| Environment variables and SSH aliases | |
| Python imports used by scripts outside this repo; list them if any | |
| Recording files, manifest fields, and output-directory layout | |
| Reading old datasets with the report tools | |

### Q17. How should deployment and configuration capture behave?

Current tools include optional post-deployment tasks, deployment before some
configuration captures, and optional Git publication of captures.

**Recommendation:** make deployment, running a task, capturing configuration,
and publishing to Git explicit operations. Use one deployment method where the
chosen laptop platforms allow it.

**Answer:**

- Desired normal workflow, in a few steps:
- Combined operations you actually want to keep:
- Does anyone need automatic Git commits or pushes from a project command?
- Must deployment work without internet on the Pi?

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

Optional example of code whose style you like:

### Q19. Where should project configuration live?

The connection code currently embeds a Pi username, Tailscale hostname, and
addresses, with environment-variable overrides.

**Recommendation:** one documented local configuration for project settings,
explicit command overrides, normal SSH configuration for login details, and
NetworkManager for Wi-Fi credentials. Avoid a new configuration framework.

**Answer:**

- One shared drone / multiple named drones:
- Preferred configuration: local file / environment variables / command flags /
  SSH configuration where applicable / you decide:
- Dependencies or development tools you specifically want removed or retained:

## E. Repository contents and documentation

### Q20. What historical and non-runtime material belongs in the checkout?

Use `keep`, `remove from checkout; retain in Git history`, `move elsewhere`, or
`unsure`. This does not propose rewriting Git history.

| Material | Decision |
| --- | --- |
| Retired person-detection/following code in `attic/` | |
| Dated observations in `state/` and old `notes/` | |
| Flight-controller parameter snapshots and firmware manifests | |
| Historical test logs and archived incident evidence | |
| Project website and poster | |
| CAD files and 3D-print assets | |
| Startup-tone media and recovery references | |

**Recommendation:** remove unused executable code from the active tree; retain
useful hardware/configuration provenance. Consolidate current instructions while
keeping dated evidence distinguishable from current procedures.

### Q21. Which documents should people actually use?

**Recommendation:** a short README, one networking/setup guide, one operator
guide, and concise development/architecture notes, with specialist procedures
linked where needed. Generate the site from those sources if the site remains.

**Answer:**

- Main audience: you / project teammates / future students / public readers:
- Documentation language: English / German / both:
- Documents or site pages that must remain available:

### Q22. Are the current local deletions intentional?

At inspection, Git already showed deletions of root `AGENTS.md`, `CLAUDE.md`,
`TONE_HANDOVER.md`, startup-tone media/checksum material, and `round5/` files.
There was also an untracked `state/2026-09-15/` directory. These changes were
present before this questionnaire was created.

**Answer:** deletions intentional / some accidental, specify / handle separately:

Also name any files or directories that cleanup must not change:

## F. Scope and acceptance

### Q23. How far should the eventual cleanup go?

**Recommendation:** implement in reviewable stages: agreed feature removals,
network policy, runtime refactoring, then documentation and evidence checks.
Prepare any installed Pi network migration as a separate step: copying source
alone does not update installed services or saved hotspot autoconnect settings.

**Answer:**

- Priority order: fewer features / clearer code / fewer files / simpler commands /
  fewer dependencies / faster startup or runtime:
- Small incremental changes / a substantial internal restructure is welcome:
- Desired deliverable: repository changes only / also prepare Pi migration /
  later supervised deployment and verification:
- Deadline or effort limit, if any:

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
- Acceptance scenario 2:
- Acceptance scenario 3:
- Performance limits that matter: startup time / Pi memory / camera frame rate /
  detection latency / recording duration / no specific targets:
- Other requirements or things you definitely do not want:
