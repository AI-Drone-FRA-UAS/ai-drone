# Historical branches preserved during consolidation

The 2026-09-07 consolidation keeps one maintained runtime under `ai_drone/`.
The earlier source revisions below remain available through annotated Git tags
under `archive/consolidation-2026-09-07/` and their immutable commit IDs.
Original authorship and all parent history are retained. The pre-consolidation
bundle also retains the local stash and its untracked-file parent.

| Original branch | Source SHA | Disposition |
| --- | --- | --- |
| `current25` | `6b9bacd46fc5c2c33f11b0a776d6a9a5e936c5db` | Maintained runtime base, integrated normally with reviewed repairs |
| `main` | `24d3bff0ad19c074be443ac5249e1b31c73d6d80` | Site, poster and unique documentation integrated through a normal merge |
| `experimental` | `d96f15384ef0645b8596df069be4fa5042c75258` | Already an ancestor of current25; draft PR #7 is superseded by the complete integration |
| `experimental-berserkan` | `f41c912c15cad952ac0b7361094b3deff9a21966` | Game-controller and mission concepts preserved in Git; not imported into runtime |
| `experimental-sebispm` | `5b490e2c2a0270abe297c261d3ac0d55c740376b` | Legacy mission/hold/motor scripts and parameter snapshots preserved in Git |
| `preflight-and-nogps-takeoff` | `f1008308165b38fb770566b555020566d25ba183` | Retired flight experiments preserved in Git; selected original incident evidence copied below |
| `exp-main-sync` | `ff4ba59b6d889cbd508529b3550bf929364e762e` | Duplicate merge with the same parents and tree as `5b4695b`, already in current25 |

## Why the old flight implementations stay inactive

The game actor sends RC overrides and may force-disarm when flight guards trip.
The mission documentation assumes an operational forward sensor, describes an
obsolete GPIO18 mapping and recommends disabling arming checks. The preflight
branch implements a different STABILIZE/ALT_HOLD control and recovery strategy.
None is interchangeable with the tested GuidedNoGPS/Loiter runtime. Their
algorithms, tests and measurements remain available for deliberate future work,
without silently changing the aircraft's behavior during repository cleanup.

- [Archived incident reports and logs](preflight-and-nogps-takeoff/README.md)
- [Berserkan source](https://github.com/AI-Drone-FRA-UAS/ai-drone/tree/f41c912c15cad952ac0b7361094b3deff9a21966)
- [Sebispm source](https://github.com/AI-Drone-FRA-UAS/ai-drone/tree/5b490e2c2a0270abe297c261d3ac0d55c740376b)
- [Preflight source](https://github.com/AI-Drone-FRA-UAS/ai-drone/tree/f1008308165b38fb770566b555020566d25ba183)

No archived parameter set, mission, calibration or executable is a deployment
input. Dated project captures under `state/` and `params/` describe provenance;
configuration changes still need individual review against the actual aircraft.
