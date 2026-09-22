# AprilTag binding patch

`apriltag-gil.patch` applies to Debian AprilTag **3.4.2**'s unmodified
`apriltag_pywrap.c` (SHA256
`7010a715a19b620879916eef0c098a91a079c3f2f87dffc372648a40d29ad55e`).
The patched file has SHA256
`d79a690e6eb871e159adeba8ebac4582262755163baae5c6b1e66530761f12b6`.
Apply with `patch -p1` from the binding source directory and package the result
as `ai-drone-debian-apriltag==3.4.2+gil1`, keeping the original wheel available.

The original wrapper holds the Python GIL throughout native detection. A busy
camera scene can therefore starve Python telemetry threads. The patch releases
the GIL only during `apriltag_detector_detect`, retains the input and detector
references, saves native errno, and cleans results on error. Overlapping calls
on one detector raise `RuntimeError`; separate detector instances remain usable.
Callers must not mutate an input array concurrently with detection.

The wrapper also previously changed process-wide SIGINT handling to `SIG_DFL`.
The patch preserves Python's handler and checks pending signals after native
completion. All Python/NumPy API calls remain under the GIL. Standard CPython
with its GIL is required; free-threaded compatibility is not claimed.

CPU-only host checks passed with CPython **3.13.15** and **3.14.7** and NumPy
2.5.3: native ID17/dictionary API, project adapter, empty input result, malformed
image recovery, overlapping-call rejection, Python thread progress, preserved
SIGINT handling and cleanup after `KeyboardInterrupt`. No permanent test files
were added. An unchanged host binding stalled another Python thread for 1.454 s;
the patched runs progressed throughout equivalent native work, with maximum
observed gaps below 5.5 ms. These are host observations, not Pi performance limits.

The ARM64 CPython3.14 wheel uses the candidate's actual Python/NumPy headers and
installed `libapriltag.so.3`; it neither replaces that library nor modifies the
working Python3.13 environment. Host compatibility does not establish ARM64
Python3.13 loading. Pi import, passive recording/transport and resource checks
must qualify the new wheel independently before an explicit native deployment.

Build scripts, exact inputs, compiler commands, wheel hashes and temporary-check
evidence are retained under the ignored
`artifacts/refactor-20260922/native-build/gil1/` kit. All Python builds use uv;
the single ARM64 C unit is cross-compiled in the existing bounded host container.
Keep the original source/wheel provenance and include this patch in the native
payload's hashed evidence. No flight or actuator qualification follows from it.
