# Further reduction of crowd-like ambience

The user liked the five polished examples and identified the remaining sound
as possibly the audience. This pass specifically targets broadband and diffuse
background energy. That acoustic profile is compatible with crowd ambience,
but the sound has not been identified conclusively by listening.

Open `listen.html` for matched-volume comparisons. Versions 31–35 correspond
to previous 21–25, which are included byte-for-byte. Start with 32 or 34.

Processing uses the same eight separated passages from the prior analysis.
A broadband floor is estimated from quieter melody frames; a frequency median
keeps narrow musical peaks out of the noise estimate. A soft power-subtraction
mask attenuates the floor, while recurring pitched components are protected.
Unprotected stereo side energy receives additional attenuation to target diffuse
ambience without collapsing the entire synth to mono. No gate mutes the gaps,
and no transposition, extra notes, or new backing sounds are introduced.

`metrics.json` records levels, pitch-preservation checks, attenuation bounds,
and energy changes in estimated broad-background regions. Those regions are a
proxy: the figures are not a measured percentage of audience removal.

All new files are 44.1 kHz stereo 16-bit PCM WAV, exactly 151,200 samples long.
RMS matches the previous versions and no output clips. Strong-frame dominant
pitch checks found no switches over 100 cents. No listening verification was
performed; further removal can also reduce desirable synth texture or room sound.
The prior versions are retained for that comparison. No drone settings changed.

`reduce_ambience.py` requires NumPy, SciPy, the prior files, and the temporary
Demucs accompaniment stem referenced in its source.
