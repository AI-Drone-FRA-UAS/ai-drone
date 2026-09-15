"""Reduce estimated diffuse ambience in the five accepted polished loops."""
from pathlib import Path
import json
import zipfile
import numpy as np
from scipy import signal, ndimage
from scipy.io import wavfile

ROOT=Path(__file__).parent
PREV=ROOT.parent/'round4'
SR=44100
LENGTH=151200
N=4096
HOP=441
F=np.fft.rfftfreq(N,1/SR)

def read(p):
    sr,y=wavfile.read(p)
    assert sr==SR
    return y.astype(float)/(32768 if np.issubdtype(y.dtype,np.integer) else 1)

def stft(y):
    return np.stack([signal.stft(y[:,c],fs=SR,nperseg=N,noverlap=N-HOP)[2] for c in range(2)],axis=2)

def inverse(z):
    return np.column_stack([signal.istft(z[:,:,c],fs=SR,nperseg=N,noverlap=N-HOP)[1] for c in range(2)])[:LENGTH]

settings=json.loads((ROOT.parent/'round3/analysis/measurements.json').read_text())
y=read('/tmp/paris-demucs/htdemucs_ft/drone-paris-analysis-long/other.wav')
start=round(settings['loop_start_within_excerpt']*SR)
indices=settings['selected_repetitions_zero_based']
Z=np.stack([stft(y[start+k*LENGTH:start+(k+1)*LENGTH]) for k in indices])
reference=Z[indices.index(settings['waveform_repetition_zero_based'])]
mag=abs(reference).mean(axis=2)
repeat=np.quantile(abs(Z),.35,axis=0).mean(axis=2)

# Build a broadband floor from quieter melody frames, across repetitions.
# Narrow pitched lines are excluded from the floor by frequency medians.
melody_energy=np.sum(repeat[(F>900)&(F<2300)]**2,axis=0)
valid=np.arange(len(melody_energy))
quiet=(melody_energy<np.quantile(melody_energy,.23))&(valid>4)&(valid<len(valid)-5)
noise=np.median(abs(Z)[:,:,quiet,:],axis=(0,2,3))
noise=ndimage.median_filter(noise,19)
noise=ndimage.gaussian_filter1d(noise,2)

# Protect lines that recur at the same point in the melody, including lower
# synth components. A one-off tonal crowd sound is less likely to be protected.
repeat_floor=ndimage.median_filter(repeat,(19,1))
tonal=np.clip(1-repeat_floor*.9/np.maximum(repeat,1e-12),0,1)
protect=np.clip((tonal-.18)/.68,0,1)
protect=ndimage.maximum_filter(protect,(3,3))
protect=ndimage.gaussian_filter(protect,(.4,.4))

floor_ref=ndimage.median_filter(mag,(19,1))
region=(F>400)&(F<12000)
local=np.median(floor_ref[region]/np.maximum(noise[region,None],1e-10),axis=0)
local=np.clip(ndimage.gaussian_filter1d(local,3),.7,2.5)
noise_tf=noise[:,None]*local[None,:]
base=np.sqrt(np.maximum(1-1.8*(noise_tf/np.maximum(mag,1e-12))**2,.055))
gain=protect+(1-protect)*base
gain=ndimage.gaussian_filter(gain,(.65,.8))
gain=np.maximum(gain,protect*.985)
side_gain=protect+(1-protect)*gain**1.35
noise_region=(protect<.25)&(F[:,None]>200)&(F[:,None]<12000)

def pitch(y):
    f,t,z=signal.stft(y.mean(axis=1),fs=SR,nperseg=8192,noverlap=7751)
    a=abs(z);a=np.maximum(a-ndimage.median_filter(a,(31,1)),0)
    ids=np.where((f>950)&(f<2350))[0]
    k=ids[np.argmax(a[ids],axis=0)]
    return f[k],a[k,np.arange(a.shape[1])]

variants=[
 ('21_natural_polished','31_natural_less_ambience','Natural'),
 ('22_clean_polished','32_clean_less_ambience','Clean / balanced'),
 ('23_extra_clean_polished','33_extra_clean_less_ambience','Extra clean'),
 ('24_fuller_polished','34_fuller_less_ambience','Fuller'),
 ('25_space_polished','35_space_less_ambience','Stereo space'),
]
metrics={}
for old,new,label in variants:
    x=read(PREV/(old+'.wav'));X=stft(x)
    mid=(X[:,:,0]+X[:,:,1])*.5
    side=(X[:,:,0]-X[:,:,1])*.5
    new_mid=mid*gain;new_side=side*side_gain
    out=inverse(np.stack([new_mid+new_side,new_mid-new_side],axis=2))
    out-=out.mean(axis=0)
    fade=round(.001*SR)
    out[:fade]*=np.linspace(0,1,fade)[:,None]
    out[-fade:]*=np.linspace(1,0,fade)[:,None]
    norm=min(np.sqrt(np.mean(x*x))/np.sqrt(np.mean(out*out)),.85/np.max(abs(out)))
    out*=norm
    p,a=pitch(x);q,_=pitch(out);active=a>a.max()*.12
    changes=int(np.sum(np.abs(1200*np.log2(q[active]/p[active]))>100))
    assert changes==0,(new,'changed dominant note')
    assert len(out)==LENGTH and np.max(abs(out))<.86 and np.isfinite(out).all()
    old_spec=np.mean(abs(X)**2,axis=2)
    new_spec=np.mean(abs(stft(out))**2,axis=2)
    attenuation=float(10*np.log10(np.sum(new_spec[noise_region])/np.sum(old_spec[noise_region])))
    metrics[new]={
      'source':old,'seconds':len(out)/SR,
      'rms_dbfs':float(20*np.log10(np.sqrt(np.mean(out*out)))),
      'peak_dbfs':float(20*np.log10(np.max(abs(out)))),
      'dominant_note_switches_over_100_cents':changes,
      'checked_strong_pitch_frames':int(active.sum()),
      'estimated_broad_background_region_energy_change_db':attenuation,
      'note':'Background-region change is a spectral proxy, not a measurement of isolated audience sound.',
    }
    wavfile.write(ROOT/(new+'.wav'),SR,np.rint(out*32767).astype(np.int16))
    (ROOT/(old+'.wav')).write_bytes((PREV/(old+'.wav')).read_bytes())

metrics['method']={
 'quiet_frame_count':int(quiet.sum()),
 'maximum_mid_attenuation_db':float(20*np.log10(gain.min())),
 'maximum_side_attenuation_db':float(20*np.log10(side_gain.min())),
 'audience_identity_verified':False,'listening_verified':False,
 'description':'Broadband floor estimated from quieter melody frames across eight repetitions, recurring-tone protection, and stronger attenuation of unprotected stereo side energy.'}
(ROOT/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
cards='\n'.join(f'''<article><h2>{label}</h2><div class="pair"><section><h3>Previous {old[:2]}</h3><audio controls preload="none" src="{old}.wav"></audio></section><section><h3>Less ambience {new[:2]}</h3><audio controls preload="none" src="{new}.wav"></audio><a href="{new}.wav" download>Download WAV</a></section></div></article>''' for old,new,label in variants)
page='''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Paris synth — less background ambience</title><style>
:root{color-scheme:dark;font:16px/1.5 system-ui,sans-serif;background:#10151e;color:#e7eef9}body{max-width:1000px;margin:auto;padding:38px 22px 70px}h1{font-size:clamp(28px,5vw,42px);letter-spacing:-1px}h2{font-size:21px;margin-top:0}h3{font-size:16px}p{color:#b6c5d9}a{color:#a7d0ff}.tools{background:#233249;padding:18px;border-radius:10px;margin:24px 0;display:flex;gap:25px;align-items:center;flex-wrap:wrap}article{background:#192230;border:1px solid #34445c;border-radius:13px;padding:23px;margin:17px 0}.pair{display:grid;grid-template-columns:1fr 1fr;gap:24px}audio{width:100%;display:block;margin:10px 0}button{background:#42699b;color:white;border:0;border-radius:6px;padding:9px 15px;cursor:pointer}footer{font-size:14px;margin-top:28px;color:#b6c5d9}@media(max-width:650px){.pair{grid-template-columns:1fr}}
</style><h1>Less background ambience</h1><p>A stronger pass targeting the broad, crowd-like sound around the synth. Compare <strong>32</strong> with 22, or <strong>34</strong> with 24. The previous files are included unchanged and volumes are matched.</p><div class="tools"><label><input id="loop" type="checkbox" checked> Loop examples</label><button id="stop">Stop playback</button><a href="NOTES.md">Processing notes</a></div>'''+cards+'''<footer>Recurring pitched components and the high ending tone are protected. No notes, transposition, or backing sounds were added. The residual sound has not been conclusively identified as audience noise, and no listening verification was performed.</footer><script>const players=[...document.querySelectorAll('audio')],loop=document.getElementById('loop');const sync=()=>players.forEach(p=>p.loop=loop.checked);sync();loop.addEventListener('change',sync);players.forEach(p=>p.addEventListener('play',()=>players.filter(q=>q!==p).forEach(q=>q.pause())));document.getElementById('stop').addEventListener('click',()=>players.forEach(p=>{p.pause();p.currentTime=0}));</script></html>'''
(ROOT/'listen.html').write_text(page)
notes='''# Further reduction of crowd-like ambience

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
'''
(ROOT/'NOTES.md').write_text(notes)
with zipfile.ZipFile(ROOT.parent/'paris-less-background.zip','w',zipfile.ZIP_DEFLATED) as z:
    for old,new,_ in variants:
        for name in [old,new]:z.write(ROOT/(name+'.wav'),arcname=name+'.wav')
    for name in ['listen.html','NOTES.md','metrics.json','reduce_ambience.py']:z.write(ROOT/name,arcname=name)
print(json.dumps(metrics,indent=2))
