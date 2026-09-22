"""Offline v8 commissioning estimates. Never sends robot commands."""
import argparse
import json
import math
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_bridge.hardware.piper6 import PiperArmController


def rotation_delta(a, b):
    return PiperArmController.rotation_distance_deg(a[3:],b[3:])


def analyze(records):
    samples = sorted((r for r in records if r.get('event') == 'sample'), key=lambda r:r['time'])
    stops = {}
    for sample in samples:
        hold = sample.get('arm', {}).get('last_hold')
        if hold:
            stops[('release', hold['time'])] = (hold['pose'], 'fresh-pose hold target')
    for record in records:
        if record.get('event') == 'fault':
            before = [s for s in samples if s['time'] <= record['time'] and s.get('arm', {}).get('pose')]
            pose = before[-1]['arm']['pose'] if before and record['time']-before[-1]['time'] <= .25 else None
            stops[('fault', record['time'])] = (pose, record.get('reason',''))
    results = []
    for (kind, when), (reference, reason) in sorted(stops.items(), key=lambda item:item[0][1]):
        # Do not include the next intentional jog in this stop measurement.
        next_motion = min((r['time'] for r in records if r.get('event')=='control'
                           and r['time'] > when and r.get('mode') == 2 and any(r.get('input',[]))), default=math.inf)
        after = [s for s in samples if when <= s['time'] <= min(when+1, next_motion)
                 and s.get('arm',{}).get('pose') is not None]
        complete = reference is not None and len(after)>=3 and after[-1]['time']-when>=.9
        if after and after[0]['time']-when > .15: complete=False
        if any(b['time']-a['time'] > .15 for a,b in zip(after,after[1:])): complete=False
        distance = max((math.dist(s['arm']['pose'][:3],reference[:3]) for s in after), default=None) if reference else None
        rotation = max((rotation_delta(s['arm']['pose'],reference) for s in after), default=None) if reference else None
        settled = None
        # Report first confirmation of three stable samples, not a physical stop guarantee.
        for i in range(2,len(after)):
            window=after[i-2:i+1]
            if all(math.dist(s['arm']['pose'][:3],window[0]['arm']['pose'][:3])<=.05
                   and rotation_delta(s['arm']['pose'],window[0]['arm']['pose'])<=.05 for s in window[1:]):
                settled=round((window[-1]['time']-when)*1000,1); break
        verdict = 'INSUFFICIENT_DATA'
        if complete:
            verdict = 'WITHIN_TRIAL_LIMITS' if distance<=2 and rotation<=1 and settled is not None else 'EXCEEDS_TRIAL_LIMITS'
        results.append(dict(kind=kind,stop_time=when,reason=reason,samples=len(after),
                            max_translation_mm=distance,max_rotation_deg=rotation,
                            stable_samples_confirmed_ms=settled,result=verdict))
    return {'measurement':'SDK feedback estimates, approximately 20 Hz; physical verification still required',
            'limits':{'translation_mm':2,'rotation_deg':1},'stops':results}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', nargs='+', type=Path)
    parser.add_argument('--output', type=Path)
    args=parser.parse_args()
    records=[]
    for path in args.logs:
        for line in path.read_text(encoding='utf-8').splitlines():
            if line.strip(): records.append(json.loads(line))
    data=json.dumps(analyze(records),ensure_ascii=False,indent=2)
    if args.output: args.output.write_text(data,encoding='utf-8')
    else: print(data)


if __name__=='__main__': main()
