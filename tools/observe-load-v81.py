#!/usr/bin/env python3
"""Read-only N100 observation; never enables, sends control or starts motion."""
import argparse,json,time,subprocess,urllib.request
from pathlib import Path
def main():
    p=argparse.ArgumentParser();p.add_argument('--seconds',type=int,default=600);p.add_argument('--can-name',default='can0');a=p.parse_args()
    if not 10<=a.seconds<=3600 or not a.can_name.replace('_','').isalnum():p.error('seconds 10..3600; CAN interface name required')
    folder=Path(__file__).resolve().parents[1]/'diagnostics'/time.strftime('load-v81-%Y%m%d-%H%M%S');folder.mkdir(parents=True)
    started=time.monotonic();deadline=started+a.seconds;next_can=0;count=0;errors=0
    with (folder/'samples.jsonl').open('w') as log:
        while time.monotonic()<deadline:
            before=time.monotonic();r={'time':before}
            try:
                with urllib.request.urlopen('http://127.0.0.1:9091',timeout=.3) as response:r['executor']=json.load(response)
                count+=1
            except Exception as exc:r['error']=str(exc);errors+=1
            if before>=next_can:
                can=subprocess.run(['ip','-j','-details','-statistics','link','show',a.can_name],capture_output=True,text=True,timeout=1)
                r['can']=can.stdout or can.stderr;next_can=before+1
            r['read_latency_ms']=(time.monotonic()-before)*1000;log.write(json.dumps(r)+'\n');log.flush()
            time.sleep(max(0,.1-(time.monotonic()-before)))
    result={'seconds':time.monotonic()-started,'samples':count,'read_errors':errors,'hardware_acceptance':'Review samples, video/control load and physical stop measurements; observation alone is not PASS'}
    (folder/'summary.json').write_text(json.dumps(result,indent=2));print(folder)
if __name__=='__main__':main()
