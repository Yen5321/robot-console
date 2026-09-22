#!/usr/bin/env python3
"""Operator-driven local CPV commissioning. No loop that repeats motion tests.

The executor, not this CLI, bounds pulse amplitude/duration and records results.
No --yes flag: each physical test requires a new attended keyboard action.
"""
import argparse,json,time,urllib.request,urllib.error,sys

def request(path='',body=None):
    data=None if body is None else json.dumps(body).encode()
    req=urllib.request.Request('http://127.0.0.1:9091'+path,data=data,headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=1) as r:result=json.load(r)
    if result.get('ok') is False:raise RuntimeError(result['error'])
    return result

def main():
    p=argparse.ArgumentParser(description='PIPER-L v8.1 本机逐步验证；启动不使能、不恢复')
    p.add_argument('action',choices=['status','sample-fk','enable','hold-test','pulse','stop','estop','recover','events'])
    p.add_argument('--joint',type=int,choices=range(1,7));p.add_argument('--sign',type=int,choices=[-1,1]);a=p.parse_args()
    if a.action in ('status','events'):
        print(json.dumps(request('/events' if a.action=='events' else ''),ensure_ascii=False,indent=2));return
    body={'action':a.action.replace('-','_')}
    if a.action=='pulse':
        if a.joint is None or a.sign is None:p.error('pulse requires --joint 1..6 --sign -1|1')
        body.update(joint=a.joint,sign=a.sign)
    if a.action in ('enable','hold-test','pulse'):
        print('实机测试：固定地面台架、夹爪空载、周围清空、准备现场停止；电子急停可能下移。')
        print('本次动作：',body,'；pulse 上限 0.01 rad/s，含减速最长 0.5 秒。')
        if input('确认只执行本次动作，输入 RUN：').strip()!='RUN':return
    print(json.dumps(request('/action',body),ensure_ascii=False))
    if a.action in ('hold-test','pulse'):
        deadline=time.monotonic()+(7 if a.action=='hold-test' else 2)
        while time.monotonic()<deadline:
            d=request()
            if d.get('fault'):raise RuntimeError(d['fault'])
            result=d.get('last_result')
            if result:print(json.dumps(result,ensure_ascii=False,indent=2));return
            time.sleep(.1)
        raise RuntimeError('未得到测试结果，请检查 diagnostics/events；不自动重试动作')
if __name__=='__main__':
    try:main()
    except urllib.error.URLError as exc:
        print('无法连接本机 CPV 执行器（127.0.0.1:9091）。请检查运行 tools/start-ros.sh real 的终端；服务未启动或已退出。未自动重试动作。',file=sys.stderr)
        raise SystemExit(2)
    except (RuntimeError,TimeoutError) as exc:
        print('操作未完成：'+str(exc)+'；请检查状态，不会自动恢复或重试运动。',file=sys.stderr)
        raise SystemExit(2)
