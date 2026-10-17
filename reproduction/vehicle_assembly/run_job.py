"""Bounded, logged local experiment runner; kills only its own child on timeout."""
from pathlib import Path
import argparse,json,subprocess,sys,time
sys.stdout.reconfigure(encoding='utf-8',errors='replace')

ROOT=Path(__file__).resolve().parent
parser=argparse.ArgumentParser()
parser.add_argument('--name',required=True)
parser.add_argument('--timeout',type=int,default=300)
parser.add_argument('script')
parser.add_argument('arguments',nargs=argparse.REMAINDER)
args=parser.parse_args()
assert args.name.replace('_','').replace('-','').isalnum()
script=(ROOT/args.script).resolve()
assert script.is_relative_to(ROOT) and script.is_file()
out=ROOT/'logs';out.mkdir(exist_ok=True)
start=time.monotonic();last=start
log=out/(args.name+'.log');status_path=out/(args.name+'.status.json')
with log.open('w',encoding='utf-8') as stream:
    process=subprocess.Popen([sys.executable,str(script),*args.arguments],cwd=ROOT,
                             stdout=stream,stderr=subprocess.STDOUT,
                             creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    status={'pid':process.pid,'command':[str(script),*args.arguments],
            'timeout_s':args.timeout,'state':'running','log':str(log)}
    status_path.write_text(json.dumps(status,indent=2))
    print('Started',args.name,'PID',process.pid,flush=True)
    timed_out=False
    while process.poll() is None:
        if time.monotonic()-start>args.timeout:
            process.kill();process.wait();timed_out=True;break
        if time.monotonic()-last>30:
            print('Alive',args.name,'elapsed',round(time.monotonic()-start),'s',flush=True)
            last=time.monotonic()
        time.sleep(1)
    status.update(state='timeout' if timed_out else 'complete',exit_code=process.returncode,
                  elapsed_s=time.monotonic()-start)
    status_path.write_text(json.dumps(status,indent=2))
print(json.dumps(status,ensure_ascii=True),flush=True)
print('\n'.join(log.read_text(encoding='utf-8',errors='replace').splitlines()[-20:]),flush=True)
raise SystemExit(124 if timed_out else process.returncode)
