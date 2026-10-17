"""Diagnose the local wheel, using Windows short paths for UTF-8 path issues."""
import ctypes
import json
from pathlib import Path
import runpy
import sys
import shutil
import tempfile

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent))
import bootstrap

def short_path(path):
    buf=ctypes.create_unicode_buffer(32768)
    n=ctypes.windll.kernel32.GetShortPathNameW(str(path),buf,len(buf))
    if not n:raise ctypes.WinError()
    return buf.value

def prepare_runtime():
    path=short_path(ROOT/'_runtime')
    print('runtime_path_ascii:',path.isascii(),flush=True)
    if not path.isascii():
        record=ROOT/'runtime_location.json'
        cached=json.loads(record.read_text())['runtime'] if record.exists() else None
        if cached and Path(cached).is_dir():
            path=cached
        else:
            cache=Path(tempfile.mkdtemp(prefix='hc-uipc-probe-'))
            assert cache.resolve().is_relative_to(Path(tempfile.gettempdir()).resolve())
            path=str(cache/'runtime')
            assert path.isascii()
            shutil.copytree(ROOT/'_runtime',path)
            record.write_text(json.dumps({'runtime':path,'reason':'Upstream native module cannot initialize from this non-ASCII workspace path.'},indent=2))
        print('using_ascii_temp_runtime:',path,flush=True)
    return path

if __name__=='__main__':
    path=prepare_runtime()
    sys.path.insert(0,path)
    sys.argv=['uipc','doctor','--probe-cuda']
    runpy.run_module('uipc',run_name='__main__')
