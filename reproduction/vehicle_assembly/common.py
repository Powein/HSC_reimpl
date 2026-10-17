from pathlib import Path
import os,sys

ROOT=Path(__file__).resolve().parent
RESEARCH=ROOT.parent/'vehicle_research'
sys.path.insert(0,str(ROOT.parent))
import bootstrap
sys.path.insert(0,str(RESEARCH/'_geometry'))

def import_uipc():
    # A custom native backend is selected explicitly, never by replacing the
    # stock wheel or silently falling back to it when HSC is requested.
    if os.environ.get('HC_UIPC_RUNTIME'):
        runtime=Path(os.environ['HC_UIPC_RUNTIME']).resolve()
        if not (runtime/'uipc/_native/uipc_backend_cuda.dll').is_file():
            raise RuntimeError('Custom HSC CUDA runtime is missing: '+str(runtime))
        sys.path.insert(0,str(runtime))
        import uipc
        return uipc,runtime.parent
    sys.path.insert(0,str(RESEARCH))
    from probe_cuda import prepare_runtime
    runtime=Path(prepare_runtime())
    sys.path.insert(0,str(runtime))
    import uipc
    return uipc,runtime.parent
