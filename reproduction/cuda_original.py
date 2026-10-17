"""Run selected, unmodified author CUDA function bodies via NVRTC/Driver API.

No CUDA Toolkit/nvcc or CuPy is needed. A compatible NVIDIA driver and the
nvidia-cuda-nvrtc-cu12 wheel suffice. This is a kernel harness, NOT a build of
the incomplete public library, nor a GPU implementation of the full HSC solver.
"""
import bootstrap
from pathlib import Path
import ctypes as C
import hashlib
import os
import re
import numpy as np


def extract_function(path, name, occurrence=0):
    text = path.read_text(encoding="utf-8")
    matches = list(re.finditer(r"\b" + re.escape(name) + r"\s*\(", text))
    start = matches[occurrence].start()
    start = text.rfind("template", 0, start)
    opening = text.index("{", matches[occurrence].end())
    depth = 1
    end = opening + 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[start:end]


def build_source(source_root, output):
    selections = [
        ("math/math_cuda.cuh", "vecSubs", 0),
        ("math/math_cuda.cuh", "dot3", 1),
        ("math/math_cuda.cuh", "affine_transform", 0),
        ("affine_projection/affine_projection_cuda.cuh", "affine_projection", 0),
        ("affine_projection/affine_projection_cuda.cuh", "affine_projection_gradient3x1", 0),
        ("affine_projection/affine_projection_cuda.cuh", "affine_projection_Hessian3x3", 0),
        ("rotational/rotational_cuda.cuh", "rotational_energy", 0),
        ("rotational/rotational_cuda.cuh", "rotational_gradient", 0),
        ("rotational/rotational_cuda.cuh", "rotational_Hessian", 0),
        ("sparse_matrix_square/spmv_cuda.cuh", "spmv_sm_kernel", 0),
        ("sparse_linear_solvers/conjugate_gradient_cuda.cuh", "dmv_accumulate_kernel", 0),
    ]
    # Only memset's zero-fill dependency is supplied locally. Selected function
    # bodies, expressions, indexing, and precision remain exactly as distributed.
    source = '''
__device__ void* memset(void* address, int value, unsigned long long size) {
    unsigned char* bytes = (unsigned char*)address;
    for (unsigned long long i=0; i<size; ++i) bytes[i] = (unsigned char)value;
    return address;
}
'''
    manifest = []
    for filename, name, occurrence in selections:
        body = extract_function(source_root / filename, name, occurrence)
        manifest.append({"file": filename, "function": name,
                         "body_sha256": hashlib.sha256(body.encode()).hexdigest()})
        source += f'\n// Extracted without edits: {filename} : {name}\n{body}\n'
    source += '''
extern "C" __global__ void validate_affine(double* rest, double* g, double* h,
                                         double* p, double* pg, double* ph) {
    affine_projection<double>(rest, p);
    affine_projection_gradient3x1<double>(rest, g, pg);
    affine_projection_Hessian3x3<double>(rest, h, ph);
}
extern "C" __global__ void validate_rotational(double* r0, double* r1,
    double* q0, double* q1, double* e, double* g, double* h) {
    e[0] = rotational_energy<double>(r0, r1, q0, q1);
    rotational_gradient<double>(r0, r1, q0, q1, g);
    rotational_Hessian<double>(r0, r1, h);
}
'''
    output.write_text(source, encoding="utf-8")
    return source, manifest


class Driver:
    def __init__(self):
        self.dll = C.WinDLL("nvcuda.dll")
        self.allocations = []
        self.context = C.c_void_p()
        self.module = C.c_void_p()
        self.call("cuInit", 0)
        device = C.c_int()
        self.call("cuDeviceGet", C.byref(device), 0)
        name = C.create_string_buffer(256)
        self.call("cuDeviceGetName", name, 256, device)
        self.name = name.value.decode()
        major, minor = C.c_int(), C.c_int()
        self.call("cuDeviceGetAttribute", C.byref(major), 75, device)
        self.call("cuDeviceGetAttribute", C.byref(minor), 76, device)
        self.arch = f"compute_{major.value}{minor.value}"
        self.call("cuCtxCreate_v2", C.byref(self.context), 0, device)

    def call(self, name, *args):
        result = getattr(self.dll, name)(*args)
        if result:
            error = C.c_char_p()
            self.dll.cuGetErrorString(result, C.byref(error))
            raise RuntimeError(f"{name}: CUDA {result}: {error.value}")

    def upload(self, value):
        value = np.ascontiguousarray(value)
        ptr = C.c_uint64()
        self.call("cuMemAlloc_v2", C.byref(ptr), C.c_size_t(value.nbytes))
        self.allocations.append(ptr)
        self.call("cuMemcpyHtoD_v2", ptr, C.c_void_p(value.ctypes.data), C.c_size_t(value.nbytes))
        return ptr

    def download(self, ptr, shape):
        out = np.empty(shape, dtype=np.float64)
        self.call("cuMemcpyDtoH_v2", C.c_void_p(out.ctypes.data), ptr, C.c_size_t(out.nbytes))
        return out

    def launch(self, name, args, blocks=1, threads=1):
        func = C.c_void_p()
        self.call("cuModuleGetFunction", C.byref(func), self.module, name.encode())
        params = (C.c_void_p * len(args))(*[C.cast(C.byref(x), C.c_void_p) for x in args])
        self.call("cuLaunchKernel", func, blocks, 1, 1, threads, 1, 1,
                  0, C.c_void_p(), params, C.c_void_p())
        self.call("cuCtxSynchronize")

    def clear(self):
        for ptr in self.allocations:
            self.call("cuMemFree_v2", ptr)
        self.allocations.clear()

    def close(self):
        self.clear()
        if self.module:
            self.call("cuModuleUnload", self.module)
        if self.context:
            self.call("cuCtxDestroy_v2", self.context)


def compile_nvrtc(source, arch, output):
    candidates = [p for p in bootstrap.DEPS.rglob("nvrtc64_*.dll")
                  if ".alt." not in p.name]
    if not candidates:
        raise FileNotFoundError("Install nvidia-cuda-nvrtc-cu12 into reproduction/_deps")
    dllpath = candidates[0]
    handle = os.add_dll_directory(str(dllpath.parent))
    # NVRTC's internal LoadLibrary call does not always inherit Python's search
    # flags (especially under non-ASCII project paths). Preload its peer DLL.
    builtins = [C.WinDLL(str(p)) for p in dllpath.parent.glob("nvrtc-builtins*.dll")]
    nvrtc = C.WinDLL(str(dllpath))
    prog = C.c_void_p()
    rc = nvrtc.nvrtcCreateProgram(C.byref(prog), source.encode(), b"author_kernels.cu", 0, None, None)
    if rc:
        raise RuntimeError(f"nvrtcCreateProgram returned {rc}")
    expressions = [b"spmv_sm_kernel<double, 3>", b"spmv_sm_kernel<double, 12>",
                   b"dmv_accumulate_kernel<double, 3>", b"dmv_accumulate_kernel<double, 12>"]
    try:
        for expression in expressions:
            if nvrtc.nvrtcAddNameExpression(prog, expression):
                raise RuntimeError("Could not register kernel template")
        opts = [b"--std=c++17", f"--gpu-architecture={arch}".encode(), b"--fmad=false"]
        options = (C.c_char_p * len(opts))(*opts)
        rc = nvrtc.nvrtcCompileProgram(prog, len(opts), options)
        size = C.c_size_t()
        nvrtc.nvrtcGetProgramLogSize(prog, C.byref(size))
        log = C.create_string_buffer(size.value)
        nvrtc.nvrtcGetProgramLog(prog, log)
        (output / "nvrtc.log").write_bytes(log.value)
        if rc:
            raise RuntimeError(f"NVRTC compile error {rc}: {log.value.decode()}")
        nvrtc.nvrtcGetPTXSize(prog, C.byref(size))
        ptx = C.create_string_buffer(size.value)
        nvrtc.nvrtcGetPTX(prog, ptx)
        (output / "author_kernels.ptx").write_bytes(ptx.value)
        names = {}
        for expression in expressions:
            name = C.c_char_p()
            nvrtc.nvrtcGetLoweredName(prog, expression, C.byref(name))
            names[expression.decode()] = name.value.decode()
        return ptx, names
    finally:
        nvrtc.nvrtcDestroyProgram(C.byref(prog))
        handle.close()


def validate(source_root, output, cases=20):
    from core import affine_projection
    output.mkdir(exist_ok=True, parents=True)
    source, manifest = build_source(source_root, output / "author_kernels.cu")
    gpu = Driver()
    rng = np.random.default_rng(2026)
    errors = {name: 0.0 for name in ["affine_P", "affine_gradient", "affine_Hessian",
              "rotational_energy", "rotational_gradient", "rotational_Hessian",
              "rotational_gradient_finite_difference", "rotational_Hessian_finite_difference",
              "spmv_3", "spmv_12", "dmv_accumulate_3", "dmv_accumulate_12"]}
    def record(name, actual, expected):
        errors[name] = max(errors[name], float(np.max(np.abs(actual - expected))))
    try:
        ptx, names = compile_nvrtc(source, gpu.arch, output)
        gpu.call("cuModuleLoadData", C.byref(gpu.module), C.cast(ptx, C.c_void_p))
        for _ in range(cases):
            r0, r1 = rng.normal(size=(2, 3))
            q0, q1 = rng.normal(size=(2, 12))
            g = rng.normal(size=3)
            Q = rng.normal(size=(3, 3))
            h = Q.T @ Q + np.eye(3)
            args = [gpu.upload(x) for x in [r0, g, h, np.zeros((3, 12)),
                    np.zeros(12), np.zeros((12, 12))]]
            gpu.launch("validate_affine", args)
            P0, P1 = affine_projection(r0), affine_projection(r1)
            record("affine_P", gpu.download(args[3], (3, 12)), P0)
            record("affine_gradient", gpu.download(args[4], (12,)), P0.T @ g)
            record("affine_Hessian", gpu.download(args[5], (12, 12)), P0.T @ h @ P0)
            args = [gpu.upload(x) for x in [r0, r1, q0, q1, np.zeros(1),
                    np.zeros(24), np.zeros((24, 24))]]
            gpu.launch("validate_rotational", args)
            B, q = np.hstack((P0, -P1)), np.concatenate((q0, q1))
            residual = B @ q
            energy = lambda w: .5 * np.dot(B @ w, B @ w)
            gradient = lambda w: B.T @ (B @ w)
            expected_h = B.T @ B
            record("rotational_energy", gpu.download(args[4], (1,)), .5 * residual @ residual)
            actual_g = gpu.download(args[5], (24,))
            actual_h = gpu.download(args[6], (24, 24))
            record("rotational_gradient", actual_g, gradient(q))
            record("rotational_Hessian", actual_h, expected_h)
            steps = np.eye(24) * 1e-5
            fd_g = np.array([(energy(q+s)-energy(q-s))/(2e-5) for s in steps])
            fd_h = np.column_stack([(gradient(q+s)-gradient(q-s))/(2e-5) for s in steps])
            record("rotational_gradient_finite_difference", actual_g, fd_g)
            record("rotational_Hessian_finite_difference", actual_h, fd_h)
            gpu.clear()
            for bs in [3, 12]:
                coords = np.array([[0,0], [0,1], [1,0], [1,1]], dtype=np.uint32)
                blocks = rng.normal(size=(4, bs, bs))
                dense = np.block([[blocks[0], blocks[1]], [blocks[2], blocks[3]]])
                vec = rng.normal(size=2*bs)
                args = [C.c_uint(4)] + [gpu.upload(v) for v in [coords, blocks, vec, np.zeros(2*bs)]]
                gpu.launch(names[f"spmv_sm_kernel<double, {bs}>"], args, threads=128)
                record(f"spmv_{bs}", gpu.download(args[-1], (2*bs,)), dense @ vec)
                diag, z = rng.normal(size=(2, bs, bs)), rng.normal(size=2*bs)
                args = [C.c_int(2)] + [gpu.upload(v) for v in [diag, vec, z]] + [C.c_double(.7)]
                gpu.launch(names[f"dmv_accumulate_kernel<double, {bs}>"], args, threads=32)
                expected = z + .7 * np.einsum("bij,bj->bi", diag, vec.reshape(2,bs)).ravel()
                record(f"dmv_accumulate_{bs}", gpu.download(args[3], (2*bs,)), expected)
                gpu.clear()
        tolerances = {k: 2e-8 if "finite_difference" in k else 1e-10 for k in errors}
        passed = all(errors[k] < tolerances[k] for k in errors)
        return {"status": "PASS" if passed else "FAIL", "gpu": gpu.name,
                "architecture": gpu.arch, "cases_per_check": cases,
                "max_absolute_errors": errors, "tolerances": tolerances,
                "source_functions": manifest,
                "scope": "Selected author function bodies compiled and executed; full public library not built",
                "rotational_scope": "Tests the distributed point-matching energy, not the complete paper Eq. (11) joint"}
    finally:
        gpu.close()
