"""Small, inspectable CPU reference for HC.pdf, Eqs. (2)--(6), Algorithm 1.

This is a new NumPy implementation, not the authors' complete simulator.
Dense matrices are deliberate: independent direct solves validate the algebra.
"""
from dataclasses import dataclass
import numpy as np


@dataclass
class ACA:
    U: np.ndarray
    V: np.ndarray
    residuals: list
    pivots: list

    @property
    def rank(self):
        return self.U.shape[1]


def aca(E, rtol=None, max_rank=None):
    """Full-pivot ACA from Algorithm 1, with a fixed maximum rank by default.

    The supplement, p. 2, specifies fixed-rank termination. A machine-precision
    zero-pivot guard handles exactly rank-deficient fixtures. Optional rtol is
    ||E-U V.T||_F / ||E||_F, used only for a separate diagnostic ablation.
    This dense validation implementation is not the paper's GPU ACA kernel.
    """
    E = np.asarray(E, dtype=np.float64)
    if E.ndim != 2 or not np.isfinite(E).all() or (rtol is not None and rtol < 0):
        raise ValueError("ACA requires a finite matrix and nonnegative tolerance")
    m, n = E.shape
    limit = min(m, n) if max_rank is None else min(max_rank, m, n)
    R = E.copy()
    scale = np.linalg.norm(E)
    residuals, pivots, us, vs = [1.0 if scale else 0.0], [], [], []
    while len(us) < limit and residuals[-1] > (0.0 if rtol is None else rtol):
        i, j = np.unravel_index(np.argmax(np.abs(R)), R.shape)
        pivot = R[i, j]
        if abs(pivot) <= np.finfo(float).eps * max(scale, np.finfo(float).tiny):
            break
        u, v = R[:, j].copy(), R[i, :].copy() / pivot
        us.append(u)
        vs.append(v)
        pivots.append([int(i), int(j), float(pivot)])
        R -= np.outer(u, v)
        residuals.append(float(np.linalg.norm(R) / scale))
    return ACA(np.column_stack(us) if us else np.zeros((m, 0)),
               np.column_stack(vs) if vs else np.zeros((n, 0)), residuals, pivots)


class BlockInverse:
    def __init__(self, H, sizes):
        self.slices, self.inverses = [], []
        offset = 0
        for size in sizes:
            s = slice(offset, offset + size)
            block = H[s, s]
            np.linalg.cholesky(block)  # Fail visibly on non-SPD inputs.
            self.slices.append(s)
            self.inverses.append(np.linalg.solve(block, np.eye(size)))
            offset += size
        if offset != H.shape[0]:
            raise ValueError("Block sizes must cover the matrix")

    def __call__(self, x):
        y = np.empty_like(x, dtype=np.float64)
        for s, inv in zip(self.slices, self.inverses):
            y[s] = inv @ x[s]
        return y


class Neumann:
    """Eq. (5): sum_{k=0}^{order-1} (I-omega B^-1 H)^k omega B^-1."""
    def __init__(self, H, block_size, order=5, omega=0.7):
        if order < 1 or not 0 < omega <= 1 or H.shape[0] % block_size:
            raise ValueError("Invalid Neumann parameters")
        self.H, self.order, self.omega = H, order, omega
        self.Binv = BlockInverse(H, [block_size] * (H.shape[0] // block_size))

    def __call__(self, rhs):
        term = self.omega * self.Binv(rhs)
        result = term.copy()
        for _ in range(1, self.order):
            term = term - self.omega * self.Binv(self.H @ term)
            result += term
        return result

    def spectral_radius(self):
        T = np.eye(len(self.H)) - self.omega * self.Binv(self.H)
        return float(np.max(np.abs(np.linalg.eigvals(T))))


def pcg(matvec, b, preconditioner=None, rtol=1e-10, maxiter=1000):
    """PCG with a true (unsquared) relative residual termination check.

    A denominator is never clamped to an absolute EPSILON, which could change
    the mathematical iteration near convergence. Non-SPD breakdown is explicit.
    """
    x = np.zeros_like(b)
    normb = np.linalg.norm(b)
    if normb == 0:
        return x, {"iterations": 0, "relative_residual": 0.0,
                   "converged": True, "history": [0.0]}
    M = preconditioner or (lambda r: r.copy())
    r = b.copy()
    z = M(r)
    p = z.copy()
    rz = float(r @ z)
    history = [1.0]
    for iteration in range(1, maxiter + 1):
        Ap = matvec(p)
        pAp = float(p @ Ap)
        if not np.isfinite(pAp) or pAp <= 0 or rz <= 0:
            raise ArithmeticError(f"PCG non-SPD breakdown at iteration {iteration}")
        alpha = rz / pAp
        x += alpha * p
        r -= alpha * Ap
        # Validate against the original operator rather than recursive residual.
        true_r = b - matvec(x)
        relative = float(np.linalg.norm(true_r) / normb)
        history.append(relative)
        if relative <= rtol:
            break
        if np.linalg.norm(r) <= rtol * normb:
            # Reliable restart if floating-point recurrence lost accuracy.
            r = true_r
            z = M(r)
            rz = float(r @ z)
            p = z.copy()
            continue
        z = M(r)
        rz_new = float(r @ z)
        p = z + (rz_new / rz) * p
        rz = rz_new
    return x, {"iterations": iteration, "relative_residual": history[-1],
               "converged": history[-1] <= rtol, "history": history}


class HSC:
    """Low-rank Schur elimination; A inverse may be exact or truncated."""
    def __init__(self, D, A, E, order=5, omega=0.7, aca_rtol=None,
                 exact_affine=False, aca_max_rank=None):
        self.D, self.A, self.E = D, A, E
        self.factors = aca(E, rtol=aca_rtol, max_rank=aca_max_rank)
        self.U, self.V = self.factors.U, self.factors.V
        self.affine_neumann = Neumann(A, 12, order, omega)
        self.affine_inverse = ((lambda x: np.linalg.solve(A, x))
                               if exact_affine else self.affine_neumann)
        self.deformable_preconditioner = Neumann(D, 3, order, omega)
        self.C = self.U.T @ self.affine_inverse(self.U)
        # Symmetry loss at roundoff level should not reach PCG.
        self.C = (self.C + self.C.T) * 0.5
        self.schur = D - self.V @ self.C @ self.V.T
        np.linalg.cholesky(self.schur)

    def matvec(self, p):
        return self.D @ p - self.V @ (self.C @ (self.V.T @ p))

    def solve(self, b, rtol=1e-10, maxiter=1000):
        n = len(self.D)
        bx, bq = b[:n], b[n:]
        rhs = bx - self.V @ (self.U.T @ self.affine_inverse(bq))
        dx, stats = pcg(self.matvec, rhs, self.deformable_preconditioner,
                        rtol, maxiter)
        dq = self.affine_inverse(bq - self.U @ (self.V.T @ dx))
        return np.concatenate((dx, dq)), stats


def refinement(H, b, solver, rtol=1e-9, maxsteps=50):
    """Inexact Newton on a manufactured quadratic, with explicit corrections.

    This outer loop is a local validation fixture, not a claim of reproducing
    the paper's nonlinear IPC time integration or its Newton iteration counts.
    """
    x, history, inner_iterations, alphas = np.zeros_like(b), [1.0], [], []
    normb = np.linalg.norm(b)
    if normb == 0:
        return x, {"steps": 0, "history": [0.0], "converged": True,
                   "inner_iterations": [], "step_lengths": []}
    for _ in range(maxsteps):
        residual = b - H @ x
        direction, info = solver.solve(residual)
        inner_iterations.append(info["iterations"])
        numerator = float(residual @ direction)
        denominator = float(direction @ H @ direction)
        if numerator <= 0 or denominator <= 0:
            raise ArithmeticError("The approximate HSC direction is not descent")
        alpha = min(1.0, numerator / denominator)
        x += alpha * direction
        alphas.append(alpha)
        history.append(float(np.linalg.norm(b - H @ x) / normb))
        if history[-1] <= rtol:
            break
    return x, {"steps": len(inner_iterations), "history": history,
               "converged": history[-1] <= rtol,
               "inner_iterations": inner_iterations, "step_lengths": alphas}


def affine_projection(rest):
    P = np.zeros((3, 12))
    for row in range(3):
        P[row, 3 * row:3 * row + 3] = rest
        P[row, 9 + row] = 1
    return P


def manufactured_system(stiffness=1e6, seed=2026, nx=10, ny=3, nz=2):
    """Contact-like SPD block fixture, not a mesh/time-step from the paper.

    3-DOF lattice nodes, three 12-DOF affine blocks, and positive rank-one
    interface stencils. All matrices and the manufactured solution are saved.
    No constitutive-model, IPC, friction, or collision fidelity is implied.
    """
    rng = np.random.default_rng(seed)
    coords = np.array([(i, j, k) for i in range(nx) for j in range(ny)
                       for k in range(nz)], dtype=float)
    nv, m = len(coords), 36
    n = 3 * nv
    L = np.zeros((nv, nv))
    for i in range(nv):
        for j in range(i):
            if np.abs(coords[i] - coords[j]).sum() == 1:
                L[i, i] += 1
                L[j, j] += 1
                L[i, j] -= 1
                L[j, i] -= 1
    material = np.array([[2., .2, .1], [.2, 1., .15], [.1, .15, 1.5]])
    D = np.kron(40 * L + np.eye(nv), material)
    A = np.zeros((m, m))
    for body in range(3):
        Q, _ = np.linalg.qr(rng.normal(size=(12, 12)))
        A[12*body:12*(body+1), 12*body:12*(body+1)] = (
            Q @ np.diag(np.geomspace(10., stiffness, 12)) @ Q.T)
    H = np.zeros((n + m, n + m))
    H[:n, :n], H[n:, n:] = D, A
    for contact in range(18):
        node = int(rng.integers(nv))
        body = contact % 3
        normal = rng.normal(size=3)
        normal /= np.linalg.norm(normal)
        rest = rng.uniform(-.5, .5, 3)
        row = np.zeros(n + m)
        row[3*node:3*node+3] = normal
        row[n+12*body:n+12*(body+1)] = -normal @ affine_projection(rest)
        weight = np.geomspace(1., 1e3, 18)[contact]
        H += weight * np.outer(row, row)
    # Weak affine-affine coupling breaks the perfectly block-diagonal A case.
    for body in range(2):
        row = np.zeros(n + m)
        row[n+12*body+9] = 1
        row[n+12*(body+1)+9] = -1
        H += 2. * np.outer(row, row)
    x_star = rng.normal(size=n + m)
    # Normalize the RHS: relative residual semantics remain visible at any scale.
    b = H @ x_star
    scale = np.linalg.norm(b)
    b, x_star = b / scale, x_star / scale
    return H[:n, :n].copy(), H[n:, n:].copy(), H[n:, :n].copy(), H, b, x_star
