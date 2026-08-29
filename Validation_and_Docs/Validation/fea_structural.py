"""Ori Robot Dog — mesh-based linear-static structural FEA.

Analysis only. This is not physical testing and does not model FDM anisotropy,
dynamic loads, fatigue, or hardware-specific contact behavior.

Optional dependencies are kept separate from the core CadQuery environment.
Run only when the FEA dependency set is installed.
"""
import os
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "Source"
CAD = SOURCE / "CAD"
for path in (SOURCE, CAD, CAD / "Master", CAD / "Legs", CAD / "Torso", SOURCE / "Parameters"):
    sys.path.insert(0, str(path))

import build_common as bc
import master_leg as ML
import torso as TR
from Parameters.master_parameters import PARAMS as P
import cadquery as cq
import meshio
import gmsh
import numpy as np
from skfem import MeshTet, Basis, ElementTetP1, ElementVectorH1, solve, enforce, asm
from skfem.models.elasticity import linear_elasticity

E = 2200.0       # N/mm^2, isotropic PETG assumption
NU = 0.38
YIELD = 45.0     # MPa conservative modeled yield-ish threshold
g = 9.81

rows = []
def ck(name, ok, detail=""):
    rows.append((name, "PASS" if ok else "FAIL", detail))


def fea_part(solid, label, load_n, fix_center=None, load_center=None, fix_r=12.0, load_r=12.0, bc_axis='x'):
    """Mesh a CAD solid and solve a simple linear-elastic static case."""
    tmp = Path(tempfile.gettempdir())
    step = tmp / f"ori-fea-{label}.step"
    msh = tmp / f"ori-fea-{label}.msh"
    vtk = tmp / f"ori-fea-{label}.vtk"
    cq.exporters.export(solid, str(step))

    gmsh.initialize()
    try:
        gmsh.option.setNumber("Mesh.MeshSizeMax", 4.0)
        gmsh.option.setNumber("Mesh.MeshSizeMin", 1.5)
        gmsh.model.add(label)
        gmsh.model.occ.importShapes(str(step))
        gmsh.model.occ.synchronize()
        gmsh.model.mesh.generate(3)
        gmsh.write(str(msh))
    finally:
        gmsh.finalize()

    m = meshio.read(str(msh))
    pts = m.points
    tet = m.cells_dict["tetra"]
    meshio.Mesh(pts, [("tetra", tet)]).write(str(vtk))
    for f in (step, msh):
        if f.exists():
            f.unlink()
    mesh = MeshTet.load(str(vtk))
    if vtk.exists():
        vtk.unlink()

    if fix_center is None or load_center is None:
        ax = {'x': 0, 'y': 1, 'z': 2}[bc_axis]
        pmin = mesh.p.min(axis=1)
        pmax = mesh.p.max(axis=1)
        length = pmax[ax] - pmin[ax]
        fixed = np.isclose(mesh.p[ax], pmin[ax], atol=max(0.15 * length, 0.5))
        loaded = np.isclose(mesh.p[ax], pmax[ax], atol=max(0.15 * length, 0.5))
    else:
        pc = mesh.p.T
        fixed = np.zeros(pc.shape[0], dtype=bool)
        for center in (fix_center if isinstance(fix_center, list) else [fix_center]):
            fixed |= np.linalg.norm(pc - np.asarray(center), axis=1) < fix_r
        loaded = np.zeros(pc.shape[0], dtype=bool)
        for center in (load_center if isinstance(load_center, list) else [load_center]):
            loaded |= np.linalg.norm(pc - np.asarray(center), axis=1) < load_r

    basis = Basis(mesh, ElementVectorH1(ElementTetP1()))
    lam = E * NU / ((1 + NU) * (1 - 2 * NU))
    mu = E / (2 * (1 + NU))
    K = linear_elasticity(lam, mu)
    A = asm(K, basis)

    nloaded = max(1, int(loaded.sum()))
    fvec = np.zeros(basis.N)
    fvec[basis.nodal_dofs[2, loaded]] += -load_n / nloaded
    A, fvec = enforce(A, fvec, D=basis.nodal_dofs[:, fixed].ravel())
    solution = solve(A, fvec)

    disp = basis.interpolate(solution)
    dvec = np.sqrt(disp[0] ** 2 + disp[1] ** 2 + disp[2] ** 2)
    max_disp = float(dvec.max())
    grad = disp.grad
    eps = np.zeros((grad.shape[1], grad.shape[2], 3, 3))
    for i in range(3):
        for j in range(3):
            eps[:, :, i, j] = 0.5 * (grad[i, :, :, j] + grad[j, :, :, i])
    tr = np.trace(eps, axis1=2, axis2=3)[..., None, None]
    sigma = lam * tr * np.eye(3) + 2 * mu * eps
    dev = sigma - (np.trace(sigma, axis1=2, axis2=3) / 3.0)[..., None, None] * np.eye(3)
    vm = np.sqrt(1.5 * np.einsum('...ij,...ij->...', dev, dev))
    return float(vm.max()), max_disp


def main():
    upper = ML.make_upper_link(P)
    wy = P.leg.link_w / 2 + 3.5
    max_vm, max_d = fea_part(
        upper,
        "upper_link",
        58.9,
        fix_center=[(12.0, +wy, 0.0), (12.0, -wy, 0.0)],
        load_center=(P.leg.upper_link_length - 11.0, 0.0, 0.0),
    )
    ck("upper-link Von Mises < modeled yield", max_vm < YIELD, f"{max_vm:.1f} MPa vs {YIELD:.0f} MPa")
    ck("upper-link deflection < 5 mm", max_d < 5.0, f"{max_d:.2f} mm")

    hip = TR._hip_bulkhead(+P.torso.hip_pitch_axis_x, +P.torso.hip_axis_y, P.torso.hip_axis_z, P)
    max_vm2, max_d2 = fea_part(hip, "hip_bulkhead", 58.9, bc_axis='y')
    ck("hip bulkhead Von Mises < modeled yield", max_vm2 < YIELD, f"{max_vm2:.1f} MPa vs {YIELD:.0f} MPa")
    ck("hip bulkhead deflection < 3 mm", max_d2 < 3.0, f"{max_d2:.2f} mm")

    print(f"{'FEA CHECK':40s} {'RESULT':6s} DETAIL")
    print("-" * 86)
    for name, result, detail in rows:
        print(f"{name:40s} {result:6s} {detail}")
    passed = sum(result == "PASS" for _, result, _ in rows)
    print("-" * 86)
    print(f"{passed}/{len(rows)} passed | isotropic PETG E={E:.0f} MPa, nu={NU}, modeled yield={YIELD:.0f} MPa")
    print("NOTE: no FDM layer anisotropy, dynamics, fatigue, or physical validation.")
    return passed == len(rows)


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
