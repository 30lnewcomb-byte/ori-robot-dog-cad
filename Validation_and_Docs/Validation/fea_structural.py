"""
Ori Robot Dog - STRUCTURAL FEA (real mesh-based linear-static, scikit-fem).

Static worst-case margin check. Isotropic PETG. NOT FDM-layer/anisotropy,
NOT dynamic/fatigue. Reports max Von Mises stress + max deflection per part.

Run: python Validation/fea_structural.py
"""
import sys, os, tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for d in ("CAD/Master", "CAD/Legs", "CAD/Torso", "Parameters"):
    sys.path.insert(0, str(ROOT / d))
sys.path.insert(0, str(ROOT))
import build_common as bc
import master_leg as ML
import torso as TR
from Parameters.master_parameters import PARAMS as P
import cadquery as cq
import meshio, gmsh
import numpy as np
from skfem import MeshTet, Basis, ElementTetP1, ElementVectorH1, solve, enforce, asm
from skfem.models.elasticity import linear_elasticity
from skfem.helpers import sym_grad, ddot

# PETG (printed, isotropic assumption) - values in N/mm^2 so geometry (mm) is consistent
E = 2200.0         # N/mm^2  (PETG ~2.0-2.3 GPa = 2000-2300 N/mm^2)
NU = 0.38
YIELD = 45.0       # N/mm^2 = MPa, conservative yield-ish for margin (PETG ~45-60)
g = 9.81

rows = []
def ck(name, ok, detail=""):
    rows.append((name, "PASS" if ok else "FAIL", detail))


def fea_part(solid, label, load_n, fix_center=None, load_center=None, fix_r=12.0, load_r=12.0, bc_axis='x'):
    """Mesh solid (gmsh tetra), clamp proximal face (min X), load distal face
    (max X) downward. Return (max_vm_MPa, max_disp_mm). If fix_center/load_center
    given, clamp/load nodes within a sphere of those points (bearing-axis BC)."""
    d = Path(tempfile.gettempdir())
    step = d / f"hermes-fea-{label}.step"
    cq.exporters.export(solid, str(step))
    msh = d / f"hermes-fea-{label}.msh"
    gmsh.initialize()
    gmsh.option.setNumber("Mesh.MeshSizeMax", 4.0)
    gmsh.option.setNumber("Mesh.MeshSizeMin", 1.5)
    gmsh.model.add(label)
    gmsh.model.occ.importShapes(str(step))
    gmsh.model.occ.synchronize()
    gmsh.model.mesh.generate(3)
    gmsh.write(str(msh))
    gmsh.finalize()
    m = meshio.read(str(msh))
    os.remove(step); os.remove(msh)
    pts = m.points
    tet = m.cells_dict["tetra"]
    m2 = meshio.Mesh(pts, [("tetra", tet)])
    vtk = d / f"hermes-fea-{label}.vtk"
    meshio.write(str(vtk), m2)
    mesh = MeshTet.load(str(vtk))
    os.remove(vtk)

    if fix_center is None or load_center is None:
        # plane-slice BC along bc_axis: clamp min face, load max face (cantilever)
        ax = {'x': 0, 'y': 1, 'z': 2}[bc_axis]
        bb = (mesh.p.min(axis=1), mesh.p.max(axis=1))
        L = bb[1][ax] - bb[0][ax]
        fx, loadx = bb[0][ax], bb[1][ax]
        fixed = np.isclose(mesh.p[ax], fx, atol=0.15 * L)
        loaded = np.isclose(mesh.p[ax], loadx, atol=0.15 * L)
    else:
        # sphere BCs; fix_center / load_center may be a single tuple or list of tuples
        fc = fix_center if isinstance(fix_center, list) else [fix_center]
        lc = load_center if isinstance(load_center, list) else [load_center]
        pc = mesh.p.T
        fixed = np.zeros(pc.shape[0], bool)
        for c in fc:
            fixed |= np.linalg.norm(pc - np.array(c), axis=1) < fix_r
        loaded = np.zeros(pc.shape[0], bool)
        for c in lc:
            loaded |= np.linalg.norm(pc - np.array(c), axis=1) < load_r

    from skfem import ElementVectorH1
    basis = Basis(mesh, ElementVectorH1(ElementTetP1()))
    # linear_elasticity takes Lame parameters (Lambda, Mu), NOT E, nu.
    lam = E * NU / ((1 + NU) * (1 - 2 * NU))
    mu = E / (2 * (1 + NU))
    K = linear_elasticity(lam, mu)
    A = asm(K, basis)

    nloaded = max(1, int(loaded.sum()))
    nodal = -load_n / nloaded
    fvec = np.zeros(basis.N)
    loaded_dofs = basis.nodal_dofs[:, loaded]
    fvec[loaded_dofs[2]] += nodal

    fixed_dofs = basis.nodal_dofs[:, fixed]
    A, fvec = enforce(A, fvec, D=fixed_dofs.ravel())
    x = solve(A, fvec)

    disp = basis.interpolate(x)
    dvec = np.sqrt(disp[0]**2 + disp[1]**2 + disp[2]**2)
    max_disp = float(dvec.max())

    g = disp.grad

    # strain_{ij}[elem][node] = 0.5*(dg_i/dx_j + dg_j/dx_i)
    eps = np.zeros((g.shape[1], g.shape[2], 3, 3))
    for i in range(3):
        for j in range(3):
            eps[:, :, i, j] = 0.5 * (g[i, :, :, j] + g[j, :, :, i])
    # stress via isotropic Hooke
    lam = E * NU / ((1 + NU) * (1 - 2 * NU))
    mu = E / (2 * (1 + NU))
    tr = np.trace(eps, axis1=2, axis2=3)[..., None, None]
    sig = lam * tr * np.eye(3) + 2 * mu * eps
    sdev = sig - (np.trace(sig, axis1=2, axis2=3) / 3.0)[..., None, None] * np.eye(3)
    vm = np.sqrt(1.5 * np.einsum('...ij,...ij->...', sdev, sdev))
    return float(vm.max()), max_disp


def main():
    # Upper link: clamp the two SOLID hip yoke plates (not the hollow tube end,
    # which creates a clamp singularity), load the SOLID knee clevis downward.
    # Worst case: full robot weight on one leg (static; dynamic amplified separately).
    up = ML.make_upper_link(P)
    wy = P.leg.link_w / 2 + 3.5  # yoke plate centreline y
    fix_cs = [(12.0, +wy, 0.0), (12.0, -wy, 0.0)]
    load_c = (P.leg.upper_link_length - 11.0, 0.0, 0.0)
    max_vm, max_d = fea_part(up, "upper_link", 58.9, fix_cs, load_c, fix_r=12.0, load_r=12.0)
    ck("upper-link Von Mises < yield", max_vm < YIELD, f"{max_vm:.1f} MPa vs {YIELD:.0f}")
    ck("upper-link deflection < 5 mm", max_d < 5.0, f"{max_d:.2f} mm")

    # Hip bulkhead: clamp inner face (min Y, toward torso centre), load servo-axis face (max Y).
    hb = TR._hip_bulkhead(+P.torso.hip_pitch_axis_x, +P.torso.hip_axis_y, P.torso.hip_axis_z, P)
    max_vm2, max_d2 = fea_part(hb, "hip_bulkhead", 58.9, bc_axis='y')
    ck("hip bulkhead Von Mises < yield", max_vm2 < YIELD, f"{max_vm2:.1f} MPa vs {YIELD:.0f}")
    ck("hip bulkhead deflection < 3 mm", max_d2 < 3.0, f"{max_d2:.2f} mm")

    print(f"{'FEA CHECK':34s} {'RESULT':6s} DETAIL")
    print("-" * 78)
    for n, r, dd in rows:
        print(f"{n:34s} {r:6s} {dd}")
    npass = sum(1 for _, r, _ in rows if r == "PASS")
    print("-" * 78)
    print(f"{npass}/{len(rows)} passed  (PETG E={E:.0f}N/mm^2 nu={NU} yield={YIELD:.0f}MPa, static worst-case)")
    print("NOTE: isotropic PETG assumption; excludes FDM layer anisotropy, dynamic & fatigue.")
    sys.exit(0 if npass == len(rows) else 1)


if __name__ == "__main__":
    main()
