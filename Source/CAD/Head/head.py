"""
Ori Robot Dog - HEAD ASSEMBLY  (2-DOF pan/pitch, actuated).

Canonical frame: head local origin at the neck base; +X = forward (robot front),
+Y = left, +Z = up.

2-DOF articulation (user-specified):
  PAN  (yaw, about Z)  at the neck base  -> HTD-45H
  PITCH (tilt, about Y) at the dome base  -> HTD-45H
Both servos are the robot baseline (HTD-45H) for parts commonality; head inertia
is tiny so torque headroom is ample.

Real hardware packed first, exterior designed around it:
  * RPi camera module (25x24 board, 8 mm lens) facing +X (forward)
  * microphone array ring (4 mics, ~50 mm ring) on the crown
  * speaker (28 mm) on the underside
  * (ToF sensors reserved as pockets)

All dims from PARAMS.head / .elec / .hw / .servo.
"""
import sys, math
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Master"))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Hardware" / "servos"))
from build_common import cq, PARAMS, cyl, box_centered, export
import hardware_lib as hw
from htd45h import make_htd45h


def _dome(p=PARAMS):
    """Inverted-dome shell: rounded box cap with a flat-ish crown. Pitches about Y."""
    L, W, H = p.head.length, p.head.width, p.head.height
    outer = cq.Workplane("XY").box(L, W, H).edges("|Z").fillet(18.0)
    inner = cq.Workplane("XY").box(L - 4, W - 4, H - 4).translate((0, 0, 2))
    shell = outer.cut(inner)
    # face opening for camera (forward +X)
    shell = shell.cut(cyl(p.head.cam_d / 2 + 2, 6.0, axis="X").translate((L / 2 - 2, 0, 4)))
    # crown hole for mic ring
    shell = shell.cut(cyl(p.head.mic_ring_d / 2, 4.0, axis="Z").translate((0, 0, H / 2 - 2)))
    # underside speaker hole (-Z)
    shell = shell.cut(cyl(p.head.speaker_d / 2, 4.0, axis="Z").translate((-L / 6, 0, -H / 2 + 2)))
    return shell


def _neck(p=PARAMS):
    """Neck column from torso; carries the PAN (yaw) servo at its top."""
    col = cyl(p.head.neck_d / 2, p.head.neck_len, axis="Z").translate((0, 0, p.head.neck_len / 2))
    flange = cyl(p.head.neck_d / 2 + 8, 6.0, axis="Z").translate((0, 0, 2))
    # pan servo seated at top of neck, shaft along Z (yaw)
    pan = make_htd45h(p).translate((0, 0, p.head.neck_len))
    return col.union(flange).union(pan)


def _pitch_yoke(p=PARAMS):
    """Yoke between neck (pan output) and dome; carries the PITCH servo (shaft Y)."""
    yoke = box_centered(p.head.neck_d + 10, 14, p.head.neck_d + 6)
    pit = make_htd45h(p).rotate((0, 0, 0), (0, 1, 0), 90).translate((0, (p.head.neck_d + 10) / 2 + 12, 0))
    return yoke.union(pit)


def _hardware(p=PARAMS):
    cam = cq.Workplane("XY").box(p.elec.cam_board_l, p.elec.cam_board_w, 4).translate((p.head.length / 2 - 6, 0, 4))
    cam = cam.union(cyl(p.elec.cam_lens_d / 2, 8, axis="X").translate((p.head.length / 2 - 4, 0, 4)))
    mic = cyl(p.head.mic_ring_d / 2, 6.0, axis="Z").translate((0, 0, p.head.height / 2 - 3))
    spk = cyl(p.head.speaker_d / 2, 6.0, axis="Z").translate((-p.head.length / 6, 0, -p.head.height / 2 + 3))
    return cam, mic, spk


def make_head(p=PARAMS, export_parts=True):
    """Build the 2-DOF head in its parked pose. Returns dict of named solids."""
    neck = _neck(p)                       # pan servo at top (shaft Z)
    pitch_yoke = _pitch_yoke(p).translate((0, 0, p.head.neck_len))   # sits on pan output
    # dome sits on the pitch yoke (pitches about Y); lift by yoke height
    dome = _dome(p).translate((0, 0, p.head.neck_len + p.head.neck_d + 4))
    cam, mic, spk = _hardware(p)
    cam = cam.translate((0, 0, p.head.neck_len + p.head.neck_d + 4))
    mic = mic.translate((0, 0, p.head.neck_len + p.head.neck_d + 4))
    spk = spk.translate((0, 0, p.head.neck_len + p.head.neck_d + 4))

    if export_parts:
        for nm, obj in (("head_neck", neck), ("head_pitch_yoke", pitch_yoke), ("head_dome", dome),
                        ("head_camera", cam), ("head_mic", mic), ("head_speaker", spk)):
            export(obj, nm, subdir="CAD/Exports/head")
        full = neck.union(pitch_yoke).union(dome).union(cam).union(mic).union(spk)
        export(full, "head_assembled", subdir="CAD/Exports/head")

    return {
        "neck": neck, "pitch_yoke": pitch_yoke, "dome": dome,
        "camera": cam, "mic": mic, "speaker": spk,
        "full": neck.union(pitch_yoke).union(dome).union(cam).union(mic).union(spk),
    }


if __name__ == "__main__":
    import build_common as bc
    out = make_head()
    for k, v in out.items():
        if k == "full":
            continue
        print(f"{k:12s} size(X,Y,Z)=", [round(r, 1) for r in bc.size_of(v)])
