// ori_torso.scad — splittable structural chassis, hips, battery bay, arm port.
// Mirrors CAD/Torso/torso.py. Readable summary, not the manufactured part.
include <ori_params.scad>

module torso_half(side=+1) {
  color("steelblue") {
    translate([0, side*torso_w/4, 0])
      cube([torso_len, torso_w/2, torso_h], center=true);
  }
  // arm port (front +X face)
  translate([arm_base_x,0,torso_h/2])
    color("gray") cylinder(r=29, h=12, center=true, $fn=24);
}

module torso() {
  torso_half(+1);
  torso_half(-1);
  // hip bulkheads (4 corners)
  for (sx=[-1,1], sy=[-1,1])
    translate([sx*stance_length/4, sy*stance_width/2, 0])
      color("gray") cylinder(r=bearing_od/2+4, h=torso_h+8, center=true, $fn=24);
}

torso();
