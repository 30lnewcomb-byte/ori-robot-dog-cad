// ori_head.scad — 2-DOF pan/pitch dome head with camera + neck.
// Mirrors CAD/Head/head.py. Readable summary, not the manufactured part.
include <ori_params.scad>

module head() {
  color("lightblue") sphere(r=head_d/2, $fn=24);
  // camera (forward +X face)
  translate([head_d/2-2,0,head_d/4]) color("black") cube([6,24,18], center=true);
  // neck (inserts into torso)
  translate([0,0,-head_d/2-10]) color("gray") cylinder(r=20,h=20,center=true,$fn=20);
}

head();
