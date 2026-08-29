// ori_leg.scad — one canonical leg (hip-yaw / hip-pitch / knee / ankle / foot)
// Mirrors CAD/Legs/master_leg.py. Readable summary, not the manufactured part.
include <ori_params.scad>

module htd45h() {
  color("silver") cube([servo_l, servo_w, servo_h], center=true);
}

module upper_link() {
  color("darkgreen") {
    cube([link_len, link_w, link_t], center=true);
    cylinder(r=bearing_od/2, h=link_t+6, center=true, $fn=24);
  }
}

module lower_link() {
  color("green") cube([link_len, link_w, link_t], center=true);
}

module foot() {
  color("orange") {
    cube([link_w, link_w, foot_h], center=true);
    translate([0,0,-foot_h/2-3]) cylinder(r=link_w*0.45, h=6, center=true, $fn=20);
  }
}

// Canonical leg: hip at origin, links run +X, foot at far end.
module leg_canonical() {
  upper_link();
  translate([link_len,0,0]) rotate([0,0,90]) htd45h();   // knee servo
  translate([link_len,0,0]) {
    lower_link();
    translate([link_len/2,0,-foot_h/2]) foot();
  }
}

leg_canonical();
