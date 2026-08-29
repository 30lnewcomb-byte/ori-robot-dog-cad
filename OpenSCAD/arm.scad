// ori_arm.scad — 6-DOF front arm + servo-in-wrist passive gear/cam gripper.
// Mirrors CAD/Arm/arm.py. Readable summary, not the manufactured part.
include <ori_params.scad>

module gripper() {
  color("gray") cylinder(r=grip_r, h=10, center=true, $fn=24);   // coupler disc (Ø34)
  // two passive fingers
  for (s=[-1,1])
    translate([grip_r+8, 0, s*(grip_r+4)])
      color("lightgray") cube([finger_len, 6, 10], center=true);
}

module arm() {
  translate([arm_base_x,0,torso_h/2]) {
    color("darkgray") cylinder(r=14,h=arm_yaw_h,center=true,$fn=20);          // yaw
    translate([0,0,arm_yaw_h/2]) {
      rotate([0,90,0]) color("gray") cylinder(r=10,h=arm_upper,center=true,$fn=20); // shoulder->upper
      translate([arm_upper/2,0,arm_upper/2]) rotate([0,90,0])
        color("gray") cylinder(r=9,h=arm_fore,center=true,$fn=20);            // elbow->fore
      translate([arm_upper/2+arm_fore/2,0,arm_fore/2]) {
        color("slategray") cube([arm_wrist,arm_wrist,arm_wrist],center=true); // wrist block
        translate([arm_wrist/2+4,0,0]) gripper();                            // passive tool
      }
    }
  }
}

arm();
