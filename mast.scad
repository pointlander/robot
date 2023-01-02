difference() {
    cube([50, 20, 30], center=false);
    translate([5, 5, -10]) cube([40, 3, 70], center=false);
    translate([7, -1, -10]) cube([36, 12, 70], center=false);
    translate([25-17/2, 13, -10]) cube([17, 3, 70], center=false);
    translate([25, 1, 6]) rotate([-90, 0, 0]) cylinder(20, 3, 3);
}
