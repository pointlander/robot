translate([0, 30, 0]) for (a=[0:30:30]) translate([0, a, 0]) difference() {
    cube([50, 20, 30], center=false);
    translate([5, 5, -10]) cube([40, 3, 70], center=false);
    translate([7, -1, -10]) cube([36, 12, 70], center=false);
    translate([25-17/2, 13, -10]) cube([17, 3, 70], center=false);
    translate([25, 1, 6]) rotate([-90, 0, 0]) cylinder(20, 3, 3);
    translate([50/2-5/2, 15, -10]) cube([5, 10, 70], center=false);
}

difference() {
    cube([35, 20, 28], center=false);
    translate([(35-26)/2, 5.5, -10]) cube([26, 5.5, 70], center=false);
    translate([7, -1, -10]) cube([35-2*4-2*3, 9, 70], center=false);
    translate([35/2-17/2, 13, -10]) cube([17, 3, 70], center=false);
    translate([(35-26)/2, -1, 28 - 3]) cube([4, 30, 4]);
    translate([35/2-5/2, 15, -10]) cube([5, 10, 70], center=false);
}