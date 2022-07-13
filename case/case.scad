$fn=100;
module tube(d=15,w=30,h=70) {
    translate([0,(w-d)/2,0])
    cylinder(d=d,h=h);
    translate([0,-(w-d)/2,0])
    cylinder(d=d,h=h);
    translate([-d/2,-(w-d)/2,0])
    cube([d,(w-d),h]);
}
module tube2(d=10,w=33,h=70,t=16) {
   
    translate([(t-d)/2,(w-d)/2,0])
    cylinder(d=d,h=h);
    translate([-(t-d)/2,(w-d)/2,0])
    cylinder(d=d,h=h);
    translate([-(t-d)/2,-(w-d)/2,0])
    cylinder(d=d,h=h);
    translate([(t-d)/2,-(w-d)/2,0])
    cylinder(d=d,h=h);
    
    translate([-(t-d)/2,-(w)/2,0])
    cube([(t-d),w,h]);
    translate([-(t)/2,-(w-d)/2,0])
    cube([t,(w-d),h]);
    
}
module board() {
    translate([4.5,-15,0])
    cube([2,30,67]);
    translate([0,-(30-12)/2,0])
    cylinder(d=12,h=12);
    translate([0,(30-12)/2,0])
    cylinder(d=12,h=12);
    translate([-(12)/2,-(30-12)/2,0])
    cube([12,(30-12),12]);
    
}

difference() {
    translate([0,0,0])
tube2(d=12,w=36,h=72,t=19);
    translate([0,0,10])
tube2(h=(70-12));
translate([0,0,-1])
board();
}
