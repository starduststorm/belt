epsilon = 0.001;

$fs = 0.2; // min angle for curved shapes
$fa = 3; // min segment/fragment size

RAD = 180 / PI;
DEG = 1;

board_thickness = 0.8;
shell_thickness = 1.4;

top_case_height = 8.2+shell_thickness; // rough measure
bottom_case_height = 2 /*rough for bottom components+pins*/ + shell_thickness + board_thickness;

board_tolerance = 1;

start_position = [119.38, 68.58, 0] /*pcb*/ - [board_tolerance/2,board_tolerance/2,0];

board_x_size = 167.64 - 119.38 /*pcb units*/ + board_tolerance;
board_y_size = 139.065 - 68.58 /*pcb units*/ + board_tolerance;
//board_size = [board_x_size, board_y_size, board_thickness];

screw_radius = 0.96 /*measured*/ + 0.4 /*tolerance*/;
screw_bump_inner_radius = 4 /*pcb*/ + 0.2 /*tolerance*/;
screw_bump_outer_radius = screw_bump_inner_radius + shell_thickness;

screw_positions = [ // pcb units
    [125.73, 69.215, 0] - start_position,
    [161.29, 69.215, 0] - start_position,
    [125.73, 138.43, 0] - start_position,
    [161.29, 138.43, 0] - start_position,
];

module arc(arcstart, arclen, height, r1, r2=0) {
    rotate(arcstart, [0,0,1]) rotate_extrude(angle=arclen) polygon([[r1,0],[r2,0],[r2,height],[r1,height]]);
}

module screw_shell(height) {
    // abandoned attempt to follow precise board curve. 
//    arc1_offset = [125.73, -69.215, 0] - [121.92, -66.04, 0];
//    arc2_offset = [125.73, -69.215, 0] - [129.54, -66.04, 0];
//    
//    arc_radius = 2.54;
    
    //            #translate(arc1_offset) arc(PI/2*RAD, PI/4*RAD, height, arc_radius-board_tolerance/2, arc_radius-shell_thickness-board_tolerance/2);

    
    union() for (i = [0:3]) {
        translate(screw_positions[i]) {
            translate([0,0, height/2]) cylinder(h = height, r = screw_bump_outer_radius, center = true);
        }
    }
}

module screw_bump(height) {
    union() for (i = [0:3]) {
        translate(screw_positions[i] + [0,0, height/2]) {
            cylinder(h = height, r = screw_bump_inner_radius, center = true);
        }
    }
}

module screw_holes(height) {
    union() for (i = [0:3]) {
        translate(screw_positions[i] + [0,0, height/2]) {
            cylinder(h = height, r = screw_radius, center = true);
        }
    }
}

module diffscale() {
     for (i = [0 : $children-1]) {
         translate([-epsilon/2,-epsilon/2,-epsilon/2]) scale([1+epsilon,1+epsilon,1+epsilon]) children(i);
     }
}

module spiral(radius, thickness, loops, height) {
    linear_extrude(height=height) polygon(points= concat(
        [for(t = [90:360*loops]) 
            [(radius-thickness+t/90)*sin(t),(radius-thickness+t/90)*cos(t)]],
        [for(t = [360*loops:-1:90]) 
            [(radius+t/90)*sin(t),(radius+t/90)*cos(t)]]
            ));
}

module rounded_rect(size, radius, epsilon=0.001) {
    module fillet(r, h) {
        translate([r/2, r/2, 0]) difference() {
            cube([r + epsilon, r + epsilon, h], center = true);
            translate([r/2, r/2, 0])
                cylinder(r = r, h = h + 1, center = true);
        }
    }
    difference() {
        cube(size);
        translate([0,0,size.z/2]) fillet(radius,size.z+0.001);
        translate([size.x,0,size.z/2]) rotate(PI/2*RAD, [0,0,1]) fillet(radius, size.z+epsilon);
        translate([0,size.y,size.z/2]) rotate(-PI/2*RAD, [0,0,1]) fillet(radius, size.z+epsilon);
        translate([size.x,size.y,size.z/2]) rotate(PI*RAD, [0,0,1]) fillet(radius, size.z+epsilon);
    }
}


power_usb_y_range = [85.8, 126.7] /*pcb*/- [start_position.y, start_position.y];
power_usb_height = 3.5 /*measured*/ - board_thickness + 0.4 /*tolerance*/;

power_switch_cutout_position = [121.3, 75.4, 0] /*pcb*/ - start_position;
power_switch_cutout_size = [3.85, 7.2, shell_thickness] /*measured*/ + [0.2, 0.6, 0.2] /*tolerance*/;

teensy_usb_size = [7.9, shell_thickness+epsilon, 3.4];
teensy_usb_position = [134.6 /*pcb*/, start_position.y-shell_thickness, 5.4-teensy_usb_size.z] - start_position;

dial_1_position = [149.225 + 10.16, 83.185, 0] /*pcb*/ - start_position;
dial_2_position = [149.225 + 10.16, 98.298, 0] /*pcb*/ - start_position;
dial_radius = 7.2 /*measured*/ + 0.5 /*tolerance*/;

wire_cutout_position_y = 132.75 /*pcb*/ - start_position.y;
wire_cutout_radius = 2.75 /*pcb*/ + 0.4 /*extra for messy cables*/;

button_column_radius = 2.2;
button_column_height = top_case_height - (6.0 /*measured*/ - board_thickness);
button_column_extra_height = 2; // I'm printing the top case upside down so this is fine, just need to flip it in the slicer
button_position = [153.3, 72.4, 0] /*pcb*/ - start_position;

board_corner_radius = 2.54 /*pcb*/;
shell_corner_radius = board_corner_radius+shell_thickness /*pcb*/;

translate([board_x_size*-1.1, 0, 0]) union() { 
    difference() {
        union() {
            // top (upside down) basic shape
            translate([-shell_thickness, -shell_thickness, 0])
                rounded_rect([board_x_size + 2*shell_thickness, board_y_size + 2*shell_thickness, top_case_height], radius=shell_corner_radius);
            screw_shell(top_case_height);
        }
        union() diffscale() {
            // hollow shell
            translate([0, 0, shell_thickness]) rounded_rect([board_x_size, board_y_size, top_case_height], radius=board_corner_radius);
            
            // power usb cutout
            translate([-shell_thickness, power_usb_y_range[0], top_case_height-power_usb_height])
                cube([shell_thickness+epsilon, power_usb_y_range[1] - power_usb_y_range[0], power_usb_height]);
    
            
            // power switch cutout
            translate(power_switch_cutout_position) cube(power_switch_cutout_size);
            
            // teensy usb cutout
            translate(teensy_usb_position) cube(teensy_usb_size);
            
            // microphone cone?
            
            
            // dials cutout
            translate(dial_1_position + [0,0,top_case_height/2]) cylinder(h = top_case_height, r=dial_radius, center = true);
            translate(dial_2_position + [0,0,top_case_height/2]) cylinder(h = top_case_height, r=dial_radius, center = true);
            
            // wire cutout
            translate([-shell_thickness/2, wire_cutout_position_y, top_case_height]) 
                rotate(PI/2*RAD, [0,1,0]) 
                    cylinder(h=shell_thickness+epsilon, r=wire_cutout_radius, center=true);
            translate([board_x_size + shell_thickness/2, wire_cutout_position_y, top_case_height]) 
                rotate(PI/2*RAD, [0,1,0]) 
                    cylinder(h=shell_thickness+epsilon, r=wire_cutout_radius, center=true);
            
            // button
            translate(button_position + [0,0,shell_thickness/2]) cylinder(h=shell_thickness+epsilon, r=4, center=true);
            
            // screws
            screw_holes(shell_thickness + epsilon);
            translate([0,0,shell_thickness]) screw_bump(top_case_height - shell_thickness);
            
            // lettering
            fonts = [   "santa fe let",
                        "blackmoor let",
                        "Apple Chancery",
                        "synchro let",
                        "herculanum",
                        "jazz let",
                    ];
            translate([board_x_size/3-3, board_y_size/4+2, 0.2]) rotate(PI*RAD, [0,1,0]) {
                for (i = [0:len(fonts)-1]) {
                    translate([-2*i,9*i,0]) linear_extrude(height=0.2, convexity=4)
                        text("motion", 
                             size=8,
                             font=fonts[i],
                             halign="center",
                             valign="center", 
                             spacing=1.1,
                             $fn=1);
                }
            }
        }
    }
    
    // button column
    translate(button_position) {
        translate([0,0,button_column_height/2]) cylinder(h=button_column_height, r=button_column_radius, center=true);
        if (button_column_extra_height) 
            cylinder(h=2*button_column_extra_height, r1=button_column_radius + button_column_extra_height/4, r2=1.5, center=true);
        translate([-0.1,0.4,0]) spiral(1, 1.3, 1, 0.5);
        translate([0.1,-0.4,0]) rotate(180, [0,0,1]) spiral(1, 1.3, 0.96, 0.5);
    }
}

translate([board_x_size*0.0, 0, 0]) difference() {
    union() {
        // bottom basic shape
        translate([-shell_thickness, -shell_thickness, 0])
            rounded_rect([board_x_size + 2*shell_thickness, board_y_size + 2*shell_thickness, bottom_case_height], radius=shell_corner_radius);
        // screws
        screw_shell(bottom_case_height);
    }
    diffscale() union() {
        // hollow out
        translate([0, 0, shell_thickness]) rounded_rect([board_x_size, board_y_size, bottom_case_height], radius=board_corner_radius);
        // screws
        screw_holes(shell_thickness + epsilon);
        translate([0,0,shell_thickness]) screw_bump(bottom_case_height - shell_thickness);
    }
}
 

