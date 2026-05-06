import FreeCAD as App
import Part

# Create a new document
doc = App.newDocument("HLK_LD2451_Complete_Enclosure")

# --- Shared Design Parameters ---
# Optimized for 0.6mm nozzle (4 wall loops = 2.4mm)
wall_thickness = 2.4 
clearance = 0.5 

# Module dimensions from documentation
board_length = 70.0 #[cite: 1]
board_width = 35.0 #[cite: 1]
hole_inset = 2.0 #[cite: 1]

# Calculated Shared Dimensions
outer_length = board_length + (2 * clearance) + (2 * wall_thickness)
outer_width = board_width + (2 * clearance) + (2 * wall_thickness)

# ==========================================
# 1. GENERATE THE BOTTOM SHELL
# ==========================================
internal_depth = 38.0 
standoff_height = 32.0 # For USB-TTL adapter clearance
outer_height = internal_depth + wall_thickness

# Main body and cavity
outer_box = Part.makeBox(outer_length, outer_width, outer_height)
inner_box = Part.makeBox(board_length + 2*clearance, board_width + 2*clearance, internal_depth)
inner_box.translate(App.Vector(wall_thickness, wall_thickness, wall_thickness))
shell = outer_box.cut(inner_box)

# USB-A Cutout
usb_width = 14.0
usb_height = 7.0
usb_cutout = Part.makeBox(wall_thickness * 3, usb_width, usb_height)
usb_y = (outer_width - usb_width) / 2
usb_z = wall_thickness + 2.0 # 2mm offset from the inner floor
usb_cutout.translate(App.Vector(-wall_thickness, usb_y, usb_z))
shell = shell.cut(usb_cutout)

# Standoffs
standoff_radius = 2.5
hole_radius = 0.8
x1 = wall_thickness + clearance + hole_inset
x2 = wall_thickness + clearance + board_length - hole_inset
y1 = wall_thickness + clearance + hole_inset
y2 = wall_thickness + clearance + board_width - hole_inset

hole_centers = [
    App.Vector(x1, y1, wall_thickness),
    App.Vector(x2, y1, wall_thickness),
    App.Vector(x1, y2, wall_thickness),
    App.Vector(x2, y2, wall_thickness)
]

for center in hole_centers:
    standoff = Part.makeCylinder(standoff_radius, standoff_height)
    standoff.translate(center)
    screw_hole = Part.makeCylinder(hole_radius, standoff_height)
    screw_hole.translate(center)
    standoff_hollow = standoff.cut(screw_hole)
    shell = shell.fuse(standoff_hollow)


# ==========================================
# 2. GENERATE THE LID
# ==========================================
lid_thickness = 1.2 # Thin face to allow 24GHz radar transmission
lip_height = 2.4    
lip_tolerance = 0.2 

# Main cover flat face
cover_base = Part.makeBox(outer_length, outer_width, lid_thickness)

# Inner friction lip
lip_length = (board_length + 2*clearance) - (lip_tolerance * 2)
lip_width = (board_width + 2*clearance) - (lip_tolerance * 2)
lip_box = Part.makeBox(lip_length, lip_width, lip_height)

lip_x = wall_thickness + lip_tolerance
lip_y = wall_thickness + lip_tolerance
lip_box.translate(App.Vector(lip_x, lip_y, lid_thickness))

# Fuse lid components
lid = cover_base.fuse(lip_box)

# Move the lid 15mm away from the shell along the Y-axis so they print side-by-side
lid.translate(App.Vector(0, outer_width + 15.0, 0))


# ==========================================
# 3. DISPLAY IN FREECAD
# ==========================================
Part.show(shell, "Bottom_Shell")
Part.show(lid, "Friction_Lid")

doc.recompute()
App.Gui.SendMsgToActiveView("ViewFit")