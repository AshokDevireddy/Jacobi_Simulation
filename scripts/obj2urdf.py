import os
import sys

"""
Simple script to wrap an .obj file into an .urdf file.
"""

# Set your filename here
filename = 'objects/box10/toilet_paper_box.obj'


def split_filename(string):
    path_to_file = os.path.dirname(string)
    filename, extension = os.path.splitext(os.path.basename(string))
    return path_to_file, filename, extension


def check_input(filename):
    _, _, ext = split_filename(filename)
    if ext != ".obj":
        print("Incorrect extension (<{}> instead of <.obj>)".format(ext))
        sys.exit()
    if not os.path.exists(filename):
        print("The file <{}> does not exist".format(filename))
        sys.exit()


def generate_output_name(filename):
    path_to_file, filename, extension = split_filename(filename)
    if path_to_file == "":
        new_name = filename + ".urdf"
    else:
        new_name = path_to_file + "/" + filename + ".urdf"
    return new_name


def check_output_file(output_name):
    if os.path.exists(output_name):
        print("Warning: <{}> already exists. Do you want to continue and overwrite it? [y, n] > ".format(output_name), end="")
        ans = input().lower()
        if ans not in ["y", "yes"]:
            sys.exit()


def write_urdf_text(filename):
    output_name = generate_output_name(filename)
    _, name, _ = split_filename(filename)
    print("Creation of <{}>...".format(output_name), end="")
    with open(output_name, "w") as f:
        text = """<?xml version="1.0" ?>
<robot name="{}.urdf">
  <link name="baseLink">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
       <mass value="0.0"/>
       <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="{}.obj" scale="1.0 1.0 1.0"/>
      </geometry>
      <material name="white">
       <color rgba="1 1 1 1"/>
     </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="{}.obj" scale="1.0 1.0 1.0"/>
        <!-- You could also specify the collision (for the {}) with a "box" tag: -->
        <!-- <box size=".06 .06 .06"/> -->
      </geometry>
    </collision>
  </link>
</robot>
        """.format(name, name, name, name)
        f.write(text)
        print(" done")


if __name__ == "__main__":
    check_input(filename)
    new_full_filename = generate_output_name(filename)
    check_output_file(new_full_filename)
    write_urdf_text(filename)
