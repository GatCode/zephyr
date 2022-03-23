#!/usr/bin/python3
import os

node_enum = "enum d_cube_nodes\n{\n"
node_enum += "    local_42 = 0x3A7E6410F8902FC1,\n"
node_enum += "    local_56 = 0xDDB146F9D7E47347,\n"

for file in os.listdir(os.getcwd() + "/logs"):
    if file.endswith(".txt"):
        node_identifier = file.split('.')[0].split('_')[1]

        f = open(os.getcwd() + "/logs/" + file)
        node_hw_identifier = f.readlines()[1].replace('\n','').split('|')[1].split(' ')[1]

        node_enum += "    remote_" + node_identifier + " = 0x" + node_hw_identifier + ",\n"

node_enum += "};"

print(node_enum)