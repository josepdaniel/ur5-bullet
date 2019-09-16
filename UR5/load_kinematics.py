"""
Usage: 

python3 load_kinematics path/to/urp/file 

Reads a urp file and extracts the kinematic model. Generates a csv file with columns: 

alpha | a | deltaTheta | d 

"""

import gzip 
import xml.etree.ElementTree as ET
import sys

if len(sys.argv) < 2:
    print("Usage: 'python3 load_kinematics path/to/urp/file'") 
    sys.exit()

elif sys.argv[1] == "-h": 
    print("Usage: 'python3 load_kinematics path/to/urp/file'") 
    sys.exit()


urpfile = sys.argv[1]
urpxml = gzip.open(urpfile, 'rb')
data = ET.fromstring(urpxml.read())

kinematics = data.find('kinematics')

outfile = open("dh_kinematics.csv", 'w+')

for param in kinematics:
    if param.tag == 'alpha':
        value = param.attrib['value']
        dh_alpha = [float(v) for v in value.replace(' ','').split(',')]

    if param.tag == 'a':
        value = param.attrib['value']
        dh_a = [float(v)*1000.0 for v in value.replace(' ','').split(',')]

    if param.tag == 'deltaTheta':
        value = param.attrib['value']
        dh_dTheta = [float(v) for v in value.replace(' ','').split(',')]

    if param.tag == 'd':
        value = param.attrib['value']
        dh_d = [float(v)*1000.0 for v in value.replace(' ','').split(',')]


line = "{:^15s} | {:^15s} | {:^15s} | {:^15s}".format("alpha", "a", "delta_theta", "d")

print(line)
print("-"*70)
line = "alpha, a, delta_theta, d\n"
outfile.write(line)


for i in range(0, len(dh_alpha)):
    print("{:<15f} | {:<15f} | {:<15f} | {:<15f}".format(dh_alpha[i], dh_a[i], dh_dTheta[i], dh_d[i]))
    outfile.write("{}, {}, {}, {}\n".format(dh_alpha[i], dh_a[i], dh_dTheta[i], dh_d[i]))
