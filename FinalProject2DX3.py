

#Modify the following line with your own serial port details
#   Currently set COM3 as serial port at 115.2kbps 8N1
#   Refer to PySerial API for other options.  One option to consider is
#   the "timeout" - allowing the program to proceed if after a defined
#   timeout period.  The default = 0, which means wait forever.
import numpy as np
import open3d as o3d
import serial
import math

z_distance_factor = 10
show_lines = True

f = open("tof_radar.xyz", "w")
s = serial.Serial('COM4', 115200, timeout = 1000)

print("Opening: " + s.name)

# reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()

# wait for user's signal to start the program
input("Press Enter to start communication...")
# send the character 's' to MCU via UART
# This will signal MCU to start the transmission
s.write('s'.encode())
# recieve 10 measurements from UART of MCU
i = 0
j = 0
while True:
    x = s.readline()
    data = x.decode().strip()  # removes any leading/trailing whitespace/newlines
    try:
        distance = float(data)
        print("distance: ", distance, "  angle: ", j*11.25)
        x_distance = math.cos(j*0.19634954) * distance
        y_distance = math.sin(j*0.19634954) * distance
        f.write('{0} {1} {2}\n'.format(x_distance, y_distance, i))
        j += 1;
    except:
        if data == "i":
            i += z_distance_factor
            j = 0
        elif data == "q":
            break
        else:
            pass
    
    
    
    
f.close()
# the encode() and decode() function are needed to convert string to bytes
# because pyserial library functions work with type "bytes"


#close the port
print("Closing: " + s.name)
pcd = o3d.io.read_point_cloud("tof_radar.xyz", format="xyz")
if show_lines:

    points = np.asarray(pcd.points)

    points_per_plane = 32  
    total_points = points.shape[0]
    num_planes = total_points // points_per_plane
    lines = []


    for plane in range(num_planes):
        base = plane * points_per_plane
        for j in range(points_per_plane):
            next_j = (j + 1) % points_per_plane 
            lines.append([base + j, base + next_j])


    for plane in range(num_planes - 1):
        base_current = plane * points_per_plane
        base_next = (plane + 1) * points_per_plane
        for j in range(points_per_plane):
            lines.append([base_current + j, base_next + j])

    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines)
    )
    o3d.visualization.draw_geometries([line_set])

else:
    print(np.asarray(pcd.points))

    o3d.visualization.draw_geometries([pcd])

s.close()
