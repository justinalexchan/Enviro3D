#Justin Chan 
import numpy as np
import open3d as o3d
import trimesh
import matplotlib.pyplot as plt
import matplotlib.tri as mtri

# This is the open3D visualization method which creates 3D contours based on the given point cloud. 
# Open3D requires lots of Graphical Processing power so without a GPU, it will crash.
# The alternative, MeshLab, was used instead.

if __name__ == "__main__":

    pcd = o3d.io.read_point_cloud("3d.xyz", format='xyz')
    print(pcd)

    print(np.asarray(pcd.points))
    #o3d.visualization.draw_geometries([pcd])


    pts = []
    x= 0 
    for i in range(2496):
        pts.append(i)

    
    lines = [] # store lines in a list
    count = 0
    # Use loops to connect points on yz plane
    for x in range(39):
        i = x * 63
        
        while i%63 != 0 or count == 0:
            count = 1
            if i%63 == 0:
                lines.append([pts[i], pts[x * 63]])
            else:
                lines.append([pts[i], pts[i+1]])
            i += 1

        count = 0

    
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)), lines=o3d.utility.Vector2iVector(lines))
    
    o3d.visualization.draw_geometries([line_set])