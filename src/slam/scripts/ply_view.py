import open3d as o3d
import sys

def view_ply(file_path):
    print(f"Reading file: {file_path}")
    try:
        # Read point cloud
        pcd = o3d.io.read_point_cloud(file_path)
        
        # Print basic information about the point cloud
        print(f"Point cloud contains {len(pcd.points)} points")
        
        # Downsample if the point cloud is too large
        if len(pcd.points) > 1000000:
            print("Point cloud is too large, downsampling...")
            downpcd = pcd.voxel_down_sample(voxel_size=0.05)
            print(f"After downsampling, contains {len(downpcd.points)} points")
            # Visualization
            print("Opening visualization window...")
            o3d.visualization.draw_geometries([downpcd])
        else:
            # Visualization
            print("Opening visualization window...")
            o3d.visualization.draw_geometries([pcd])
            
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        file_path = sys.argv[1]
        view_ply(file_path)
    else:
        print("Please provide a PLY file path")
        print("Usage: python view_ply.py file_path.ply")