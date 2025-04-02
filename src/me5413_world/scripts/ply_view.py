import open3d as o3d
import sys

def view_ply(file_path):
    print(f"正在读取文件: {file_path}")
    try:
        # 读取点云
        pcd = o3d.io.read_point_cloud(file_path)
        
        # 打印点云基本信息
        print(f"点云数据包含 {len(pcd.points)} 个点")
        
        # 如果点云太大，进行下采样
        if len(pcd.points) > 1000000:
            print("点云过大，进行下采样...")
            downpcd = pcd.voxel_down_sample(voxel_size=0.05)
            print(f"下采样后包含 {len(downpcd.points)} 个点")
            # 可视化
            print("正在打开可视化窗口...")
            o3d.visualization.draw_geometries([downpcd])
        else:
            # 可视化
            print("正在打开可视化窗口...")
            o3d.visualization.draw_geometries([pcd])
            
    except Exception as e:
        print(f"出错了: {e}")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        file_path = sys.argv[1]
        view_ply(file_path)
    else:
        print("请提供PLY文件路径")
        print("用法: python view_ply.py 文件路径.ply")
