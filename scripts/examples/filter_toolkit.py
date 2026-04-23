"""
03_filter_toolkit.py - 函数：滤波工具箱
在 Python Console 中调用:
    filter_toolkit.downsample("cloud_name", 0.1)
    filter_toolkit.remove_outliers("cloud_name")
    filter_toolkit.crop_z("cloud_name", 0.0, 10.0)
"""

import ct

def downsample(name, voxel_size=0.1):
    """体素降采样"""
    cloud = ct.get_cloud(name)
    if cloud is None:
        ct.printE(f"Cloud '{name}' not found!")
        return
    ct.printI(f"Downsampling '{name}' (voxel={voxel_size})...")
    result = cloud.voxel_down_sample(voxel_size, voxel_size, voxel_size)
    ct.printI(f"  {cloud.size()} -> {result.size()} points")

def remove_outliers(name, k=30, stddev=2.0):
    """统计离群点移除"""
    cloud = ct.get_cloud(name)
    if cloud is None:
        ct.printE(f"Cloud '{name}' not found!")
        return
    ct.printI(f"Removing outliers (k={k}, stddev={stddev})...")
    result = cloud.remove_outliers(k, stddev)
    ct.printI(f"  {cloud.size()} -> {result.size()} points")

def crop_z(name, z_min, z_max):
    """Z 轴范围裁剪"""
    bbox = ct.get_cloud(name).bounding_box()
    ct.printI(f"Cropping '{name}' Z=[{z_min}, {z_max}]...")
    result = ct.get_cloud(name).crop_by_box(
        bbox["cx"] - 9999, bbox["cy"] - 9999, z_min,
        bbox["cx"] + 9999, bbox["cy"] + 9999, z_max
    )
    ct.printI(f"  -> {result.size()} points")

def batch_process(name):
    """一键批处理：降采样 -> 去噪"""
    cloud = ct.get_cloud(name)
    if cloud is None:
        ct.printE(f"Cloud '{name}' not found!")
        return
    ct.printI(f"Batch processing '{name}'...")
    ct.printI(f"  Original: {cloud.size()} points")
    step1 = cloud.voxel_down_sample(0.05, 0.05, 0.05)
    ct.printI(f"  After downsample: {step1.size()} points")
    step2 = step1.remove_outliers(30, 2.0)
    ct.printI(f"  After outlier removal: {step2.size()} points")
    ct.zoom_to_bounds()
    ct.printI("Done!")
