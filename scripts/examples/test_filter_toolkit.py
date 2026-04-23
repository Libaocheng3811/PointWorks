"""
test_filter_toolkit.py - 测试 filter_toolkit.py

数据条件: 需要加载至少1个点云文件
建议: 加载一个包含较多点数的点云（>10000点），以便观察滤波效果
"""

import ct
import filter_toolkit

ct.printI("=== Test: filter_toolkit ===")
ct.printI("")

names = ct.get_all_cloud_names()
if not names:
    ct.printE("ERROR: No cloud loaded! Load a point cloud first.")
    ct.printI("=== FAILED ===")
else:
    name = names[0]
    ct.printI(f"Using cloud: {name} ({ct.get_cloud(name).size()} pts)")
    ct.printI("")

    ct.printI("[1/4] downsample(0.5):")
    filter_toolkit.downsample(name, 0.5)
    ct.printI("")

    ct.printI("[2/4] remove_outliers(k=20, stddev=1.5):")
    filter_toolkit.remove_outliers(name, k=20, stddev=1.5)
    ct.printI("")

    cloud = ct.get_cloud(name)
    bbox = cloud.bounding_box()
    z_min = bbox["cz"]
    z_max = bbox["cz"] + bbox["depth"] / 4
    ct.printI(f"[3/4] crop_z({z_min:.2f}, {z_max:.2f}):")
    filter_toolkit.crop_z(name, z_min, z_max)
    ct.printI("")

    ct.printI("[4/4] batch_process():")
    filter_toolkit.batch_process(name)

    ct.printI("")
    ct.printI("=== PASSED ===")
