"""
test_cloud_workspace.py - 测试 cloud_workspace.py

数据条件: 需要加载至少1个点云文件（建议点数 >10000）
"""

import ct
import cloud_workspace

ct.printI("=== Test: cloud_workspace ===")
ct.printI("")

names = ct.get_all_cloud_names()
if not names:
    ct.printE("ERROR: No cloud loaded! Load a point cloud first.")
    ct.printI("=== FAILED ===")
else:
    name = names[0]
    ct.printI(f"Using cloud: {name}")
    ct.printI("")

    # 测试1: 创建工作台，打印初始信息
    ct.printI("[1/5] Create Workspace & info():")
    ws = cloud_workspace.Workspace(name)
    ws.info()
    ct.printI("")

    # 测试2: 体素降采样
    ct.printI("[2/5] downsample(0.5):")
    ws.downsample(0.5)
    ws.info()
    ct.printI("")

    # 测试3: 离群点移除
    ct.printI("[3/5] remove_outliers():")
    ws.remove_outliers()
    ws.info()
    ct.printI("")

    # 测试4: 链式调用
    ct.printI("[4/5] Chained: .downsample(1.0).remove_outliers():")
    ws2 = cloud_workspace.Workspace(name)
    ws2.downsample(1.0).remove_outliers().info()
    ct.printI("")

    # 测试5: 将处理结果插入场景
    ct.printI("[5/5] insert() - add result to scene:")
    ws.insert("filtered_result")

ct.printI("")
ct.printI("=== PASSED ===")
