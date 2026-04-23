"""
test_cloud_info.py - 测试 cloud_info.py

数据条件: 需要加载至少1个点云文件
建议: 加载一个 LAS/LAZ/PCD 格式的点云
"""

import ct
import cloud_info

ct.printI("=== Test: cloud_info ===")
ct.printI("")

# 测试1: 列出所有点云
ct.printI("[1/3] list_all():")
cloud_info.list_all()
ct.printI("")

# 测试2: 打印第一个点云的摘要
ct.printI("[2/3] summary() - auto:")
cloud_info.summary()
ct.printI("")

# 测试3: 如果有多个点云，打印指定的
names = ct.get_all_cloud_names()
if len(names) >= 2:
    ct.printI(f"[3/3] summary('{names[-1]}') - by name:")
    cloud_info.summary(names[-1])
else:
    ct.printI("[3/3] Skipped - need >= 2 clouds for this test.")

ct.printI("")
ct.printI("=== PASSED ===")
