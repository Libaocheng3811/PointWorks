"""
test_scene_analyzer.py - 测试 scene_analyzer.py

数据条件: 需要加载至少1个点云文件
建议: 加载2-3个点云以充分测试场景分析功能
"""

import ct
import scene_analyzer

ct.printI("=== Test: scene_analyzer ===")
ct.printI("")

# 测试1: 创建分析器并打印报告
ct.printI("[1/4] SceneAnalyzer.report():")
sa = scene_analyzer.SceneAnalyzer()
sa.report()
ct.printI("")

# 测试2: 给每个点云添加名称标签
ct.printI("[2/4] label_clouds():")
sa.label_clouds()
ct.printI("")

# 测试3: 给每个点云添加包围盒箭头
ct.printI("[3/4] add_bounding_box():")
sa.add_bounding_box()
ct.printI("")

# 测试4: 切换暗色主题
ct.printI("[4/4] set_dark_theme():")
sa.set_dark_theme()

ct.printI("")
ct.printI("=== PASSED ===")
