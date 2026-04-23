"""
test_hello_world.py - 测试 hello_world.py

数据条件: 无需任何点云，直接运行即可
"""

import ct
import hello_world

ct.printI("=== Test: hello_world ===")
ct.printI("")

ct.printI("[1/2] Testing ct.printI...")
ct.printI("  This is an info message.")

ct.printW("[2/2] Testing ct.printW...")
ct.printW("  This is a warning message.")

ct.printI("")
ct.printI("=== PASSED ===")
