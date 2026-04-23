"""
02_cloud_info.py - 函数：查询点云信息
在 Python Console 中调用:
    cloud_info.summary()       # 打印第一个点云的摘要
    cloud_info.summary("xxx")  # 打印指定名称的点云摘要
    cloud_info.list_all()      # 列出所有点云
"""

import ct

def summary(name=None):
    if name is None:
        names = ct.get_all_cloud_names()
        if not names:
            ct.printE("No point cloud loaded!")
            return
        name = names[0]

    cloud = ct.get_cloud(name)
    if cloud is None:
        ct.printE(f"Cloud '{name}' not found!")
        return

    bbox = cloud.bounding_box()
    ct.printI(f"Cloud: {cloud.name()}")
    ct.printI(f"  Points: {cloud.size()}")
    ct.printI(f"  Has colors: {cloud.has_colors()}")
    ct.printI(f"  Has normals: {cloud.has_normals()}")
    ct.printI(f"  Center: ({bbox['cx']:.3f}, {bbox['cy']:.3f}, {bbox['cz']:.3f})")
    ct.printI(f"  Size: {bbox['width']:.3f} x {bbox['height']:.3f} x {bbox['depth']:.3f}")
    fields = cloud.get_scalar_field_names()
    if fields:
        ct.printI(f"  Scalar fields: {', '.join(fields)}")

def list_all():
    names = ct.get_all_cloud_names()
    if not names:
        ct.printI("No point clouds in scene.")
    else:
        ct.printI(f"Loaded clouds ({len(names)}):")
        for n in names:
            c = ct.get_cloud(n)
            ct.printI(f"  {n} - {c.size()} pts")
