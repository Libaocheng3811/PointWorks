"""
05_cloud_workspace.py - 类进阶：点云工作台
演示更完整的面向对象设计，支持链式调用

在 Python Console 中调用:
    import cloud_workspace
    ws = cloud_workspace.Workspace("my_cloud_name")
    ws.info().downsample(0.1).remove_outliers().info()
    ws.save("output.las")
"""

import ct


class Workspace:
    """点云工作台：对一个点云执行链式操作"""

    def __init__(self, name):
        self._name = name
        self._cloud = ct.get_cloud(name)
        if self._cloud is None:
            ct.printE(f"Cloud '{name}' not found!")
        else:
            ct.printI(f"Workspace: '{name}' ({self._cloud.size()} pts)")

    def info(self):
        """打印当前点云信息"""
        if self._cloud is None:
            return self
        bbox = self._cloud.bounding_box()
        ct.printI(f"[{self._name}] {self._cloud.size()} pts, "
                   f"center=({bbox['cx']:.2f}, {bbox['cy']:.2f}, {bbox['cz']:.2f})")
        return self

    def downsample(self, voxel_size):
        """体素降采样"""
        if self._cloud is None:
            return self
        ct.printI(f"  Downsample voxel={voxel_size}: {self._cloud.size()}", end=" -> ")
        self._cloud = self._cloud.voxel_down_sample(voxel_size, voxel_size, voxel_size)
        ct.printI(f"{self._cloud.size()}")
        return self

    def remove_outliers(self, k=30, stddev=2.0):
        """统计离群点移除"""
        if self._cloud is None:
            return self
        ct.printI(f"  Outlier removal: {self._cloud.size()}", end=" -> ")
        self._cloud = self._cloud.remove_outliers(k, stddev)
        ct.printI(f"{self._cloud.size()}")
        return self

    def estimate_normals(self, k=30):
        """估计法线"""
        if self._cloud is None:
            return self
        ct.printI(f"  Estimating normals (k={k})...")
        self._cloud = self._cloud.estimate_normals(k)
        return self

    def save(self, filepath):
        """保存点云"""
        if self._cloud is None:
            return self
        ct.printI(f"  Saving to {filepath}...")
        ct.save_cloud(self._name, filepath, binary=True)
        return self

    def insert(self, new_name=""):
        """将结果插入场景"""
        if self._cloud is None:
            return self
        self._cloud.show(new_name)
        ct.printI(f"  Inserted as new cloud in scene.")
        return self
