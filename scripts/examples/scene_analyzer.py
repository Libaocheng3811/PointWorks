"""
04_scene_analyzer.py - 类：场景分析器
演示面向对象的脚本组织方式

在 Python Console 中调用:
    import scene_analyzer
    sa = scene_analyzer.SceneAnalyzer()
    sa.report()              # 打印场景报告
    sa.add_bounding_box()    # 给所有点云添加包围盒
    sa.set_dark_theme()      # 切换暗色主题
    sa.label_clouds()        # 在每个点云上方添加标签
"""

import ct


class SceneAnalyzer:
    """场景分析器：通过类组织相关功能"""

    def __init__(self):
        self._cloud_names = ct.get_all_cloud_names()

    def _get_cloud(self, name):
        cloud = ct.get_cloud(name)
        if cloud is None:
            ct.printE(f"Cloud '{name}' not found!")
        return cloud

    def refresh(self):
        """刷新点云列表"""
        self._cloud_names = ct.get_all_cloud_names()

    def report(self):
        """打印场景摘要报告"""
        self.refresh()
        if not self._cloud_names:
            ct.printI("Scene is empty.")
            return
        ct.printI(f"=== Scene Report ({len(self._cloud_names)} clouds) ===")
        total = 0
        for name in self._cloud_names:
            cloud = self._get_cloud(name)
            if cloud is None:
                continue
            total += cloud.size()
            bbox = cloud.bounding_box()
            ct.printI(f"  [{name}]")
            ct.printI(f"    Points: {cloud.size()}  Colors: {cloud.has_colors()}")
            ct.printI(f"    Center: ({bbox['cx']:.2f}, {bbox['cy']:.2f}, {bbox['cz']:.2f})")
        ct.printI(f"Total points: {total}")

    def add_bounding_box(self):
        """给每个点云添加包围盒箭头"""
        self.refresh()
        for name in self._cloud_names:
            cloud = self._get_cloud(name)
            if cloud is None:
                continue
            bbox = cloud.bounding_box()
            x0 = bbox["cx"] - bbox["width"] / 2
            y0 = bbox["cy"] - bbox["height"] / 2
            z0 = bbox["cz"] - bbox["depth"] / 2
            x1 = bbox["cx"] + bbox["width"] / 2
            y1 = bbox["cy"] + bbox["height"] / 2
            z1 = bbox["cz"] + bbox["depth"] / 2
            ct.add_arrow(x0, y0, z0, x1, y0, z0, r=1, g=0, b=0, id=f"{name}_x")
            ct.add_arrow(x0, y0, z0, x0, y1, z0, r=0, g=1, b=0, id=f"{name}_y")
            ct.add_arrow(x0, y0, z0, x0, y0, z1, r=0, g=0, b=1, id=f"{name}_z")
            ct.printI(f"  Added bbox arrows for '{name}'")
        ct.zoom_to_bounds()

    def set_dark_theme(self):
        """切换暗色主题"""
        ct.set_background_color(0.12, 0.12, 0.12)
        ct.show_axes(True)
        ct.show_fps(True)

    def label_clouds(self):
        """在每个点云中心上方添加名称标签"""
        self.refresh()
        for name in self._cloud_names:
            cloud = self._get_cloud(name)
            if cloud is None:
                continue
            bbox = cloud.bounding_box()
            ct.add_3d_label(
                f"{name}\n{cloud.size()} pts",
                bbox["cx"], bbox["cy"],
                bbox["cz"] + bbox["depth"] / 2 + 0.5,
                id=f"label_{name}"
            )
            ct.printI(f"  Labeled '{name}'")
