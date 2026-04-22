#include "bind_surface.h"
#include "bind_core.h"
#include "mesh_utils.h"

void registerSurfaceBindings(py::module_& m)
{
    py::class_<PyMesh>(m, "Mesh")
        .def("vertices", &PyMesh::vertices, "Mesh vertices as numpy array (N, 3) float32")
        .def("faces", &PyMesh::faces, "Mesh faces as numpy array (M, 3) int32")
        .def("num_vertices", &PyMesh::numVertices, "Number of vertices")
        .def("num_faces", &PyMesh::numFaces, "Number of faces")
        .def("show", [](PyMesh& self, const std::string& id) {
            auto mesh_ptr = self.meshPtr();
            if (!mesh_ptr) throw std::runtime_error("Mesh is empty");
            auto* bridge = ct::PythonManager::instance().bridge();
            bridge->addMesh(mesh_ptr, QString::fromStdString(id));
        }, py::arg("id"), "Display this mesh in the 3D view.");

    auto meshResultToPyMesh = [](const std::shared_ptr<pcl::PolygonMesh>& mesh,
                                  const std::string& error) -> py::object {
        if (!mesh) throw std::runtime_error(error.empty()
            ? "Surface reconstruction produced no result" : error);
        return py::cast(PyMesh(mesh));
    };

    m.def("poisson", [meshResultToPyMesh](const std::string& name,
                         int depth, int min_depth, float point_weight,
                         float scale, int solver_divide, int iso_divide,
                         float samples_per_node, bool confidence,
                         bool output_polygons, bool manifold) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        std::string err;
        auto mesh = surfacePoisson(cloud, depth, min_depth, point_weight, scale,
                                   solver_divide, iso_divide, samples_per_node,
                                   confidence, output_polygons, manifold, err);
        return meshResultToPyMesh(mesh, err);
    }, py::arg("name"),
       py::arg("depth") = 8,
       py::arg("min_depth") = 5,
       py::arg("point_weight") = 4.0f,
       py::arg("scale") = 1.1f,
       py::arg("solver_divide") = 8,
       py::arg("iso_divide") = 8,
       py::arg("samples_per_node") = 1.5f,
       py::arg("confidence") = false,
       py::arg("output_polygons") = false,
       py::arg("manifold") = false,
       "Poisson surface reconstruction. Returns ct.Mesh object.");

    m.def("greedy_triangulation", [meshResultToPyMesh](const std::string& name,
                                       double mu, int nnn, double radius,
                                       double min_angle, double max_angle, double ep,
                                       bool consistent, bool consistent_ordering) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        std::string err;
        auto mesh = surfaceGreedyTriangulation(
            cloud, mu, nnn, radius, min_angle, max_angle, ep, consistent, consistent_ordering, err);
        return meshResultToPyMesh(mesh, err);
    }, py::arg("name"),
       py::arg("mu") = 2.5,
       py::arg("nnn") = 12,
       py::arg("radius") = 0.025,
       py::arg("min_angle") = 12.0,
       py::arg("max_angle") = 150.0,
       py::arg("ep") = 180.0,
       py::arg("consistent") = false,
       py::arg("consistent_ordering") = false,
       "Greedy projection triangulation. Returns ct.Mesh object.");

    m.def("marching_cubes_hoppe", [meshResultToPyMesh](const std::string& name,
                                         float iso_level, int res_x, int res_y, int res_z,
                                         float percentage, float dist_ignore) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        std::string err;
        auto mesh = surfaceMarchingCubesHoppe(cloud, iso_level, res_x, res_y, res_z,
                                              percentage, dist_ignore, err);
        return meshResultToPyMesh(mesh, err);
    }, py::arg("name"),
       py::arg("iso_level") = 0.0f,
       py::arg("res_x") = 50,
       py::arg("res_y") = 50,
       py::arg("res_z") = 50,
       py::arg("percentage") = 10.0f,
       py::arg("dist_ignore") = 0.0f,
       "Marching Cubes Hoppe surface reconstruction. Returns ct.Mesh object.");

    m.def("convex_hull", [meshResultToPyMesh](const std::string& name,
                              bool compute_area_volume, int dimension) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        std::string err;
        auto mesh = surfaceConvexHull(cloud, compute_area_volume, dimension, err);
        return meshResultToPyMesh(mesh, err);
    }, py::arg("name"),
       py::arg("compute_area_volume") = false,
       py::arg("dimension") = 3,
       "Convex hull reconstruction. Returns ct.Mesh object.");

    m.def("concave_hull", [meshResultToPyMesh](const std::string& name,
                               double alpha, bool keep_information, int dimension) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        std::string err;
        auto mesh = surfaceConcaveHull(cloud, alpha, keep_information, dimension, err);
        return meshResultToPyMesh(mesh, err);
    }, py::arg("name"),
       py::arg("alpha") = 0.1,
       py::arg("keep_information") = false,
       py::arg("dimension") = 3,
       "Concave hull (alpha shape) reconstruction. Returns ct.Mesh object.");

    m.def("marching_cubes_rbf", [meshResultToPyMesh](const std::string& name,
                                         float iso_level, int res_x, int res_y, int res_z,
                                         float percentage, float epsilon) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        std::string err;
        auto mesh = surfaceMarchingCubesRBF(cloud, iso_level, res_x, res_y, res_z,
                                            percentage, epsilon, err);
        return meshResultToPyMesh(mesh, err);
    }, py::arg("name"),
       py::arg("iso_level") = 0.0f,
       py::arg("res_x") = 50,
       py::arg("res_y") = 50,
       py::arg("res_z") = 50,
       py::arg("percentage") = 10.0f,
       py::arg("epsilon") = 0.0f,
       "Marching Cubes RBF surface reconstruction. Returns ct.Mesh object.");

    m.def("grid_projection", [meshResultToPyMesh](const std::string& name,
                                       double resolution, int padding_size, int k,
                                       int max_binary_search_level) -> py::object {
        auto* bridge = ct::PythonManager::instance().bridge();
        auto cloud = bridge->getCloud(QString::fromStdString(name));
        if (!cloud) throw std::runtime_error("Cloud not found: " + name);
        std::string err;
        auto mesh = surfaceGridProjection(cloud, resolution, padding_size, k,
                                          max_binary_search_level, err);
        return meshResultToPyMesh(mesh, err);
    }, py::arg("name"),
       py::arg("resolution") = 0.001,
       py::arg("padding_size") = 3,
       py::arg("k") = 0,
       py::arg("max_binary_search_level") = 10,
       "Grid Projection surface reconstruction. Returns ct.Mesh object.");

    // ================================================================
    // 网格显示/移除
    // ================================================================
    m.def("show_mesh", [](PyMesh& mesh, const std::string& id) {
        auto mesh_ptr = mesh.meshPtr();
        if (!mesh_ptr) throw std::runtime_error("Mesh is empty");
        auto* bridge = ct::PythonManager::instance().bridge();
        bridge->addMesh(mesh_ptr, QString::fromStdString(id));
    }, py::arg("mesh"), py::arg("id"),
       "Display a ct.Mesh in the 3D view under the given id.");

    m.def("remove_mesh", [](const std::string& id) {
        auto* bridge = ct::PythonManager::instance().bridge();
        bridge->removeMesh(QString::fromStdString(id));
    }, py::arg("id"),
       "Remove a displayed mesh from the 3D view by id.");
}
