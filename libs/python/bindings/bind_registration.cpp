#include "algorithm/registration.h"
#include "bind_registration.h"
#include "bind_core.h"

void registerRegistrationBindings(py::module_& m)
{
    auto makeRegContext = [](const std::string& src_name, const std::string& tgt_name,
                             int max_iter, double corr_dist) -> pw::RegistrationContext {
        auto src = getRegistry().getCloud(src_name);
        if (!src) throw std::runtime_error("Source cloud not found: " + src_name);
        auto tgt = getRegistry().getCloud(tgt_name);
        if (!tgt) throw std::runtime_error("Target cloud not found: " + tgt_name);
        pw::RegistrationContext ctx;
        ctx.source_cloud = src;
        ctx.target_cloud = tgt;
        ctx.params.max_iterations = max_iter;
        ctx.params.distance_threshold = corr_dist;
        return ctx;
    };

    auto regResultToDict = [](const pw::RegistrationResult& result) -> py::object {
        if (!result.success) return py::none();
        result.aligned_cloud->makeAdaptive();
        getRegistry().registerCloud(result.aligned_cloud);
        getRegistry().holdCloud(result.aligned_cloud);
        if (shouldAutoInsert()) {
            auto* bridge = pw::PythonManager::instance().bridge();
            if (bridge) bridge->insertCloud(result.aligned_cloud);
        }

        py::dict dict;
        dict["aligned"] = py::cast(PyCloud(result.aligned_cloud));
        dict["score"] = result.score;
        dict["time_ms"] = result.time_ms;
        py::list rows;
        for (int i = 0; i < 4; ++i) {
            py::list row;
            for (int j = 0; j < 4; ++j)
                row.append(static_cast<double>(result.matrix(i, j)));
            rows.append(row);
        }
        dict["matrix"] = rows;
        return dict;
    };

    m.def("icp", [makeRegContext, regResultToDict](const std::string& src, const std::string& tgt,
                                                      int max_iter, double corr_dist,
                                                      bool use_reciprocal) -> py::object {
        auto ctx = makeRegContext(src, tgt, max_iter, corr_dist);
        auto result = pw::Registration::IterativeClosestPoint(ctx, use_reciprocal);
        return regResultToDict(result);
    }, py::arg("source"), py::arg("target"),
       py::arg("max_iterations") = 50, py::arg("correspondence_distance") = 1.0,
       py::arg("use_reciprocal") = false,
       "ICP registration. Returns dict with 'aligned', 'score', 'matrix', 'time_ms' or None");

    m.def("icp_with_normals", [makeRegContext, regResultToDict](const std::string& src, const std::string& tgt,
                                                                    int max_iter, double corr_dist,
                                                                    bool use_reciprocal,
                                                                    bool use_symmetric,
                                                                    bool enforce_same_direction) -> py::object {
        auto ctx = makeRegContext(src, tgt, max_iter, corr_dist);
        auto result = pw::Registration::IterativeClosestPointWithNormals(
            ctx, use_reciprocal, use_symmetric, enforce_same_direction);
        return regResultToDict(result);
    }, py::arg("source"), py::arg("target"),
       py::arg("max_iterations") = 50, py::arg("correspondence_distance") = 1.0,
       py::arg("use_reciprocal") = false, py::arg("use_symmetric") = false,
       py::arg("enforce_same_direction") = false,
       "ICP with normals registration. Returns dict or None");

    m.def("icp_nonlinear", [makeRegContext, regResultToDict](const std::string& src, const std::string& tgt,
                                                                int max_iter, double corr_dist,
                                                                bool use_reciprocal) -> py::object {
        auto ctx = makeRegContext(src, tgt, max_iter, corr_dist);
        auto result = pw::Registration::IterativeClosestPointNonLinear(ctx, use_reciprocal);
        return regResultToDict(result);
    }, py::arg("source"), py::arg("target"),
       py::arg("max_iterations") = 50, py::arg("correspondence_distance") = 1.0,
       py::arg("use_reciprocal") = false,
       "Non-linear ICP registration. Returns dict or None");

    m.def("gicp", [makeRegContext, regResultToDict](const std::string& src, const std::string& tgt,
                                                       int max_iter, int k, double tra_tol,
                                                       double rol_tol, bool use_reciprocal) -> py::object {
        auto ctx = makeRegContext(src, tgt, max_iter, std::sqrt(std::numeric_limits<double>::max()));
        auto result = pw::Registration::GeneralizedIterativeClosestPoint(
            ctx, k, max_iter, tra_tol, rol_tol, use_reciprocal);
        return regResultToDict(result);
    }, py::arg("source"), py::arg("target"),
       py::arg("max_iterations") = 200, py::arg("k") = 30,
       py::arg("translation_tolerance") = 1e-6, py::arg("rotation_tolerance") = 1e-6,
       py::arg("use_reciprocal") = false,
       "Generalized ICP registration. Returns dict or None");

    m.def("ndt", [makeRegContext, regResultToDict](const std::string& src, const std::string& tgt,
                                                      float resolution, double step_size,
                                                      double outlier_ratio) -> py::object {
        auto ctx = makeRegContext(src, tgt, 35, std::sqrt(std::numeric_limits<double>::max()));
        auto result = pw::Registration::NormalDistributionsTransform(ctx, resolution, step_size, outlier_ratio);
        return regResultToDict(result);
    }, py::arg("source"), py::arg("target"),
       py::arg("resolution") = 1.0, py::arg("step_size") = 0.1, py::arg("outlier_ratio") = 0.05,
       "Normal Distributions Transform registration. Returns dict or None");

    m.def("fpcs", [makeRegContext, regResultToDict](const std::string& src, const std::string& tgt,
                                                       float delta, float approx_overlap,
                                                       float score_threshold, int nr_samples,
                                                       float max_norm_diff, int max_runtime) -> py::object {
        auto ctx = makeRegContext(src, tgt, 0, std::sqrt(std::numeric_limits<double>::max()));
        auto result = pw::Registration::FPCSInitialAlignment(
            ctx, delta, true, approx_overlap, score_threshold, nr_samples, max_norm_diff, max_runtime);
        return regResultToDict(result);
    }, py::arg("source"), py::arg("target"),
       py::arg("delta") = 1.0, py::arg("approx_overlap") = 0.1,
       py::arg("score_threshold") = 0.6, py::arg("nr_samples") = 3000,
       py::arg("max_norm_diff") = 0.1, py::arg("max_runtime") = 60,
       "FPCS initial alignment. Returns dict or None");

    m.def("kfpcs", [makeRegContext, regResultToDict](const std::string& src, const std::string& tgt,
                                                        float delta, float approx_overlap,
                                                        float score_threshold, int nr_samples,
                                                        float max_norm_diff, int max_runtime,
                                                        float upper_trl, float lower_trl,
                                                        float lambda) -> py::object {
        auto ctx = makeRegContext(src, tgt, 0, std::sqrt(std::numeric_limits<double>::max()));
        auto result = pw::Registration::KFPCSInitialAlignment(
            ctx, delta, true, approx_overlap, score_threshold, nr_samples,
            max_norm_diff, max_runtime, upper_trl, lower_trl, lambda);
        return regResultToDict(result);
    }, py::arg("source"), py::arg("target"),
       py::arg("delta") = 1.0, py::arg("approx_overlap") = 0.1,
       py::arg("score_threshold") = 0.6, py::arg("nr_samples") = 3000,
       py::arg("max_norm_diff") = 0.1, py::arg("max_runtime") = 60,
       py::arg("upper_trl_boundary") = 2.0, py::arg("lower_trl_boundary") = 0.05,
       py::arg("lambda") = 0.5,
       "KFPCS initial alignment. Returns dict or None");
}
