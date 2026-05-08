#pragma once
#include "bind_common.h"

class PyCloud;
namespace pw { struct FeatureType; }

void registerFeatureBindings(py::module_& m);

py::object extractDescriptorToPy(const std::shared_ptr<pw::FeatureType>& feature);
