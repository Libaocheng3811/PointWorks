#pragma once

// Common includes for all bind_*.cpp files
// Provides pybind11 includes and cloud registry access

#include "core/cloud.h"
#include "core/octree.h"
#include "python/cloud_registry.h"
#include "python_manager.h"

// Qt's <QObject> defines slots macro, conflicts with Python's object.h
#undef slots
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

namespace py = pybind11;

// Get the global PythonCloudRegistry (Qt-free)
inline pw::PythonCloudRegistry& getRegistry()
{
    return *pw::PythonManager::instance().registry();
}

// Check whether algorithm results should auto-insert into UI
inline bool shouldAutoInsert()
{
    return !getRegistry().isScriptMode();
}
