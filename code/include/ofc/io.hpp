#pragma once
// Small input/output helpers: JSON case files and CSV outputs.

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "ofc/types.hpp"

namespace ofc::io {

nlohmann::json readJson(const std::string& path);

Vec3 vec3(const nlohmann::json& j, const char* key, const Vec3& fallback);

// Writes a CSV file, creating parent directories when needed.
void writeCsv(const std::string& path, const std::vector<std::string>& header, const std::vector<Vec>& rows);

// Writes indented JSON, creating parent directories when needed.
void writeJson(const std::string& path, const nlohmann::json& j);

} // namespace ofc::io
