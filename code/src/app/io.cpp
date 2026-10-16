#include "ofc/io.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace ofc::io {

nlohmann::json readJson(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open " + path);
    return nlohmann::json::parse(in, nullptr, true, /*ignore_comments=*/true);
}

Vec3 vec3(const nlohmann::json& j, const char* key, const Vec3& fallback) {
    if (!j.contains(key)) return fallback;
    const auto& a = j.at(key);
    if (!a.is_array() || a.size() != 3) throw std::invalid_argument(std::string("'") + key + "' must be [x, y, z]");
    return {a[0].get<double>(), a[1].get<double>(), a[2].get<double>()};
}

void writeCsv(const std::string& path, const std::vector<std::string>& header, const std::vector<Vec>& rows) {
    const std::filesystem::path p(path);
    if (p.has_parent_path()) std::filesystem::create_directories(p.parent_path());
    std::ofstream out(p);
    if (!out) throw std::runtime_error("cannot write " + path);
    for (std::size_t i = 0; i < header.size(); ++i) out << (i ? "," : "") << header[i];
    out << '\n' << std::setprecision(10);
    for (const Vec& r : rows) {
        for (std::size_t i = 0; i < r.size(); ++i) out << (i ? "," : "") << r[i];
        out << '\n';
    }
}

} // namespace ofc::io
