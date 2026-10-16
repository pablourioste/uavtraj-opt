#pragma once
// Minimal dependency-free test registry.

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace check {

struct Case {
    const char* name;
    std::function<void()> fn;
};

inline std::vector<Case>& registry() {
    static std::vector<Case> r;
    return r;
}

inline int& failures() {
    static int f = 0;
    return f;
}

struct Register {
    Register(const char* name, std::function<void()> fn) { registry().push_back({name, std::move(fn)}); }
};

inline void fail(const char* file, int line, const std::string& msg) {
    std::printf("    FAIL %s:%d  %s\n", file, line, msg.c_str());
    ++failures();
}

} // namespace check

#define OFC_CAT2(a, b) a##b
#define OFC_CAT(a, b)  OFC_CAT2(a, b)
#define TEST(name)                                                 \
    static void name();                                            \
    static const check::Register OFC_CAT(reg_, name)(#name, name); \
    static void name()

#define CHECK(cond)                                          \
    do {                                                     \
        if (!(cond)) check::fail(__FILE__, __LINE__, #cond); \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                                \
    do {                                                                                                     \
        const double va_ = (a), vb_ = (b);                                                                   \
        if (!(std::fabs(va_ - vb_) <= (tol)))                                                                \
            check::fail(__FILE__, __LINE__,                                                                  \
                        std::string(#a " ~ " #b ": ") + std::to_string(va_) + " vs " + std::to_string(vb_)); \
    } while (0)

#define CHECK_THROWS(expr)                                                           \
    do {                                                                             \
        bool thrown_ = false;                                                        \
        try {                                                                        \
            (void)(expr);                                                            \
        } catch (...) {                                                              \
            thrown_ = true;                                                          \
        }                                                                            \
        if (!thrown_) check::fail(__FILE__, __LINE__, "expected exception: " #expr); \
    } while (0)
