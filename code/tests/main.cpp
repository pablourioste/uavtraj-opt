#include <cstdio>
#include <exception>

#include "check.hpp"

int main() {
    for (const auto& c : check::registry()) {
        const int before = check::failures();
        try {
            c.fn();
        } catch (const std::exception& e) {
            std::printf("    EXCEPTION %s\n", e.what());
            ++check::failures();
        }
        std::printf("[%s] %s\n", check::failures() == before ? " ok " : "FAIL", c.name);
    }
    std::printf("\n%zu tests, %d failed checks\n", check::registry().size(), check::failures());
    return check::failures() == 0 ? 0 : 1;
}
