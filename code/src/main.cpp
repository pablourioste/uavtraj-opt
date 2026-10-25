// ofc: command-line entry point.
//
//   ofc <case.json> [<case.json> ...]   run each case file (examples in data/cases/)
//   ofc --info                          build information (is the optimiser available?)
//
// Each case file names its use case ("use_case": "UC-01" ... "UC-09"), its
// aircraft and its output folder. Run from code/ so that relative paths resolve.

#include <chrono>
#include <cstdio>
#include <exception>
#include <string>
#include <vector>

#include "ofc/solver.hpp"
#include "ofc/use_cases.hpp"

namespace {

void usage() {
    std::printf("usage: ofc <case.json> [<case.json> ...]\n"
                "       ofc --info\n\n"
                "use cases (examples in data/cases/):\n"
                "  UC-01 aircraft report       UC-06 design scenarios\n"
                "  UC-02 trim                  UC-07 missions with no-fly zones (planner + guidance)\n"
                "  UC-03 open-loop simulation  UC-08 optimal missions (warm start -> UF -> FC, IPOPT)\n"
                "  UC-04 transcription check   UC-09 warm-start benchmark (IPOPT)\n"
                "  UC-05 design charts\n\n"
                "run from code/ so that relative paths in the case files resolve.\n");
}

void info() {
    std::printf("ofc %s\noptimiser: %s\n", OFC_VERSION,
                ofc::nlp::available() ? "IPOPT (UC-08 and UC-09 available)"
                                      : "not built (UC-08 and UC-09 are skipped; see code/README.md)");
}

} // namespace

int main(int argc, char** argv) {
    // 1. Arguments.
    std::vector<std::string> cases;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--info") {
            info();
            return 0;
        }
        if (a == "-h" || a == "--help") {
            usage();
            return 0;
        }
        cases.push_back(a);
    }
    if (cases.empty()) {
        usage();
        return 1;
    }

    // 2. Run every case: load, validate, solve, export. A failing case does
    //    not stop the others; the exit code reports the worst outcome.
    int status = 0;
    std::vector<double> seconds;
    for (const std::string& path : cases) {
        const auto t0 = std::chrono::steady_clock::now();
        try {
            const int rc = ofc::cases::run(path);
            if (rc != 0) status = rc;
        } catch (const std::exception& e) {
            std::fprintf(stderr, "error (%s): %s\n", path.c_str(), e.what());
            status = 1;
        }
        seconds.push_back(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
    }

    // 3. Timing summary.
    if (cases.size() > 1) {
        std::printf("\n== Summary ==\n");
        for (std::size_t i = 0; i < cases.size(); ++i) std::printf("%8.2f s  %s\n", seconds[i], cases[i].c_str());
    }
    return status;
}
