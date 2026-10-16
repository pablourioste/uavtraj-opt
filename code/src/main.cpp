#include <cstdio>
#include <exception>
#include <string>

#include "ofc/use_cases.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("usage: ofc <case.json> [<case.json> ...]\n"
                    "example cases in data/cases/:\n"
                    "  uc01_aircraft_report.json  uc02_trim.json\n"
                    "  uc03_simulation.json       uc04_transcription.json\n"
                    "run from the project root so relative paths resolve.\n");
        return 1;
    }
    int status = 0;
    for (int i = 1; i < argc; ++i) {
        try {
            const int rc = ofc::cases::run(argv[i]);
            if (rc != 0) status = rc;
        } catch (const std::exception& e) {
            std::fprintf(stderr, "error (%s): %s\n", argv[i], e.what());
            status = 1;
        }
    }
    return status;
}
