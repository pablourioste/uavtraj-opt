# Notice

Copyright (c) 2026 Pablo Urioste Alarcón

| Part | Licence |
|---|---|
| Documentation (`docs/`), figures (`docs/assets/`), aggregated results (`results/`) | CC BY 4.0, see [LICENSE](LICENSE) |
| Source code (`code/`, except `code/third_party/`) | MIT, see [LICENSE-CODE](LICENSE-CODE) |
| `code/third_party/nlohmann/json.hpp` (JSON for Modern C++ 3.12.0) | MIT, © 2013–2025 Niels Lohmann; licence header kept in the file |
| Thesis PDF (`thesis/`) | Published for reading. All rights reserved by the author unless stated otherwise; third-party material inside it keeps its original rights. |

## Exclusions and trademarks

- The production solver was designed exclusively as a product for Singular Aircraft (https://singularaircraft.com/inicio) and, for privacy reasons towards the company, is not part of this repository. Only the research, the mathematical methods and their application in code are shared. The code in `code/` is an independent, restricted re-implementation written from the thesis equations; it contains no production code or company data.
- "Singular Aircraft", "Flyox I" and "MSN012" are names and data of their owners and are used here to describe the reference aircraft only.
- The generic aircraft in `code/data/aircraft/generic_uav.json` uses mass, geometry, inertia and stability derivatives of the Aerosonde-class model published in R. W. Beard & T. W. McLain, *Small Unmanned Aircraft: Theory and Practice*, Princeton University Press, 2012. Its drag polar, propulsion and envelope values are illustrative.
- Third-party papers are cited, never redistributed (see `docs/references.md`).
