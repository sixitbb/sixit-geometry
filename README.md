# sixit-geometry
sixit/geometry provides geometrical primitives and algorithms (both 2D and 3D), with support for cross-platform deterministic from [sixit/dmath](https://github.com/sixitbb/sixit-dmath/tree/main).

## Included 
- 2D geometry primitives, with not-so-primitive classes and algos including:
   + curve2 - curves for pathfinding, _and rendering of SVG-like primitives_.
   + TBD
- 3D geometry primitives, with not-so-primitive classes and algos including:
   + finding the best plane to approximate N>3 3D points best
   + TBD

## Support for sixit/dmath floating-point determinism
ALL the classes and algos in sixit/geometry are templatized and support ALL the fp classes provided by sixit/dmath. 

## WARNING: APIs are SUBJECT TO CHANGE WITHOUT NOTICE
We are currently at version 0.0.1, and we will NOT commit to APIs being stable at least until v0.1.0... 

## Supported Platforms
In our speak, "platform" = "CPU+Compiler+OS". Currently, we're supporting {x64|ARM64|RISC-V64|WASM32}, {Clang|MSVC|GCC}, and {Android|iOS|Linux|MacOS|Windows|WASM}. For details, please refer to [Supported platforms for sixit C++ Libs](https://github.com/sixitbb/.github/blob/main/profile/cpp-supported-platforms.md).

## Dependencies, Install and Building
Dependencies:
- [sixit/core](https://github.com/sixitbb/sixit-core/tree/main)
- [sixit/dmath](https://github.com/sixitbb/sixit-dmath/tree/main)
Install: just put all sixit libs under the same sixit folder.

Building:
sixit/geometry is a HEADER-ONLY LIB, no build is really necessary. 

## Plans for v0.0.2
- code cleanup
- fixing and adding more tests
- replacing numerically-unstable algorithms wherever feasible. 
