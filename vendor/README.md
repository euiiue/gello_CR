# Bundled device libraries

`vendor/` contains device libraries used by this project. Application code lives
under `src/gello_cr/`; automated tests stay under `tests/`.

- `gello/`: the GELLO Python package subset and FK URDF used by the master adapter.
- `nrc/`: the NRC Python wrapper and its matching native extension. The extension
  is built for Linux x86-64 and Python 3.12; keep the wrapper and extension together.
- `linker_hand_python_sdk/`: upstream Git submodule pinned by the parent repository;
  the O6 adapter imports its `LinkerHand` package. Initialize it with
  `git submodule update --init --recursive` after cloning.
