# Pinned x87 constants

The headers are unmodified from Bochs commit
`c38d6a4ab10e084b7d52d6606e90b7395c6e688f`, under `bochs/cpu/fpu`:
https://github.com/bochs-emu/Bochs/tree/c38d6a4ab10e084b7d52d6606e90b7395c6e688f/bochs/cpu/fpu

They support the exact-rational diagnostic `tools/check-x87-model.py` and the
derived portable Rust constants. Notices for the derived algorithm are retained
in `crates/hw-flashmath/src/x87-LICENSE.txt` and in the diagnostic's docstring.
Source inspection alone is not evidence of hardware equivalence; see MATH.md
for measured comparisons and their limits.
