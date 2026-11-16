# Contract kernels

`kernel_reference.hpp` defines the shared K1/K2/K6 numerical workload. Guest
programs use the public Core API and authored ISA encodings. No firmware input
belongs in these benchmarks.

## Numerical definition

Samples and coefficients are signed Q1.Bits. DSP56300 uses Bits=23; the reference
also supports Bits=15 and Bits=31. Quantization rounds to nearest, ties to even,
then saturates to the signed sample range. All fixture intermediates fit int64.

Each workload processes 256 consecutive blocks of 1024 samples. Its fixed input
comes from the header's 32-bit LCG, seed 0x243f6a88. Signed values are halved with
truncation toward zero; each 97-sample period starts with maximum and minimum
samples. Preparation creates these inputs before timing.

- K1 is a causal 64-tap FIR. Initial history is zero; history continues across
  block boundaries. Coefficient k is `((17*k mod 31)-15)/1024`. The complete
  dot product is quantized once per output sample.
- K2 is a cascade of four transposed direct-form-II biquads. Coefficients and
  initial states are defined in the header. For each stage, quantize
  `y=z1+b0*x`, then `z1=z2+b1*x-a1*y` and `z2=b2*x-a2*y`, using the old state
  and the quantized y. All state persists across blocks. Initial states exercise
  saturation as well as feedback and rounding.
- K6 executes the same FIR with a run horizon every 256 base cycles and an IRQ
  requested every 4096 base cycles. The handler increments an independent
  counter; input samples and output data are unchanged. Report any architectural
  delay between the requested IRQ time and acceptance.

## Measurement

Compare complete guest outputs and filter state with the reference outside the
timed interval. Each run must retire at least ten million guest instructions.
Report preparation, native bytes, guest instructions/s, hardware host
instructions and cycles per guest instruction, and the Clang19 O3 C++ ratio.
Hardware-counter unavailability is explicit.

Use the same fixed memory map and code for the ordinary and optimized-region
measurements. An explicit empty region-mode set selects ordinary native code;
the optimized run selects the actual numeric mode. Measure optional retirement
observation separately and require its region and precise counts to sum to
Progress. Counter instrumentation is outside the ordinary timing control.

The host algorithm receives runtime input and coefficient spans through an
opaque translation-unit boundary, so known fixture coefficients do not become
constant-folded host instructions. Report flags and timing scope. Kernel
results do not qualify whole-instrument throughput.

Build and run the public-API DSP controls with:

```sh
cmake --preset release
cmake --build build/release --target dsp56300_kernel_bench
timeout 120s build/release/dsp56300_kernel_bench
```

The command emits JSON lines for ordinary native code and mode-zero regions.
Each timed guest run follows a complete untimed warm run. All 262144 outputs,
all final biquad states, exact instruction totals and guest IRQ counts are checked.
The native Q1.23 reference is compiled at O3 for the local host in a separate
translation unit without LTO. Its reported time is the median of three warmed
runs adjacent to each guest measurement; setup and checks are outside timing.
`guest_over_host_time` is guest elapsed time divided by reference elapsed time;
one is equal speed. Repeat the command to report variation.

K1 also runs with bounds alone, using the identical prepared image. K6 adds the
two-instruction counter vector. The two differences isolate the incremental cost
of bounded calls and delivered interrupts; do not divide total K6 time by either
count. Interrupt admission waits for actual architectural eligibility; the maximum
requested-to-admitted delay is in nominal guest cycles. Bounds are absolute and
retain phase. Preparation is reported on first use; zero on the following rows
means the same prepared image is reused.

Linux PMCs cover the calling thread's user code from immediately before to
immediately after the timed guest call, including the benchmark's bounded-call
driver. Only unscaled groups with equal enabled/running time are accepted; raw
counts and times are emitted. Unavailable counters are null. Other hosts report
null until a native counter provider exists. Observed runs have separate records
and code, so retirement counting never contributes to the timing controls.

Native bytes cover the complete prepared module, including the unused IRQ code
in K1/K2. `foreground_guest_sites` counts unique foreground instructions, while
`prepared_guest_sites` also counts ordinary and inserted-vector preparation of
the two IRQ instructions. Static bytes per site are not dynamic host instructions
per guest retirement.
