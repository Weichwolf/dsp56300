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
