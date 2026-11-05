# Target architecture

The library executes the DSP56300 ISA through native JIT code. This design is the
implementation target, not a claim that the current migration is complete.

```text
consumer: memory + peripheral state + virtual time + event arbitration
                            |
                  Core(Memory, Io*, Options)
                            |
               ISA frontend -> host-JIT concept -> native code
                                   |                    |
                        x86/SSE/AVX today        direct RAM access
                        future ARM/NEON          explicit I/O effects
```

The public headers are core.hpp, isa.hpp and memory.hpp. Both libraries expose
the same minimal Core API. ISA-specific register layouts, address units and
peripheral contracts account for their differences; internal JIT mechanics do not.
Core owns its complete state. Normal consumers need no state management; optional
read-only state access occurs at safe owner boundaries and adds no work otherwise.
Opaque checkpoint handles and staged effect tickets are outside the normal API.

Only JIT execution belongs in production. Host emitters satisfy one host concept;
there is no interpreter/reference/cached backend selector. Native specialization
must preserve all ISA effects and precise exits. Unknown operations and accesses
fail with operation, address and PC. The general ISA extends beyond Blofeld needs.

Memory uses 24-bit P/X/Y words and 56-bit accumulators. Typed borrowed views preserve bounds,
permissions, knownness, aliases and owner lifetimes. Harvard spaces are logical
views over ordinary host RAM. Program mutation invalidates affected translations.
One execution owner controls each core; external events use bounded communication.

The consumer owns peripheral clocks and arbitration. Native runs stop at valid
event, interrupt, fault or sleep boundaries; internal grants preserve deadlines
and bounded response. A due event is handled before executing an instruction
beyond its boundary. Caller-visible instruction counts are optional test control,
not a substitute for event handling. Physical buses and pipeline phases are not
implementation goals. ISA-visible cache, loop and interrupt semantics remain.

Strict compilation, static analysis and independently expected ISA/state tests
qualify changes. Core tests remain active before instrument real-time operation;
fuzzing runs only on explicit request. Primary references are in references.json.
