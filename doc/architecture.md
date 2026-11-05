# Target architecture

This is the intended design, not a claim of complete ISA or performance coverage.

```text
consumer: memory + peripheral state + virtual time + event arbitration
                            |
                  Core(Memory, Io*, Options)
                            |
           bounded run -> native region cache -> native code
                            |                        |
                  precise side exit          direct RAM access
                            |                        |
                 fault / event / budget      explicit I/O callback
```

The consumer owns memory and all device behavior. The DSP56300 core owns guest
registers, instruction semantics, native code and continuation state. One thread
owns each core; callers synchronize external events outside its mutable state.

Memory uses 24-bit P/X/Y words and 56-bit accumulators. Typed views make storage bounds,
aliases and lifetimes explicit. Ordinary RAM accesses use direct native loads and
stores; device accesses retain exactly-once effects. Program mutations invalidate
all affected native regions, including aliased storage.

Compile regions on demand into writable memory, then execute from read/execute
pages. Chain valid regions and keep live guest values in host registers. Guards
cover numerical modes, addresses, code versions and admissible event budgets.
Every exit materializes the architectural state required by its observer. Failed
guards must preserve the original operation, ordering and fault context.

The ordinary entry selects the fastest qualified native path. Incomplete lowering
remains an explicit implementation gap. Callbacks implement device effects,
not guest instruction interpretation. Diagnostics are optional and must not force
per-instruction snapshots in normal execution.

Runs stop at meaningful event, fault or budget boundaries. Consumers own clocks,
interrupt arbitration, pacing and inter-core queues. Checkpoints pair complete CPU
continuations with matching consumer-owned memory and peripheral state. No host
synthesizer, board simulator or physical wire model belongs in this library.

Instruction expectations and complete qualification requirements belong in test/.
Primary architecture references are listed in references.json.
