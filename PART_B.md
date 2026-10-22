# Part B — Risk ranking and the case for what was built

Part A describes a design that is internally consistent on paper. The purpose
of Part B is to say which parts of it are most likely to be wrong in a way that
matters, rank them, and justify spending the implementation effort of Part C on
the top one. Section references point into `docs/design_part_a.md` and
`docs/design_rationale.md`.

Ranking criterion: **probability the implementation is wrong × consequence of
being wrong × difficulty of detecting it before hardware.** The third factor
is weighted heavily: a fault that only appears on the bench is far more
expensive than one a unit test can catch.

---

## 1. Ranked risks

| Rank | Risk | Where | Score rationale |
|---|---|---|---|
| **1** | **Delayed, asynchronous camera fusion in `state_estimator`** — correcting a 20–30 ms old (±5 ms jittered) camera frame at its true time, replaying forward, rejecting stale and out-of-order frames, and handing the result to the 1 kHz tick without a mutex | A.0.3, A.3.2 Path 2, A.3.3 items 2–4, A.4.3, TD-4, TD-7; listed in A.6 as *designed, not implemented* | High probability (the only algorithm the design leaves as a sketch), total consequence (defeats fine lock), invisible in static testing |
| **2** | **Fast-path timing on E1** — the 0.9 ms IMU→FSM budget, IMU-arrival triggering, no allocation/mutex/logging in the tick, deadline-miss escalation | A.3, A.4.2–A.4.3, TD-1, TD-3, TD-5, TD-6 | High consequence (fine-loop bandwidth in CD-3 derives from it), but the design already hedges it explicitly, most of it is discipline rather than algorithm, and it **cannot be verified on this host** (no PREEMPT_RT, no isolated core) |
| **3** | **Mode-manager logic** — bumpless HANDOFF, RECENTRE via FSM mean, loss-of-lock hysteresis + dwell + rate watchdog, 3-attempts-in-10-s guard | A.2, CD-4, CD-7, CD-8, A.5.3 | Largest surface area for logic bugs and chatter is a real operational failure, but it is a fully specified deterministic state machine that exhaustive unit tests can pin down cheaply |

Considered and ranked lower: command-authority enforcement (one integer
compare in each driver, AD-5); QoS mismatch (structurally prevented by the
shared header, AD-7); CycloneDDS discovery and late join (lifecycle-gated,
A.4.4).

---

## 2. Risk 1 — delayed camera fusion

### 2.1 What can go wrong

The camera measures the residual at exposure time `t_k`; the message arrives
≈ 25 ms later. In those 25 ms the IMU-propagated estimate has moved by whatever
the platform did. At the assumed 1°/s base motion that is 17 453 µrad/s ×
0.025 s ≈ **436 µrad**. If the frame is applied as though it were current (TD-4
option a), the estimator is pulled toward a value that was true 25 ms ago, the
IMU propagation then carries that error forward, and the next frame does it
again. The steady-state error converges to the full rate × delay — twenty
times the 20 µrad fine-lock budget — and it arrives as a 50 Hz excitation of the
FSM. The design chose buffer-and-replay (option c) precisely to avoid this, and
then left it unimplemented.

### 2.2 Why it is ranked first

1. **It is the one algorithm the design admits is not done.** A.6 lists it under
   "Designed, not implemented"; TD-4 says Part C implements it "only if time
   permits". Every other Part C item (handoff ramp, offload loop, state machine)
   has a complete recipe with every constant specified. Unimplemented design is
   where errors live.

2. **Failure is silent in every test that does not include motion.** With a
   static platform, options (a) and (c) give identical results. A bench test
   with the terminal bolted down will pass; the first platform manoeuvre will
   fail. This is the defining property of a high-ranked risk under the
   criterion above.

3. **Everything downstream trusts `ê`.** The HANDOFF pre-condition
   (`valid=true`, `camera_age_ms < 60`), the RECOVER coast, the 100/200 ms
   camera-gap thresholds (CD-6), the 3σ outlier gate, and the gimbal-command
   feedforward that stops the two loops fighting (A.2.3) all go through the
   estimate. A wrong estimator makes a correct mode manager take correct
   decisions on wrong data — there is no second line of defence.

4. **It sits on the hardest concurrency boundary in the system.** The correction
   runs at 50 Hz on E2 and must update state that E1 reads at 1 kHz with no
   mutex (TD-7). Replaying 25–100 ms of stored increments inside a bounded
   callback while E1 keeps appending is exactly where a data race or an
   unbounded-time path hides, and such bugs do not reproduce on demand.

5. **It is fully verifiable in software now.** Unlike risk 2 it needs no RT
   kernel: the measurement equation in A.0.3 is written explicitly so a
   synthetic plant and the estimator can be checked against each other to
   µrad precision, and the apply-at-arrival baseline can be measured alongside
   it rather than asserted.

### 2.3 Why not risk 2 first

The timing budget is a hypothesis the design already states it cannot prove
without the target SBC (TD-10). On this host — generic kernel, no isolated
core, RT priorities requiring privilege — any percentile figure is evidence
about the laptop, not about the design. Building the `SCHED_FIFO` / affinity /
`mlockall` layer first would consume effort on something whose result cannot be
interpreted here, while leaving the algorithm that *can* be settled unsettled.
What was taken from risk 2 is the part inseparable from risk 1: the
two-callback-group, two-executor split, because the lock-free hand-off only
makes sense if the two sides really are different threads.

### 2.4 Why not risk 3 first

The state machine has the most transitions and therefore the most places for a
bug, but every threshold, dwell and exit is specified in A.2.1 and CD-8, the
logic is deterministic, and it can be unit-tested exhaustively by feeding it
scripted lock-quality sequences in milliseconds. Its risk is *volume*, not
*uncertainty*. It is also meaningless to exercise before the estimator exists:
its inputs are `valid`, `camera_age_ms` and the windowed RMS of `ê`.

---

## 3. What Part C therefore had to demonstrate

For the ranking to be justified, Part C needed to show four things, each of
which is reported in `PART_C.md`:

| Claim | Evidence required | Result |
|---|---|---|
| Apply-at-arrival really fails under motion | Measured error ≈ rate × delay at 1°/s | 444 µrad live, 436 µrad in unit test |
| Buffer-and-replay really fixes it | Error inside the 20 µrad budget on the same stream | 11.4 µrad live (noise-limited); 0.4 µrad with perfect inputs |
| The gates work | Stale, out-of-order, future, low-confidence-outlier frames rejected and counted; validity expires at 200 ms | All verified; `valid` drops exactly in the > 200 ms portions of live camera gaps |
| The E1/E2 hand-off is sound | No mutex, no allocation, no torn state under concurrent operation | Zero allocations in the tick; two-thread stress test and live node both consistent |

One side finding supports the ranking after the fact: the tests found a real
defect that none of the design review had — a half-sample lag from rectangular
gyro integration worth 10 µrad at 37 Hz, half the fine-lock budget. It was
invisible by inspection and would have been invisible on a static bench.

---

## 4. Residual risk after Part C

- Risk 2 is untouched in substance. The node has the right executor shape and
  an optional RT priority, but no timing evidence exists and none can be
  gathered on this host.
- Risk 3 is untouched. Nothing consumes `ê` yet.
- Within risk 1: the drift-rate state stands in for an explicit gyro-bias state
  (AD-4 defers this to real Allan-variance data); the reported covariance is
  ~1.6× conservative and untuned; camera stamps are assumed to be exposure
  mid-time on the IMU clock (TD-4 "depends on").
