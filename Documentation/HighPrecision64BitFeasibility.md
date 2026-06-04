# Feasibility analysis: 64-bit coordinates / large worlds in bepuphysics2

> This document answers issue [#1](https://github.com/MiheevN/bepuphysics2/issues/1):
> *"Analyze the possibility of implementing 64-bit coordinates so the engine can
> simulate much larger spaces and seamless movement through them — or is it more
> sensible to keep one 32-bit space per server and handle movement with another
> library layered on top?"*
>
> It is an **evaluation**, not an implementation. No engine code is changed. All
> claims below are backed by direct citations into this repository and into the
> upstream design discussion.

## Краткий ответ (Russian TL;DR)

Коротко, на прямые вопросы из задачи:

1. **Насколько это сложно и логично?** — Перевести *весь* движок на 64-битную
   арифметику (FP64) **нелогично**: это примерно вдвое урезало бы пропускную
   способность SIMD в самых дорогих стадиях и упёрло бы симуляцию в пропускную
   способность памяти (масштабирование всего до 2–3 ядер). Это прямо
   подтверждает автор движка (см. цитату ниже).
2. **Но точечное решение возможно и заложено в архитектуру.** Высокая точность
   нужна **только в двух местах**: (а) мировые позы тел и статики и (б)
   ограничивающие объёмы (AABB) широкой фазы. Весь решатель и узкая фаза уже
   работают в *относительных* координатах, где FP32 достаточно. Автор движка
   специально оставил «дверь открытой» для этого в ревизии решателя 2.4 и в коде
   уже расставлены пометки `//TODO ... high precision poses` в нужных местах.
   Это работа **средней** сложности и хорошо локализованная (если ограничиться
   мирами «среднего» размера), либо **высокой** сложности (если нужны
   планетарные/космические масштабы, требующие переделки широкой фазы).
3. **«Хватит ли 1 сервера на одно 32-битное пространство? Делать перемещение
   другими методами/библиотеками поверх?»** — **Да, это и есть рекомендованный
   автором подход** для по-настоящему огромных бесшовных миров: дробить
   симуляцию на FP32-острова и хранить «мета-начало координат» (meta-origin) в
   произвольной точности на уровне приложения / надстройки. Полноценная
   64-битная поза — это *удобство* для умеренного расширения диапазона, а не
   замена секционированию для космических масштабов.

Подробное техническое обоснование — ниже на английском (язык всей остальной
документации репозитория).

## 1. The question is not new — the original author already analyzed it

This is one of the most-requested features of bepuphysics2 and the original
author (Ross Nordby) has discussed it at length. It appears on the project
roadmap as item #15:

> *"High precision body and static poses, plus associated broad phase changes,
> for worlds exceeding 32 bit floating point precision. This isn't actually too
> difficult, but it would come with tradeoffs. See
> [bepu/bepuphysics2#13](https://github.com/bepu/bepuphysics2/issues/13). 2.4's
> revamp of the solver and body data layouts intentionally left the door open
> for higher precision poses."*
> — `Documentation/roadmap.md:31`

The upstream tracking issue [bepu/bepuphysics2#13 "High precision
poses"](https://github.com/bepu/bepuphysics2/issues/13) frames the problem
precisely:

> *"While the vast majority of the engine works in relative space where single
> precision floating point numbers are good enough, there are two places where
> it can be problematic: 1. Body/static world space poses, and 2. Broad phase
> bounding boxes. While it wouldn't be a trivial change, it is relatively
> localized. We could use conditional compilation to swap out singles for
> doubles or even fixed point representations without much issue in poses."*

That single paragraph is the crux of the whole analysis: **the problem is
localized to two subsystems, not the whole engine.**

## 2. Current state: where coordinates live today (all FP32)

| Subsystem | Type | Storage | Citation |
|---|---|---|---|
| Body pose (scalar) | `Vector3 Position` (3×FP32) | `RigidPose` is 32 bytes | `BepuPhysics/BodyProperties.cs:54-68` |
| Body motion state | `RigidPose` + `BodyVelocity` | `MotionState` 64 bytes | `BepuPhysics/BodyProperties.cs:11-46` |
| Body solver bundle (SoA) | `Vector3Wide` = 3×`Vector<float>` | SIMD wide | `BepuUtilities/Vector3Wide.cs:9-22` |
| Static pose | `RigidPose Pose` | same FP32 | `BepuPhysics/Statics.cs:61-85` |
| Broad-phase AABB | `Vector3 Min/Max` (3×FP32) | 32 bytes | `BepuUtilities/BoundingBox.cs:44-57` |
| Tree node AABB | `Vector3 Min/Max` per child | 32-byte child, 64-byte node | `BepuPhysics/Trees/Node.cs:7-33` |

Everything that touches a *world-space* coordinate is single precision. There is
no `Vector<double>` anywhere in the core libraries; the only `double` usage is in
profiling/statistics and isolated convex-hull preprocessing.

### Why the engine was *designed* to allow swapping this out

The pose type is deliberately kept separate from the general-purpose transform
type so its representation can change later:

> *"It's a little odd that this exists alongside the BepuUtilities.RigidTransform.
> The original reasoning was that rigid poses may end up having a non-FP32
> representation. … When/if we take advantage of larger sizes, we'll have to
> closely analyze every use case of RigidPose to see if we need the higher
> precision or not."*
> — `BepuPhysics/BodyProperties.cs:48-50`

The bound-computation sites that would need to change are even pre-flagged in the
code:

> *"Note: the min and max here are in absolute coordinates, which means this is a
> spot that has to be updated in the event that positions use a higher precision
> representation."*
> — `BepuPhysics/Statics.cs:376` and `:467`

And the gather/scatter, compound, and query paths already carry explicit notes:

- `BepuPhysics/Bodies_GatherScatter.cs:630` — *"High precision poses means we'll
  end up with 64 bits of the second lane … That'll require a revamp of this
  approach to mask out any writes to the pose components, but it'll all be
  compile time conditional."*
- `BepuPhysics/Collidables/Compound.cs:173` — *"This is an area that has to be
  updated for high precision poses. May be able to centralize positional work by
  deferring it until the final bounds scatter step."*
- `BepuPhysics/Simulation_Queries.cs:102` — *"This is all sensitive to pose
  precision. If you change broadphase or pose precision, this will have to
  change."*

These breadcrumbs are exactly the seam along which a high-precision mode would be
implemented.

## 3. Why "just make everything 64-bit" is the wrong move

Naively switching the engine to FP64 (or `Vector<double>`) would be a regression,
for two independent reasons the author spells out:

> *"v2 uses SIMD widely. Bumping up to 64-bit as a default would cut ALU
> throughput in the most expensive stages by around a factor of 2. … More
> concerning would be memory bandwidth. If, for example, the solver defaulted to
> storing everything in full 64 bit precision, bandwidth would become such a
> bottleneck that you'd likely only be able to scale to about 2-3 cores on a
> system with dual channel memory, even considering the ALU performance cut."*
> — Ross Nordby, bepu/bepuphysics2#13

Concretely in this codebase:

- The whole solver is built on `Vector<float>` bundles. `Vector3Wide`,
  `QuaternionWide`, `Symmetric3x3Wide`, `BodyVelocityWide`, `BodyInertiaWide` and
  every constraint's prestep/solve operate on `Vector<float>`
  (`BepuUtilities/Vector3Wide.cs:9`, the ~57 files in `BepuPhysics/Constraints/`).
  Doubling the element size **halves the SIMD lane count** (AVX: 8→4 lanes),
  doubling the number of bundles per constraint batch.
- The body data layout is hand-tuned for cache behavior: `BodyDynamics` is a
  128-byte block deliberately co-locating motion state and inertia for L2
  prefetch (`BepuPhysics/BodyProperties.cs:312-338`). FP64 positions would inflate
  these hot structures and defeat that tuning.
- The gather/scatter fast paths use hardcoded 8-wide AVX shuffles
  (`BepuPhysics/Bodies_GatherScatter.cs`), which assume a float lane width.

**The decisive insight: you don't need high precision in the solver or narrow
phase at all,** because they already operate in *relative* space.

### The narrow phase is already precision-robust

Collision detection never receives two absolute world positions. It receives the
*offset* between the two bodies:

```text
BepuPhysics/CollisionDetection/NarrowPhase.cs:562  …, poseB.Position - poseA.Position, poseB.Orientation, …
BepuPhysics/CollisionDetection/NarrowPhase.cs:576  var offsetB = poseB.Position - poseA.Position + (velocityB.Linear - velocityA.Linear) * t1;
BepuPhysics/CollisionDetection/NarrowPhase.cs:591  poseB.Position - poseA.Position, poseA.Orientation, …
```

So a contact between two bodies 10 m apart has the **same** precision whether
they sit at the origin or at (1e6, 1e6, 1e6): the error scales with the distance
*between* the bodies, not with the distance from the world origin. This is why
high precision only ever needs to reach poses and the broad phase — and confirms
the upstream framing in section 1.

## 4. Where precision actually breaks today, and the usable range

FP32 has ~24 bits of mantissa. At distance `D` from the origin, the smallest
representable step is roughly `D · 2⁻²³`. With a target precision of 1 mm
(0.001 unit), single precision degrades past roughly **±16 000 units**
(Ross's own figure). Past ~10 km from origin, jitter and integration drift become
visible for human-scale objects.

The author enumerated the achievable ranges per representation (assuming a
0.001-unit target precision, bepu/bepuphysics2#13):

| Mode | Usable range | If 1 unit = 1 m |
|---|---|---|
| 1. 32-bit float (today) | 16 384 units | ~16 km |
| 2. 32-bit fixed-point pose, FP32 broad phase rounded conservatively | 81 920 units | ~82 km |
| 3. 32-bit fixed-point | 2 097 152 units | ~2 000 km |
| 4. 64-bit float poses | 8.79e12 units | ~8 light-hours (≈ Pluto's orbit) |
| 5. 64-bit fixed-point | 9e15 units | ~1 light-year |

Note that two FP32 simulations placed at different origins will **not** produce
bit-identical results, and accumulated collision/response error is
position-dependent. A fixed-point pose representation additionally buys
*translation-invariant determinism*, which a floating representation cannot
(several participants in #13 cared more about this than about raw range).

## 5. Two distinct goals — keep them separate

The issue conflates two things that have different best answers:

### Goal A — "much larger spaces" (bigger usable range)

A localized **high-precision-pose mode** is the right tool. Keep the solver and
narrow phase in FP32; change only:

1. **Pose storage** — `RigidPose.Position` and `Static.Pose.Position` become FP64
   or 64-bit fixed point (behind conditional compilation / a build flag, as the
   author intends).
2. **Pose integration** — `PoseIntegrator` accumulates position in the high
   precision type, then produces FP32 *relative* offsets for the solver
   (`BepuPhysics/PoseIntegrator.cs:113-119` and the wide path).
3. **Gather/scatter** — emit body-relative FP32 positions into the solver bundles
   (the masking work flagged at `Bodies_GatherScatter.cs:630`).
4. **Broad phase** — two sub-options:
   - *Medium worlds:* keep FP32 AABBs but round outward to the next FP32 value so
     the conservative bounds never under-report. Cheap; minimal broad-phase
     change. (Ross's option 2.)
   - *Extreme worlds:* widen the tree/AABB representation to high precision. This
     is the expensive part — the broad phase is intensely memory-bandwidth- and
     layout-sensitive (`BepuPhysics/Trees/Node.cs:35-36`,
     `BepuPhysics/CollisionDetection/BroadPhase.cs`), and the author notes it
     *"would come with a measurable performance penalty."*

Bounds, queries, and compound child placement (`Statics.cs:376/467`,
`Simulation_Queries.cs:102`, `Compound.cs:173`) must also be revisited — they are
already marked.

### Goal B — "seamless movement through huge spaces" (e.g. a whole planet/solar system)

For genuinely enormous, seamless worlds, **the recommended approach is exactly
what the issue proposes as an alternative**: keep FP32 simulation(s) and manage a
high-precision world origin *above* the engine. The author is explicit:

> *"Splitting simulations is indeed the current recommendation to deal with
> enormous worlds. Individual simulations operating in FP32 while having a
> meta-origin stored in arbitrary precision does work. I have no plans to include
> auto-distribution into the core library at this time … It would end up looking
> a lot more like general infrastructure than physics."*
> — Ross Nordby, bepu/bepuphysics2#13 (2020)

Two practical patterns:

- **Floating origin / origin re-centering:** keep a single FP32 simulation but
  periodically translate every body so the active region (e.g. around the player)
  stays near the origin, while an `Int64`/`BigInteger` meta-origin tracks the true
  position. Rendering is already typically camera-relative, so this composes
  cleanly.
- **Region/island splitting:** run multiple FP32 simulations, each owning a sector
  with its own origin, and hand objects between them at the seams. Good for
  servers; the hard part (border handover, objects spanning two regions) is
  application logic, not physics.

Both are best implemented as a **thin library on top of bepuphysics2**, not inside
it — which directly validates the issue author's intuition ("реализовывать
перемещение другими методами … другими библиотеками поверх этой").

## 6. Effort and risk estimate

| Scope | Difficulty | Notes |
|---|---|---|
| FP64/fixed-point **poses only**, FP32 conservative broad phase (Goal A, medium worlds, ~±2000 km) | **Medium** | Localized; seams already marked in code; conditional-compilation friendly. Author: *"isn't actually too difficult."* Touches pose storage, integrator, gather/scatter masking, bounds computation, queries. |
| High-precision **broad phase** too (Goal A, planetary/space scale) | **High** | Broad-phase tree is the most bandwidth/layout-sensitive subsystem; measurable perf penalty; the genuinely hard part. |
| Full FP64 **everywhere** | **High and ill-advised** | ~2× ALU cost in hot stages + memory-bandwidth wall (scales to ~2–3 cores). Not recommended. |
| Floating origin / region splitting **on top of FP32** (Goal B) | **Low–Medium, outside the engine** | No core changes; the recommended path for truly seamless huge worlds. |

Surface area for a core change is broad in *reference count* (~130 files mention
`Vector3`/`Vector<float>`) but narrow in *conceptual scope*: only world-space pose
and broad-phase code paths carry absolute coordinates; the solver and narrow phase
stay FP32 by design.

## 7. Recommendation

1. **Do not** convert the engine to FP64 wholesale — it sacrifices the SIMD/memory
   design that makes bepuphysics2 fast, for no benefit the localized approach can't
   provide.
2. **If** the requirement is "a moderately larger single world" (up to ~thousands
   of km), implement a **conditional high-precision pose mode** plus a
   **conservative FP32 broad phase**. This is the medium-effort, architecturally
   anticipated path, and upstream issue #13 is the design reference.
3. **If** the requirement is "seamless movement across a planet/solar-system-scale
   world," build a **floating-origin / region-splitting layer on top** of an
   unmodified FP32 engine. This is the maintainer's standing recommendation and is
   the most robust answer to the issue's own proposed alternative — one FP32 space
   per region/server, with movement handled by an outer coordinator.
4. **Before** doing core work, follow `CONTRIBUTING.md` and coordinate with
   upstream (issue #13) — a conditional-compilation precision mode is a
   maintenance-sensitive change the original author has strong opinions about.

## References (in-repo)

- `Documentation/roadmap.md:31` — roadmap item #15 (high-precision poses)
- `BepuPhysics/BodyProperties.cs:48-50` — pose kept non-FP32-ready by design
- `BepuPhysics/BodyProperties.cs:54-68`, `:11-46`, `:312-338` — pose / motion
  state / cache-tuned body layout
- `BepuUtilities/Vector3Wide.cs:9-22` — FP32 SIMD bundle type
- `BepuPhysics/Statics.cs:376`, `:467` — absolute-coordinate bound sites flagged
- `BepuPhysics/Bodies_GatherScatter.cs:630` — high-precision gather/scatter note
- `BepuPhysics/Collidables/Compound.cs:173`, `BepuPhysics/Simulation_Queries.cs:102`
  — further high-precision TODOs
- `BepuPhysics/CollisionDetection/NarrowPhase.cs:562/576/591` — narrow phase uses
  relative offsets (precision-robust)
- `BepuUtilities/BoundingBox.cs:44-57`, `BepuPhysics/Trees/Node.cs:7-36` — FP32
  broad-phase AABBs

## References (upstream design discussion)

- [bepu/bepuphysics2#13 — High precision poses](https://github.com/bepu/bepuphysics2/issues/13)
  (39 comments; the canonical analysis by the original author and community)
