# Fusing COIN-LIO's Photometric Residual into EllipseLIO

*A technical report, written for a master's student familiar with LiDAR-inertial odometry
(LIO) and the basics of Kalman filtering, but not with the internals of these two systems.*

---

## 1. The problem: LiDAR-inertial odometry goes blind on smooth surfaces

A LiDAR-inertial odometry system estimates the 6-DoF pose of a robot by combining a high-rate
IMU (which predicts motion but drifts) with a LiDAR (which corrects that drift by registering
each scan against a map). Modern LIO systems such as **FAST-LIO2**, **EllipseLIO**, and
**COIN-LIO** do this inside an *iterated error-state Kalman filter* (IKFoM — more on this below).

The LiDAR correction works by matching points to **local surface geometry**: each scan point is
associated with a small plane in the map, and the filter is nudged so the point lies on that
plane. This is the classic **point-to-plane residual**. It works beautifully when the environment
is geometrically rich (corners, edges, varied surfaces).

It **fails** when the geometry is *degenerate* — when the visible surfaces don't constrain all
six degrees of freedom. The textbook cases:

- A long, smooth corridor or tunnel: nothing constrains motion *along* the corridor.
- A drone facing a large flat wall, an aircraft fuselage, or hovering above a flat surface
  (nadir): the plane constrains the direction *normal* to the surface (and you know your distance
  to it), but you can **slide freely in the two in-plane directions and in yaw** and the
  point-to-plane residual never complains. Worse, planar features *flood* the scan, so naive
  "do I have enough features?" health checks read **green** exactly when you are most degenerate.

This is the failure mode that motivates the whole project: how do you constrain those sliding
directions when the surface is geometrically featureless?

### The key idea: a flat surface is geometrically blank but *photometrically* textured

A LiDAR doesn't only measure range — it also measures **intensity** (a.k.a. reflectivity): how
strongly each beam reflects. A flat aircraft skin or a painted wall is *geometrically* flat but is
covered in **intensity texture**: rivet lines, panel seams, paint markings, scuffs, signage. That
texture moves predictably as the sensor moves, so it can pin down exactly the in-plane translation
and yaw that the geometry cannot.

**COIN-LIO** (ETH ASL, ICRA 2024) exploits this. It projects the LiDAR intensity returns into an
image and adds a **direct photometric residual** — the same idea as direct visual odometry (DSO),
but on the LiDAR's own reflectivity image instead of a camera. **EllipseLIO** (Oxford, 2026)
instead attacks degeneracy on the *geometric* side, with an adaptive "ellipsoid" residual and an
observability score (`obs_score`) that flags when the geometry is starving.

This report describes **grafting COIN-LIO's photometric residual onto EllipseLIO**, so a single
estimator gets both: EllipseLIO's degeneracy-aware geometric front-end *and* COIN-LIO's
intensity-based constraint for when geometry alone isn't enough.

---

## 2. Background you need

### 2.1 The IKFoM measurement model in one paragraph

Both systems are built on **IKFoM** (Iterated Kalman Filter on Manifolds, the engine inside
FAST-LIO2). You don't need its full machinery — just this contract. The filter keeps a state
`x` (position, orientation, IMU-to-LiDAR extrinsics, velocity, IMU biases, gravity). The IMU
*predicts* `x` forward. To *correct* it with a LiDAR scan, you supply a single callback that, for
the current state estimate, returns:

- **`h`** — a vector of *residuals* (how wrong the current estimate is, one entry per measurement),
- **`h_x`** — the *Jacobian* of those residuals with respect to the state (∂h/∂x),

and the filter computes a Kalman update `δx` that reduces the residuals, **iterating** a few times
(re-linearising at the new estimate each time). The update is *information-form*:

```
(P⁻¹ + Hᵀ R⁻¹ H) · δx = Hᵀ R⁻¹ · (−h)
```

where `P` is the prior covariance and `R` is the measurement noise. The crucial intuition for this
report: **`Hᵀ R⁻¹ H` is how strongly each measurement pulls on the state.** A measurement with a
large Jacobian, or a small noise (large `R⁻¹`), dominates the update. Keep that in mind — it
becomes the central tuning problem in §6.3.

### 2.2 The geometric (EllipseLIO) residual, at a glance

EllipseLIO's callback is `TensorRegistration(state, ekfom_data)`. For each scan point it finds the
nearest map point, decides whether the local neighbourhood looks like a plane / line / blob
(via *tensor voting*), and builds a residual measuring how far the point is from that local
structure. Each residual row's Jacobian `h_x` is **6-dimensional** — three columns for position,
three for rotation — and each row carries a **weight** combining recency, gravity alignment, and
the observability score. The geometric residuals all get stacked into `h` / `h_x`, and the filter
updates. **We will append the photometric residuals to the bottom of those same stacks.**

### 2.3 The direct photometric residual, at a glance

A "feature" here is a small patch of pixels in the LiDAR intensity image, together with the 3D
positions of the points behind those pixels. When a feature is first detected, we record (a) the
3D point in the **global frame** and (b) the **reference intensity** at its pixel.

On a later scan, we take that stored global point, transform it into the current LiDAR frame using
the current state estimate, project it back into the *new* intensity image, and read the intensity
there. If the pose estimate is correct, the new intensity matches the reference. The mismatch is
the residual:

```
r = I_current( π( p_LiDAR(x) ) )  −  I_reference
```

where `π(·)` is the projection of a 3D point into the image. This residual depends on the state
`x` (because the projected pixel moves as the pose changes), so it produces a Jacobian we can feed
to the filter — exactly the same contract as the geometric residual.

### 2.4 Why this fusion is a *port*, not a *rewrite*

The reason this is tractable at all: **EllipseLIO and COIN-LIO both descend from FAST-LIO2 and use
an identical state definition** (`state_ikfom`) and the **same IKFoM `esekfom` update engine**.
Verified by diffing the two repos, the state manifolds are byte-for-byte equivalent (position,
rotation, both extrinsics, velocity, biases, an `S2` gravity manifold). That means:

- No change to the state vector is needed to add the photometric channel.
- COIN-LIO's photometric residual already produces a Jacobian in the **same 6-DoF (pos+rot)
  layout** EllipseLIO uses — it never touches the extrinsic columns.
- The measurement-model callback has the same signature in both.

So the work is: **lift COIN-LIO's image/feature/residual machinery into EllipseLIO's package, and
append its residual rows to EllipseLIO's existing measurement stack.** No new estimator.

---

## 3. System architecture

Per LiDAR scan, the fused pipeline runs:

```
                         ┌─────────────────────────── EllipseLIO (unchanged) ───────────┐
   /ouster/points  ──►   IMU de-skew  ──►  geometric residual (TensorRegistration)  ──►  IEKF update  ──►  map
   (full-res cloud)             │                         ▲                                   ▲
        │                       │                         │ stack rows                        │
        │   ┌───────────────────┴─── PHOTOMETRIC ADD-ON ──┴───────────────────────────┐       │
        └─► │ 1. build intensity image (Projector: project points → 64×1024 image)    │       │
            │ 2. process image (ImageProcessor: scale, normalise, gradients, mask)     │       │
            │ 3. residual: reproject stored features → r = I_cur − I_ref, Jacobian ────┼───────┘
            │ 4. AFTER update: detect/track features for the NEXT scan (FeatureManager)│
            └──────────────────────────────────────────────────────────────────────────┘
```

The ordering matters:

1. **Before** the IEKF update, build and process the current intensity image (so the residual can
   project features into it).
2. **During** the update, `TensorRegistration` computes geometric rows *and then* appends
   photometric rows for the features detected on the *previous* scan.
3. **After** the update (now that we have a refined pose), detect new features and track existing
   ones on the current image, ready for the next scan's residual.

This is the standard direct-odometry rhythm: features detected on frame *N* constrain the pose on
frame *N+1*.

---

## 4. Implementation walkthrough

All photometric code lives under `include/photometric/` and `src/photometric/`, plus the wiring in
`src/map_processing.cpp`. The three COIN-LIO modules were ported from ROS 1 to ROS 2 (namespaced
`photometric::`, ROS-handle constructors replaced with explicit parameters, no `ros::` includes).

### 4.1 The intensity image (`Projector`)

The image is a **range image**: rows = LiDAR beams (64 for an OS1-64), columns = azimuth bins
(1024). A 3D point is placed into it with a spherical projection that accounts for the Ouster's
beam-origin offset:

```
L     = √(x² + y²) − beam_offset        # horizontal range, beam-offset corrected
R     = √(z² + L²)                       # corrected 3D range
φ     = atan2(y, x)                      # azimuth  → column  u = K₀₀·φ + cₓ
θ     = asin(z / R)                      # elevation → row    v  (via a per-beam angle lookup)
```

Every point's intensity is written to its `(v, u)` pixel, building the intensity image; we also
store, per pixel, the index of the 3D point behind it (needed later to attach a feature to a real
point). **An important practical point:** this image must be built from the **full-resolution**
driver cloud (`/ouster/points`), *not* from EllipseLIO's internal cloud, which is heavily
downsampled (~11 % of pixels) and far too sparse to form a usable image. Empty / no-return pixels
are simply skipped. (See §6.1 — this was a real bug.)

### 4.2 Processing the image (`ImageProcessor`)

Raw LiDAR intensity is noisy and has strong per-beam brightness variation. `ImageProcessor`
applies: an intensity scale, an **adaptive brightness normalisation** (`140·I / (blur(I)+1)`,
which flattens beam-to-beam bias so a fixed gradient threshold works), an optional blur, and then
computes the **image gradients** `∂I/∂u`, `∂I/∂v` (used both to pick high-texture features and
inside the residual Jacobian). It also builds a validity **mask** (in-range, non-empty pixels).

### 4.3 Feature selection and tracking (`FeatureManager`)

Each scan, the manager:

- **Tracks** existing features into the new image (via normalised cross-correlation of their
  patches), dropping ones that fail.
- **Detects** new features to top up to a target count (default 65), choosing the highest-gradient
  pixels (the `"strongest"` mode) inside the valid mask, with non-maximum suppression so they
  spread out.

Each feature stores its patch's **reference intensities** and the **global-frame 3D positions** of
the points behind its pixels — the two things the residual needs.

### 4.4 The photometric residual and its Jacobian (the heart of it)

This is inserted directly into `TensorRegistration`, right after the geometric rows are written.
For each tracked feature pixel with stored global point `p_G` and reference intensity `I_ref`, at
the current iterated state `s`:

**Step 1 — bring the global point into the current LiDAR frame** (so we can project it):

```
p_I  = R_IG · (p_G − pos)              # world → IMU      (R_IG = sᵣₒₜᵀ)
p_Li = R_LI · (p_I − offset_T_L_I)     # IMU   → LiDAR     (R_LI = offset_R_L_Iᵀ)
```

**Step 2 — project and read the current intensity, forming the residual:**

```
(u, v) = π(p_Li)            # spherical projection from §4.1
r      = I_current(u, v) − I_ref
```

**Step 3 — the Jacobian, by the chain rule.** The residual depends on the state only through the
projected pixel, so:

```
∂r/∂x  =  ∂I/∂(u,v)  ·  ∂(u,v)/∂p_Li  ·  ∂p_Li/∂x
          └─ dI_du ─┘   └──  du_dp  ──┘   └─ dp_dtf ─┘
           (1×2)            (2×3)            (3×6)
```

- `dI_du` — the image gradient at the pixel (central differences).
- `du_dp` — the projection Jacobian (how the pixel moves when the 3D point moves); the closed form
  of the spherical model above.
- `dp_dtf` — how the LiDAR-frame point moves with the state. Using the right-perturbation
  convention that EllipseLIO's geometric rows use:

  ```
  ∂p_Li/∂pos = −R_LI · R_IG          (3×3, position columns 0–2)
  ∂p_Li/∂rot =  R_LI · ⌊p_I⌋ₓ        (3×3, rotation columns 3–5)   ⌊·⌋ₓ = skew-symmetric
  ```

Multiplying gives a **1×6 row** that fills exactly the position+rotation columns — the same layout
as the geometric rows. We store `h = −r` (matching EllipseLIO's geometric sign convention) and the
1×6 Jacobian. Rows where any term is non-finite are dropped (a single `Inf` would poison the whole
filter — see §6.2).

### 4.5 Stacking into EllipseLIO's weighted update

EllipseLIO's update is information-form with **per-row weights baked into the Jacobian**: it forms
`HᵀH = Σᵢ wᵢ · hᵢᵀhᵢ` via a pre-weighted Jacobian-transpose buffer (`h_x_R`). All we do is extend
the loop over `feat_tot` geometric rows to `feat_tot + n_photo` rows, giving each photometric row a
weight `w = photo_scale`. The existing weighting and Kalman-gain code then runs unchanged over the
combined stack. Conceptually:

```
H = [ H_geometric ]     ( ~thousands of rows, |h| ~ O(1) )
    [ H_photometric]     ( ~hundreds  of rows, |h| ~ O(10³), weighted by photo_scale )
```

That single shared update is the entire fusion: one estimator, two residual types, jointly solved.

---

## 5. How to read the code

| File | Role |
|---|---|
| `include/photometric/photometric_common.h` | Foundation: `LidarFrame`, `Feature`, types, bilinear sampler |
| `src/photometric/projector.cpp` | Spherical projection, builds the intensity image, projection Jacobian |
| `src/photometric/image_processing.cpp` | Intensity scaling, brightness normalisation, gradients, mask |
| `src/photometric/feature_manager.cpp` | Feature detection (`"strongest"`) + NCC tracking |
| `src/map_processing.cpp` → `InitPhotometric()` | Builds the modules + subscribes to `/ouster/points` |
| `src/map_processing.cpp` → `TimerCallback()` | The pipeline ordering of §3 |
| `src/map_processing.cpp` → `TensorRegistration()` | **The residual block of §4.4–4.5** (search "PHOTOMETRIC RESIDUAL ROWS") |
| `config/os1_64_ouster.yaml` → `photometric:` | Enable flag + all tunables |

---

## 6. Engineering challenges (the parts that actually took the time)

The algorithm is the easy half. Three problems consumed most of the effort, and each is
instructive.

### 6.1 The image was too sparse — *use the right cloud*

The first working version built the image from EllipseLIO's internal `scan_cloud_`, not realising
it is **downsampled** (~7 400 of 65 536 pixels, ~11 % coverage). The image was mostly empty; the
validity mask's erosion step (which needs solid neighbourhoods) wiped it out, and **zero features**
were ever detected — the photometric residual silently contributed nothing.

*Lesson:* a direct photometric method needs a **dense** image. The fix was to subscribe to the
full-resolution `/ouster/points` directly, buffer the clouds by timestamp, and feed the one
matching the current scan into the image builder. (A subtlety worth knowing: the *density* comes
from using the full driver cloud, not from the cloud being "organised" — points are projected
individually, so an organised or unorganised driver cloud both work.)

### 6.2 The filter diverged to NaN — *guard, then balance*

With features finally flowing, the trajectory went to **NaN**. Two distinct causes:

- **Non-finite Jacobians.** The projection Jacobian divides by terms that blow up for points near
  the sensor axis/origin, producing `Inf`. A single `Inf` row propagates through the matrix
  multiply and poisons the entire state. *Fix:* drop any row whose residual or Jacobian isn't
  finite, and skip non-finite (no-return) points when building the image.

- **Catastrophic over-weighting** (the real one — see next).

### 6.3 The weighting problem — *why `photo_scale ≈ 1e-9`*

This is the most important lesson, and a general one for fusing heterogeneous residuals. Recall
from §2.1 that a measurement's pull on the state scales with `wᵢ · |hᵢ|²`. Now compare the two
residual types:

- **Geometric** Jacobian rows have magnitude `|h| ~ O(1)` (they're built from unit surface
  normals).
- **Photometric** Jacobian rows are `dI_du · du_dp · dp_dtf`. The image gradient is `O(10²)` (an
  8-bit-ish image), the projection Jacobian is `O(K/range) ~ O(10²)`, so `|h| ~ O(10³)`.

In the information form, the photometric contribution `w·|h|²` is therefore `~10⁶×` larger than the
geometric one *for the same weight*. At a naïve weight, the photometric rows utterly dominate the
update; driven by a noisy image residual, the filter diverges. To make the photometric rows
**comparable** to (not dominant over) the geometric rows, the weight must be roughly
`1 / (10³)² = 10⁻⁶` smaller — i.e. `photo_scale ~ 10⁻⁹`.

A scale sweep on real data confirmed the analysis exactly:

| `photo_scale` | result |
|---|---|
| `1e-6` | diverges → NaN |
| **`1e-9`** | **stable; loop error 0.004 m, max step 0.058 m (≈ geometric baseline)** |
| `1e-12` | stable but the channel is effectively switched off |

*Lesson:* when stacking residuals of different physical units into one filter, you must match their
**information contributions**, not just pick "a small number". The right scale follows from the
Jacobian magnitudes. (A small but real trap: in YAML, `1e-9` parses as a *string*; you must write
`1.0e-9` or the parameter is silently rejected.)

---

## 7. Results and what they mean

Validated on a handheld OS1-64 dataset (an apartment) at `photo_scale = 1e-9`:

- The photometric residual is **active** (~350 rows added per filter update).
- The filter is **stable** — 0 NaN over the whole sequence.
- The fused trajectory **matches the geometry-only baseline** (loop-closure error 0.004 m vs
  0.003 m; max single-step 0.058 m vs 0.059 m).

At first glance "same as before" sounds like a non-result. It is exactly the **correct** v1
outcome, and understanding why is the point:

> In a feature-rich apartment, the geometry already fully constrains the pose. A well-behaved
> additional residual should **change nothing** there — it should neither help (no help is needed)
> nor hurt (it must not fight a healthy geometric solution). Demonstrating *stable and neutral on
> easy data* is the precondition for trusting it on hard data.

The **benefit** of the photometric channel only appears where geometry is degenerate — the blank
wall / fuselage / nadir cases of §1 — which this dataset does not contain. Quantifying that benefit
on a deliberately degenerate sequence is the next experiment, not a property you can read off an
easy bag.

---

## 8. Limitations of v1 and where to go next

Deliberately minimal first version; each simplification is a known lever:

1. **Image construction by splatting.** Points are projected individually, which leaves small gaps
   and collisions. The faithful COIN-LIO approach uses the organised grid index for a dense,
   collision-free image (and would then *require* an organised cloud).
2. **No rolling-shutter undistortion of the image.** COIN-LIO re-distorts feature points through a
   per-sub-scan transform; v1 uses identity. Fine for moderate motion, lossy at speed.
3. **`"strongest"`-gradient feature selection.** COIN-LIO's *complementary* mode picks features
   specifically in the geometrically **un**constrained direction — which is the whole point in a
   degenerate scene. EllipseLIO already computes that direction (its `obs_score` / observability
   vectors); wiring it into feature selection is the highest-value upgrade.
4. **Constant weight.** A natural v2 is an **observability-modulated** weight: lean on the
   photometric channel *only when* `obs_score` says the geometry is degenerate, and let geometry
   dominate when it's healthy. The hook (`photo_deg_gain`) is already stubbed in the code.
5. **Tuning on a degenerate scene.** Record a slow close pass at a blank, intensity-textured
   surface and raise `photo_scale` toward the stability boundary (between `1e-9` and `1e-6`), where
   the channel should visibly arrest the in-plane drift that geometry cannot see.

---

## 9. Glossary

- **LIO** — LiDAR-Inertial Odometry.
- **IKFoM** — Iterated Kalman Filter on Manifolds; the on-manifold iterated error-state EKF used by
  FAST-LIO2 and its descendants.
- **Residual / measurement model** — the function `h(x)` whose value the filter drives toward zero;
  its Jacobian `H = ∂h/∂x` tells the filter which way to move the state.
- **Point-to-plane residual** — geometric residual: signed distance from a scan point to its local
  map plane.
- **Direct photometric residual** — intensity difference between a reprojected point and its stored
  reference intensity; no feature *descriptors*, the image intensities are compared directly (cf.
  DSO).
- **Degeneracy** — a viewpoint where the measurements don't constrain all 6 DoF (e.g. a flat
  surface leaves the in-plane translation and yaw unobserved).
- **Information form** — Kalman update written in terms of `HᵀR⁻¹H` (the information matrix);
  convenient when stacking many measurements.
- **`obs_score`** — EllipseLIO's scalar observability/degeneracy signal.
- **`photo_scale`** — the constant weight applied to photometric residual rows (see §6.3).

## 10. References

- Border & Chli, *EllipseLIO: Adaptive LiDAR-Inertial Odometry with an Ellipsoid Representation*,
  arXiv 2026.
- Pfreundschuh et al., *COIN-LIO: Complementary Intensity-Augmented LiDAR Inertial Odometry*,
  ICRA 2024.
- Xu et al., *FAST-LIO2: Fast Direct LiDAR-Inertial Odometry*, and the IKFoM toolkit (HKU-MARS).
- Engel et al., *Direct Sparse Odometry (DSO)* — the direct-photometric idea this borrows.
