Brian Wang

**Contents**

- [Problem](#problem)
- [Objective](#objective)
- [Constraints](#constraints)
- [Proposed Solution](#proposed-solution)
  - [Interfaces](#interfaces)
      - [Rocket Interface](#rocket-interface)
      - [Controller interface](#controller-interface)
        - [Simulink Exporting](#simulink-exporting)
      - [Sensor model](#sensor-model)
      - [State estimate](#state-estimate)
      - [Actuator model (within the plant)](#actuator-model-within-the-plant)
  - [Performance Aware Design Decisions](#performance-aware-design-decisions)
  - [Monte Carlo methodology](#monte-carlo-methodology)
      - [Selection of QMC Sampling](#selection-of-qmc-sampling)
      - [Error Modeling](#error-modeling)
      - [Epistemic Error Modeling](#epistemic-error-modeling)
      - [Aleatory Error Modeling](#aleatory-error-modeling)
      - [Approach](#approach)
        - [σ table:](#σ-table)
        - [Exploration Strategy (approach to sampling epistemic points)](#exploration-strategy-approach-to-sampling-epistemic-points)
        - [Example MonteCarlo Driver Config](#example-montecarlo-driver-config)
  - [Results and the Deliverable](#results-and-the-deliverable)
- [Verification and validation](#verification-and-validation)
- [Timeline](#timeline)
- [Out of Scope](#out-of-scope)
- [Other Notes](#other-notes)

## Problem
The current Simulink flight sim takes ~100s per flight on Rohan's machine and ~600s on Christopher's. Furthermore, the numbers we intend to test are already known to be guesses (the 'guesstimated values in `Models/Aero/initRocketAeroModel.m`'). There has been no way to quantify the rocket's stability margins in a reliable and fast way, i.e.,
> **How far can our aerodynamic coefficients and physical parameters be off before
> the rocket becomes unstable or the controller fails?'

## Objective
We seek to implement a full 6-DOF rocket flight simulator in C++ fast enough to run large ensembles of Monte Carlo. The same system will serve as a test harness for HPRC controller development and eventually drive a HITL rig.

## Constraints
- Simulations run from launch through drogue deployment. Real-time flights are ~300s total with roughly 40s to apogee. Results should be reproducible and bit-exact for a given binary (requires pining the compiler + C library in a container). Any run must be replayable and visualizable. Target run latency is 10ms with baseline of 100s. 
- Controllers are implemented in Simulink and can be exported in a C++ class. We assume all controllers run at the same control rate. 
- x86-64 Linux is priority, followed by Windows.

## Proposed Solution
![[Screenshot 2026-09-19 at 7.47.51 PM.png]]

### Interfaces
##### Rocket Interface
The simulation consumes a sensor config, state estimate error, N controllers, and actuator settings from config at runtime (in addition to the intervals and distributions being swept). 

We define a RunConfig for the Rocket (SensorConfig, FilterChoice, ControllerSet, ActuatorConfig). Below is a rough example:
```json
{
  "vehicle": {    // paths, not values — the yellow box
    "parameters":  "Data/30k_v2/parameters.json",
    "aero_tables": "Data/30k_v2/drag_curve.csv",
    "thrust":      "Data/30k_v2/thrust_source.csv"
  },

  "sim": { "dt_s": 0.001, "ctl_period_s": 0.01, "t_max_s": 60, "integrator": "rk4" },

  "sensors": { "imu": { "...": "..." }, "baro": {}, "mag": {}, "gps": {} },

  "estimate_error": {                   // truth + sampled error, no filter
    "att_bias_rad": 0.01, "att_drift_radps": 0.001,
    "vel_bias_mps": 0.5, "corr_time_s": 10.0, "latency_s": 0.02
  },

  "controllers": [
    { "name": "canard_lqr", "type": "GainSchedulerLQR", "rate_hz": 100,
      "drives": ["canard_yp", "canard_ym", "canard_zp", "canard_zm"],
      "params": { "gain_table": "Data/30k_v2/K.csv" } },
    { "name": "apogee_ctl", "type": "AirbrakeController", "rate_hz": 100,
      "drives": ["airbrake"],
      "params": { "target_apogee_m": 9144 } }
  ],

  "actuators": [ { "name": "canard_yp", "...": "..." } ],

  "events": {
    "apogee": { "method": "brent" },
    "deploy": { "rule": "vel_ned_window", "threshold_mps": 0.2,
                "source": "estimate", "t_end_buffer_s": 2.0 }
  },

  "logging": { "coarse_hz":0, "apogee_window_hz": 100 }
}
```
##### Controller interface
Controllers are to be declared with the actuator(s) they drive, the number of outputs, and a single update function called by both the sim and flight computer. 
```cpp
enum class Actuator { CANARD_YP, CANARD_YM, CANARD_ZP, CANARD_ZM, AIRBRAKE };

class IController {
public:
    virtual void reset() = 0;
    virtual const Actuator* drives() const = 0;     // which surfaces, declared once. Setup catches multiple controllers on same actuator
    virtual int  n_outputs() const = 0;
    virtual void update(const StateEstimate& x, float t, float dt, float* u) = 0;
    virtual ~IController() {}
};
```

Everything the controller needs should arrive through the constructor or update() arguments (ex. reference trajectories). We model the controller's gain (if the controller is stronger or weaker than designed) and delay (which we can then compute phase margin from). 

Note on implementation: each sim owns one array of actuator commands (where each controller has it's own offset determined before the run). When every controller has written, the sim passes the whole array to the plant, which holds it constant until the next control tick.

###### Simulink Exporting
Simulink controllers must be exported as a C++ class. For sims, we simply write a wrapper.

```cpp
template <class Gen>
class SimulinkController : public IController {
    Gen gen_;   // Simulink output, untouched
public:
    void reset() override { gen_.initialize(); }
    void update(const StateEstimate& x, float t, float dt, float* u) override {
        gen_.rtU.roll_rad = x.rpy[0];             // standard bus, same every model
        gen_.rtU.altitude_m = x.altitude;
        // ...
        gen_.step();
        for (int i = 0; i < n_; ++i) u[i] = gen_.rtY.u[i];
    }
};
```
Template + inheritance allows abstraction of which generated model and allows all controllers to sit in one array 🤩
If all models use the same Simulink bus object, this wrapper is written once. 

- Export Requirements (which later can be enforced on PR)
	- C++ class code interface packaging
	- disabled memory allocation
	- non-finite support enabled
	- MAT-file logging
	- fixed-step discrete solver
	- data type that matches the data type of the state estimate
	- hardware implementation matching the build target (x86-64)
##### Sensor model
We need to model:
- individual rate of the sensor
- sensor latencies, biases, noise
- mounting error
- dropout
- saturation/clipping

Each added sensor is simply a new struct + config entry in the RunConfig, along with a defined reading function from a truth vector. The "corruption" function of a perfect reading is simply defined once. 

```json
"sensors": {
  "imu":  { "rate_hz": 100, "latency_s": 0.0,
            "accel_noise_mps2": 0.05, "accel_bias_mps2": 0.1, "accel_bias_rw": 0.001,
            "gyro_noise_radps": 0.002, "gyro_bias_radps": 0.01, "gyro_bias_rw": 1e-5,
            "accel_range_g": 32, "gyro_range_dps": 2000,
            "misalign_rad": 0.002, "dropout_prob": 0.0 },
  "baro": { "rate_hz": 25, "latency_s": 0.02, "noise_pa": 5.0, "bias_pa": 20.0 },
  "mag":  { "rate_hz": 50, "...": "..." },
  "gps":  { "rate_hz": 1,  "latency_s": 0.1, "noise_m": 2.5 }
}
```


##### State estimate
We define a state estimate struct owned by the simulation driver, filled from truth plus a sampled error. There is no estimator module. Abhay's MEKF is tuned against real flight logs, so any sensor noise we invent here would only tune a filter against our own fiction. What we take from him instead is the error characterization from real flight residuals.
```cpp
void estimate(const State& truth, const EstError& err, double t, StateEstimate& out);
```
Error is bias, drift rate, correlation time, and latency. Zero error is a sample rather than a separate mode, so nominal and perturbed runs go through identical code. White noise is not the interesting part: zero-mean noise on the estimate averages out in the controller, while bias and latency eat phase margin. For HITL the real filter runs onboard and we send raw measurements instead.
##### Actuator model (within the plant)
To consider both the dynamics of the actuator and its effectiveness, we model the dynamics with first order lag, rate/movement limit, and position limit. Dynamics go straight into the integrator. The effectiveness is determined by an aerodynamics and environment submodule in the plant that takes in the rocket's physical configuration. 
```json
"actuators": [
  { "name": "canard_yp", "type": "canard",
    "phi_deg": 0, "x_m": 0.65, "area_m2": 0.0034, "cl_delta": 0.25,
    "tau_s": 0.02, "rate_max_radps": 5.2, "pos_lim_rad": [-0.26, 0.26] },
  { "name": "canard_ym", "phi_deg": 180, "...": "..." },
  { "name": "canard_zp", "phi_deg":  90, "...": "..." },
  { "name": "canard_zm", "phi_deg": 270, "...": "..." },
  { "name": "airbrake", "type": "airbrake", "x_m": 2.9,
    "cd_table": "airbrake_cd.csv",
    "tau_s": 0.15, "rate_max": 0.8, "pos_lim": [0.0, 1.0] }
]
```


### Performance Aware Design Decisions
- Analysis of the step size and convergence should be done early as possible. It is the biggest performance lever. 
- Interleave coefficient table. For a given mach and $\alpha$, we get each coefficient. For locality, we fetch them all in the same lookup. We trip the mach range too, which allows use of doubles. (Tables can still stay in L1). Cache the lookup cursor since the mach changes slowly between steps. 
- Divergent samples stop sooner than drogue deploy.
- Per-phase (aero, EOM, state estimate, etc.) timing counters are placed behind a compile flag. Not really a profiler, just accumulated counters. Allows targeted optimization. 
- Persistent worker pool that fetch from a atomic job queue. Writing of results requires no synchronization as each index is written by exactly one thread. At ~8 KB per record (summary + coarse trajectory + apogee window) a 20,000-sample sweep holds ~160 MB in RAM and writes it once, after every worker joins
- Pre-touch the results array before the sweep, or first-write page faults land on the sim thread inside the hot loop.
- Have each worker thread grabs 64 indices at a time (conventional chunk size, can be adjusted) that they then work on rather than grabbing one index at a time from the job queue in the MonteCarlo Driver. 

### Monte Carlo methodology

##### Selection of QMC Sampling
Plain MC has error of $\frac{\sigma}{\sqrt{N}}$. So standard error shrinks with $N^{-0.5}$. To halve the error you need 4x more samples. 

Quasi-Monte Carlo replaces random points with a low discrepancy sequence (a set of deterministic numbers that fills a space more evenly and with fewer gaps than random or pseudorandom numbers). Simply put, we get a standard error that shrinks with $N^{-1}$ instead. 
##### Error Modeling
We consider two types of uncertainty, Aleatory (inherent randomness modeled by a probability distribution) and Epistemic (uncertainty due to lack of knowledge, modeled with an interval). 
- Aleatory example: wind on launch day. Epistemic example: Coeff. of Normal Force at $8° \alpha$ 

Adding a new source of error requires a simple entry into the $\sigma$ table (described further below) and the parameter manifest. Based on the blast radius of the change, the impact can be as simple as tweaking a model (a couple lines of change) to refactoring code (adding new physics/dynamics). 

All QMC sampled values, along with a seed, are stored in a sample struct per simulation. 

The `Sample` struct is generated at build time via a python script on a YAML config file. Names, units, model consuming it are dclared in the YAML and the python script emits the enum, the struct, and the name table:

```cpp
enum Param : int { k_CN, k_CD, k_Cm, rail_angle, /* ... */ N_PARAMS };
inline constexpr const char* kParamNames[] = { "k_CN", "k_CD", ... };

struct Sample {
    double   v[N_PARAMS];
    uint64_t seed;
    constexpr double operator[](Param p) const { return v[p]; }
    void dump(FILE*) const;     // named output for logs and the debugger
};
```

Three things follow. The $\sigma$ table's `parameter` column is validated against `kParamNames` at load, so a typo becomes a startup error naming the row rather than a parameter silently dropped from the sweep. The sampler becomes a loop over `v[]` instead of one assignment per parameter. And the manifest and the $\sigma$ table are separate files on purpose: the manifest changes when new physics is added, the $\sigma$ table changes every study, and only the first triggers a rebuild. Every swept parameter is a continuous `double`. 
##### Epistemic Error Modeling
Interval selected throughs sources such as research papers, manufacturing tolerance/spec sheet, spread from estimation methods, or documented engineering judgement. 

| Source                           | Current value                                      | Lands in                                           |
| -------------------------------- | -------------------------------------------------- | -------------------------------------------------- |
| normal force multiplier          | 1.0                                                | Aerodynamics                                       |
| drag multiplier                  | 1.0                                                | Aerodynamics                                       |
| pitch moment multiplier          | 1.0                                                | Aerodynamics                                       |
| canard effectiveness `CL_delta`  | 0.25, labelled a guess                             | Aerodynamics                                       |
| roll damping `Cd_x`              | 0.3, labelled a guess                              | Aerodynamics                                       |
| pitch/yaw damping `Cd_y`, `Cd_z` | 0.5, labelled guesses                              | Aerodynamics                                       |
| CP location, transonic shift     | held constant through transonic                    | Aerodynamics                                       |
| controller gain error            | 1.0                                                | Controllers                                        |
| controller delay                 | 0                                                  | Controllers                                        |
| state estimate error             | bias, drift, corr time, latency — from Abhay       | State Estimate                                     |
| sensor dropout rate              | 0                                                  | Sensor Models                                      |
| IMU bias vs temperature coeff    | absent                                             | Sensor Models                                      |
| actuator `tau`                   | 0.02 s                                             | Actuator Dynamics                                  |
| actuator `rate_max`              | **0.2 vs 5.2 rad/s unresolved** (MATLAB vs Config) | Actuator Dynamics                                  |
| bearing coulomb                  | absent                                             | Spin can submodel in Mass/Inertia and Aerodynamics |
| bearing viscous                  | absent                                             | Spin can submodel in Mass/Inertia and Aerodynamics |

##### Aleatory Error Modeling
Access distributions from sources such as launch site climatology, manufacture specs, or build tolerance. 

| Source                    | Lands in                   |
| ------------------------- | -------------------------- |
| wind speed                | Aerodynamics + Environment |
| wind direction            | Aerodynamics + Environment |
| turbulence realisation    | Aerodynamics + Environment |
| launch day temperature    | Aerodynamics + Environment |
| launch day pressure       | Aerodynamics + Environment |
| rail angle                | Initial Conditions         |
| rail azimuth              | Initial Conditions         |
| motor lot to lot impulse  | Mass, Inertia + Propulsion |
| thrust misalignment angle | Mass, Inertia + Propulsion |
| mass                      | Mass, Inertia + Propulsion |
| lateral CG offset         | Mass, Inertia + Propulsion |
| inertia                   | Mass, Inertia + Propulsion |
| per fin cant error        | Aerodynamics               |
| sensor bias realisation   | Sensor Models              |
| sensor noise sequence     | Sensor Models              |
| which samples drop        | Sensor Models              |

##### Approach
We run a sample over a nested loop as the different sources of error that we do not combine because they represent fundamentally different types of uncertainty. Cost multiplies as `(epistemic points) × (aleatory samples)`, so keep the outer loop coarse: corners and a few interior points of the interval box, not a dense grid.

Raw output is a p-box: a bounded family of CDFs rather than one curve. So the deliverable is not *"3% of flights go unstable"* but *"between 1% and 9%, and the width of that band is the cost of not knowing our aerodynamics."* 

###### σ table:

| Column         | What it holds                                                               |
| -------------- | --------------------------------------------------------------------------- |
| `parameter`    | the binding key, must match the `Sample` field name exactly                 |
| `nominal`      | value when nothing is perturbed, `1.0` for every multiplier by construction |
| `variation`    | how far from nominal, read according to `distribution`                      |
| `distribution` | how to interpret `variation`, and which loop the row lives in               |
| `correlation`  | other parameters this one moves with, and how strongly                      |
| `bounds`       | hard physical limits, separate from `variation`                             |
| `applies_to`   | which model consumes it                                                     |
| `source`       | provenance, including "guess"                                               |
| `role`         | `swept` or `fixed`, carrying the evidence if fixed                          |

- Correlation column is required to prevent independent sampling from producing physically impossible vehicles that inflates the reported tolerance. 
- Given that we are using correlation, we use Shapley effects over Sobol indices (which assumes independence). 
###### Exploration Strategy (approach to sampling epistemic points)

| `strategy`  | Outer points                          | Gives                                                                      | Misses               |
| ----------- | ------------------------------------- | -------------------------------------------------------------------------- | -------------------- |
| `corners`   | `2^k` vertices of the box             | a bound, if monotone (P(fail) increasing/decreasing as single param moves) | nothing, if monotone |
| `levels`    | `k x levels`, one parameter at a time | a tolerance per parameter                                                  | interactions         |
| `hypercube` | `n` points scattered through the box  | interactions, an observed max                                              | no guarantee         |


###### Example MonteCarlo Driver Config
```json
{
  "name":        "k_CN_tolerance_v3",
  "run_config":  "config.json",
  "sigma_table": "sigma_table.csv",
  "output_dir":  "results/2026-09-19_kCN_v3",

  "explore": {
    "strategy": "levels",     // "levels" | "corners" | "hypercube"
    "levels":   25,           // used by "levels"
    "points":   500           // used by "hypercube"
  },

  "samples_per_point": 1000,  // aleatory draws at each outer point
  "seed":              20260919,

  "failure": {
    "margin_below_cal": 1.0,
    "alpha_above_deg":  25,
    "sustained_s":      0.5,
    "acceptable_rate":  0.05
  },

  "threads": 0, // 0 means all cores

  "output": {
    "animate_worst":  5,
    "animate_random": 3
  }
}
```




### Results and the Deliverable
The following is tentative. "Failure" has not been formally defined and require team lead input. Result format is also an open question for the team leads. 

Results of each simulation are written into memory. Once all simulations are finished, the MC thread writes a directory of results. Failures can be found by sorting summary.parquet by peak $\alpha$ and taking the worst ones. 

Note that provenance refers to git SHA, binary hash (of the executable), compiler + flags, CPU, libc, wall time. Allows results to be defended and reproduced. 

```
results/<timestamp>_<config_hash>/
  manifest.json      provenance
  config.json        the exact RunConfig
  sigma_table.csv    the exact σ table
  summary.parquet    one row per sample, the thing you query
  traj.bin           coarse trajectories, fixed stride, memory-mappable by index
  report.md          the deliverable, one per sweep
  figs/*.png         referenced by report.md
```

The report contains summary of run config, along with sensitivity to different coefficients/parameters, failure rates, failure-mode breakdown, and command to replay any sample.

## Verification and validation
Post verification (does the code solve the equations correctly), we can validate the simulations against: 
- RocketPy: Python 6-DOF, validated against real flights. Computes normal force from dimensions itself, but drag must be supplied. 
- jsbsim: Independent EOM and integrated. Not independent aero.
- Or upcoming flight data, wind tunnel data, ANSYS CFD. 


## Out of Scope
- The simulation focuses on testing the rocket's performance rather than elements such as MEKF implementation validation, Code quality CI. 
- Descent/parachute simulation is considered a reach end-of-year goal by Rohan. 
- MacOS deployment of the simulations. Priority is x86-64 Linux, followed by Windows. 
- Optimization such as SIMD (single instruction, multiple data), custom allocators (like object pools), lock free structures, GPU offload, distributed execution beyond a job pool are all out of scope. 
- An originally proposed "estimator model" where an MEKF/direct truth state could be interchangeable. Abhay, the student working on the MEKF, can provide empirical error bounds from real flight residuals. Integrating an MEKF into the sim yields nothing about estimator accuracy, since the sensor noise would be our own invention. So we scope hammered it down to no estimator module. The controller reads a state estimate filled from truth plus sampled error (bias, drift rate, correlation time, latency). 
- RTOS integration from the flight software. The sim calls the shared state machine and controllers once per control tick, as if the flight computer runs instantly. Scheduling delay is covered by the swept controller delay and estimate latency rather than a model of tasks and priorities. The real scheduler is only exercised in HITL, where the board runs its own firmware. Current flight software is an Arduino superloop with no RTOS anyway. 
- NASA has a library called Trick recommended by Rohan. It is more of a scheduling framework where you write models and its interface code generator parses your headers and schedules jobs. It was built for multiple subsystem models from many teams over decades, a pressure that does not exist with WPI's HPRC team. May revisit when designing HITL system. 
- Any built in screening (ex. Morris pass) to cut down on the number of epistemic parameters to those that actually matter. Any inputted epistemic parameters and intervals are assumed to be important to test. 
- We do not model chirp. We instead use gain and pure delay. The chirp would need to be at a frozen operating point that a rocket never holds. 
- We do not consider frequency-dependent uncertainty. We assume that coefficient error is frequency-flat across the loop bandwidth. Thus, any unsteady aerodynamics (flow lags a changing $\alpha$) and structural bending modes are absent. 
- A SysId optimizer wrapped around the plant that fits parameters to recorded flight telemetry. Out of scope for now. 
- for the HITL: the HITL would be single thread, real-time driver instead of MonteCarlo driver that logs overruns such as issues with OS scheduler preemption, page faults, cache misses, interrupt handling, transport latency. The driver would sleep until the start of the next time frame. The controllers would run on an onboard MEKF while the servos within the Gimbal are driven by the simulation plant. 
- Note the eventual goal of also having any changes to the open rocket file across subteams runs a set of C++ sims and automatically creates a PR to update the simulation config.


## Other Notes
- Scope of Multithreading: One thread per complete flight due to sequential nature of time stepping. Parallelism is to occur across the Monte Carlo samples.
- Flight computer: STM32H753ZIT6: Cortex-M7 at 240 MHz with a hardware double-precision FPU, 512 KB RAM, 2 MB flash.
- Development: I have a Mac machine and a mini-pc with x86-64 Linux Ubuntu. 
- No global mutable state in the plant. 
- No heap allocation inside integration loop.
- Seed per sample index, not per thread.
- Reproducibility by pinning the build. Record the build env in every results file. Determinism tests. 
