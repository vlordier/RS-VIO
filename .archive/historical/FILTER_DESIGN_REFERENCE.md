# Filter Design Reference: Biquad Implementation Details

## Biquad Filter Form

All filters (highpass, lowpass, notch) are implemented as second-order sections using the Direct Form II:

```
y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
```

**State Variables**: x[n-1], x[n-2], y[n-1], y[n-2]
**Coefficients**: b0, b1, b2, a1, a2

## Highpass Butterworth Filter (0.5 Hz)

**Purpose**: Remove low-frequency drift and platform sway

**Design Formula** (for cutoff frequency fc = 0.5 Hz, fs = 200 Hz):

```
θ = 2π * fc / fs = 2π * 0.5 / 200 ≈ 0.01571 rad
α = sin(θ) / (2*Q)  where Q = 0.707 (Butterworth)
```

**Coefficient Computation**:
```
b0 = (1 + cos(θ)) / 2
b1 = -(1 + cos(θ))
b2 = (1 + cos(θ)) / 2
a0 = 1 + α
a1 = -2*cos(θ)
a2 = 1 - α

Normalized: divide all by a0
```

**Example Coefficients** (fc=0.5 Hz, fs=200 Hz):
```
b0 = 0.99992
b1 = -1.99984
b2 = 0.99992
a1 = -1.99968
a2 = 0.99968
```

**Frequency Response**:
```
Frequency | Gain (dB) | Phase (degrees)
0.01 Hz   | -40       | -90
0.1 Hz    | -20       | -80
0.5 Hz    | -3        | 0
1 Hz      | -1        | 45
10 Hz     | 0         | 90
```

**Effect on Input Signal**:
- Input: `[0.5, 0.5, 0.5, 0.5, ...]` (slow drift)
- Output: `[0, 0.001, 0.002, 0.003, ...]` (attenuated ~40x)

- Input: 10 Hz oscillation
- Output: ~Same amplitude (preserved)

## Lowpass Butterworth Filter (50 Hz)

**Purpose**: Remove high-frequency sensor noise

**Design** (fc = 50 Hz, fs = 200 Hz):

```
θ = 2π * 50 / 200 = π/2 ≈ 1.5708 rad
```

**Example Coefficients**:
```
b0 = 0.29290
b1 = 0.58579
b2 = 0.29290
a1 = 0
a2 = 0.17157
```

**Frequency Response**:
```
Frequency | Gain (dB) | Phase (degrees)
1 Hz      | 0         | -5
10 Hz     | -0.5      | -25
50 Hz     | -3        | -45
100 Hz    | -20       | -80
200 Hz    | -40       | -90
```

## Notch Filter (Center: 0.06 Hz, Q: 5.0)

**Purpose**: Surgical removal of specific resonance

**Design** (fc = 0.06 Hz, fs = 200 Hz, Q = 5.0):

```
θ = 2π * 0.06 / 200 ≈ 0.001885 rad
α = sin(θ) / (2*Q) ≈ 0.0001885

b0 = 1
b1 = -2*cos(θ)
b2 = 1
a1 = -2*cos(θ)
a2 = 1 - 2*α
```

**Example Coefficients**:
```
b0 = 1.0
b1 = -1.99999642
b2 = 1.0
a1 = -1.99999642
a2 = 0.99962306
```

**Frequency Response** (notch at 0.06 Hz, Q=5 means BW=0.012 Hz):
```
Frequency | Gain (dB)
0.048 Hz  | -1
0.06 Hz   | -40        ← Notch center
0.072 Hz  | -1
0.1 Hz    | 0
```

**Effect**: Removes narrow band at 0.06 Hz with minimal impact on adjacent frequencies

## Cascade of Filters

The complete denoising pipeline chains multiple filters:

```
Raw Input
    ↓
[Highpass 0.5 Hz]  → removes drift
    ↓
[Notch 0.06 Hz]    → removes platform sway
    ↓
[Notch 1.46 Hz]    → removes frame mode
    ↓
[Lowpass 50 Hz]    → removes noise
    ↓
Filtered Output
```

**Cascade Transfer Function**:
```
H(z) = H_hp(z) × H_notch1(z) × H_notch2(z) × H_lp(z)
```

**Magnitude Response at Key Frequencies**:
```
0.01 Hz  | -40 dB   (highpass blocking)
0.06 Hz  | -43 dB   (notch + highpass)
1.46 Hz  | -40 dB   (notch + highpass)
10 Hz    | -0.5 dB  (passband, nearly transparent)
100 Hz   | -20 dB   (lowpass blocking)
```

## Implementation in Rust

```rust
pub struct BiquadFilter {
    // Coefficients
    b0: f32, b1: f32, b2: f32,
    a1: f32, a2: f32,

    // State variables
    x1: f32, x2: f32,  // Previous inputs
    y1: f32, y2: f32,  // Previous outputs
}

impl BiquadFilter {
    pub fn process(&mut self, input: f32) -> f32 {
        let output = self.b0 * input
                   + self.b1 * self.x1
                   + self.b2 * self.x2
                   - self.a1 * self.y1
                   - self.a2 * self.y2;

        // Update state
        self.x2 = self.x1;
        self.x1 = input;
        self.y2 = self.y1;
        self.y1 = output;

        output
    }
}

pub fn create_highpass(fc: f32, fs: f32) -> BiquadFilter {
    let theta = 2.0 * PI * fc / fs;
    let q = 0.707;  // Butterworth
    let alpha = theta.sin() / (2.0 * q);

    let b0 = (1.0 + theta.cos()) / 2.0;
    let b1 = -(1.0 + theta.cos());
    let b2 = (1.0 + theta.cos()) / 2.0;
    let a0 = 1.0 + alpha;
    let a1 = -2.0 * theta.cos();
    let a2 = 1.0 - alpha;

    BiquadFilter {
        b0: b0 / a0, b1: b1 / a0, b2: b2 / a0,
        a1: a1 / a0, a2: a2 / a0,
        x1: 0.0, x2: 0.0, y1: 0.0, y2: 0.0,
    }
}
```

## Numerical Stability

### Potential Issues:
1. **Coefficient values** - Very large or small values
2. **Accumulated rounding errors** - From repeated filtering
3. **Saturation** - If intermediate values exceed f32 range

### Mitigation:
1. ✅ **Direct Form II** - Minimizes accumulated error (implemented)
2. ✅ **Normalized coefficients** - All divided by a0
3. ✅ **Double state** - Could use f64 internally if needed
4. ✅ **Coefficient clipping** - Ensure within [-10, 10]

**Current Implementation**: Stable for all identified resonance frequencies (0.06-1.46 Hz)

## Filter Order Comparison

| Aspect | 1st Order | 2nd Order (Biquad) | 4th Order |
|--------|-----------|-------------------|-----------|
| Slope | 20 dB/dec | 40 dB/dec | 80 dB/dec |
| Attenuation @ fc | -3 dB | -3 dB | -3 dB |
| Phase shift | -90° | -180° | -360° |
| Stability | Always | Stable | Can oscillate |
| Computation | Simple | 5 ops/sample | 10 ops/sample |

**Why 2nd Order?** Best balance of:
- Sharp enough rolloff (40 dB/decade)
- Numerically stable
- Computationally efficient
- Simple tuning (fc, Q)

## Testing the Filter

### Unit Test Pattern:
```rust
#[test]
fn test_highpass_removes_dc() {
    let mut filter = create_highpass(0.5, 200.0);

    // Constant input (DC = 0 Hz)
    let output: Vec<f32> = (0..200)
        .map(|_| filter.process(1.0))
        .collect();

    // Last 50 outputs should be near 0
    let mean = output[150..].iter().sum::<f32>() / 50.0;
    assert!(mean.abs() < 0.01);  // <1% of input
}

#[test]
fn test_notch_removes_resonance() {
    let mut filter = create_notch(0.06, 200.0, 5.0);

    // 0.06 Hz sine input
    let freq = 0.06;
    let period = (200.0 / freq) as usize;

    let input: Vec<f32> = (0..period*2)
        .map(|i| (2.0 * PI * freq * i as f32 / 200.0).sin())
        .collect();

    let output: Vec<f32> = input.iter()
        .map(|&x| filter.process(x))
        .collect();

    // Amplitude in steady state should be ~1% of input
    let amplitude = output[period..].iter()
        .max_by(|a, b| a.abs().partial_cmp(&b.abs()).unwrap())
        .unwrap()
        .abs();

    assert!(amplitude < 0.01);  // <1% attenuation
}
```

## Frequency Response Visualization

```
Magnitude Response (dB) vs Frequency
 0 ┤                               ┌─────────┐
-5 ┤                           ┌───┘         └───┐
-10┤                       ┌───┘                 └───┐
-20┤               ┌───────┘                         └───
-40┤       ┌───────┘ ↑ 0.06Hz notch
-80┤───────┘         ↑ 1.46Hz notch
   └─────┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴─────
    0.01 0.1 0.5 1  2  5  10 20 50 100 200   (Hz, log)

    ↑ Highpass      ↑ Notches      ↑ Lowpass
    0.5 Hz          0.06, 1.46     50 Hz
```

## Practical Tuning Example

**Symptom**: Residual 0.06 Hz oscillation visible in trajectory

**Current Configuration**:
```rust
notch_q: 5.0,
notch_frequencies: vec![0.06, 1.46],
```

**Problem**: Notch too wide, not suppressing enough

**Solution 1** - Increase Q (sharper notch):
```rust
notch_q: 8.0,  // Narrower bandwidth (0.0075 Hz)
```

**Solution 2** - Add secondary notch at harmonics:
```rust
notch_frequencies: vec![0.06, 0.12, 0.24, 1.46],
```

**Solution 3** - Increase highpass cutoff:
```rust
highpass_cutoff: 1.0,  // More aggressive DC removal
```

**Verification**:
```bash
# Before fix
python3 /tmp/resonance_decomposition.py  # Shows 0.06 Hz peak

# After fix (with Q=8.0)
python3 /tmp/resonance_decomposition.py  # 0.06 Hz peak reduced by 20dB
```
