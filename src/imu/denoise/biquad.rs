/// Second-order Butterworth filter stage
#[derive(Debug, Clone)]
pub struct BiquadFilter {
    b0: f32,
    b1: f32,
    b2: f32, // Numerator coefficients
    a1: f32,
    a2: f32, // Denominator coefficients
    x1: f32,
    x2: f32, // Input history
    y1: f32,
    y2: f32, // Output history
}

impl BiquadFilter {
    pub fn identity() -> Self {
        Self {
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
            a1: 0.0,
            a2: 0.0,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }

    /// Create high-pass Butterworth filter
    pub fn highpass(sample_rate: f32, cutoff_hz: f32) -> Self {
        let wc = 2.0 * std::f32::consts::PI * cutoff_hz / sample_rate;
        let c = wc.cos();
        let s = wc.sin();
        let alpha = s / (2.0 * 0.707); // Q = 0.707 for Butterworth

        let b0 = (1.0 + c) / 2.0;
        let b1 = -(1.0 + c);
        let b2 = (1.0 + c) / 2.0;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * c / a0;
        let a2 = (1.0 - alpha) / a0;

        Self {
            b0: b0 / a0,
            b1: b1 / a0,
            b2: b2 / a0,
            a1,
            a2,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }

    /// Create low-pass Butterworth filter
    pub fn lowpass(sample_rate: f32, cutoff_hz: f32) -> Self {
        let wc = 2.0 * std::f32::consts::PI * cutoff_hz / sample_rate;
        let c = wc.cos();
        let s = wc.sin();
        let alpha = s / (2.0 * 0.707); // Q = 0.707 for Butterworth

        let b0 = (1.0 - c) / 2.0;
        let b1 = 1.0 - c;
        let b2 = (1.0 - c) / 2.0;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * c / a0;
        let a2 = (1.0 - alpha) / a0;

        Self {
            b0: b0 / a0,
            b1: b1 / a0,
            b2: b2 / a0,
            a1,
            a2,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }

    /// Create notch filter (band-stop)
    pub fn notch(sample_rate: f32, center_hz: f32, q: f32) -> Self {
        let wc = 2.0 * std::f32::consts::PI * center_hz / sample_rate;
        let c = wc.cos();
        let s = wc.sin();
        let alpha = s / (2.0 * q);

        let b0 = 1.0;
        let b1 = -2.0 * c;
        let b2 = 1.0;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * c / a0;
        let a2 = (1.0 - alpha) / a0;

        Self {
            b0: b0 / a0,
            b1: b1 / a0,
            b2: b2 / a0,
            a1,
            a2,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }

    /// Apply filter to a single sample
    pub fn process(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2;

        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;

        y
    }
}
