mod optimization;
mod window;

#[cfg(test)]
mod tests;

pub use window::SlidingWindow;

pub struct Backend {
    pub sliding_window: SlidingWindow,
}

impl Backend {
    pub fn new(config: &crate::datasets::config::Config) -> Self {
        Self {
            sliding_window: SlidingWindow::from_config(config),
        }
    }
}
