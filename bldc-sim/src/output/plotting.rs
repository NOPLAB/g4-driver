//! Plot generation for simulation data

use crate::motor_model::StateSnapshot;
use plotters::prelude::*;
use std::path::Path;

/// Plot configuration
#[derive(Debug, Clone)]
pub struct PlotConfig {
    /// Image width in pixels
    pub width: u32,
    /// Image height in pixels
    pub height: u32,
    /// Title for the plot
    pub title: String,
}

impl Default for PlotConfig {
    fn default() -> Self {
        Self {
            width: 1024,
            height: 768,
            title: "Motor Simulation".to_string(),
        }
    }
}

impl PlotConfig {
    /// Set dimensions
    pub fn with_size(mut self, width: u32, height: u32) -> Self {
        self.width = width;
        self.height = height;
        self
    }

    /// Set title
    pub fn with_title(mut self, title: impl Into<String>) -> Self {
        self.title = title.into();
        self
    }
}

/// Generate speed plot from simulation history
pub fn plot_speed<P: AsRef<Path>>(
    path: P,
    history: &[StateSnapshot],
    target_rpm: Option<f32>,
    config: &PlotConfig,
) -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::new(path.as_ref(), (config.width, config.height))
        .into_drawing_area();
    root.fill(&WHITE)?;

    // Find data range
    let t_max = history.last().map(|s| s.time).unwrap_or(1.0);
    let speed_max = history
        .iter()
        .map(|s| s.speed_rpm)
        .fold(0.0f32, |a, b| a.max(b))
        .max(target_rpm.unwrap_or(0.0))
        * 1.2;

    let mut chart = ChartBuilder::on(&root)
        .caption(&config.title, ("sans-serif", 30).into_font())
        .margin(10)
        .x_label_area_size(40)
        .y_label_area_size(60)
        .build_cartesian_2d(0.0f32..t_max, 0.0f32..speed_max)?;

    chart
        .configure_mesh()
        .x_desc("Time [s]")
        .y_desc("Speed [RPM]")
        .draw()?;

    // Plot speed
    chart
        .draw_series(LineSeries::new(
            history.iter().map(|s| (s.time, s.speed_rpm)),
            &BLUE,
        ))?
        .label("Speed")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLUE));

    // Plot target if provided
    if let Some(target) = target_rpm {
        chart
            .draw_series(LineSeries::new(
                [(0.0, target), (t_max, target)].iter().cloned(),
                &RED.mix(0.5),
            ))?
            .label("Target")
            .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], RED));
    }

    chart
        .configure_series_labels()
        .background_style(WHITE.mix(0.8))
        .border_style(BLACK)
        .draw()?;

    root.present()?;
    Ok(())
}

/// Generate multi-panel plot (speed, current, torque)
pub fn plot_multi_panel<P: AsRef<Path>>(
    path: P,
    history: &[StateSnapshot],
    config: &PlotConfig,
) -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::new(path.as_ref(), (config.width, config.height))
        .into_drawing_area();
    root.fill(&WHITE)?;

    let panels = root.split_evenly((3, 1));

    let t_max = history.last().map(|s| s.time).unwrap_or(1.0);

    // Panel 1: Speed
    {
        let speed_max = history
            .iter()
            .map(|s| s.speed_rpm)
            .fold(0.0f32, |a, b| a.max(b.abs()))
            * 1.2;

        let mut chart = ChartBuilder::on(&panels[0])
            .caption("Speed", ("sans-serif", 20).into_font())
            .margin(5)
            .x_label_area_size(30)
            .y_label_area_size(50)
            .build_cartesian_2d(0.0f32..t_max, 0.0f32..speed_max.max(1.0))?;

        chart.configure_mesh().y_desc("RPM").draw()?;

        chart.draw_series(LineSeries::new(
            history.iter().map(|s| (s.time, s.speed_rpm)),
            &BLUE,
        ))?;
    }

    // Panel 2: Current
    {
        let current_max = history
            .iter()
            .map(|s| libm::sqrtf(s.i_d * s.i_d + s.i_q * s.i_q))
            .fold(0.0f32, |a, b| a.max(b))
            * 1.2;

        let mut chart = ChartBuilder::on(&panels[1])
            .caption("Current", ("sans-serif", 20).into_font())
            .margin(5)
            .x_label_area_size(30)
            .y_label_area_size(50)
            .build_cartesian_2d(0.0f32..t_max, 0.0f32..current_max.max(0.1))?;

        chart.configure_mesh().y_desc("A").draw()?;

        chart
            .draw_series(LineSeries::new(
                history.iter().map(|s| (s.time, s.i_q)),
                &RED,
            ))?
            .label("Iq");

        chart
            .draw_series(LineSeries::new(
                history.iter().map(|s| (s.time, s.i_d)),
                &GREEN,
            ))?
            .label("Id");
    }

    // Panel 3: Torque
    {
        let torque_max = history
            .iter()
            .map(|s| s.torque.abs())
            .fold(0.0f32, |a, b| a.max(b))
            * 1.2;

        let mut chart = ChartBuilder::on(&panels[2])
            .caption("Torque", ("sans-serif", 20).into_font())
            .margin(5)
            .x_label_area_size(30)
            .y_label_area_size(50)
            .build_cartesian_2d(0.0f32..t_max, 0.0f32..torque_max.max(0.001))?;

        chart.configure_mesh().x_desc("Time [s]").y_desc("N·m").draw()?;

        chart.draw_series(LineSeries::new(
            history.iter().map(|s| (s.time, s.torque)),
            &MAGENTA,
        ))?;
    }

    root.present()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_history() -> Vec<StateSnapshot> {
        (0..100)
            .map(|i| {
                let t = i as f32 * 0.01;
                StateSnapshot {
                    time: t,
                    speed_rpm: 500.0 * (1.0 - libm::expf(-t * 5.0)),
                    omega_m: 0.0,
                    theta_m: 0.0,
                    theta_e: 0.0,
                    i_d: 0.1,
                    i_q: 2.0 * libm::expf(-t * 5.0),
                    torque: 0.05 * libm::expf(-t * 5.0),
                    rotations: 0,
                }
            })
            .collect()
    }

    #[test]
    fn test_plot_speed() {
        let history = make_test_history();
        let config = PlotConfig::default().with_title("Test Speed Plot");

        let path = "/tmp/bldc_sim_speed_test.png";
        let result = plot_speed(path, &history, Some(500.0), &config);

        assert!(result.is_ok());
        assert!(std::path::Path::new(path).exists());

        // Cleanup
        std::fs::remove_file(path).ok();
    }

    #[test]
    fn test_plot_multi_panel() {
        let history = make_test_history();
        let config = PlotConfig::default();

        let path = "/tmp/bldc_sim_multi_test.png";
        let result = plot_multi_panel(path, &history, &config);

        assert!(result.is_ok());
        assert!(std::path::Path::new(path).exists());

        // Cleanup
        std::fs::remove_file(path).ok();
    }
}
