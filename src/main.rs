use nannou::prelude::*;
use nannou_conrod as ui;
use ui::prelude::*;
mod dbl_pendulum;

use dbl_pendulum::{DoublePendulumState, DoublePendulumSystem};

const LEN_SCALE: f64 = 100.;
const WIDTH: u32 = 1024;
const HEIGHT: u32 = 1024;

widget_ids! {
    struct Ids {
        title,
        g_label,
        g,
        m1_label,
        m1,
        m2_label,
        m2,
        l1_label,
        l1,
        l2_label,
        l2,
    }
}

struct Model {
    system: DoublePendulumSystem,
    state: DoublePendulumState,
    main_window: WindowId,
    ui: Ui,
    ids: Ids,
}

impl Model {
    fn step(&mut self, t: f64) {
        self.state = self.system.step(self.state, t);
    }

    /// Get the location of the top pendulum (relative to the pivot)
    fn top_pendulum_loc(&self) -> Vec2 {
        let (s, c) = self.state.θ1.sin_cos();
        Vec2::new(
            (s * self.system.l1 * LEN_SCALE) as f32,
            (c * self.system.l1 * LEN_SCALE) as f32,
        )
    }

    /// Get the location of the bottom pendulum (relative to the top pendulum)
    fn bottom_pendulum_loc(&self) -> Vec2 {
        let (s, c) = self.state.θ2.sin_cos();
        Vec2::new(
            (s * self.system.l2 * LEN_SCALE) as f32,
            (c * self.system.l2 * LEN_SCALE) as f32,
        )
    }
}

fn main() {
    nannou::app(model).update(update).run();
}

fn model(app: &App) -> Model {
    let main_window = app
        .new_window()
        .title(app.exe_name().unwrap())
        .size(WIDTH, HEIGHT)
        .view(view)
        .key_pressed(key_pressed)
        .build()
        .unwrap();

    let ui_window = app
        .new_window()
        .title(app.exe_name().unwrap() + " controls")
        .size(350, 400)
        .view(ui_view)
        .event(ui_event)
        .key_pressed(key_pressed)
        .build()
        .unwrap();

    let mut ui = ui::Builder::new(app).window(ui_window).build().unwrap();
    let ids = Ids::new(ui.widget_id_generator());

    ui.clear_with(color::DARK_CHARCOAL);
    let mut theme = ui.theme_mut();
    theme.label_color = color::WHITE;
    theme.shape_color = color::CHARCOAL;

    Model {
        system: Default::default(),
        state: DoublePendulumState::new(2., 2., 0., 0.),
        main_window,
        ui,
        ids,
    }
}

fn key_pressed(_app: &App, _model: &mut Model, key: Key) {}

fn update(_app: &App, model: &mut Model, update: Update) {
    model.step(update.since_last.as_secs_f64());
    //println!("{:?} {:?}", model.state, update.since_last.as_secs_f64());
}

fn view(app: &App, model: &Model, frame: Frame) {
    fn mass_to_size(mass: f64) -> f32 {
        10. + (mass as f32 - 1.) * 2.
    }

    frame.clear(BLACK);
    let draw = app.draw();
    let top = model.top_pendulum_loc();
    let btm = model.bottom_pendulum_loc();
    draw.translate(Vec3::new(0., 100., 0.));
    draw.line().x_y(0., 0.).end(-top).color(BLUE);
    draw.line().xy(-top).end(-btm).color(BLUE);
    draw.ellipse()
        .radius(mass_to_size(model.system.m1))
        .xy(-top)
        .color(srgb(1., 0., 0.));
    draw.ellipse()
        .radius(mass_to_size(model.system.m2))
        .xy(-top - btm)
        .color(srgb(1., 0., 0.));
    draw.to_frame(app, &frame).unwrap();
}

fn ui_event(_app: &App, model: &mut Model, _event: WindowEvent) {
    const LABEL_WIDTH: f64 = 175.;
    let ui = &mut model.ui.set_widgets();

    // Control panel title
    widget::Text::new("Double Pendulum")
        .top_left_with_margin(10.0)
        .w_h(300.0, 40.0)
        .font_size(24)
        .set(model.ids.title, ui);

    // Gravity label
    widget::Text::new("Gravity")
        .down_from(model.ids.title, 15.0)
        .w_h(LABEL_WIDTH, 30.0)
        .set(model.ids.g_label, ui);

    // Gravity slider
    for value in widget::Slider::new(model.system.g, 0.0, 20.0)
        .enabled(true)
        .right_from(model.ids.g_label, 10.0)
        .w_h(150.0, 30.0)
        .label(&format!("{:.4}", model.system.g))
        .set(model.ids.g, ui)
    {
        model.system.g = value;
    }

    // First pendulum mass label
    widget::Text::new("Pendulum 1 mass")
        .down_from(model.ids.g_label, 15.0)
        .w_h(LABEL_WIDTH, 30.0)
        .set(model.ids.m1_label, ui);

    // First pendulum mass slider
    for value in widget::Slider::new(model.system.m1, 0.1, 100.0)
        .enabled(true)
        .skew(8.)
        .right_from(model.ids.m1_label, 10.0)
        .w_h(150.0, 30.0)
        .label(&format!("{:.4}", model.system.m1))
        .set(model.ids.m1, ui)
    {
        model.system.m1 = value;
    }

    // First pendulum length label
    widget::Text::new("Pendulum 1 length")
        .down_from(model.ids.m1_label, 15.0)
        .w_h(LABEL_WIDTH, 30.0)
        .set(model.ids.l1_label, ui);

    // First pendulum length slider
    for value in widget::Slider::new(model.system.l1, 0.5, 5.0)
        .enabled(true)
        .right_from(model.ids.l1_label, 10.0)
        .w_h(150.0, 30.0)
        .label(&format!("{:.4}", model.system.l1))
        .set(model.ids.l1, ui)
    {
        model.system.l1 = value;
    }

    // Second pendulum mass label
    widget::Text::new("Pendulum 2 mass")
        .down_from(model.ids.l1_label, 15.0)
        .w_h(LABEL_WIDTH, 30.0)
        .set(model.ids.m2_label, ui);

    // First pendulum mass slider
    for value in widget::Slider::new(model.system.m2, 0.1, 100.0)
        .enabled(true)
        .skew(10.)
        .right_from(model.ids.m2_label, 10.0)
        .w_h(150.0, 30.0)
        .label(&format!("{:.4}", model.system.m2))
        .set(model.ids.m2, ui)
    {
        model.system.m2 = value;
    }

    // Second pendulum length label
    widget::Text::new("Pendulum 2 length")
        .down_from(model.ids.m2_label, 15.0)
        .w_h(LABEL_WIDTH, 30.0)
        .set(model.ids.l2_label, ui);

    // Second pendulum length slider
    for value in widget::Slider::new(model.system.l2, 0.5, 5.0)
        .enabled(true)
        .right_from(model.ids.l2_label, 10.0)
        .w_h(150.0, 30.0)
        .label(&format!("{:.4}", model.system.l2))
        .set(model.ids.l2, ui)
    {
        model.system.l2 = value;
    }
}

fn ui_view(app: &App, model: &Model, frame: Frame) {
    model.ui.draw_to_frame_if_changed(app, &frame).unwrap();
}
