extern crate ggez;
extern crate lerp;
extern crate ncollide;
extern crate nphysics2d;
extern crate tiled;

use ggez::*;
use ggez::event::{EventHandler, Keycode, Mod};
use ggez::timer;
use ggez::graphics::{DrawMode, Matrix4, Point2, Rect, Vector2};
use ggez::nalgebra::Translation2;
use ggez::conf::{WindowMode, WindowSetup};

use nphysics2d::world::World;
use nphysics2d::object::{RigidBody, RigidBodyHandle};

use ncollide::shape::{Ball, Ball2, Cuboid, Cuboid2};

use lerp::Lerp;

const PADDLE_SPEED: f32 = 1000.0;

struct Paddle {
    dy: f32,
    rb: Option<RigidBodyHandle<f32>>,
}

impl Default for Paddle {
    fn default() -> Self {
        Paddle { dy: 0.0, rb: None }
    }
}

struct MainState {
    world: World<f32>,
    axis: f32,
    player: Paddle,
    ball: RigidBodyHandle<f32>,
    ai: Paddle,
}

fn make_cuboid_rb(object: &tiled::Object, dynamic: bool) -> RigidBody<f32> {
    let half_extents = match object.shape {
        tiled::ObjectShape::Rect { width, height } => Vector2::new(width, height) / 2.0,
        _ => panic!("cuboid must be rect"),
    };
    let cuboid = Cuboid::new(half_extents);

    let mut rb = if dynamic {
        RigidBody::new_dynamic(cuboid, 1.0, 1.0, 0.0)
    } else {
        RigidBody::new_static(cuboid, 1.0, 0.0)
    };

    rb.append_translation(&Translation2::new(
        object.x + half_extents.x,
        object.y + half_extents.y,
    ));
    rb
}

impl MainState {
    fn new(map: tiled::Map) -> Self {
        let mut world = World::new();

        let mut player = Paddle::default();
        let mut ai = Paddle::default();
        let mut ball = None;

        for group in &map.object_groups {
            for object in &group.objects {
                match object.obj_type.as_ref() {
                    "wall" => {
                        let mut rb = make_cuboid_rb(object, false);
                        rb.set_user_data(Some(Box::new(())));
                        world.add_rigid_body(rb);
                    }
                    "paddle" => {
                        let mut rb = make_cuboid_rb(object, true);
                        rb.set_inv_mass(0.0);

                        let handle = world.add_rigid_body(rb);
                        match object.name.as_ref() {
                            "player_paddle" => {
                                player.rb = Some(handle);
                            }
                            "ai_paddle" => {
                                ai.rb = Some(handle);
                            }
                            _ => panic!("unknown paddle name"),
                        }
                    }
                    "ball" => {
                        let radius = match object.shape {
                            tiled::ObjectShape::Ellipse { width, height } => {
                                width.hypot(height) / 2.0
                            }
                            _ => panic!("ball must be an ellipse"),
                        };
                        let mut rb = RigidBody::new_dynamic(Ball::new(radius), 1.0, 1.0, 0.0);
                        rb.append_translation(&Translation2::new(
                            object.x + radius,
                            object.y + radius,
                        ));
                        // rb.set_inv_mass(std::f32::MAX);
                        rb.set_lin_vel(Vector2::new(1000.0, 0.0));
                        ball = Some(world.add_rigid_body(rb));
                    }
                    _ => {}
                }
            }
        }

        MainState {
            world,
            player,
            ai,
            ball: ball.unwrap(),
            axis: 0.0,
        }
    }
}

impl EventHandler for MainState {
    fn update(&mut self, ctx: &mut Context) -> GameResult<()> {
        const DESIRED_FPS: u32 = 60;
        const LERP_TWEAK: f32 = 10.0;

        while timer::check_update_time(ctx, DESIRED_FPS) {
            self.world.step(0.016);

            let vel = self.ball.borrow().lin_vel();
            println!("{:?}", vel.x.hypot(vel.y));

            let seconds = 1.0 / (DESIRED_FPS as f32);
            self.player.dy = self.player
                .dy
                .lerp(self.axis * PADDLE_SPEED, LERP_TWEAK * seconds);

            if let Some(ref mut player_rb) = self.player.rb {
                let mut rb = player_rb.borrow_mut();
                rb.set_lin_vel(Vector2::new(0.0, self.player.dy));
            }
        }
        Ok(())
    }

    fn key_down_event(&mut self, ctx: &mut Context, keycode: Keycode, _keymod: Mod, _repeat: bool) {
        match keycode {
            Keycode::Up => {
                self.axis = -1.0;
            }
            Keycode::Down => {
                self.axis = 1.0;
            }
            Keycode::Escape | Keycode::Q => ctx.quit().unwrap(),
            _ => (), // Do nothing
        }
    }

    fn key_up_event(&mut self, _ctx: &mut Context, _keycode: Keycode, _keymod: Mod, _repeat: bool) {
        self.axis = 0.0;
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult<()> {
        graphics::clear(ctx);

        for body in self.world.rigid_bodies() {
            let body = body.borrow();
            if body.user_data().is_some() {
                graphics::set_color(ctx, (100, 100, 100).into())?;
            } else {
                graphics::set_color(ctx, (255, 255, 255).into())?;
            }
            if let Some(shape) = body.shape().as_shape::<Cuboid2<f32>>() {
                let h = shape.half_extents();
                let pos = body.position().translation.vector;
                graphics::rectangle(
                    ctx,
                    DrawMode::Fill,
                    Rect {
                        x: pos.x - h.x,
                        y: pos.y - h.y,
                        w: h.x * 2.0,
                        h: h.y * 2.0,
                    },
                )?;
            } else if let Some(shape) = body.shape().as_shape::<Ball2<f32>>() {
                let pos = body.position().translation.vector;
                let radius = shape.radius();
                graphics::circle(
                    ctx,
                    DrawMode::Fill,
                    Point2::from_coordinates(pos),
                    radius,
                    0.1,
                )?;
            }
        }

        graphics::present(ctx);
        Ok(())
    }
}

pub fn main() {
    let cb = ContextBuilder::new("pong", "TatriX")
        .window_setup(WindowSetup::default().title("Pong!").samples(8).unwrap());

    let ctx = &mut cb.add_resource_path("./resources").build().unwrap();

    let file = ctx.filesystem.open("/main.tmx").unwrap();
    let map = tiled::parse(file).unwrap();

    let scaling_factor = 0.5;
    let screen_width = (map.width * map.tile_width) as f32 * scaling_factor;
    let screen_height = (map.height * map.tile_height) as f32 * scaling_factor;

    graphics::set_mode(
        ctx,
        WindowMode::default().dimensions(screen_width as u32, screen_height as u32),
    ).unwrap();
    graphics::set_screen_coordinates(ctx, Rect::new(0.0, 0.0, screen_width, screen_height))
        .unwrap();

    graphics::set_transform(ctx, Matrix4::new_scaling(scaling_factor));
    graphics::apply_transformations(ctx).unwrap();

    // println!("{:#?}", map);

    let state = &mut MainState::new(map);
    event::run(ctx, state).unwrap();
}
