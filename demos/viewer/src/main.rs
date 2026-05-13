// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

// Programming of this file was entirely done by prompting Cursor Composer 2.0
// Fast.

use anyangle::{DelaunayNavmesh, Navmesher, PolygonWithData};
use macroquad::prelude::*;
use macroquad::rand::gen_range;

fn layer_color(layer_index: usize, layer_count: usize) -> Color {
    let hue_deg = (layer_index as f32 / layer_count.max(1) as f32) * 360.0;
    let (r, g, b) = hsv_to_rgb(hue_deg, 0.72, 1.0);
    Color::new(r, g, b, 1.0)
}

fn hsv_to_rgb(h: f32, s: f32, v: f32) -> (f32, f32, f32) {
    let c = v * s;
    let hh = (h / 60.0).rem_euclid(6.0);
    let x = c * (1.0 - (hh % 2.0 - 1.0).abs());
    let m = v - c;
    let (rp, gp, bp) = match hh as i32 {
        0 => (c, x, 0.0),
        1 => (x, c, 0.0),
        2 => (0.0, c, x),
        3 => (0.0, x, c),
        4 => (x, 0.0, c),
        _ => (c, 0.0, x),
    };
    (rp + m, gp + m, bp + m)
}

fn world_to_screen(p: [i32; 2], origin: Vec2, zoom: f32) -> Vec2 {
    vec2(origin.x + p[0] as f32 * zoom, origin.y - p[1] as f32 * zoom)
}

fn screen_to_world(screen: Vec2, origin: Vec2, zoom: f32) -> [i32; 2] {
    let wx = (screen.x - origin.x) / zoom;
    let wy = (origin.y - screen.y) / zoom;
    [wx.round() as i32, wy.round() as i32]
}

fn cross_i64(o: [i32; 2], a: [i32; 2], b: [i32; 2]) -> i64 {
    let oa_x = (a[0] - o[0]) as i64;
    let oa_y = (a[1] - o[1]) as i64;
    let ob_x = (b[0] - o[0]) as i64;
    let ob_y = (b[1] - o[1]) as i64;
    oa_x * ob_y - oa_y * ob_x
}

fn convex_hull(mut points: Vec<[i32; 2]>) -> Vec<[i32; 2]> {
    points.sort_by(|p, q| p[0].cmp(&q[0]).then_with(|| p[1].cmp(&q[1])));
    points.dedup_by(|a, b| a[0] == b[0] && a[1] == b[1]);
    if points.len() <= 2 {
        return points;
    }

    let mut lower = Vec::new();
    for &p in points.iter() {
        while lower.len() >= 2 && cross_i64(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 0
        {
            lower.pop();
        }
        lower.push(p);
    }

    let mut upper = Vec::new();
    for &p in points.iter().rev() {
        while upper.len() >= 2 && cross_i64(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 0
        {
            upper.pop();
        }
        upper.push(p);
    }

    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

fn random_convex_polygon(
    center: [i32; 2],
    radius_max: i32,
    count: usize,
) -> PolygonWithData<i32, ()> {
    let mut pts = Vec::with_capacity(count);
    for _ in 0..count {
        let angle = gen_range(0.0f32, std::f32::consts::TAU);
        let rf = gen_range(radius_max as f32 * 0.35, radius_max as f32);
        let x = center[0] as f32 + rf * angle.cos();
        let y = center[1] as f32 + rf * angle.sin();
        pts.push([x.round() as i32, y.round() as i32]);
    }

    let mut hull = convex_hull(pts);
    if hull.len() < 3 {
        let r = radius_max.max(12);
        hull = vec![
            [center[0] - r, center[1] - r],
            [center[0] + r, center[1] - r],
            [center[0], center[1] + r],
        ];
    }

    PolygonWithData {
        exterior: hull,
        interiors: Vec::new(),
        data: (),
    }
}

fn layer_row_top(layer_index: usize) -> f32 {
    let y0 = 96.0_f32;
    let h = 34.0_f32;
    let gap = 6.0_f32;
    y0 + layer_index as f32 * (h + gap)
}

fn ui_blocks_click(mouse: Vec2, n: usize) -> bool {
    if mouse.x >= 235.0 {
        return false;
    }
    let bottom = if n == 0 {
        94.0_f32
    } else {
        layer_row_top(n - 1) + 34.0 + 8.0
    };
    mouse.y >= 18.0 && mouse.y <= bottom
}

fn layer_panel_rect(layer_index: usize) -> Rect {
    let h = 34.0_f32;
    Rect::new(12.0, layer_row_top(layer_index), 220.0, h)
}

fn layer_radio_center(layer_index: usize) -> Vec2 {
    vec2(26.0, layer_row_top(layer_index) + 17.0)
}

fn layer_visibility_toggle_rect(layer_index: usize) -> Rect {
    let h = 34.0_f32;
    Rect::new(46.0, layer_row_top(layer_index), 174.0, h)
}

fn layer_radio_contains(mouse: Vec2, layer_index: usize) -> bool {
    layer_radio_center(layer_index).distance(mouse) <= 14.0
}

#[macroquad::main("Navmesher layers")]
async fn main() {
    let boundary = [[0_i32, 0], [420, 0], [380, 300], [200, 340], [40, 280]];
    let num_layers = 4_usize;
    let parallel_inflations = [12_i32, 28];

    let mut navmesher = Navmesher::<i32, DelaunayNavmesh<i32>, ()>::new(
        boundary,
        num_layers,
        parallel_inflations,
        core::iter::empty(),
    );

    let mut layer_visible = vec![true; navmesher.navmesh().layers().len()];
    let mut active_layer = 0_usize;

    let mut zoom = 1.8_f32;
    let mut origin = vec2(
        screen_width() * 0.5 - 200.0 * zoom,
        screen_height() * 0.5 + 150.0 * zoom,
    );
    let mut last_mouse: Option<Vec2> = None;

    loop {
        let (mx, my) = mouse_position();
        let mouse = vec2(mx, my);

        let (_, scroll_y) = mouse_wheel();
        if scroll_y != 0.0 {
            let zoom_prev = zoom;
            zoom *= 1.0 + scroll_y * 0.12;
            zoom = zoom.clamp(0.15, 12.0);
            let ratio = zoom / zoom_prev;
            origin.x = mouse.x - (mouse.x - origin.x) * ratio;
            origin.y = mouse.y + (origin.y - mouse.y) * ratio;
        }

        if is_mouse_button_down(MouseButton::Middle) {
            if let Some(prev) = last_mouse {
                origin += mouse - prev;
            }
            last_mouse = Some(mouse);
        } else {
            last_mouse = None;
        }

        let n = navmesher.navmesh().layers().len();

        if is_mouse_button_pressed(MouseButton::Left) {
            let mut consumed = false;
            for i in 0..n {
                if layer_radio_contains(mouse, i) {
                    active_layer = i;
                    consumed = true;
                    break;
                }
            }
            if !consumed {
                for i in 0..n {
                    if layer_visibility_toggle_rect(i).contains(mouse) {
                        layer_visible[i] = !layer_visible[i];
                        consumed = true;
                        break;
                    }
                }
            }
            if !consumed && !ui_blocks_click(mouse, n) {
                let center = screen_to_world(mouse, origin, zoom);
                let r_max = gen_range(28_i32, 96_i32);
                let poly = random_convex_polygon(center, r_max, 9);
                navmesher.insert_polygon(active_layer, poly);
            }
        }

        clear_background(BLACK);

        let mesh = navmesher.navmesh();

        for (layer_i, layer) in mesh.layers().iter().enumerate().rev() {
            if !layer_visible[layer_i] {
                continue;
            }

            let fill = layer_color(layer_i, n);
            let mut fill_a = fill;
            let active = layer_i == active_layer;
            fill_a.a = if active { 0.20 } else { 0.12 };

            let stroke = Color::new(fill.r * 0.6, fill.g * 0.6, fill.b * 0.6, 0.85);
            let line_w = if active { 2.35 } else { 1.2 };

            let tri = layer.triangulation();
            for verts in tri.vertices() {
                let a = world_to_screen(verts[0], origin, zoom);
                let b = world_to_screen(verts[1], origin, zoom);
                let c = world_to_screen(verts[2], origin, zoom);
                draw_triangle(a, b, c, fill_a);
                draw_triangle_lines(a, b, c, line_w, stroke);
            }
        }

        draw_text(
            &format!("layers: {} (semi-transparent overlay)", n),
            12.0,
            24.0,
            22.0,
            LIGHTGRAY,
        );
        draw_text(
            &format!("active layer: {}", active_layer),
            12.0,
            46.0,
            20.0,
            YELLOW,
        );
        draw_text(
            "middle-drag pan; wheel zoom; left = insert polygon (outside panel)",
            12.0,
            70.0,
            18.0,
            GRAY,
        );

        for i in 0..n {
            let r = layer_panel_rect(i);
            let on = layer_visible[i];
            let accent = layer_color(i, n);
            let bg = if on {
                Color::new(accent.r * 0.35, accent.g * 0.35, accent.b * 0.35, 0.92)
            } else {
                Color::new(0.12, 0.12, 0.12, 0.92)
            };
            draw_rectangle(r.x, r.y, r.w, r.h, bg);
            let c = layer_radio_center(i);
            draw_circle_lines(c.x, c.y, 11.0, 2.0, LIGHTGRAY);
            if active_layer == i {
                draw_circle(c.x, c.y, 6.5, accent);
            }
            draw_rectangle_lines(r.x, r.y, r.w, r.h, 2.0, LIGHTGRAY);
            let label = if on {
                format!("layer {} visible", i)
            } else {
                format!("layer {} hidden", i)
            };
            draw_text(&label, r.x + 44.0, r.y + 23.0, 18.0, WHITE);
        }

        next_frame().await;
    }
}
