// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

// Programming of this file was assisted by OpenAI Codex 5.1/5.2/5.3 and Cursor
// Composer 2.0 Fast.
//
// Other files were all written by hand, without LLM assistance.

use std::ops::ControlFlow;

use anyangle::{ChainVertexType, Diagonal, Monotonizer, Point};
use macroquad::prelude::*;
use macroquad::rand::gen_range;
use polygon_unionfind::{Polygon, PolygonUnionFind};

/// Delay between monotonizer sweep steps.
const MONOTONIZER_STEP_DELAY_SECS: f64 = 0.1;

fn monotonizer_from_first_poly_geom(
    geom: &(Vec<[i64; 2]>, Vec<Vec<[i64; 2]>>),
) -> Option<Monotonizer<i64>> {
    let (ext, interiors) = geom;
    if ext.len() < 3 {
        return None;
    }
    let rings: Vec<Vec<[i64; 2]>> = std::iter::once(ext.clone())
        .chain(interiors.iter().cloned().filter(|ring| ring.len() >= 3))
        .collect();
    Some(Monotonizer::new(rings))
}

/// Monotone-chain convex hull; returns vertices in counter-clockwise order.
fn convex_hull(points: &[[i32; 2]]) -> Vec<[i32; 2]> {
    if points.len() < 3 {
        return points.to_vec();
    }

    let mut pts: Vec<[i32; 2]> = points.to_vec();
    pts.sort_by(|a, b| a[0].cmp(&b[0]).then_with(|| a[1].cmp(&b[1])));

    fn cross(o: [i32; 2], a: [i32; 2], b: [i32; 2]) -> i64 {
        (a[0] as i64 - o[0] as i64) * (b[1] as i64 - o[1] as i64)
            - (a[1] as i64 - o[1] as i64) * (b[0] as i64 - o[0] as i64)
    }

    let mut lower = Vec::new();
    for &p in &pts {
        while lower.len() >= 2 && cross(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 0 {
            lower.pop();
        }
        lower.push(p);
    }

    let mut upper = Vec::new();
    for &p in pts.iter().rev() {
        while upper.len() >= 2 && cross(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 0 {
            upper.pop();
        }
        upper.push(p);
    }

    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

fn random_convex_polygon_at_point(center: [i32; 2], radius: i32, count: usize) -> Vec<[i32; 2]> {
    let mut points = Vec::with_capacity(count);
    for _ in 0..count {
        let angle = gen_range(0.0f32, std::f32::consts::TAU);
        let r = gen_range(radius as f32 * 0.5, radius as f32);
        let x = center[0] + (r * angle.cos()) as i32;
        let y = center[1] + (r * angle.sin()) as i32;
        points.push([x, y]);
    }

    let mut hull = convex_hull(&points);
    if hull.len() < 3 {
        hull = vec![
            [center[0] - radius, center[1] - radius],
            [center[0] + radius, center[1] - radius],
            [center[0], center[1] + radius],
        ];
    }
    hull
}

fn polygon_from_ring_i32(ring: Vec<[i32; 2]>) -> Polygon<i64, ()> {
    Polygon {
        exterior: ring
            .into_iter()
            .map(|[x, y]| [i64::from(x), i64::from(y)])
            .collect(),
        interiors: vec![],
        weight: (),
    }
}

fn draw_diagonal(d: &Diagonal<i64>, center: Vec2, zoom: f32, thickness: f32, color: Color) {
    let from = d.from.vertex.this;
    let to = d.to.vertex.this;
    let a = center + vec2(from.x as f32, -from.y as f32) * zoom;
    let b = center + vec2(to.x as f32, -to.y as f32) * zoom;
    draw_line(a.x, a.y, b.x, b.y, thickness, color);
}

fn world_point_to_screen(p: Point<i64>, center: Vec2, zoom: f32) -> Vec2 {
    center + vec2(p.x as f32, -p.y as f32) * zoom
}

fn chain_vertex_type_label(t: ChainVertexType) -> &'static str {
    match t {
        ChainVertexType::Start => "Start",
        ChainVertexType::Split => "Split",
        ChainVertexType::Regular => "Regular",
        ChainVertexType::Merge => "Merge",
        ChainVertexType::End => "End",
    }
}

#[macroquad::main("Polygon Union-Find Viewer")]
async fn main() {
    let mut polygon_unionfind: PolygonUnionFind<i64> = PolygonUnionFind::new();

    let mut zoom = 1.0f32;
    let mut offset = vec2(0.0, 0.0);
    let mut last_mouse_pos: Option<Vec2> = None;
    let mut show_insert_hint = true;

    let mut monotonizer: Option<Monotonizer<i64>> = None;
    let mut monotonizer_diagonals: Vec<Diagonal<i64>> = Vec::new();
    let mut monotonizer_done = false;

    let mut monotonizer_first_poly_snapshot: Option<(Vec<[i64; 2]>, Vec<Vec<[i64; 2]>>)> = None;
    let mut monotonizer_auto_run = false;
    let mut monotonizer_step_accum: f64 = 0.0;

    loop {
        let (mx, my) = mouse_position();
        let left_pressed = is_mouse_button_pressed(MouseButton::Left);
        if show_insert_hint && left_pressed {
            show_insert_hint = false;
        }

        let center = vec2(screen_width() * 0.5, screen_height() * 0.5) + offset;

        let (_, scroll_y) = mouse_wheel();
        if scroll_y != 0.0 {
            zoom *= 1.0 + scroll_y * 0.1;
            zoom = zoom.clamp(0.1, 20.0);
        }

        if left_pressed {
            let click_world = vec2((mx - center.x) / zoom, -(my - center.y) / zoom);
            let radius = (60.0 / zoom).max(10.0).round() as i32;
            let count = gen_range(3, 10) as usize;
            let ring = random_convex_polygon_at_point(
                [click_world.x.round() as i32, click_world.y.round() as i32],
                radius,
                count,
            );

            polygon_unionfind.insert(polygon_from_ring_i32(ring));
        }

        // Monotonize the first inserted polygon (index 0), including its merge result — all rings.
        let first_poly_geom = if polygon_unionfind.raw_polygons().get(0).is_some() {
            let p = polygon_unionfind.find_compress(0);
            Some((p.exterior.clone(), p.interiors.clone()))
        } else {
            None
        };
        let needs_mono_reset = match (&monotonizer_first_poly_snapshot, &first_poly_geom) {
            (None, Some(_)) | (Some(_), None) => true,
            (Some(a), Some(b)) => a != b,
            (None, None) => false,
        };
        if needs_mono_reset {
            monotonizer_first_poly_snapshot = first_poly_geom.clone();
            monotonizer_diagonals.clear();
            monotonizer_done = false;
            monotonizer_auto_run = false;
            monotonizer_step_accum = 0.0;
            monotonizer = first_poly_geom
                .as_ref()
                .and_then(monotonizer_from_first_poly_geom);
        }

        if is_key_pressed(KeyCode::Enter) {
            if let Some(ref geom) = first_poly_geom {
                if let Some(m) = monotonizer_from_first_poly_geom(geom) {
                    monotonizer = Some(m);
                    monotonizer_diagonals.clear();
                    monotonizer_done = false;
                    monotonizer_auto_run = true;
                    monotonizer_step_accum = MONOTONIZER_STEP_DELAY_SECS;
                }
            }
        }

        if monotonizer_auto_run && !monotonizer_done {
            monotonizer_step_accum += get_frame_time() as f64;

            while monotonizer_step_accum >= MONOTONIZER_STEP_DELAY_SECS && !monotonizer_done {
                monotonizer_step_accum -= MONOTONIZER_STEP_DELAY_SECS;
                match &mut monotonizer {
                    Some(m) => match m.step() {
                        ControlFlow::Continue(diagonals) => {
                            monotonizer_diagonals.extend(diagonals);
                        }
                        ControlFlow::Break(()) => {
                            monotonizer_done = true;
                            monotonizer_auto_run = false;
                        }
                    },
                    None => {
                        monotonizer_done = true;
                        monotonizer_auto_run = false;
                    }
                }
            }
        }

        if is_mouse_button_down(MouseButton::Middle) {
            let (mx, my) = mouse_position();
            let current = vec2(mx, my);
            if let Some(previous) = last_mouse_pos {
                offset += current - previous;
            }
            last_mouse_pos = Some(current);
        } else {
            last_mouse_pos = None;
        }

        clear_background(BLACK);

        if show_insert_hint {
            let hint = "Click to insert a new polygon.";
            let hint_size = 22.0;
            let hint_dims = measure_text(hint, None, hint_size as u16, 1.0);
            draw_text(
                hint,
                (screen_width() - hint_dims.width) * 0.5,
                screen_height() - 32.0,
                hint_size,
                GRAY,
            );
        }

        let enter_hint = "Press Enter to monotonize red polygon.";
        let enter_size = 18.0;
        let enter_dims = measure_text(enter_hint, None, enter_size as u16, 1.0);
        draw_text(
            enter_hint,
            (screen_width() - enter_dims.width) * 0.5,
            screen_height() - if show_insert_hint { 64.0 } else { 32.0 },
            enter_size,
            GRAY,
        );

        for geom_with_data in polygon_unionfind.rtree().as_ref().iter() {
            let [bbox_min_x, bbox_min_y] = geom_with_data.geom().lower();
            let [bbox_max_x, bbox_max_y] = geom_with_data.geom().upper();

            let bbox_origin = center + vec2(bbox_min_x as f32, -bbox_max_y as f32) * zoom;
            let bbox_width = (bbox_max_x as f32 - bbox_min_x as f32) * zoom;
            let bbox_height = (bbox_max_y as f32 - bbox_min_y as f32) * zoom;
            draw_rectangle_lines(
                bbox_origin.x,
                bbox_origin.y,
                bbox_width,
                bbox_height,
                2.0,
                DARKGRAY,
            );
        }

        for (i, polygon) in polygon_unionfind.polygons().into_iter().enumerate() {
            let color = if i == 0 { RED } else { DARKGRAY };

            let rings: Vec<&[[i64; 2]]> = std::iter::once(polygon.exterior.as_slice())
                .chain(polygon.interiors.iter().map(Vec::as_slice))
                .collect();
            for ring in rings.iter().copied() {
                for window in ring
                    .iter()
                    .zip(ring.iter().cycle().skip(1))
                    .take(ring.len())
                {
                    let (from, to) = window;
                    let start = center + vec2(from[0] as f32, -from[1] as f32) * zoom;
                    let end = center + vec2(to[0] as f32, -to[1] as f32) * zoom;
                    draw_line(start.x, start.y, end.x, end.y, 3.0, color);
                }
            }
        }

        for d in &monotonizer_diagonals {
            draw_diagonal(d, center, zoom, 2.0, ORANGE);
        }

        if let Some(m) = monotonizer.as_ref() {
            let label_size = 16.0;
            for i in 0..m.sweep_vertex_count() {
                let Some(cv) = m.sweep_vertex(i) else {
                    continue;
                };
                let pos = world_point_to_screen(cv.this, center, zoom);
                let text = format!("{i}");
                let dims = measure_text(&text, None, label_size as u16, 1.0);
                let base_y = pos.y + dims.height * 0.35;
                draw_text(&text, pos.x - dims.width * 0.5, base_y, label_size, SKYBLUE);

                let ty = chain_vertex_type_label(cv.typ());
                let ty_size = 13.0;
                let ty_dims = measure_text(ty, None, ty_size as u16, 1.0);
                draw_text(
                    ty,
                    pos.x - ty_dims.width * 0.5,
                    base_y + label_size + 2.0,
                    ty_size,
                    LIGHTGRAY,
                );
            }

            if !monotonizer_done {
                let cur = m.current_sweep_index();
                if cur < m.sweep_vertex_count() {
                    if let Some(cv) = m.sweep_vertex(cur) {
                        let r = (12.0 * zoom).clamp(8.0, 28.0);
                        let r_adj = r * 0.88;

                        let pos_prev = world_point_to_screen(cv.prev, center, zoom);
                        draw_circle(
                            pos_prev.x,
                            pos_prev.y,
                            r_adj,
                            Color::from_rgba(255, 0, 255, 40),
                        );
                        draw_circle_lines(pos_prev.x, pos_prev.y, r_adj, 2.0, MAGENTA);

                        let pos_next = world_point_to_screen(cv.next, center, zoom);
                        draw_circle(
                            pos_next.x,
                            pos_next.y,
                            r_adj,
                            Color::from_rgba(0, 255, 160, 40),
                        );
                        draw_circle_lines(pos_next.x, pos_next.y, r_adj, 2.0, LIME);

                        let pos = world_point_to_screen(cv.this, center, zoom);
                        draw_circle(pos.x, pos.y, r, Color::from_rgba(255, 220, 0, 45));
                        draw_circle_lines(pos.x, pos.y, r, 2.5, YELLOW);
                    }
                }
            }
        }

        next_frame().await;
    }
}
