package main

import "core:fmt"
import "core:mem"
import "core:math/linalg"
import "vendor:raylib"

rgb :: proc(r, g, b: f32) -> raylib.Color {
    return raylib.Color{cast(u8) (r*255.0), cast(u8) (g*255.0), cast(u8) (b*255.0), 255}
}

grey :: proc(brightness: f32) -> raylib.Color {
    return rgb(brightness, brightness, brightness)
}

blendalpha :: proc(col: raylib.Color, alpha: f32) -> raylib.Color {
    to_return := col
    to_return.a = cast(u8) (( (cast(f32)to_return.a / 255.0) * alpha) * 255.0)
    return to_return
}

V2 :: linalg.Vector2f32
float :: f32

TIMESTEP :: 1.0 / 60.0

// generational reference. generation = 0 is a null value
Generation :: int
GenHandle :: struct {
    index: int,
    gen: Generation,
}

Body :: struct {
    gen: Generation,
    exists: bool,

    pos, vel: V2,
    ang, angvel: float,
    
    shape_list: GenHandle,
}

Shape :: struct {
    gen: Generation,
    exists: bool,

    next: GenHandle,

    // offset from parent body
    offset: V2, 
    angle: float,

    // assumed to be rect shape for now
    size: V2, // halfsize, from the center of the rect shape
}

World :: struct {
    bodies: [dynamic]Body,
    shapes: [dynamic]Shape,
}

allocate_from_pool :: proc(p: ^[dynamic]$T) -> ^T {
    for elem, index in p {
        if !elem.exists {
            last_gen := elem.gen
            p[index] = T{}
            p[index].gen = last_gen + 1
            p[index].exists = true
            return &p[index]
        }
    }

    // no available holes
    append(p, T{})
    p[len(p) - 1].exists = true
    p[len(p) - 1].gen = 1
    return &p[len(p) - 1]
}

get_pool_handle :: proc(pool: [dynamic]$T, elem: ^T) -> GenHandle {
    index: = mem.ptr_sub(elem, &pool[0])
    assert(index >= 0)
    assert(index < len(pool))
    return GenHandle{index=index, gen=elem.gen}
}

pool_dereference :: proc(pool: [dynamic]$T, handle: GenHandle) -> Maybe(^T) {
    assert(handle.index >= 0)
    assert(handle.index < len(pool))
    if handle.gen == 0 { 
        return nil
    }
    if pool[handle.index].gen != handle.gen {
        return nil
    }
    if !pool[handle.index].exists {
        return nil
    }

    return &pool[handle.index]
}

make_rect_body :: proc(w: ^World, at: V2) -> GenHandle {
    new_body := allocate_from_pool(&w.bodies)
    new_shape := allocate_from_pool(&w.shapes)
    new_shape.size = V2{1, 1}
    new_body.pos = at
    new_body.shape_list = get_pool_handle(w.shapes, new_shape)
    return get_pool_handle(w.bodies, new_body)
}

rotate :: proc(v: V2, angle: float) -> V2 {
    return V2{
        linalg.cos(angle) * v.x - linalg.sin(angle) * v.y,
        linalg.sin(angle) * v.x + linalg.cos(angle) * v.y
    }
}

body_local_to_world :: proc(b: Body, point: V2) -> V2 {
    rotated := rotate(point, b.ang)
    return rotated + b.pos
}

shape_points_in_body :: proc(b: Body, s: Shape) -> [4]V2 {
    points := [4]V2{ 
        V2{-s.size.x, s.size.y},
        V2{ s.size.x, s.size.y},
        V2{ s.size.x,-s.size.y},
        V2{-s.size.x,-s.size.y},
    }
    for p, index in points {
        points[index] += s.offset
        points[index] = body_local_to_world(b, points[index])
    }
    return points
}

/*
    In world coordinates Y+ is up
    In screen and drawing coordinates Y+ is down
*/
Camera :: struct {
    offset: V2, // in meters
    scale: float,
}

camera := Camera{scale = 100.0} // 100 pixels = 1 meter

into_world :: proc(v: V2) -> V2 {
    // screen =  (camera.offset + world)*scale
    // screen/scale =  camera.offset + world
    // screen/scale - camera.offset = world

    return v/camera.scale - camera.offset
}

into_screen :: proc(v: V2) -> V2 {
    return (camera.offset + v)*camera.scale
}

// the world is for getting the shapes in the body
draw_body :: proc(w: World, body: Body) {
    cur_shape, shape_ok := pool_dereference(w.shapes, body.shape_list).?
    for shape_ok {
        points := shape_points_in_body(body, cur_shape^)

        for i in 0..=len(points)-1 {
            raylib.DrawLineV(into_screen(points[i]), into_screen(points[(i + 1) % len(points)]), rgb(1, 0, 0))
        }

        cur_shape, shape_ok = pool_dereference(w.shapes, cur_shape.next).?
    }
}

dbgtext :: proc(text: cstring, at: V2, size: float, color: raylib.Color) {
    raylib.DrawText(text, cast(i32)at.x, cast(i32)at.y, cast(i32)size, color)
}

process :: proc(world: ^World, timestep: float) {

}

main :: proc() {
    raylib.SetConfigFlags({raylib.ConfigFlag.WINDOW_RESIZABLE})
    screen_size := V2{1280, 720}
    raylib.InitWindow(cast(i32)screen_size.x, cast(i32)screen_size.y, "Hello")
    fmt.println("What's up dawg")

    world := World{}
    time: f64 = 0.0
    unprocessed_time: f64 = 0.0
    body := make_rect_body(&world, V2{0, 0})
    {
        body_unstable, ok := pool_dereference(world.bodies, body).?
        assert(ok)
        body_unstable.pos = V2{2, 1}
    }

    for !raylib.WindowShouldClose() {
        defer free_all(context.temp_allocator)

        // physics processing
        unprocessed_time += cast(f64) raylib.GetFrameTime()
        for unprocessed_time > TIMESTEP {
            defer unprocessed_time -= TIMESTEP
            process(&world, TIMESTEP)
        }

        {
            raylib.BeginDrawing()
            defer raylib.EndDrawing()

            // ui processing and drawing
            if raylib.IsMouseButtonDown(raylib.MouseButton.LEFT) {
                camera.offset += raylib.GetMouseDelta() / camera.scale
            }

            // draw reference grid in world
            grid_size: float = 5.0
            between_grid_lines: float = 1.0
            col := grey(0.3)
            raylib.DrawLineV(into_screen(V2{0, -grid_size}), into_screen(V2{0, grid_size}), col)
            raylib.DrawLineV(into_screen(V2{-grid_size, 0}), into_screen(V2{grid_size, 0}), col)
            for x := -grid_size; x <= grid_size; x += between_grid_lines {
                raylib.DrawLineV(into_screen(V2{x, grid_size}), into_screen(V2{x, -grid_size}), blendalpha(col, 0.3))
            }
            for y := -grid_size; y <= grid_size; y += between_grid_lines {
                raylib.DrawLineV(into_screen(V2{-grid_size, y}), into_screen(V2{grid_size, y}), blendalpha(col, 0.3))
            }
            // meters
            for x: f32 = 1.0; x <= grid_size; x += 1.0 { 
                dbgtext(fmt.ctprintf("%.1fm", x), into_screen(V2{x, 0}), 25, grey(0.6))
            }

            for body in world.bodies {
                if body.exists {
                    draw_body(world, body)
                }
            }

            raylib.ClearBackground(grey(0.1))

            raylib.DrawText("Congrats bro", 190, 200, 20, raylib.LIGHTGRAY)
            raylib.DrawLineV(V2{0, 0}, V2{100, 100}, rgb(0, 1, 0))
        }
    }

    raylib.CloseWindow()
}