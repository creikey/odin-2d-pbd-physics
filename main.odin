package main

import "core:fmt"
import "core:mem"
import "core:math/linalg"
import "core:strings"
import "vendor:raylib"

import "cute_c2"

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

	inverse_mass: float,

	// for PBD bodies
	prev_pos: V2, 
	prev_ang: float,

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

make_rect_body :: proc(w: ^World, b: Body) -> GenHandle {
    new_body := allocate_from_pool(&w.bodies)
    new_shape := allocate_from_pool(&w.shapes)
    new_shape.size = V2{1, 1}
	old_gen := new_body.gen
	new_body^ = b
	new_body.exists = true
	new_body.gen = old_gen
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

body_world_to_local :: proc(b: Body, world: V2) -> V2 {
	// world = rotated(local) + pos
	// world - pos = rotated(local)
	// rotated_negative(world - pos) = local

	return rotate(world - b.pos, -b.ang)
}

// points are in world coordinates
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

camera := Camera{scale = 25.0} // 25 pixels = 1 meter

flip :: proc(v: V2) -> V2 {
	to_return := v
	to_return.y *= -1.0
	return to_return
}

into_world :: proc(v: V2) -> V2 {
    // screen =  (camera.offset + world)*scale
    // screen/scale =  camera.offset + world
    // screen/scale - camera.offset = world

	return (flip(v)/camera.scale - camera.offset)
}

into_screen :: proc(v: V2) -> V2 {
	return flip((camera.offset + v)*camera.scale)
}

// the world is for getting the shapes in the body
draw_body :: proc(w: World, body: Body, color: raylib.Color) {
    cur_shape, shape_ok := pool_dereference(w.shapes, body.shape_list).?
    for shape_ok {
        points := shape_points_in_body(body, cur_shape^)

        for i in 0..=len(points)-1 {
            raylib.DrawLineV(into_screen(points[i]), into_screen(points[(i + 1) % len(points)]), color)
        }

        cur_shape, shape_ok = pool_dereference(w.shapes, cur_shape.next).?
    }
}

dbgtext :: proc(text: cstring, at: V2, size: float, color: raylib.Color) {
    raylib.DrawText(text, cast(i32)at.x, cast(i32)at.y, cast(i32)size, color)
}

make_c2poly :: proc(body: Body, shape: Shape) -> cute_c2.c2Poly {
	points := shape_points_in_body(body, shape)
	to_return := cute_c2.c2Poly{count = 4}
	for i in 0..=3 {
		to_return.verts[i] = points[i]
	}
	cute_c2.c2MakePoly(&to_return)
	return to_return
}

DbgBlip :: struct {
	lifetime: f32
	at_world: V2,
}

debug: [dynamic]DbgBlip

dbgblip :: proc(point: V2) {
	found := false
	new_blip := DbgBlip{lifetime = 1.0, at_world = point}
	for d, index in debug {
		if d.lifetime == 0.0 { 
			debug[index] = new_blip
			found = true
			break
		}
	}
	if !found {
		append(&debug, new_blip)
	}
}

// as if v1 and v2 were 3d vectors, with no depth, and it returns
// the Z value of the resulting actual cross product
cross :: proc(v1, v2: V2) -> float {
	return v1.x*v2.y - v1.y*v2.x;
}

do_position_correction :: proc(world: ^World, from_body, other_body: ^Body) {
	from_shape, ok := pool_dereference(world.shapes, from_body.shape_list).?
	assert(ok)
	other_shape: ^Shape
	other_shape, ok = pool_dereference(world.shapes, other_body.shape_list).?
	assert(ok)

	out: cute_c2.c2Manifold = cute_c2.c2Manifold{}
	from_c2poly := make_c2poly(from_body^, from_shape^)
	to_c2poly := make_c2poly(other_body^, other_shape^)
	cute_c2.c2PolytoPolyManifold(&from_c2poly, nil, &to_c2poly, nil, &out)

	if out.count > 0 {
		for i in 0..=out.count-1 {
			delta_x := out.depths[i]
			r1 := body_world_to_local(from_body^, out.contact_points[i])
			r2 := body_world_to_local(other_body^, out.contact_points[i])

			n := out.n
			c := out.depths[i]

			w1 := cross(r1, n) * cross(r1, n) + from_body.inverse_mass
			w2 := cross(r2, n) * cross(r2, n) + other_body.inverse_mass

			lambda_delta := (-c) / (w1 + w2)

			p := lambda_delta * n

			from_body.pos += p * from_body.inverse_mass
			other_body.pos -= p * other_body.inverse_mass

			from_body.ang +=  cross(r1, p)
			other_body.ang -=  cross(r2, p)

			dbgblip(out.contact_points[i])
		}
	}
}

process :: proc(world: ^World, timestep: float) {
	num_sub_steps := 1

	for sub_step in 0..=num_sub_steps-1 {
		dt: float = timestep / cast(float)num_sub_steps

		for body, index in world.bodies {
			if body.exists {
				world.bodies[index].prev_pos = body.pos
				world.bodies[index].prev_ang = body.ang

				world.bodies[index].pos += body.vel * dt
				world.bodies[index].ang += body.angvel * dt
			}
		}

		// solve positions
		for from_body, from_index in world.bodies {
			if from_body.exists {
				for other_body, other_index in world.bodies {
					if other_index != from_index && other_body.exists {
						do_position_correction(world, &world.bodies[from_index], &world.bodies[other_index])
					}
				}
			}
		}

		for body, index in world.bodies {
			if body.exists {
				world.bodies[index].vel = (body.pos - body.prev_pos) / dt
				world.bodies[index].angvel = (body.ang - body.prev_ang) / dt
			}
		}
	}

	total_energy: f32 = 0.0
	for body in world.bodies {
		total_energy += 0.5 * (linalg.length(body.vel) * linalg.length(body.vel)) / body.inverse_mass
		total_energy += abs(body.angvel)
	}
	fmt.printf("Total Energy: %v\n", total_energy)
}

// state used by some scenes
SceneState :: struct {
	unprocessed_time: f64,
	paused: bool,
}


scene_position_testing_init :: proc(state: ^SceneState, world: ^World) {
    make_rect_body(world, Body{pos = V2{2, 1}, vel = V2{-1, 0}, inverse_mass = 1})
    make_rect_body(world, Body{pos = V2{-1, 1.5}, vel = V2{2, 0}, inverse_mass = 1})
}

scene_position_testing_process :: proc(state: ^SceneState, world: ^World) {
	stationary_body := &world.bodies[0]
	moving_body := &world.bodies[1]
	assert(stationary_body.exists)
	assert(moving_body.exists)
	stationary_body.pos = V2{0,0}
	stationary_body.ang = 0.0
	moving_body.pos = into_world(raylib.GetMousePosition())
	moving_body.ang = 0.0
	draw_body(world^, moving_body^, rgb(0, 1, 0))
	draw_body(world^, stationary_body^, rgb(0, 1, 0))
	do_position_correction(world, stationary_body, moving_body)
}

scene_simple_boxes_init :: proc(state: ^SceneState, world: ^World) {
    make_rect_body(world, Body{pos = V2{2, 1}, vel = V2{-1, 0}, inverse_mass = 1})
    make_rect_body(world, Body{pos = V2{-1, 1.5}, vel = V2{2, 0}, inverse_mass = 1})
}

scene_simple_boxes_process :: proc(state: ^SceneState, world: ^World) {
	if !state.paused {
		state.unprocessed_time += cast(f64) raylib.GetFrameTime()
		for state.unprocessed_time > TIMESTEP {
			defer state.unprocessed_time -= TIMESTEP
			process(world, TIMESTEP)
		}
	}
}

scene_chaos_init :: proc(state: ^SceneState, world: ^World) {
	hash11 :: proc(p: f32) -> f32 {
		p := p
		p = linalg.fract(p * .1031)
		p *= p + 33.33
		p *= p + p
		return linalg.fract(p)
	}

	random_symmetric :: proc(p: f32) -> f32 {
		p := p
		p *= 219.0
		return 2.0 * hash11(p) - 1.0
	}

	for i in 0..=50 {
		make_rect_body(world, Body{pos = V2{random_symmetric(cast(f32)i + 20.0), random_symmetric(cast(f32)i + 1298.0)}*10.0, vel = V2{random_symmetric(cast(f32)i + 1289.0),random_symmetric(cast(f32)i + 192.0)}, inverse_mass = 1})
	}
}

SceneInfo :: struct {
	name: string,
	init_function: proc(state: ^SceneState, world: ^World),
	process_function: proc(state: ^SceneState, world: ^World),
}

scenes := []SceneInfo {
	SceneInfo{"position testing", scene_position_testing_init, scene_position_testing_process}
	SceneInfo{"simple boxes", scene_simple_boxes_init, scene_simple_boxes_process}
	SceneInfo{"chaos", scene_chaos_init, scene_simple_boxes_process}
}

switch_to_scene :: proc(state: ^SceneState, world: ^World, to: SceneInfo) {
	clear(&world.bodies)
	clear(&world.shapes)
	state^ = SceneState{}
	world^ = World{}
	state.paused = true
	state.unprocessed_time = 0.0
	to.init_function(state, world)
}

main :: proc() {
    raylib.SetConfigFlags({raylib.ConfigFlag.WINDOW_RESIZABLE})
    screen_size := V2{1280, 720}
	camera.offset = into_world(screen_size/2.0)
    raylib.InitWindow(cast(i32)screen_size.x, cast(i32)screen_size.y, "Hello")
    fmt.println("What's up dawg")

	do_position_testing: bool = false

	// body world to local and local to world testing
	{
		approx_eq :: proc(v1, v2: V2) -> bool {
			dist := v1 - v2
			return linalg.length(dist) < 0.0001
		}
		b: Body
		b.pos = V2{2, 1}
		b.ang = 12.0
		equivalent_point := V2{2, 2}
		in_world := body_local_to_world(b, equivalent_point)
		back_to_local := body_world_to_local(b, in_world)
		//fmt.printf("%v - %v - %v\n", equivalent_point, in_world, back_to_local)
		assert(approx_eq(back_to_local, equivalent_point))
	}

    world := World{}
	state := SceneState{}
	cur_scene_index := 0
	cur_scene: SceneInfo = scenes[cur_scene_index]
	switch_to_scene(&state, &world, cur_scene)
	defer delete(world.bodies)
	defer delete(world.shapes)
    time: f64 = 0.0
	scene_notif_fade: f32 = 1.0

    for !raylib.WindowShouldClose() {
		// drawing can happen all the time only for debug purposes right now...
		raylib.BeginDrawing()
		defer {
			raylib.EndDrawing()
			free_all(context.temp_allocator)
		}
		
		cur_scene.process_function(&state, &world)


		// do general frame stuff and drawing stuff
        {
            // ui processing and drawing
			for d, index in debug {
				if d.lifetime > 0.0 {
					debug[index].lifetime -= raylib.GetFrameTime()*7.0
					raylib.DrawCircleV(into_screen(debug[index].at_world), 5.0, blendalpha(rgb(1,0,0), debug[index].lifetime))
				} else {
					debug[index].lifetime = 0.0
				}
			}
            if raylib.IsMouseButtonDown(raylib.MouseButton.LEFT) {
                camera.offset += flip(raylib.GetMouseDelta()/camera.scale)
            }
			if raylib.IsKeyPressed(raylib.KeyboardKey.SPACE) {
				state.paused = !state.paused
			}
			if raylib.IsKeyPressed(raylib.KeyboardKey.RIGHT) {
				cur_scene_index += 1
				cur_scene_index %= len(scenes)
				cur_scene = scenes[cur_scene_index]
				switch_to_scene(&state, &world, cur_scene)
				scene_notif_fade = 1.0
			}
			raylib.DrawText(strings.clone_to_cstring(cur_scene.name, context.temp_allocator), 200, 0, 40, blendalpha(rgb(1, 1, 1), scene_notif_fade))
			scene_notif_fade = linalg.lerp(scene_notif_fade, 0.0, raylib.GetFrameTime()*3.0)

			text: cstring
			color: raylib.Color
			if state.paused {
				text = "paused"
				color = rgb(1, 0, 0)
			} else {
				text = "unpaused"
				color = rgb(0, 1, 0)
			}
			raylib.DrawText(text, 0, 0, 15, color)

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
                    draw_body(world, body, rgb(1, 0, 0))
                }
            }

            raylib.ClearBackground(grey(0.1))
        }
    }

    raylib.CloseWindow()
}
