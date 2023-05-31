package cute_c2

import "core:math/linalg"

c2v :: linalg.Vector2f32

foreign import lib {
	"cute_c2_impl.lib"
}

C2_MAX_POLYGON_VERTS :: 8

c2x :: struct {
	p, r: c2v,
}

c2Poly :: struct {
	count: i32,
	verts: [C2_MAX_POLYGON_VERTS]c2v,
	norms: [C2_MAX_POLYGON_VERTS]c2v,
}

c2Manifold :: struct {
	count: i32,
	depths: [2]f32,
	contact_points: [2]c2v,
	n: c2v,
}

@(default_calling_convention="c")
foreign lib {
	c2PolytoPolyManifold :: proc(A: ^c2Poly, ax: ^c2x, B: ^c2Poly, bx: ^c2x, m: ^c2Manifold) ---
	c2MakePoly :: proc(p: ^c2Poly) ---
}
