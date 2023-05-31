package cute_c2

import "core:math/linalg"
import "core:c"

c2v :: linalg.Vector2f32


#assert(size_of(c2v{}.x) == size_of(c.float))
#assert(size_of(c2v{}.y) == size_of(c.float))


foreign import lib {
	"cute_c2_impl.lib"
}

C2_MAX_POLYGON_VERTS :: 8

c2x :: struct {
	p, r: c2v,
}

c2Poly :: struct {
	count: c.int,
	verts: [C2_MAX_POLYGON_VERTS]c2v,
	norms: [C2_MAX_POLYGON_VERTS]c2v,
}

c2Manifold :: struct {
	count: c.int,
	depths: [2]c.float,
	contact_points: [2]c2v,

	// always points from shape A to shape B (first and second shapes passed into
	// any of the c2***to***Manifold functions)
	n: c2v,
}

@(default_calling_convention="c")
foreign lib {
	c2PolytoPolyManifold :: proc(A: ^c2Poly, ax: ^c2x, B: ^c2Poly, bx: ^c2x, m: ^c2Manifold) ---
	c2MakePoly :: proc(p: ^c2Poly) ---
}
