#define CUTE_C2_IMPLEMENTATION
#include "cute_c2.h"


#include <stdio.h>

c2v V2(float x, float y) {
	return (c2v){x, y};
}


void set_poly_norms(c2Poly *poly)
{
	c2v center = {0};
	for(int i = 0; i < poly->count; i++) {
		center = c2Add(center, poly->verts[i]);
	}
	center.x /= (float)poly->count;
	center.y /= (float)poly->count;

	for(int i = 0; i < poly->count; i++) {
		poly->norms[i] = c2Norm(c2Sub(poly->verts[i], center));
	}
}

int main(int argc, char ** argv)
{
	c2Poly from_poly = {
		.count = 4,
		.verts = {V2(2.2, 2), V2(4.2, 2), V2(4.2, 0), V2(2.2, 0)},
	};
	c2Poly to_poly = {
		.count = 4,
		.verts = {V2(0.4, 2.5), V2(2.4, 2.5), V2(2.4, 0.5), V2(0.4, 0.5)},
	};
	//set_poly_norms(&from_poly);
	//set_poly_norms(&to_poly);
	c2MakePoly(&from_poly);
	c2MakePoly(&to_poly);
	c2Manifold out;
	c2PolytoPolyManifold(&from_poly, 0, &to_poly, 0, &out);

	printf("Out manifold: count = %d n.x = %f n.y = %f", out.count, out.n.x, out.n.y);
}
