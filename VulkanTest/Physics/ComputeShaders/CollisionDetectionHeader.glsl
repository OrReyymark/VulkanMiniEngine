#ifndef COLLISION_DETECTION_HEADER
#define COLLISION_DETECTION_HEADER

struct RigidBody
{
	vec4 inertia;
	vec4 position;
	mat3 orientation;
	vec3 extents;
	vec4 linear_velocity;
	vec4 angular_velocity;
};

struct Collision
{
	vec4 intersection_points[48];
	vec4 best_saperator;
};

#endif
