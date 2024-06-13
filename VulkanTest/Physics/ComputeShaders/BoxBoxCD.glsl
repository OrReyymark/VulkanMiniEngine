#version 460
#include "CollisionDetectionHeader.glsl"


//The next function find the advences on the line of the tow closest points between the lines...
void LineClosestApproach(in vec3 pa, in vec3 ua, in vec3 pb, in vec3 ub, out float alpha, out float beta)
{
	vec3 p = pb - pa;

	float uaub = dot(ua, ub);
	float q1   = dot(ua, p);
    float q2   = -dot(ub, p);
    float d = 1.0 - uaub * uaub;

	if (d <= 0.0001)
	{
		alpha	= 0.0;
		beta	= 0.0;
	}
	else
	{
		d = 1.0 / d;
		alpha = (q1 + uaub * q2) * d;
		beta  = (q2 + uaub * q1) * d;
	}
}

// find all the intersection points between the 2D rectangle with vertices
// at (+/-h[0],+/-h[1]) and the 2D quadrilateral with vertices (p[0],p[1]),
// (p[2],p[3]),(p[4],p[5]),(p[6],p[7]).
//
// the intersection points are returned as x,y pairs in the 'ret' array.
// the number of intersection points is returned by the function (this will
// be in the range 0 to 8).
uint RectQuadIntersect2D(in vec2 rect_dimantions, in vec2 vertices[4], out vec2 ips[8], out uint ids[8])
{
	//Zero output memory...
    int i;
	for (i = 0; i < 8; i ++)
    {
        ips[i] = vec2(0.0f, 0.0f);
        ids[i] = 100;
    }
	
	//Setup varibles...
    uint vertices_number = 4;
	
    vec3 swap_buffer[2][8];
    swap_buffer[0][0] = vec3(vertices[0], 0);
    swap_buffer[0][1] = vec3(vertices[1], 1);
    swap_buffer[0][2] = vec3(vertices[2], 2);
    swap_buffer[0][3] = vec3(vertices[3], 3);
	
    uint sorce = 0, target = 1;
    uint founed_vertices_number = 0;
    bool some_cross				= false;
	
	//Sort points...
	for (uint dir = 0 ; dir <= 1 ; dir ++)
	{
		for (int sgn = -1; sgn <= 1; sgn += 2)
		{
            founed_vertices_number	= 0;
            some_cross				= false;
			
            for (uint i = 0; i < vertices_number ; i++)
			{
                if (sgn * (swap_buffer[sorce])[i][dir] < rect_dimantions[dir]) //If this point is inside the chopping line
				{
                    swap_buffer[target][founed_vertices_number] = swap_buffer[sorce][i];//Add the point to the target buffer
                    founed_vertices_number++;
					
					if ((founed_vertices_number & 8) != 0)
					{
						//Copy result to function output acoording to the numeration...
                        for (i = 0; i < 8; i++)
                        {
                            ips[i] = (swap_buffer[target])[i].xy;
                            ids[i] = int((swap_buffer[target])[i].z);
                        }
						
                        return 8;
                    }
				}
				
                uint next_i = (i + 1) % vertices_number;
				
                bool current_in = (sgn * (swap_buffer[sorce])[i][dir] < rect_dimantions[dir]);
                bool next_in	= (sgn * (swap_buffer[sorce])[next_i][dir] < rect_dimantions[dir]);
				
                if (current_in ^^ next_in)//If this line crosses the chopping line[if(current_in xor next_in)]
                {
                    (swap_buffer[target])[founed_vertices_number][dir] = sgn * rect_dimantions[dir];
                    (swap_buffer[target])[founed_vertices_number][1 - dir] = (swap_buffer[sorce])[i][1 - dir] +
						(((swap_buffer[sorce])[next_i][1 - dir] - (swap_buffer[sorce])[i][1 - dir]) / ((swap_buffer[sorce])[next_i][dir] - (swap_buffer[sorce])[i][dir])) * 
						(sgn * rect_dimantions[dir] - (swap_buffer[sorce])[i][dir]);
                    (swap_buffer[target])[founed_vertices_number][2] = (4 + (dir * 2 + sorce) * 2) + uint(some_cross); //There maximum tow crossing points of eath chopping line
                    founed_vertices_number++;
                    some_cross = true;
					
                    if ((founed_vertices_number & 8) != 0)
                    {
                       //Copy result to function output acoording to the numeration...
                        for (i = 0; i < 8; i++)
                        {
                            ips[i] = (swap_buffer[target])[i].xy;
                            ids[i] = int((swap_buffer[target])[i].z);
                        }
						
                        return 8;
                    }
                }
            }
			
            vertices_number = founed_vertices_number;
			
			//Swap the sorce and target buffer index...
            sorce  = 1 - sorce;
            target = 1 - target;
        }
	}
	
	//Copy result to function output acoording to the numeration...
    target = sorce;
    for (i = 0 ; i < founed_vertices_number ; i ++)
    {
        ips[i] = (swap_buffer[target])[i].xy;
        ids[i] = int((swap_buffer[target])[i].z);
    }
	
    return founed_vertices_number;
}


bool BoxBoxCD(in RigidBody box1, in RigidBody box2, out vec4 ips[12], out uint ips_number, out vec4 best_saperator, out bool incident_first)
{
    ips_number = 0;
	
	//Zero intersection memory...
	ips[0] = vec4(0.0f, 0.0f, 0.0f, 0.0f);
    ips[1] = vec4(0.0f, 0.0f, 0.0f, 0.0f);
    ips[2] = vec4(0.0f, 0.0f, 0.0f, 0.0f);
    ips[3] = vec4(0.0f, 0.0f, 0.0f, 0.0f);
    ips[4] = vec4(0.0f, 0.0f, 0.0f, 0.0f);
    ips[5] = vec4(0.0f, 0.0f, 0.0f, 0.0f);
    ips[6] = vec4(0.0f, 0.0f, 0.0f, 0.0f);
    ips[7] = vec4(0.0f, 0.0f, 0.0f, 0.0f);
    ips[8] = vec4(0.0f, 0.0f, 0.0f, 0.0f);
	ips[9] = vec4(0.0f, 0.0f, 0.0f, 0.0f);
    ips[10] = vec4(0.0f, 0.0f, 0.0f, 0.0f);
    ips[11] = vec4(0.0f, 0.0f, 0.0f, 0.0f);
	
    best_saperator = vec4(0.0f, 0.0f, 0.0f, 0.0f);
	
    incident_first = false;
	//
	
	const float fudge_factor = 1.05f;
    const float fudge2 = 1e-4;
	
	vec3 normalC = vec3(0.0f, 0.0f, 0.0f);
    vec3 normalR = vec3(0.0f, 0.0f, 0.0f);
	uint i, j, case_code;

	// get vector from centers of box 1 to box 2, relative to box 1
    vec3 displacment_b1s = (box2.position.xyz - box1.position.xyz) * box1.orientation;
    vec3 displacment_b2s = (box2.position.xyz - box1.position.xyz) * box2.orientation;
    mat3 R = transpose(box1.orientation) * box2.orientation;
    mat3 Q = abs(R);
	
	// for all 15 possible separating axes:
	//   * see if the axis separates the boxes. if so, return 0.
	//   * find the depth of the penetration along the separating axis (s2)
	//   * if this is the largest depth so far, record it.
	// the normal vector will be set to the separating axis with the smallest
	// depth. note: normalR is set to point to a column of R1 or R2 if that is
	// the smallest depth normal so far. otherwise normalR is 0 and normalC is
	// set to a vector relative to body 1. invert_normal is 1 if the sign of
	// the normal should be flipped.
	
	
    vec3 A = box1.extents;
    vec3 B = box2.extents;
	
	
    vec3 u[3], v[3];
    u[0] = vec3(box1.orientation[0][0], box1.orientation[1][0], box1.orientation[2][0]);
    u[1] = vec3(box1.orientation[0][1], box1.orientation[1][1], box1.orientation[2][1]);
    u[2] = vec3(box1.orientation[0][2], box1.orientation[1][2], box1.orientation[2][2]);
	
    v[0] = vec3(box2.orientation[0][0], box2.orientation[1][0], box2.orientation[2][0]);
    v[1] = vec3(box2.orientation[0][1], box2.orientation[1][1], box2.orientation[2][1]);
    v[2] = vec3(box2.orientation[0][2], box2.orientation[1][2], box2.orientation[2][2]);

#define TST(expr1, expr2, normal, cc)		\
	p = abs(expr1) - (expr2);				\
	if(p > 0.000f) return 0;				\
	if (p > penetration)					\
	{										\
		penetration		= p;				\
		normalR			= normal;			\
		invert_normal	= ((expr1) < 0);	\
		case_code		= (cc);				\
	}

    float p;
    float penetration  = -1e30;
	bool invert_normal = false;
	case_code		   = 0;
	
	
	// separating axis = u[0],u[1],u[2]
	TST(displacment_b1s[0], (A[0] + B[0] * Q[0][0] + B[1] * Q[0][1] + B[2] * Q[0][2]), u[0], 1);
	TST(displacment_b1s[1], (A[1] + B[0] * Q[1][0] + B[1] * Q[1][1] + B[2] * Q[1][2]), u[1], 2);
	TST(displacment_b1s[2], (A[2] + B[0] * Q[2][0] + B[1] * Q[2][1] + B[2] * Q[2][2]), u[2], 3);
	
	// separating axis = v[0],v[1],v[3]
	TST(displacment_b2s[0], (B[0] + A[0] * Q[0][0] + A[1] * Q[1][0] + A[2] * Q[2][0]), v[0], 4);
	TST(displacment_b2s[1], (B[1] + A[0] * Q[0][1] + A[1] * Q[1][1] + A[2] * Q[2][1]), v[1], 5);
	TST(displacment_b2s[2], (B[2] + A[0] * Q[0][2] + A[1] * Q[1][2] + A[2] * Q[2][2]), v[2], 6);
	
	
	// note: cross product axes need to be scaled when s is computed.
	// normal (n1,n2,n3) is relative to box 1.
#undef TST
#define TST(expr1, expr2, normal, cc)					 \
	p = abs(expr1) - (expr2);                            \
	if(p > 0.1) return 0;								\
	l = length(normal);									 \
	if (l > 0.00001)									 \
	{                                                    \
		p /= l;                                          \
		if (p * fudge_factor > penetration)				 \
		{                                                \
			penetration		= p;                         \
			normalC			= (normal) / l;              \
			invert_normal	= ((expr1) < 0);             \
			case_code		= (cc);                      \
		}                                                \
	}													 \
	
    float l;
    Q += fudge2;
	
	// separating axis = u1 x (v1,v2,v3)
	TST(displacment_b1s[2] * R[1][0] - displacment_b1s[1] * R[2][0], (A[1] * Q[2][0] + A[2] * Q[1][0] + B[1] * Q[0][2] + B[2] * Q[0][1]), vec3(0, -R[2][0], R[1][0]), 7);
	TST(displacment_b1s[2] * R[1][1] - displacment_b1s[1] * R[2][1], (A[1] * Q[2][1] + A[2] * Q[1][1] + B[0] * Q[0][2] + B[2] * Q[0][0]), vec3(0, -R[2][1], R[1][1]), 8);
	TST(displacment_b1s[2] * R[1][2] - displacment_b1s[1] * R[2][2], (A[1] * Q[2][2] + A[2] * Q[1][2] + B[0] * Q[0][1] + B[1] * Q[0][0]), vec3(0, -R[2][2], R[1][2]), 9);

	// separating axis = u2 x (v1,v2,v3)
    TST(displacment_b1s[0] * R[2][0] - displacment_b1s[2] * R[0][0], (A[0] * Q[2][0] + A[2] * Q[0][0] + B[1] * Q[1][2] + B[2] * Q[1][1]), vec3(R[2][0], 0, -R[0][0]), 10);
	TST(displacment_b1s[0] * R[2][1] - displacment_b1s[2] * R[0][1], (A[0] * Q[2][1] + A[2] * Q[0][1] + B[0] * Q[1][2] + B[2] * Q[1][0]), vec3(R[2][1], 0, -R[0][1]), 11);
	TST(displacment_b1s[0] * R[2][2] - displacment_b1s[2] * R[0][2], (A[0] * Q[2][2] + A[2] * Q[0][2] + B[0] * Q[1][1] + B[1] * Q[1][0]), vec3(R[2][2], 0, -R[0][2]), 12);

	// separating axis = u3 x (v1,v2,v3)
    TST(displacment_b1s[1] * R[0][0] - displacment_b1s[0] * R[1][0], (A[0] * Q[1][0] + A[1] * Q[0][0] + B[1] * Q[2][2] + B[2] * Q[2][1]), vec3(-R[1][0], R[0][0], 0), 13);
	TST(displacment_b1s[1] * R[0][1] - displacment_b1s[0] * R[1][1], (A[0] * Q[1][1] + A[1] * Q[0][1] + B[0] * Q[2][2] + B[2] * Q[2][0]), vec3(-R[1][1], R[0][1], 0), 14);
	TST(displacment_b1s[1] * R[0][2] - displacment_b1s[0] * R[1][2], (A[0] * Q[1][2] + A[1] * Q[0][2] + B[0] * Q[2][1] + B[1] * Q[2][0]), vec3(-R[1][2], R[0][2], 0), 15);

#undef TST
	
	if (!case_code) return false;
	
    vec3 normal;
	
	// if we get to this point, the boxes interpenetrate. compute the normal
	// in global coordinates.
    if (case_code > 6)
        normal = normalize(mul(box1.orientation, normalC));
	else
        normal = normalize(normalR);
	
	if (invert_normal)
        normal = -normal;
	
    float sap_w;
    if (1 <= case_code && case_code <= 3)
        sap_w = dot(box1.position, normal) + A[case_code - 1];
	else if(case_code <= 6)
        sap_w = dot(box2.position, normal) - B[case_code - 4];

    best_saperator = vec4(normal, sap_w);
	
	// compute contact point(s)

	if (case_code > 6)
	{
		//In this cases we can said approximately that edge from box 1 touches an edge from box 2!!!
		//So first we find a point on the box1's intersetion-egde and point a point on the box2's intersetion-egde
		//We do this her by get the box1's vertex that founded in the box1-octette that containe the normal and the box2's vertex that founded in the box2-octette that containe the normal!(Its not hard to show that it a point of the intersection egde)
		//An evry vertex of a 3D-box can write like this : v = center (+/-) width * axis[0] (+/-) height * axis[1] (+/-) depth * axis[2]
		//So is easy those vertex
		vec3 p1 = box1.position;
        vec3 p2 = box2.position;

		for (j = 0; j < 3; j++)
		{
            float sign = (dot(normal, u[j]) > 0) ? 1.0f : -1.0f;
            p1 += sign * A[j] * u[j];
        }

		for (j = 0; j < 3; j++)
		{
            float sign = (dot(normal, v[j]) > 0) ? -1.0f : 1.0f;
			p2 += sign * B[j] * v[j];
		}

		//Now we restore box1's intersect edge direction from the case_code...
		//Les notic that the direction of box1's intersect-edge must by one of the box1's-aligent-axis and the direction of the box2's intersect-edge must by one of the box2's-aligent-axis
		//And we now that the seperate axis of this case by the cross-product of those.
		//So we are only shold find the indexies of those axis from the specific case_code. Remember the case_code table...
		//case 07 : u1 x v1, case 08 : u1 x v2, case 09 : u1 x v3
		//case 10 : u2 x v1, case 11 : u2 x v2, case 12 : u2 x v3
		//case 13 : u3 x v1, case 14 : u3 x v2, case 15 : u3 x v3
		//So case i is u[(i - 7) / 3] x v[(i - 7) % 3]!!!
	
		
		//So lets get the currect vectors from the box1's, box2's orientation matrix
        vec3 u_ = u[(case_code - 7) / 3];
        vec3 v_ = v[(case_code - 7) % 3];
		
		float alpha, beta;
		LineClosestApproach(p1, u_, p2, v_, alpha, beta);

		//Now we advence on the intesection edge antile we get the closest points on between thos egdes... 
		p1 += u_ * alpha;
		p2 += v_ * beta;

		//Lets notice thet now points p1 and p2 should by very close so we can chose one of them
		
        if (abs(p2.y) > 0.004)
        {
            best_saperator.w = dot(p2, normal);
            ips[0] = vec4(p2, length(p2 - p1));
            ips_number = 1;
        }
		
		return true;
		//We are can get the averge point too if we want!
	}

	// Okay, we have a face-something intersection (because the separating
	// axis is perpendicular to a face). define face 'a' to be the reference
	// face (i.e. the normal vector is perpendicular to this) and face 'b' to be
	// the incident face (the closest face of the other box).

    vec3		Pa, Pb;
	vec3		Da, Db;
    vec3x3	Oa, Ob;
	
    vec3 axis_a[3], axis_b[3];
	
	if (case_code <= 3)
	{
		Pa = box1.position;
        Da = box1.dimantions;
		Oa = box1.orientation; 
		
        Pb = box2.position;
        Db = box2.dimantions;
        Ob = box2.orientation;
		
		axis_a[0] = u[0];
        axis_a[1] = u[1];
        axis_a[2] = u[2];
		
        axis_b[0] = v[0];
        axis_b[1] = v[1];
        axis_b[2] = v[2];
    }
	else
	{
        Pa = box2.position;
        Da = box2.dimantions;
        Oa = box2.orientation;
		
        Pb = box1.position;
        Db = box1.dimantions;
        Ob = box1.orientation;
		
        axis_a[0] = v[0];
        axis_a[1] = v[1];
        axis_a[2] = v[2];
		
        axis_b[0] = u[0];
        axis_b[1] = u[1];
        axis_b[2] = u[2];
		
        incident_first = true;
    }

	// nr = normal vector of reference face dotted with axes of incident box.
	// anr = fabsfolute values of nr.
	vec3 inc_normal;//The normal in the incident-box-space
    vec3 abs_inc_normal;
    vec3 normal2 = (case_code <= 3) ? normal : -normal;

	inc_normal = mul(normal2, Ob);
    abs_inc_normal = abs(inc_normal);

	// find the largest compontent of anr: this corresponds to the normal
	// for the indident face. the other axis numbers of the indicent face
	// are stored in a1,a2.
	int lanr, a1, a2;
    if (abs_inc_normal[1] > abs_inc_normal[0])
	{
        if (abs_inc_normal[1] > abs_inc_normal[2])
		{
			a1		= 0;
			lanr	= 1;
			a2		= 2;
		}
		else
		{
			a1		= 0;
			a2		= 1;
			lanr	= 2;
		}
	}
	else
	{
        if (abs_inc_normal[0] > abs_inc_normal[2])
		{
			lanr	= 0;
			a1		= 1;
			a2		= 2;
		}
		else
		{
			a1		= 0;
			a2		= 1;
			lanr	= 2;
		}
	}

	// compute center point of incident face, in reference-face coordinates
	vec3 center;
	if (inc_normal[lanr] < 0)
		center = Pb - Pa + Db[lanr] * vec3(Ob[0][lanr], Ob[1][lanr], Ob[2][lanr]);
	else
        center = Pb - Pa - Db[lanr] * vec3(Ob[0][lanr], Ob[1][lanr], Ob[2][lanr]);


	// find the normal and non-normal axis numbers of the reference box
	int codeN, code1, code2;
	if (case_code <= 3)
		codeN = case_code - 1;
	else
		codeN = case_code - 4;
	if (codeN == 0)
	{
		code1 = 1;
		code2 = 2;
	}
	else if (codeN == 1)
	{
		code1 = 0;
		code2 = 2;
	}
	else
	{
		code1 = 0;
		code2 = 1;
	}
	
	// find the four corners of the incident face, in reference-face coordinates
	vec2 quad[4];  // 2D coordinate of incident face (x,y pairs)
    float c1 = dot(center, axis_a[code1]);
    float c2 = dot(center, axis_a[code2]);
	// optimize this? - we have already computed this data above, but it is not
	// stored in an easy-to-index format. for now it's quicker just to recompute
	// the four dot products.
    float m11 = dot(axis_a[code1], axis_b[a1]);
    float m12 = dot(axis_a[code1], axis_b[a2]);
    float m21 = dot(axis_a[code2], axis_b[a1]);
    float m22 = dot(axis_a[code2], axis_b[a2]);
	{
		float k1 = m11 * Db[a1];
        float k2 = m21 * Db[a1];
        float k3 = m12 * Db[a2];
        float k4 = m22 * Db[a2];
		quad[0].x = c1 - k1 - k3;
		quad[0].y = c2 - k2 - k4;
		quad[1].x = c1 - k1 + k3;
		quad[1].y = c2 - k2 + k4;
		quad[2].x = c1 + k1 + k3;
		quad[2].y = c2 + k2 + k4;
		quad[3].x = c1 + k1 - k3;
		quad[3].y = c2 + k2 - k4;
	}
	
	// find the size of the reference face
	// intersect the incident and reference faces
	vec2 ret[8];
    uint   ids[8];//For numeration the points uniformly
    uint n = RectQuadIntersect2D(vec2(Da[code1], Da[code2]), quad, ret, ids);
	if (n < 1) return 0;  // this should never happen
	
	// convert the intersection points into reference-face coordinates,
	// and compute the contact position and depth for each point. only keep
	// those points that have a positive (penetrating) depth. delete points in
	// the 'ret' array as necessary so that 'point' and 'ret' correspond.
	float dep[8];        // depths for those points
    float det1 = 1.0f / (m11 * m22 - m12 * m21);
	m11 *= det1;
	m12 *= det1;
	m21 *= det1;
	m22 *= det1;
	
	for (j = 0 ; j < n ; j++)
	{
        float k1 =  m22 * (ret[j].x - c1) - m12 * (ret[j].y - c2);
        float k2 = -m21 * (ret[j].x - c1) + m11 * (ret[j].y - c2);
        vec3 p = Pa + center + k1 * axis_b[a1] + k2 * axis_b[a2];
        float depth = Da[codeN] - dot(normal2, p - Pa);
		
        if (0.0f < depth)
        {
            ips[ids[j]] = vec4(p, depth);
			ips_number++;
        }
    }
	
	return true;
}
