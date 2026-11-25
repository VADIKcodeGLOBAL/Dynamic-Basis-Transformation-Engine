shader_type spatial;
// VERTEX, NORMAL, TANGENT and BINORMAL arrive in world space, so the basis is applied to
// world (logical) positions, exactly like the CPU mirror in main.gd.
render_mode world_vertex_coords;

// Declared in project.godot ([shader_globals]) and set every frame from main.gd with
// RenderingServer.global_shader_parameter_set().
global uniform vec3 dbte_origin;
global uniform vec3 dbte_basis_i;
global uniform vec3 dbte_basis_j;
global uniform vec3 dbte_basis_k;

uniform vec4 albedo : source_color = vec4(0.8, 0.8, 0.8, 1.0);
uniform float roughness : hint_range(0.0, 1.0) = 0.85;
uniform float grid_strength : hint_range(0.0, 1.0) = 0.45;

varying vec3 logical_position;
varying vec3 logical_normal;

void vertex() {
	logical_position = VERTEX;
	logical_normal = NORMAL;

	// p_W = o + B (p_L - o): the columns of B are the dynamic basis vectors i, j, k.
	mat3 basis = mat3(dbte_basis_i, dbte_basis_j, dbte_basis_k);
	VERTEX = dbte_origin + basis * (VERTEX - dbte_origin);

	// Tangents follow B. Normals follow the inverse transpose, which equals the cofactor
	// matrix up to a positive factor while det B > 0, so lighting stays right under shear.
	mat3 normal_matrix = transpose(inverse(basis));
	NORMAL = normalize(normal_matrix * NORMAL);
	TANGENT = normalize(basis * TANGENT);
	BINORMAL = normalize(basis * BINORMAL);
}

void fragment() {
	// A 1 m grid drawn in logical coordinates shows the "fabric" of space being skewed.
	vec3 width = max(fwidth(logical_position), vec3(0.0001));
	vec3 dist = abs(fract(logical_position - 0.5) - 0.5) / width;
	// Skip the axis the surface faces: its coordinate is constant across the face.
	vec3 in_plane = step(abs(normalize(logical_normal)), vec3(0.7));
	dist = mix(vec3(1000.0), dist, in_plane);
	float line = 1.0 - clamp(min(min(dist.x, dist.y), dist.z), 0.0, 1.0);

	ALBEDO = mix(albedo.rgb, albedo.rgb * 0.25, line * grid_strength);
	ROUGHNESS = roughness;
}
