extends Node3D
## Stage 1 prototype: one global basis applied to the whole scene in the vertex shader.
##
## What to look for:
## - geometry AND its shadows follow the shear and scale, because Godot runs the same
##   vertex shader in the shadow pass;
## - the red marker is placed on the CPU with the same formula as the shader. It must stay
##   glued to the top corner of the pillar in the +X +Z corner while the basis changes.

const SHADER: Shader = preload("res://dbte_proto.gdshader")

## Columns of each basis: the dynamic vectors i, j, k.
const PRESETS := [
	{"name": "Identity", "i": Vector3(1, 0, 0), "j": Vector3(0, 1, 0), "k": Vector3(0, 0, 1)},
	{"name": "Shear: up leans along X", "i": Vector3(1, 0, 0), "j": Vector3(0.6, 1, 0), "k": Vector3(0, 0, 1)},
	{"name": "Stretch and squash", "i": Vector3(1.5, 0, 0), "j": Vector3(0, 0.7, 0), "k": Vector3(0, 0, 1.3)},
	{"name": "Skew on all axes", "i": Vector3(1, 0, 0.3), "j": Vector3(0.4, 1, 0), "k": Vector3(0, 0.2, 1)},
]

const ORIGIN := Vector3.ZERO
const BLEND_SECONDS := 1.2
const WAVE_SPEED := 1.3
## Logical position of the top corner of the pillar in grid cell (2, 2).
const MARKER_LOGICAL := Vector3(9.75, 5.0, 9.75)

var _preset := 1
var _enabled := true
var _wave := false
var _wave_phase := 0.0
var _show_marker := true

var _from := Basis.IDENTITY
var _current := Basis.IDENTITY
var _blend := 1.0

var _marker: MeshInstance3D
var _camera: Camera3D
var _hud: Label
var _orbit_yaw := deg_to_rad(35.0)
var _orbit_pitch := deg_to_rad(28.0)
var _orbit_distance := 30.0
var _auto_orbit := true
var _dragging := false


func _ready() -> void:
	_build_environment()
	_build_level()
	_build_marker()
	_build_camera()
	_build_hud()
	_apply_basis(_current)
	_start_blend()


func _process(delta: float) -> void:
	if _blend < 1.0:
		_blend = minf(_blend + delta / BLEND_SECONDS, 1.0)
		_current = _lerp_basis(_from, _target(), smoothstep(0.0, 1.0, _blend))
	elif _wave and _enabled:
		# Phase 0 is the target itself; the wave swings back to the identity and forth.
		_wave_phase += delta * WAVE_SPEED
		_current = _lerp_basis(Basis.IDENTITY, _target(), 0.5 + 0.5 * cos(_wave_phase))
	else:
		_current = _target()

	if _auto_orbit:
		_orbit_yaw += delta * 0.12
		_update_camera()

	_apply_basis(_current)


func _unhandled_input(event: InputEvent) -> void:
	var key := event as InputEventKey
	if key != null:
		if key.pressed and not key.echo:
			match key.keycode:
				KEY_1, KEY_2, KEY_3, KEY_4:
					_preset = key.keycode - KEY_1
					_start_blend()
				KEY_SPACE:
					_wave = not _wave
					_start_blend()
				KEY_TAB:
					_enabled = not _enabled
					_start_blend()
				KEY_M:
					_show_marker = not _show_marker
				KEY_ESCAPE:
					get_tree().quit()
		return

	var button := event as InputEventMouseButton
	if button != null:
		if button.button_index == MOUSE_BUTTON_LEFT or button.button_index == MOUSE_BUTTON_RIGHT:
			_dragging = button.pressed
			_auto_orbit = false
		elif button.pressed and button.button_index == MOUSE_BUTTON_WHEEL_UP:
			_orbit_distance = maxf(_orbit_distance - 1.5, 8.0)
			_update_camera()
		elif button.pressed and button.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			_orbit_distance = minf(_orbit_distance + 1.5, 70.0)
			_update_camera()
		return

	var motion := event as InputEventMouseMotion
	if motion != null and _dragging:
		_orbit_yaw -= motion.relative.x * 0.005
		_orbit_pitch = clampf(_orbit_pitch + motion.relative.y * 0.005, deg_to_rad(5.0), deg_to_rad(85.0))
		_update_camera()


func _target() -> Basis:
	if not _enabled:
		return Basis.IDENTITY
	var preset: Dictionary = PRESETS[_preset]
	return Basis(preset["i"], preset["j"], preset["k"])


func _start_blend() -> void:
	_from = _current
	_blend = 0.0
	_wave_phase = 0.0


## Blends the basis vectors component-wise. Fine for these presets (det B stays positive);
## dbte-core will interpolate through the polar decomposition instead (docs/ARCHITECTURE.md).
static func _lerp_basis(a: Basis, b: Basis, t: float) -> Basis:
	return Basis(a.x.lerp(b.x, t), a.y.lerp(b.y, t), a.z.lerp(b.z, t))


func _apply_basis(b: Basis) -> void:
	RenderingServer.global_shader_parameter_set(&"dbte_origin", ORIGIN)
	RenderingServer.global_shader_parameter_set(&"dbte_basis_i", b.x)
	RenderingServer.global_shader_parameter_set(&"dbte_basis_j", b.y)
	RenderingServer.global_shader_parameter_set(&"dbte_basis_k", b.z)

	# CPU mirror of the vertex shader: p_W = o + B (p_L - o).
	_marker.position = ORIGIN + b * (MARKER_LOGICAL - ORIGIN)
	_marker.visible = _show_marker
	_update_hud(b)


func _update_hud(b: Basis) -> void:
	var preset: Dictionary = PRESETS[_preset]
	var lines := PackedStringArray([
		"DBTE prototype: one global basis",
		"Preset %d: %s   deformation %s%s" % [
			_preset + 1, preset["name"], "on" if _enabled else "off", "   wave" if _wave else ""],
		"i = %s" % _format_vector(b.x),
		"j = %s" % _format_vector(b.y),
		"k = %s" % _format_vector(b.z),
		"det B = %.3f" % b.determinant(),
		"",
		"1-4 presets   Space wave   Tab on/off   M marker   drag to orbit   wheel to zoom   Esc quit",
	])
	_hud.text = "\n".join(lines)


static func _format_vector(v: Vector3) -> String:
	return "(%5.2f, %5.2f, %5.2f)" % [v.x, v.y, v.z]


func _build_environment() -> void:
	var sky := Sky.new()
	sky.sky_material = ProceduralSkyMaterial.new()
	var environment := Environment.new()
	environment.background_mode = Environment.BG_SKY
	environment.sky = sky
	environment.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	environment.tonemap_mode = Environment.TONE_MAPPER_FILMIC
	var world_environment := WorldEnvironment.new()
	world_environment.environment = environment
	add_child(world_environment)

	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-55.0, -30.0, 0.0)
	sun.shadow_enabled = true
	sun.directional_shadow_max_distance = 90.0
	add_child(sun)


func _build_level() -> void:
	var ground := PlaneMesh.new()
	ground.size = Vector2(44.0, 44.0)
	ground.subdivide_width = 22
	ground.subdivide_depth = 22
	_add_mesh(ground, Vector3.ZERO, Color(0.62, 0.64, 0.68))

	# A 5 x 5 grid of pillars, taller towards the corners.
	for gx in range(-2, 3):
		for gz in range(-2, 3):
			var height := 1.0 + float(absi(gx) + absi(gz))
			var pillar := BoxMesh.new()
			pillar.size = Vector3(1.5, height, 1.5)
			var tint := Color.from_hsv(0.05 + 0.04 * float(gx + gz + 4), 0.45, 0.9)
			_add_mesh(pillar, Vector3(gx * 4.5, height * 0.5, gz * 4.5), tint)

	var wall := BoxMesh.new()
	wall.size = Vector3(16.0, 3.0, 0.6)
	_add_mesh(wall, Vector3(0.0, 1.5, -15.0), Color(0.8, 0.76, 0.7))

	for n in 3:
		var ball := SphereMesh.new()
		ball.radius = 1.0
		ball.height = 2.0
		_add_mesh(ball, Vector3(-15.0, 1.0, -6.0 + 6.0 * n), Color(0.35, 0.55, 0.9))


func _add_mesh(mesh: Mesh, at: Vector3, color: Color) -> MeshInstance3D:
	var material := ShaderMaterial.new()
	material.shader = SHADER
	material.set_shader_parameter(&"albedo", color)
	var instance := MeshInstance3D.new()
	instance.mesh = mesh
	instance.material_override = material
	instance.position = at
	# The deformation happens on the GPU, so the engine's bounding boxes no longer match
	# what is drawn. A generous margin keeps deformed meshes and their shadows from being
	# culled; the plugin will compute exact bounds instead (Stage 3).
	instance.extra_cull_margin = 40.0
	add_child(instance)
	return instance


func _build_marker() -> void:
	var sphere := SphereMesh.new()
	sphere.radius = 0.3
	sphere.height = 0.6
	var material := StandardMaterial3D.new()
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.albedo_color = Color(1.0, 0.15, 0.1)
	_marker = MeshInstance3D.new()
	_marker.mesh = sphere
	_marker.material_override = material
	_marker.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	add_child(_marker)


func _build_camera() -> void:
	_camera = Camera3D.new()
	_camera.fov = 55.0
	add_child(_camera)
	_update_camera()


func _update_camera() -> void:
	var target := Vector3(0.0, 2.0, 0.0)
	var direction := Vector3(
		cos(_orbit_pitch) * sin(_orbit_yaw),
		sin(_orbit_pitch),
		cos(_orbit_pitch) * cos(_orbit_yaw))
	_camera.look_at_from_position(target + direction * _orbit_distance, target)


func _build_hud() -> void:
	var layer := CanvasLayer.new()
	add_child(layer)
	_hud = Label.new()
	_hud.position = Vector2(16.0, 12.0)
	_hud.add_theme_color_override(&"font_color", Color.WHITE)
	_hud.add_theme_color_override(&"font_outline_color", Color.BLACK)
	_hud.add_theme_constant_override(&"outline_size", 6)
	layer.add_child(_hud)
