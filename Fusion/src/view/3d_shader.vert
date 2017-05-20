R"(
#version 150

in vec3 position;
in vec3 norm;

out vec3 v_norm;
out vec3 v_pos;
out vec4 v_light_pos;

uniform mat4 proj;
uniform mat4 model;
uniform mat4 light_model;

void main() {
    v_norm = norm;
    v_pos = position;
    v_light_pos = light_model * vec4(position, 1.0);
    gl_Position = proj * model * vec4(position, 1.0);
}
)"
