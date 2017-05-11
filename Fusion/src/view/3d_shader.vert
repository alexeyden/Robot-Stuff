R"(
#version 150

in vec3 position;
in vec3 norm;

out vec3 v_norm;
out vec3 v_pos;

uniform mat4 proj;
uniform mat4 model;

void main() {
    v_norm = norm;
    v_pos = position;
    gl_PointSize = 2.0f;
    gl_Position = proj * model * vec4(position, 1.0);
}
)"
