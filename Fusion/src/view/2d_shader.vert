R"(
#version 150

in vec2 position;
in vec2 uvp;
out vec2 uv;

uniform mat4 proj;
uniform mat4 model;

void main() {
    uv = uvp;
    gl_Position = proj * model * vec4(position, 0.0, 1.0);
}
)"
