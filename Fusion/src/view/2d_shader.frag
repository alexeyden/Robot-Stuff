R"(
#version 150

in vec2 uv;
out vec4 outColor;

uniform vec3 color;
uniform sampler2D tex;

void main() {
    outColor = vec4(color, 1.0) * texture2D(tex, uv);
}
)"
