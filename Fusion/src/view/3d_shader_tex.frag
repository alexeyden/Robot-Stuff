R"(
#version 150

in vec3 v_norm;
in vec3 v_pos;
in vec2 v_uv;
in vec4 v_light_pos;

uniform vec3 eye;
uniform vec3 light_dir;
uniform sampler2D map;
uniform bool light;
uniform bool shadow;
uniform sampler2D tex;

out vec4 outColor;
)"
#include "light.glsl"
R"(
void main() {
    calc_light(texture2D(tex, v_uv).rgb);
    float k = 1.0 - clamp((length(v_pos) - 40.0)/10.0, 0.0, 1.0);
    outColor = vec4(outColor.rgb, k);
}
)"
