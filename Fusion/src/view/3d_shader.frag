R"(
#version 150

in vec3 v_norm;
in vec3 v_pos;
in vec4 v_light_pos;
in vec4 v_color;

uniform vec3 eye;
uniform vec3 light_dir;
uniform sampler2D map;
uniform bool light;
uniform bool shadow;

out vec4 outColor;
)"
#include "light.glsl"
R"(
void main() {
    calc_light(v_color.xyz);
	float k = 1.0 - clamp((length(v_pos) - 40.0)/10.0, 0.0, 1.0); 
    outColor = vec4(outColor.rgb, min(k, v_color.a));
}
)"
