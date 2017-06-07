R"(
#version 150

in vec3 position;
in vec3 norm;
in float cluster;

out vec3 v_norm;
out vec3 v_pos;
out vec4 v_light_pos;
out vec4 v_color;

uniform mat4 proj;
uniform mat4 model;
uniform mat4 light_model;
uniform vec3 sva;
uniform vec4 override_color = vec4(0.0, 0.0, 0.0, 0.0);

vec3 hsv2rgb(vec3 c)
{
    vec4 K = vec4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
    vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);
    return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);
}

void main() {
    v_norm = norm;
    v_pos = position;
    if(length(override_color) > 0.0)
        v_color = override_color;
    else
        v_color = vec4(hsv2rgb(vec3(cluster, sva.x, sva.y)), sva.z);
    v_light_pos = light_model * vec4(position, 1.0);
    gl_Position = proj * model * vec4(position, 1.0);
}
)"
