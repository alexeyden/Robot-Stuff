R"(
#version 150

in vec3 position;
in float prob;

out vec3 v_color;

uniform mat4 proj;
uniform mat4 model;
uniform float thresh;

void main() {
    vec3 c0 = vec3(1.0, 0.3, 0.3);
    vec3 c1 = vec3(1.0, 1.0, 0.3);
    vec3 c2 = vec3(0.3, 1.0, 0.3);

    vec3 v1 = mix(c0, c1, pow(smoothstep(0.0, 1.0, prob*2.0), 0.6));
    vec3 v2 = mix(v1, c2, pow(smoothstep(0.0, 1.0, prob*2.0 - 1.0), 1.6));

    v_color = v2;

    float a = 1.0;

    if(prob < thresh)
        a = 0.0;

    gl_PointSize = 2.0;
    gl_Position = proj * model * vec4(position, a);
}
)"
