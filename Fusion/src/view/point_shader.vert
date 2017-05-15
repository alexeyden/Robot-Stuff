R"(
#version 150

in vec3 position;
in float prob;

out vec3 v_color;

uniform mat4 proj;
uniform mat4 model;

void main() {
    v_color =  mix(vec3(0.0, 1.0, 0.0), vec3(1.0, 0.0, 0.0), prob);

    gl_PointSize = 4.0;
    gl_Position = proj * model * vec4(position, 1.0);
}
)"
