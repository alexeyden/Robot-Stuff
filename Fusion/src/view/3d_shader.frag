R"(
#version 150

in vec3 v_norm;
in vec3 v_pos;

uniform vec3 color;
uniform vec3 eye;

out vec4 outColor;

void main() {
        vec3 l = normalize(vec3(10.0, 10.0, 5.0) - v_pos);
        vec3 view = normalize(eye - v_pos);

        vec3 ambient = vec3(0.5, 0.5, 0.5);
        vec3 diffuse = 0.3 * vec3(1.0, 1.0, 1.0) * max(0.0, dot(v_norm, l));
        vec3 specular = vec3(0.0, 0.0, 0.0);
        if(dot(v_norm, l) > 0.0)
                specular = vec3(1.0, 1.0, 1.0) * 0.2 * pow(max(0.0, dot(reflect(-l, v_norm), view)), 1);
        outColor = vec4(color * (diffuse + ambient + specular), 1.0f);
}
)"
