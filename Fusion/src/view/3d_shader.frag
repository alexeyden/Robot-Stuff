R"(
#version 150

in vec3 v_norm;
in vec3 v_pos;

uniform vec3 color;
uniform vec3 eye;
uniform bool light;

out vec4 outColor;

void main() {
        if(light) {
            vec3 l = -vec3(20.0, 20.0, 20.0) + vec3(0.0, 8.0, 0.0);
            vec3 view = normalize(eye - v_pos);
            vec3 norm = normalize(v_norm);

            vec3 ambient = vec3(0.5, 0.5, 0.5);
            vec3 diffuse = 0.1 * vec3(1.0, 1.0, 1.0) * max(0.0, dot(norm, l));
            vec3 specular = vec3(0.0, 0.0, 0.0);
            float spec_fac = dot(view, normalize(reflect(l, norm)));
            specular = vec3(1.0, 1.0, 1.0) * 0.3 * pow(spec_fac, 2.0);
            outColor = vec4(color * (diffuse + ambient + specular), 1.0);
        } else {
            outColor = vec4(color, 1.0);
        }
	float k = 1.0 - clamp((length(v_pos) - 40.0)/10.0, 0.0, 1.0); 
	outColor = vec4(outColor.rgb, k);
}
)"
