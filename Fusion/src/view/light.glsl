R"(
float CalcShadowFactor()
{
    if(!shadow)
        return 1.0;
    if(v_light_pos.w <= 0)
        return 1.0;

    vec3 ProjCoords = v_light_pos.xyz / v_light_pos.w;
    vec2 UVCoords;
    UVCoords.x = 0.5 * ProjCoords.x + 0.5;
    UVCoords.y = 0.5 * ProjCoords.y + 0.5;
    if(UVCoords.x > 1.0 || UVCoords.y > 1.0)
        return 1.0;
    float z = 0.5 * ProjCoords.z + 0.5 - 0.00001;
    float Depth = texture2D(map, UVCoords).x;

    if (Depth < z)
        return 0.5;
    else
        return 1.0;
}

void calc_light(vec3 clr) {
        if(light) {
            vec3 l = light_dir;
            vec3 view = normalize(eye - v_pos);
            vec3 norm = normalize(v_norm);

            vec3 ambient = vec3(0.3, 0.3, 0.3);
            vec3 diffuse = 0.1 * vec3(1.0, 1.0, 1.0) * abs(normalize(dot(norm, l)));
            vec3 specular = vec3(0.0, 0.0, 0.0);
            float spec_fac = dot(view, normalize(reflect(l, norm)));
            specular = vec3(1.0, 1.0, 1.0) * 0.3 * pow(spec_fac, 2.0);
            float sh = CalcShadowFactor();
            vec3 light_color = sh * (diffuse + specular) + ambient;
            outColor = vec4(clr * light_color, length(v_norm));
        } else {
            outColor = vec4(clr, 1.0);
        }
}
)"
