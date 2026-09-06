#version 330 core

in vec3 vWorldPos;
in vec3 vNormal;
in vec2 vTexCoord;

//uniform sampler2D uAlbedo;
uniform vec3 uLightPos;
uniform vec3 uLightColor;
uniform vec3 uCameraPos;

out vec4 fragColor;

void main() {
    //vec3 albedo = texture(uAlbedo, vTexCoord).rgb;
    vec3 albedo = vec3(0.5, 0.5, 0.5);

    vec3 N = normalize(vNormal);
    vec3 L = normalize(uLightPos - vWorldPos);
    vec3 V = normalize(uCameraPos - vWorldPos);
    vec3 H = normalize(L + V);

    vec3 ambient  = 0.12 * albedo;
    vec3 diffuse  = max(dot(N, L), 0.0) * albedo * uLightColor;
    vec3 specular = pow(max(dot(N, H), 0.0), 64.0) * 0.35 * uLightColor;

    fragColor = vec4(ambient + diffuse + specular, 1.0);
}
