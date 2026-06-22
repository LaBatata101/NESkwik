#version 450

/*
   Animated Border Slang shader
   Copyright (c) 2014 Mudlord licensed under the WTFPL.
*/

layout(push_constant) uniform Push
{
   float PARAM1;
   float PARAM2;
   float PARAM3;
   float PARAM4;
   float PARAM5;
   float PARAM6;
   float R;
   float G;
   float B;
} params;

#pragma parameter PARAM1 "Border Zoom" 10.0 1.0 50.0 1.0
#pragma parameter PARAM2 "Border Pan" 10.0 1.0 50.0 1.0
#pragma parameter PARAM3 "Border Animation Speed" 1.0 0.5 5.0 0.5
#pragma parameter PARAM4 "Factor 1" 2.0 0.1 3.0 0.1
#pragma parameter PARAM5 "Factor 2" 1.0 0.1 2.0 0.1
#pragma parameter PARAM6 "Factor 3" 0.1 0.01 0.2 0.01
#pragma parameter R "Red" 0.0 0.0 1.0 0.1
#pragma parameter G "Green" 0.3 0.0 1.0 0.1
#pragma parameter B "Blue" 0.5 0.0 1.0 0.1

layout(std140, set = 0, binding = 0) uniform UBO
{
   mat4 MVP;
   vec4 SourceSize;
   vec4 OriginalSize;
   vec4 OutputSize;
   uint FrameCount;
} global;

#pragma stage vertex
layout(location = 0) in vec4 Position;
layout(location = 1) in vec2 TexCoord;
layout(location = 0) out vec2 vTexCoord;

void main()
{
   gl_Position = global.MVP * Position;
   vTexCoord = TexCoord;
}

#pragma stage fragment
layout(location = 0) in vec2 vTexCoord;
layout(location = 0) out vec4 FragColor;
layout(set = 0, binding = 2) uniform sampler2D Source;

vec4 mudscape(vec2 texture_size, float frame_count, vec2 uv)
{
   vec2 sp = uv;
   vec2 p = sp * params.PARAM1 - vec2(params.PARAM2, params.PARAM2);
   vec2 i = p;
   float c = 1.0;
   float inten = params.PARAM6;
   float t = 0.01 * params.PARAM3 * frame_count * (1.0 - (3.0 / float(0 + 1)));
   i = p + vec2(cos(t - i.x) + sin(t + i.y), sin(t - i.y) + cos(t + i.x));
   c += 1.2 / length(vec2(p.x / (sin(i.x + t) / inten), p.y / (cos(i.y + t) / inten)));
   c /= params.PARAM5;
   c = params.PARAM4 - sqrt(c);
   vec4 color = vec4(params.R, params.G, params.B, 1.0);
   
   return vec4(vec3(c * c * c * c, c * c * c * c, c * c * c * c), 9999.0) + color;
}

void main()
{
   vec2 effect_uv = vTexCoord * vec2(global.OutputSize.x / global.OutputSize.y, 1.0);
   FragColor = mudscape(global.OutputSize.xy, float(global.FrameCount), effect_uv);
}
