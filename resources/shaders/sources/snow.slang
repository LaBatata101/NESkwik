#version 450

/*
   Snowy Border Slang shader, based on Andrew Baldwin's 'Just Snow' shadertoy
   Renders snow effect in letterbox areas only
   
   Copyright (c) 2013 Andrew Baldwin (twitter: baldand, www: http://thndl.com)
   License = Attribution-NonCommercial-ShareAlike (http://creativecommons.org/licenses/by-nc-sa/3.0/deed.en_US)
*/

layout(push_constant) uniform Push
{
   float R;
   float G;
   float B;
   float LAYERS;
   float DEPTH;
   float WIDTH;
   float SPEED;
   float FALL_DIRECTION;
   float HORIZONTAL_SPEED;
} params;

#pragma parameter R "Red" 0.20 0.0 1.0 0.01
#pragma parameter G "Green" 0.1 0.0 1.0 0.01
#pragma parameter B "Blue" 0.5 0.0 1.0 0.01
#pragma parameter LAYERS "Snow Layers" 15.0 5.0 30.0 1.0
#pragma parameter DEPTH "Parallax Depth" 1.0 0.1 3.0 0.1
#pragma parameter WIDTH "Flake Spread" 1.0 0.1 3.0 0.1
#pragma parameter SPEED "Fall Speed" 0.01 0.0 0.1 0.01
// 0 = straight down, 1 = diagonal down-left, 2 = diagonal down-right, 3 = up
#pragma parameter FALL_DIRECTION "Fall Direction" 1.0 0.0 3.0 1.0
#pragma parameter HORIZONTAL_SPEED "Horizontal Speed" 0.0 -0.05 0.05 0.001

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

vec4 snow(vec2 texture_size, float frame_count, vec2 uv)
{
   const mat3 p = mat3(13.323122, 23.5112, 21.71123,
                       21.1212, 28.7312, 11.9312,
                       21.8112, 14.7212, 61.3934);
   vec3 acc = vec3(params.R, params.G, params.B);
   float dof = 5.0 * sin(frame_count * 0.1);
   
   vec2 fall_dir;
   int direction = int(params.FALL_DIRECTION);
   if (direction == 0) {
      // Straight down (default)
      fall_dir = vec2(params.HORIZONTAL_SPEED, -params.SPEED);
   } else if (direction == 1) {
      // Diagonal down-left
      fall_dir = vec2(-params.SPEED * 0.5 + params.HORIZONTAL_SPEED, -params.SPEED);
   } else if (direction == 2) {
      // Diagonal down-right
      fall_dir = vec2(params.SPEED * 0.5 + params.HORIZONTAL_SPEED, -params.SPEED);
   } else {
      // Falling up (reverse)
      fall_dir = vec2(params.HORIZONTAL_SPEED, params.SPEED);
   }
   
   for (int i = 0; i < int(params.LAYERS); i++) {
      float fi = float(i);
      vec2 q = uv * (1.0 + fi * params.DEPTH);
      
      // Apply fall direction with parallax
      float parallax_factor = 1.0 / (1.0 + fi * params.DEPTH * 0.03);
      vec2 motion = fall_dir * frame_count * parallax_factor;
      
      // Add horizontal spread based on WIDTH parameter
      q += vec2(q.y * (params.WIDTH * mod(abs(fi * 7.238917), 1.0) - params.WIDTH * 0.5), 0.0);
      q += motion;
      
      vec3 n = vec3(floor(q), 31.189 + fi);
      vec3 m = floor(n) * 0.00001 + fract(n);
      vec3 mp = (31415.9 + m) / fract(p * m);
      vec3 r = fract(mp);
      vec2 s = abs(mod(q, 1.0) - 0.5 + 0.9 * r.xy - 0.45);
      s += 0.01 * abs(2.0 * fract(10.0 * q.yx) - 1.0); 
      float d = 0.6 * max(s.x - s.y, s.x + s.y) + max(s.x, s.y) - 0.01;
      float edge = 0.05 + 0.05 * min(0.5 * abs(fi - 5.0 - dof), 1.0);
      float acc_mod = smoothstep(edge, -edge, d) * (r.x / (1.0 + 0.02 * fi * params.DEPTH));
      acc += vec3(acc_mod, acc_mod, acc_mod);
   }
   return vec4(vec3(acc), 1.0);
}

void main()
{
   vec2 snow_uv = vTexCoord * vec2(global.OutputSize.x / global.OutputSize.y, 1.0);
   FragColor = snow(global.OutputSize.xy, float(global.FrameCount), snow_uv);
}
