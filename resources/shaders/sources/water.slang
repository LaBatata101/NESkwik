#version 450

/*
   Author: Themaister
   License: Public domain
   
   Border shader with water wave effect
   Converted to slang format
*/

/*-------------------- Parameters --------------------*/
layout(push_constant) uniform Push
{
   float aspect_x;
   float aspect_y;
   float integer_scale;
   float overscale;
   float scanline_toggle;
   float THICKNESS;
   float DARKNESS;
   float interp_toggle;
   float OS_MASK_TOP;
   float OS_MASK_BOTTOM;
   float OS_MASK_LEFT;
   float OS_MASK_RIGHT;
} params;

#pragma parameter aspect_x "Aspect Ratio Numerator" 64.0 1.0 256.0 1.0
#pragma parameter aspect_y "Aspect Ratio Denominator" 49.0 1.0 256.0 1.0
#pragma parameter integer_scale "Force Integer Scaling" 1.0 0.0 1.0 1.0
#pragma parameter overscale "Integer Overscale" 0.0 0.0 1.0 1.0
#pragma parameter scanline_toggle "Scanline Toggle" 0.0 0.0 1.0 1.0
#pragma parameter interp_toggle "Sharpen Linear Scaling" 0.0 0.0 1.0 1.0
#pragma parameter THICKNESS "Scanline Thickness" 2.0 1.0 12.0 1.0
#pragma parameter DARKNESS "Scanline Darkness" 0.35 0.0 1.0 0.05
#pragma parameter OS_MASK_TOP "OS Mask Top" 0.0 0.0 1.0 0.005
#pragma parameter OS_MASK_BOTTOM "OS Mask Bottom" 0.0 0.0 1.0 0.005
#pragma parameter OS_MASK_LEFT "OS Mask Left" 0.0 0.0 1.0 0.005
#pragma parameter OS_MASK_RIGHT "OS Mask Right" 0.0 0.0 1.0 0.005

/*-------------------- Built-in Uniforms --------------------*/
layout(std140, set = 0, binding = 0) uniform UBO
{
   mat4 MVP;
   vec4 SourceSize;
   vec4 OriginalSize;
   vec4 OutputSize;
   uint FrameCount;
} global;

/*------------------------- Vertex Shader -------------------------*/
#pragma stage vertex
layout(location = 0) in vec4 Position;
layout(location = 1) in vec2 TexCoord;
layout(location = 0) out vec2 vTexCoord;
layout(location = 1) out vec2 vTexBorder;

void main()
{
   gl_Position = global.MVP * Position;
   
   vec2 out_res = global.OutputSize.xy;
   vec2 corrected_size = global.OriginalSize.xy * vec2(params.aspect_x / params.aspect_y, 1.0)
      * vec2(global.OriginalSize.y / global.OriginalSize.x, 1.0);
   float full_scale = (params.integer_scale > 0.5) ? floor(global.OutputSize.y /
      global.OriginalSize.y) + params.overscale : global.OutputSize.y / global.OriginalSize.y;
   vec2 scale = (global.OutputSize.xy / corrected_size) / full_scale;
   vec2 middle = vec2(0.49999, 0.49999) * global.OriginalSize.xy / global.SourceSize.xy;
   vec2 diff = TexCoord.xy - middle;
   vTexCoord = middle + diff * scale;
   
   vTexBorder = TexCoord;
}

/*--------------------- Fragment Shader ---------------------*/
#pragma stage fragment
layout(location = 0) in vec2 vTexCoord;
layout(location = 1) in vec2 vTexBorder;
layout(location = 0) out vec4 FragColor;
layout(set = 0, binding = 2) uniform sampler2D Source;

#define fragcoord (vTexCoord.xy * (global.SourceSize.xy / global.OriginalSize.xy))

vec2 interp_coord(vec2 coord, vec2 texture_size)
{
   vec2 p = coord.xy;
   
   p = p * texture_size.xy + vec2(0.5, 0.5);
   
   vec2 i = floor(p);
   vec2 f = p - i;
   
   // Smoothstep
   f = f * f * f * f * (f * (f * (-20.0 * f + vec2(70.0, 70.0)) - vec2(84.0, 84.0)) + vec2(35.0, 35.0));
   
   p = i + f;
   
   p = (p - vec2(0.5, 0.5)) * 1.0 / texture_size;
   return vec2(p.x, coord.y);
}

vec4 scanlines(vec4 frame, vec2 coord, vec2 texture_size, vec2 video_size, vec2 output_size)
{
   float lines = fract(coord.y * texture_size.y);
   float scale_factor = floor((output_size.y / video_size.y) + 0.4999);
   return (params.scanline_toggle > 0.5 && (lines < (1.0 / scale_factor * params.THICKNESS)))
      ? frame * vec4(1.0 - params.DARKNESS) : frame;
}

float apply_wave(vec2 pos, vec2 src, float cnt)
{
   vec2 diff = pos - src;
   float dist = 300.0 * sqrt(dot(diff, diff));
   dist -= 0.15 * cnt;
   return sin(dist);
}

const vec2 src0 = vec2(0.6, 0.7);
const vec2 src1 = vec2(0.9, 0.9);
const vec2 src2 = vec2(-0.6, 0.3);
const vec2 src3 = vec2(0.1, 0.4);
const vec2 src4 = vec2(0.1, 0.4);
const vec2 src5 = vec2(0.5, 0.5);
const vec2 src6 = vec2(-1.0, 1.0);

vec4 conv_background(vec4 back, vec2 coord, float frame_count)
{
   float cnt = frame_count;
   float res = apply_wave(coord, src0, cnt);
   res +=      apply_wave(coord, src1, cnt);
   res +=      apply_wave(coord, src2, cnt);
   res +=      apply_wave(coord, src3, cnt);
   res +=      apply_wave(coord, src4, cnt);
   res +=      apply_wave(coord, src5, cnt);
   res +=      apply_wave(coord, src6, cnt);
   
   return vec4(back.rgb * (0.7 + 0.05 * res), back.a);
}

void main()
{
   vec2 coord = (params.interp_toggle < 0.5) ? vTexCoord : interp_coord(vTexCoord, global.SourceSize.xy);
   vec4 frame = texture(Source, coord);
   frame = scanlines(frame, vTexCoord, global.SourceSize.xy, global.OriginalSize.xy, global.OutputSize.xy);
   
   vec4 background = conv_background(
      vec4(0.25, 0.25, 1.0, 1.0), 
      vTexBorder * (global.SourceSize.xy / global.OriginalSize.xy) * vec2(global.OutputSize.x / global.OutputSize.y, 1.0), 
      float(global.FrameCount)
   );
   
   if (fragcoord.x < 1.0 - params.OS_MASK_RIGHT && fragcoord.x > 0.0 + params.OS_MASK_LEFT &&
       fragcoord.y < 1.0 - params.OS_MASK_BOTTOM && fragcoord.y > 0.0 + params.OS_MASK_TOP)
      FragColor = frame;
   else
      FragColor = background;
}
