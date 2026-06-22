#version 450

/*
   A recreation of the blur border commonly used for portrait cell phone videos
   Created by hunterk
   License: public domain
*/

layout(push_constant) uniform Push
{
   float ZOOM;
   float BRIGHTNESS;
} params;

#pragma parameter ZOOM "Border Zoom" 1.25 0.5 10.0 0.5
#pragma parameter BRIGHTNESS "Border Brightness Mod" 0.0 -1.0 1.0 0.05

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

void main()
{
   // Apply zoom and aspect correction for the blurred border effect
   vec2 middle = vec2(0.5, 0.5);
   vec2 zoom_coord = ((vTexCoord - middle) / params.ZOOM) * vec2(global.OutputSize.x / global.OutputSize.y, 1.0) + middle;
   
   // Sample the blurred source (this comes from the blur passes)
   vec4 effect = texture(Source, zoom_coord);
   
   // Apply brightness adjustment
   effect += vec4(vec3(params.BRIGHTNESS), 0.0);
   
   FragColor = effect;
}
