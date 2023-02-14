#include "fog.h"

Fog::Fog()
{
	fogColor[0] = 0.5f;
	fogColor[1] = 0.5f;
	fogColor[2] = 0.5f;
	fogColor[3] = 0.2f;
}

void Fog::setup() {
	glClearColor(fogColor[0], fogColor[1], fogColor[2], fogColor[3]);          // We'll Clear To The Color Of The Fog
	glFogi(GL_FOG_MODE, GL_EXP2);        // Fog Mode
	glFogfv(GL_FOG_COLOR, fogColor);            // Set Fog Color
	glFogf(GL_FOG_DENSITY, 0.009f);              // How Dense Will The Fog Be
	glFogf(GL_FOG_START, 1.0f);             // Fog Start Depth
	glFogf(GL_FOG_END, 25.0f);               // Fog End Depth
	glEnable(GL_FOG);                   // Enables GL_FOG
}