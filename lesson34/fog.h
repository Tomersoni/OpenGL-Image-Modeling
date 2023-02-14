#ifndef FOG_H
#define FOG_H

#include <GL/glut.h>
class Fog
{
public:
	Fog();
	void setup();
private:
	GLfloat fogColor[4];      // Fog Color
};

#endif // FOG_H