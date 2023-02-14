/*
 *		This Code Was Created By Dolev Dor and Tomer Zonenfeld
 */


#include <windows.h>									// Header File For Windows
#include <stdio.h>										// Header file for standard input output (IE, "FILE") (NEW)
#include <gl\gl.h>										// Header File For The OpenGL32 Library
#include <gl\glu.h>										// Header File For The GLu32 Library
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include "fog.h"
#include <SDL.h>

using namespace cv;

namespace std {
	template <>
	struct hash<Vec3f> {
		std::size_t operator()(const Vec3f& v) const {
			std::size_t h1 = std::hash<float>()(v[0]);
			std::size_t h2 = std::hash<float>()(v[1]);
			std::size_t h3 = std::hash<float>()(v[2]);
			return h1 ^ h2 ^ h3;
		}
	};
}


struct Triangle {
	Vec3f vertexA;
	Vec3f vertexB;
	Vec3f vertexC;
	bool picked;

	Triangle() : vertexA(), vertexB(), vertexC() {}  // Default constructor

	Triangle(Vec3f A, Vec3f B, Vec3f C) {
		vertexA = A;
		vertexB = B;
		vertexC = C;
		picked = false;
	}
	bool operator!=(const Triangle& other) const {
		return vertexA != other.vertexA || vertexB != other.vertexB || vertexC != other.vertexC;
	}
	bool operator==(const Triangle& other) const {
		return vertexA == other.vertexA && vertexB == other.vertexB && vertexC == other.vertexC;
	}
};

struct PairCompare {
	bool operator()(const std::pair<int, Triangle>& a, const std::pair<int, Triangle>& b) const {
		return a.first < b.first;
	}
};

bool operator<(const std::pair<int, Triangle>& lhs, const std::pair<int, Triangle>& rhs) {
	return lhs.first < rhs.first;
}

namespace std {
	template <>
	struct hash<Triangle> {
		std::size_t operator()(const Triangle& t) const {
			std::size_t h1 = std::hash<Vec3f>()(t.vertexA);
			std::size_t h2 = std::hash<Vec3f>()(t.vertexB);
			std::size_t h3 = std::hash<Vec3f>()(t.vertexC);
			return h1 ^ h2 ^ h3;
		}
	};
}

#pragma comment(lib, "opengl32.lib")					// Link OpenGL32.lib
#pragma comment(lib, "glu32.lib")						// Link Glu32.lib

#define		STEP_SIZE_LOW	  16							// Width And Height Of Each Triangle - Low
#define		STEP_SIZE_HIGH	  8				// Width And Height Of Each Triangle - High
#define		HEIGHT_RATIO  1.5f							// Ratio That The Y Is Scaled According To The X And Z

HDC			hDC = NULL;									// Private GDI Device Context
HGLRC		hRC = NULL;									// Permanent Rendering Context
HWND		hWnd = NULL;									// Holds Our Window Handle
HINSTANCE	hInstance;									// Holds The Instance Of The Application

bool		keys[256];									// Array Used For The Keyboard Routine
bool		active = TRUE;								// Window Active Flag Set To TRUE By Default
bool		bRender = TRUE;								// Polygon Flag Set To TRUE By Default
int mouseX, mouseY;
bool picked = FALSE;
bool wasPaved = FALSE;
bool highRes = FALSE;
bool shouldFog = false;
GLuint	texture[1];

Mat g_HeightMap = imread("C:/CPP Libs/opencv/OIP.jpg", IMREAD_COLOR);		// Holds The Height Map Data

float scaleValue = 0.3f;								// Scale Value For The Terrain
float heading;

int numOfTriangles = 0;
int numOfTrianglesHigh = 0;

std::vector<Vec3f> vertices;
std::vector<Triangle> triangles;
std::vector<int> pickedTriangles;
std::vector<Triangle> trianglesHigh;

GLfloat	yrot;				// Y Rotation
GLfloat walkbias = 0;
GLfloat walkbiasangle = 0;
GLfloat lookupdown = 0.0f;
GLfloat rotationAngle = 0.0f;
GLfloat yAxisTranslation = 0.0f;
GLfloat	z = 0.0f;				// Depth Into The Screen

LRESULT	CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);	// Declaration For WndProc

SDL_Surface* LoadImg(const char* file) {
	SDL_Surface* image = SDL_LoadBMP(file);
	if (image == NULL) {
		fprintf(stderr, "Error loading image: %s\n", SDL_GetError());
		return NULL;
	}
	return image;
}

int LoadGLTextures()									// Load Bitmaps And Convert To Textures
{
	int Status = FALSE;
	SDL_Surface* TextureImage[1];
	memset(TextureImage, 0, sizeof(void*) * 1);

	if (TextureImage[0] = LoadImg("C:/CPP Libs/opencv/TGP.bmp"))
	{
		Status = TRUE;
		glGenTextures(1, &texture[0]);
		glBindTexture(GL_TEXTURE_2D, texture[0]);
		glTexImage2D(GL_TEXTURE_2D, 0, 3, TextureImage[0]->w, TextureImage[0]->h, 0, GL_RGB, GL_UNSIGNED_BYTE, TextureImage[0]->pixels);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		SDL_FreeSurface(TextureImage[0]);
	}

	return Status;
}

float Height(Mat *pHeightMap, int X, int Y)				// This Returns The Height From A Height Map Index
{
	int rows = pHeightMap->rows;
	int cols = pHeightMap->cols;
	int x = X % rows; 								// Error Check Our x Value
	int y = Y % cols;								// Error Check Our y Value

	if (!pHeightMap) return 0;							// Make Sure Our Data Is Valid

	return 80 * ((pHeightMap->at<Vec3b>(x, y).val[0] / 255.0f +
		pHeightMap->at<Vec3b>(x, y).val[1] / 255.0f +
		pHeightMap->at<Vec3b>(x, y).val[2] / 255.0f) / 3.0f);				// Index Into Our Height Array And Return The Height
}

void initTrianglesLow(Mat pHeightMap)
{
	int X = 0, Y = 0;									// Create Some Variables To Walk The Array With.
	float x, y, z;										// Create Some Variables For Readability

	int rows = pHeightMap.rows;
	int cols = pHeightMap.cols;

	int index = 0;

	for (X = 0; X < (rows - STEP_SIZE_LOW); X += STEP_SIZE_LOW) {

		for (Y = 0; Y < (cols - STEP_SIZE_LOW); Y += STEP_SIZE_LOW)
		{
			x = X;
			y = Height(&pHeightMap, X, Y);
			z = Y;

			Vec3f verticeA = Vec3f(x, y, z);

			x = X + STEP_SIZE_LOW;
			y = Height(&pHeightMap, X + STEP_SIZE_LOW, Y);
			z = Y;

			Vec3f verticeB = Vec3f(x, y, z);

			x = X;
			y = Height(&pHeightMap, X, Y + STEP_SIZE_LOW);
			z = Y + STEP_SIZE_LOW;

			Vec3f verticeC = Vec3f(x, y, z);

			triangles.push_back(Triangle(verticeA, verticeB, verticeC));
			index++;

			x = X;
			y = Height(&pHeightMap, X, Y + STEP_SIZE_LOW);
			z = Y + STEP_SIZE_LOW;

			verticeA = Vec3f(x, y, z);

			x = X + STEP_SIZE_LOW;
			y = Height(&pHeightMap, X + STEP_SIZE_LOW, Y);
			z = Y;

			verticeB = Vec3f(x, y, z);

			x = X + STEP_SIZE_LOW;
			y = Height(&pHeightMap, X + STEP_SIZE_LOW, Y + STEP_SIZE_LOW);
			z = Y + STEP_SIZE_LOW;

			verticeC = Vec3f(x, y, z);

			triangles.push_back(Triangle(verticeA, verticeB, verticeC));
			index++;
		}

		glEnd();

	}

	numOfTriangles = index;
}

void initTrianglesHigh(Mat pHeightMap)
{
	int X = 0, Y = 0;									// Create Some Variables To Walk The Array With.
	float x, y, z;										// Create Some Variables For Readability

	int rows = pHeightMap.rows;
	int cols = pHeightMap.cols;

	int index = 0;

	for (X = 0; X < (rows - STEP_SIZE_HIGH); X += STEP_SIZE_HIGH) {

		for (Y = 0; Y < (cols - STEP_SIZE_HIGH); Y += STEP_SIZE_HIGH)
		{
			x = X;
			y = Height(&pHeightMap, X, Y);
			z = Y;

			Vec3f verticeA = Vec3f(x, y, z);

			x = X + STEP_SIZE_HIGH;
			y = Height(&pHeightMap, X + STEP_SIZE_HIGH, Y);
			z = Y;

			Vec3f verticeB = Vec3f(x, y, z);

			x = X;
			y = Height(&pHeightMap, X, Y + STEP_SIZE_HIGH);
			z = Y + STEP_SIZE_HIGH;

			Vec3f verticeC = Vec3f(x, y, z);

			trianglesHigh.push_back(Triangle(verticeA, verticeB, verticeC));
			index++;

			x = X;
			y = Height(&pHeightMap, X, Y + STEP_SIZE_HIGH);
			z = Y + STEP_SIZE_HIGH;

			verticeA = Vec3f(x, y, z);

			x = X + STEP_SIZE_HIGH;
			y = Height(&pHeightMap, X + STEP_SIZE_HIGH, Y);
			z = Y;

			verticeB = Vec3f(x, y, z);

			x = X + STEP_SIZE_HIGH;
			y = Height(&pHeightMap, X + STEP_SIZE_HIGH, Y + STEP_SIZE_HIGH);
			z = Y + STEP_SIZE_HIGH;

			verticeC = Vec3f(x, y, z);

			trianglesHigh.push_back(Triangle(verticeA, verticeB, verticeC));
			index++;
		}

		glEnd();

	}

	numOfTrianglesHigh = index;
}

struct TriangleComp {
	bool operator() (const std::pair<int, Triangle>& p1, const std::pair<int, Triangle>& p2) const {
		// Compare the pairs based on their distance
		return p1.first < p2.first;
	}
};

bool trianglesShareEdge(const Triangle& t1, const Triangle& t2) {
	std::unordered_set<Vec3f> t1Vertices{ t1.vertexA, t1.vertexB, t1.vertexC };
	if (t1Vertices.count(t2.vertexA) && t1Vertices.count(t2.vertexB) ||
		t1Vertices.count(t2.vertexA) && t1Vertices.count(t2.vertexC) ||
		t1Vertices.count(t2.vertexB) && t1Vertices.count(t2.vertexC)) {
		return true;
	}
	return false;
}


void transferPickingInfo() {
	if (pickedTriangles.size() == 0) {
		for (int j = 0; j < numOfTrianglesHigh; j++) {
			trianglesHigh[j].picked = false;
		}
	}
	else {
		float diagonal = sqrt(2 * pow(STEP_SIZE_LOW, 2)) / 2;
		for (int i = 0; i < numOfTriangles; i++) {
			if (triangles[i].picked) {
				float xMiddle, yMiddle;
				xMiddle = (triangles[i].vertexA[0] + triangles[i].vertexB[0] + triangles[i].vertexC[0]) / 3;
				yMiddle = (triangles[i].vertexA[2] + triangles[i].vertexB[2] + triangles[i].vertexC[2]) / 3;

				float minDistance = INT16_MAX;
				float distance;
				int jMin;

				for (int j = 0; j < numOfTrianglesHigh; j++) {
					distance = sqrt(pow(trianglesHigh[j].vertexA[0] - xMiddle, 2) + pow(trianglesHigh[j].vertexA[2] - yMiddle, 2)) +
						sqrt(pow(trianglesHigh[j].vertexB[0] - xMiddle, 2) + pow(trianglesHigh[j].vertexB[2] - yMiddle, 2)) +
						sqrt(pow(trianglesHigh[j].vertexC[0] - xMiddle, 2) + pow(trianglesHigh[j].vertexC[2] - yMiddle, 2));
					if (distance < minDistance) {
						minDistance = distance;
						jMin = j;
					}
				}
				trianglesHigh[jMin].picked = !trianglesHigh[jMin].picked;
			}
		}
	}
}


Triangle getClosestTriangle(const Triangle& t) {
	Triangle closestTriangle;
	float minDistance = std::numeric_limits<float>::max();
	for (const Triangle& candidate : trianglesHigh) {
		float distance = cv::norm(t.vertexA - candidate.vertexA) +
			cv::norm(t.vertexB - candidate.vertexB) +
			cv::norm(t.vertexC - candidate.vertexC);
		if (distance < minDistance) {
			minDistance = distance;
			closestTriangle = candidate;
		}
	}
	return closestTriangle;
}

// Returns true if t1 and t2 are neighbors, false otherwise.
bool isNeighbor(const Triangle& t1, const Triangle& t2) {
	return (t1.vertexA == t2.vertexA || t1.vertexA == t2.vertexB || t1.vertexA == t2.vertexC ||
		t1.vertexB == t2.vertexA || t1.vertexB == t2.vertexB || t1.vertexB == t2.vertexC ||
		t1.vertexC == t2.vertexA || t1.vertexC == t2.vertexB || t1.vertexC == t2.vertexC);
}

// Returns the cost of traversing from t1 to t2.
int calculateCost(const Triangle& t1, const Triangle& t2) {
	// Calculate the distance between the centroids of the triangles.
	Vec3f t1Centroid = (t1.vertexA + t1.vertexB + t1.vertexC) / 3;
	Vec3f t2Centroid = (t2.vertexA + t2.vertexB + t2.vertexC) / 3;
	float distance = cv::norm(t1Centroid - t2Centroid);

	int cost = static_cast<int>(distance);
	return cost;
}

std::vector<std::pair<Triangle, int>> getNeighbors(const Triangle& t) {
	std::vector<std::pair<Triangle, int>> neighbors;

	// Find the neighbors of t and calculate their costs.
	for (const Triangle& other : triangles) {
		if (t == other) continue;
	//	if (isNeighbor(t, other) && trianglesShareEdge(t, other)) { // Should use this in case we want path of triangles with sam
		if (isNeighbor(t, other)) {
			int cost = calculateCost(t, other);
			neighbors.push_back({ other, cost });
		}
	}

	return neighbors;
}

std::vector<std::pair<Triangle, int>> getNeighborsHigh(const Triangle& t) {
	std::vector<std::pair<Triangle, int>> neighbors;

	// Find the neighbors of t and calculate their costs.
	// For example, you could use the distance between the centroids of the triangles
	// as the cost, and add an additional cost based on the difference in height.
	for (const Triangle& other : trianglesHigh) {
		if (t == other) continue;
//		if (isNeighbor(t, other) && trianglesShareEdge(t, other)) { // Should use this in case we want path of triangles with same edge
		if (isNeighbor(t, other)) {
			int cost = calculateCost(t, other);
			neighbors.push_back({ other, cost });
		}
	}

	return neighbors;
}

std::vector<int> findShortestPathDijkstraHigh(Triangle start, Triangle end) {
	std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>,
		std::greater<std::pair<int, int>>>
		q;
	std::unordered_map<int, int> predecessor;
	std::unordered_map<int, int> cost;

	int startIndex = std::find(trianglesHigh.begin(), trianglesHigh.end(), start) - trianglesHigh.begin();
	int endIndex = std::find(trianglesHigh.begin(), trianglesHigh.end(), end) - trianglesHigh.begin();

	// Initialize the cost of the start triangle to 0.
	q.push({ 0, startIndex });
	cost[startIndex] = 0;
	predecessor[startIndex] = startIndex;

	while (!q.empty()) {
		int currentIndex = q.top().second;
		q.pop();

		if (currentIndex == endIndex) {
			break;
		}

		Triangle current = trianglesHigh[currentIndex];
		std::vector<std::pair<Triangle, int>> neighbors = getNeighborsHigh(current);
		for (const std::pair<Triangle, int>& p : neighbors) {
			Triangle neighbor = p.first;
			int neighborCost = p.second;
			int neighborIndex = std::find(trianglesHigh.begin(), trianglesHigh.end(), neighbor) - trianglesHigh.begin();
			int newCost = cost[currentIndex] + neighborCost;
			if (cost.count(neighborIndex) == 0 || newCost < cost[neighborIndex]) {
				// Update the cost and predecessor of the neighbor.
				cost[neighborIndex] = newCost;
				predecessor[neighborIndex] = currentIndex;
				q.push({ newCost, neighborIndex });
			}
		}
	}

	// Construct the shortest path by following the predecessors from the end triangle back to the start triangle.
	std::vector<int> path;
	int currentIndex = endIndex;
	while (currentIndex != startIndex) {
		path.push_back(currentIndex);
		currentIndex = predecessor[currentIndex];
	}
	path.push_back(startIndex);
	std::reverse(path.begin(), path.end());
	return path;
}

bool areConnected(Triangle t1, Triangle t2) {
	if (t1.vertexA == t2.vertexA ||
		t1.vertexA == t2.vertexB ||
		t1.vertexA == t2.vertexC ||

		t1.vertexB == t2.vertexA ||
		t1.vertexB == t2.vertexB ||
		t1.vertexB == t2.vertexC ||

		t1.vertexC == t2.vertexA ||
		t1.vertexC == t2.vertexB ||
		t1.vertexC == t2.vertexC)
		return true;
	return false;
}

void transferPathToHighResolution() {
	std::vector<int> highResPath;
	for (int i : pickedTriangles) {
		Triangle lowResTriangle = triangles[i];
		Triangle highResTriangle = getClosestTriangle(lowResTriangle);
		int highResTriangleIndex = std::find(trianglesHigh.begin(), trianglesHigh.end(), highResTriangle) - trianglesHigh.begin();
		highResPath.push_back(highResTriangleIndex);
	}

	trianglesHigh[highResPath[0]].picked = true; // Color first triangle in path
	trianglesHigh[highResPath[1]].picked = true; // Color last triangle in path

	if (highResPath.size() > 2) {
//		if (!trianglesShareEdge(trianglesHigh[highResPath[0]], trianglesHigh[highResPath[2]])) { // In case we want edges version
		if (!areConnected(trianglesHigh[highResPath[0]], trianglesHigh[highResPath[2]])) { // Connect first and second triangles
			std::vector<int> toColor = findShortestPathDijkstraHigh(trianglesHigh[highResPath[0]], trianglesHigh[highResPath[2]]);
			for (int j : toColor) {
				trianglesHigh[j].picked = true;
			}
		}

//		if (!trianglesShareEdge(trianglesHigh[highResPath[highResPath.size() - 1]], trianglesHigh[highResPath[1]])) { // In case we want edges version
		if (!areConnected(trianglesHigh[highResPath[highResPath.size() - 1]], trianglesHigh[highResPath[1]])) { // Connect before last and last triangles
			std::vector<int> toColor = findShortestPathDijkstraHigh(trianglesHigh[highResPath[highResPath.size() - 1]], trianglesHigh[highResPath[1]]);
			for (int j : toColor) {
				trianglesHigh[j].picked = true;
			}
		}
	}

	else {
	//	if (!trianglesShareEdge(trianglesHigh[highResPath[0]], trianglesHigh[highResPath[1]])) { // In case we want edges version
		if (!areConnected(trianglesHigh[highResPath[0]], trianglesHigh[highResPath[1]])) { // Connect both triangles
			std::vector<int> toColor = findShortestPathDijkstraHigh(trianglesHigh[highResPath[0]], trianglesHigh[highResPath[1]]);
			for (int j : toColor) {
				trianglesHigh[j].picked = true;
			}
		}
	}

	for (int i = 2; i < highResPath.size() - 1; i++) { // Connect and color all other triangles in path
	//	if (!trianglesShareEdge(trianglesHigh[highResPath[i]], trianglesHigh[highResPath[i + 1]])) { // In case we want edges version
		if (!areConnected(trianglesHigh[highResPath[i]], trianglesHigh[highResPath[i + 1]])) {
			std::vector<int> toColor = findShortestPathDijkstraHigh(trianglesHigh[highResPath[i]], trianglesHigh[highResPath[i + 1]]);
				for (int j : toColor) {
					trianglesHigh[j].picked = true;
				}
		}
	}
	
}


std::vector<int> findShortestPathDijkstra(Triangle start, Triangle end) {
	std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>,
		std::greater<std::pair<int, int>>>
		q;
	std::unordered_map<int, int> predecessor;
	std::unordered_map<int, int> cost;

	int startIndex = std::find(triangles.begin(), triangles.end(), start) - triangles.begin();
	int endIndex = std::find(triangles.begin(), triangles.end(), end) - triangles.begin();

	// Initialize the cost of the start triangle to 0.
	q.push({ 0, startIndex });
	cost[startIndex] = 0;
	predecessor[startIndex] = startIndex;

	while (!q.empty()) {
		int currentIndex = q.top().second;
		q.pop();

		if (currentIndex == endIndex) {
			break;
		}

		Triangle current = triangles[currentIndex];
		std::vector<std::pair<Triangle, int>> neighbors = getNeighbors(current);
		for (const std::pair<Triangle, int>& p : neighbors) {
			Triangle neighbor = p.first;
			int neighborCost = p.second;
			int neighborIndex = std::find(triangles.begin(), triangles.end(), neighbor) - triangles.begin();
			int newCost = cost[currentIndex] + neighborCost;
			if (cost.count(neighborIndex) == 0 || newCost < cost[neighborIndex]) {
				// Update the cost and predecessor of the neighbor.
				cost[neighborIndex] = newCost;
				predecessor[neighborIndex] = currentIndex;
				q.push({ newCost, neighborIndex });
			}
		}
	}

	// Construct the shortest path by following the predecessors from the end triangle back to the start triangle.
	std::vector<int> path;
	int currentIndex = endIndex;
	while (currentIndex != startIndex) {
		path.push_back(currentIndex);
		currentIndex = predecessor[currentIndex];
	}
	path.push_back(startIndex);
	std::reverse(path.begin(), path.end());
	return path;
}

void pickPathTriangles() {
	std::vector<int> path = findShortestPathDijkstra(triangles[pickedTriangles[0]], triangles[pickedTriangles[1]]);
	for (int i = 0; i < path.size(); i++) {
		triangles[path[i]].picked = true;
		if (i != 0 && i != path.size() - 1)
			pickedTriangles.push_back(path[i]);
	}
}

void clearPickedTriangles() {
	for (int i = 0; i < triangles.size(); i++) {
		triangles[i].picked = false;
	}
	pickedTriangles.clear();
}

GLvoid ReSizeGLScene(GLsizei width, GLsizei height)		// Resize And Initialize The GL Window
{
	if (height == 0)										// Prevent A Divide By Zero By
		height = 1;										// Making Height Equal One

	glViewport(0, 0, width, height);						// Reset The Current Viewport

	glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
	glLoadIdentity();									// Reset The Projection Matrix

	// Calculate The Aspect Ratio Of The Window.  Farthest Distance Changed To 500.0f (NEW)
	gluPerspective(45.0f, (GLfloat)width / (GLfloat)height, 0.1f, 500.0f);
	
	glMatrixMode(GL_MODELVIEW);							// Select The Modelview Matrix
	glLoadIdentity();									// Reset The Modelview Matrix
}

int InitGL(GLvoid)										// All Setup For OpenGL Goes Here
{
	/**
	if (!LoadGLTextures())								// Jump To Texture Loading Routine ( NEW )
	{
		return FALSE;									// If Texture Didn't Load Return FALSE
	}
	*/

//	glEnable(GL_TEXTURE_2D);							// Enable Texture Mapping
	glShadeModel(GL_SMOOTH);							// Enable Smooth Shading
	glClearColor(0.0f, 0.0f, 0.0f, 0.5f);				// Black Background
	glClearDepth(1.0f);									// Depth Buffer Setup
	glEnable(GL_DEPTH_TEST);							// Enables Depth Testing
	glDepthFunc(GL_LEQUAL);								// The Type Of Depth Testing To Do
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculations

	initTrianglesLow(g_HeightMap);
	initTrianglesHigh(g_HeightMap);

	return TRUE;										// Initialization Went OK
}

void SetVertexColor(Mat *pHeightMap, int x, int y)		// Sets The Color Value For A Particular Index, Depending On The Height Index
{
	if (!pHeightMap) return;								// Make Sure Our Height Data Is Valid

	float bColor = pHeightMap->at<Vec3b>(x, y).val[0] / 255.0f;
	float gColor = pHeightMap->at<Vec3b>(x, y).val[1] / 255.0f;
	float rColor = pHeightMap->at<Vec3b>(x, y).val[2] / 255.0f;

	glColor3f(rColor, gColor, bColor);
}

void renderMap(Mat pHeightMap)
{
	glPolygonOffset(-1.0f, -1.0f);
	glEnable(GL_DEPTH_TEST);

	for (int i = 0; i < numOfTriangles; i++) {
		glBegin(GL_TRIANGLES);

		if (triangles[i].picked) {
			glColor3f(0.0f, 0.0f, 1.0f);
			glVertex3f(triangles[i].vertexA[0], triangles[i].vertexA[1], triangles[i].vertexA[2]);
			glColor3f(0.0f, 0.0f, 1.0f);
			glVertex3f(triangles[i].vertexB[0], triangles[i].vertexB[1], triangles[i].vertexB[2]);
			glColor3f(0.0f, 0.0f, 1.0f);
			glVertex3f(triangles[i].vertexC[0], triangles[i].vertexC[1], triangles[i].vertexC[2]);
		}

		else {
			SetVertexColor(&pHeightMap, triangles[i].vertexA[0], triangles[i].vertexA[2]);
			glVertex3f(triangles[i].vertexA[0], triangles[i].vertexA[1], triangles[i].vertexA[2]);
			SetVertexColor(&pHeightMap, triangles[i].vertexB[0], triangles[i].vertexB[2]);
			glVertex3f(triangles[i].vertexB[0], triangles[i].vertexB[1], triangles[i].vertexB[2]);
			SetVertexColor(&pHeightMap, triangles[i].vertexC[0], triangles[i].vertexC[2]);
			glVertex3f(triangles[i].vertexC[0], triangles[i].vertexC[1], triangles[i].vertexC[2]);
		}

		glEnd();
	}
	// Another rendering, but this time outlines rendering

	glPolygonOffset(1.0f, 1.0f);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	for (int i = 0; i < triangles.size(); i++) {
		glBegin(GL_TRIANGLES);

		glColor3f(1.0f, 1.0f, 1.0f);
		glVertex3f(triangles[i].vertexA[0], triangles[i].vertexA[1], triangles[i].vertexA[2]);
		glColor3f(1.0f, 1.0f, 1.0f);
		glVertex3f(triangles[i].vertexB[0], triangles[i].vertexB[1], triangles[i].vertexB[2]);
		glColor3f(1.0f, 1.0f, 1.0f);
		glVertex3f(triangles[i].vertexC[0], triangles[i].vertexC[1], triangles[i].vertexC[2]);

		glEnd();
	}

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

	void renderMapHigh(Mat pHeightMap)
	{
		glPolygonOffset(-1.0f, -1.0f);
		glEnable(GL_DEPTH_TEST);

		for (int i = 0; i < numOfTrianglesHigh; i++) {
			glBegin(GL_TRIANGLES);

			if (trianglesHigh[i].picked) {
				glColor3f(0.0f, 0.0f, 1.0f);
				glVertex3f(trianglesHigh[i].vertexA[0], trianglesHigh[i].vertexA[1], trianglesHigh[i].vertexA[2]);
				glColor3f(0.0f, 0.0f, 1.0f);
				glVertex3f(trianglesHigh[i].vertexB[0], trianglesHigh[i].vertexB[1], trianglesHigh[i].vertexB[2]);
				glColor3f(0.0f, 0.0f, 1.0f);
				glVertex3f(trianglesHigh[i].vertexC[0], trianglesHigh[i].vertexC[1], trianglesHigh[i].vertexC[2]);
			}

			else {
				SetVertexColor(&pHeightMap, trianglesHigh[i].vertexA[0], trianglesHigh[i].vertexA[2]);
				glVertex3f(trianglesHigh[i].vertexA[0], trianglesHigh[i].vertexA[1], trianglesHigh[i].vertexA[2]);
				SetVertexColor(&pHeightMap, trianglesHigh[i].vertexB[0], trianglesHigh[i].vertexB[2]);
				glVertex3f(trianglesHigh[i].vertexB[0], trianglesHigh[i].vertexB[1], trianglesHigh[i].vertexB[2]);
				SetVertexColor(&pHeightMap, trianglesHigh[i].vertexC[0], trianglesHigh[i].vertexC[2]);
				glVertex3f(trianglesHigh[i].vertexC[0], trianglesHigh[i].vertexC[1], trianglesHigh[i].vertexC[2]);
			}

			glEnd();
		}

	// Another rendering, but this time outlines rendering

	glPolygonOffset(1.0f, 1.0f);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	for (int i = 0; i < numOfTrianglesHigh; i++) {
		glBegin(GL_TRIANGLES);

		glColor3f(1.0f, 1.0f, 1.0f);
		glVertex3f(trianglesHigh[i].vertexA[0], trianglesHigh[i].vertexA[1], trianglesHigh[i].vertexA[2]);
		glColor3f(1.0f, 1.0f, 1.0f);
		glVertex3f(trianglesHigh[i].vertexB[0], trianglesHigh[i].vertexB[1], trianglesHigh[i].vertexB[2]);
		glColor3f(1.0f, 1.0f, 1.0f);
		glVertex3f(trianglesHigh[i].vertexC[0], trianglesHigh[i].vertexC[1], trianglesHigh[i].vertexC[2]);

		glEnd();
	}

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

Vec3f idToColor(int i) {
	int r = (i & 0x000000FF) >> 0;
	int g = (i & 0x0000FF00) >> 8;
	int b = (i & 0x00FF0000) >> 16;

	return Vec3f(r / 255.0f, g / 255.0f, b / 255.0f);
}

void fogScreen() {
	Fog fog;
	fog.setup();
}

void pick(Mat pHeightMap) {
	glDisable(GL_FOG);
	Vec3f colors;
	// Rendering for picking
		
	GLint buffer;
	glGetIntegerv(GL_DRAW_BUFFER, &buffer);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	int X = 0, Y = 0;									// Create Some Variables To Walk The Array With.
	float x, y, z;										// Create Some Variables For Readability

	int rows = pHeightMap.rows;
	int cols = pHeightMap.cols;

	float index = 0;

	glPolygonOffset(-1.0f, -1.0f);
	glEnable(GL_DEPTH_TEST);

	for (X = 0; X < (rows - STEP_SIZE_LOW); X += STEP_SIZE_LOW) {
		glBegin(GL_TRIANGLES);							

		for (Y = 0; Y < (cols - STEP_SIZE_LOW); Y += STEP_SIZE_LOW)
		{
			x = X;
			y = Height(&pHeightMap, X, Y);
			z = Y;

			colors = idToColor(index);
			glColor3f(colors[0], colors[1], colors[2]);

			glVertex3f(x, y, z);

			x = X + STEP_SIZE_LOW;
			y = Height(&pHeightMap, X + STEP_SIZE_LOW, Y);
			z = Y;

			colors = idToColor(index);
			glColor3f(colors[0], colors[1], colors[2]);

			glVertex3f(x, y, z);

			x = X;
			y = Height(&pHeightMap, X, Y + STEP_SIZE_LOW);
			z = Y + STEP_SIZE_LOW;

			colors = idToColor(index);
			glColor3f(colors[0], colors[1], colors[2]);

			glVertex3f(x, y, z);

			x = X;
			y = Height(&pHeightMap, X, Y + STEP_SIZE_LOW);
			z = Y + STEP_SIZE_LOW;

			index += 1;
			colors = idToColor(index);
			glColor3f(colors[0], colors[1], colors[2]);

			glVertex3f(x, y, z);

			x = X + STEP_SIZE_LOW;
			y = Height(&pHeightMap, X + STEP_SIZE_LOW, Y);
			z = Y;

			colors = idToColor(index);
			glColor3f(colors[0], colors[1], colors[2]);

			glVertex3f(x, y, z);	

			x = X + STEP_SIZE_LOW;
			y = Height(&pHeightMap, X + STEP_SIZE_LOW, Y + STEP_SIZE_LOW);
			z = Y + STEP_SIZE_LOW;

			colors = idToColor(index);
			glColor3f(colors[0], colors[1], colors[2]);

			glVertex3f(x, y, z);

			index += 1;
		}

		glEnd();
	}

	// Another rendering, but this time outlines rendering

	glPolygonOffset(1.0f, 1.0f);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	for (X = 0; X < (rows - STEP_SIZE_LOW); X += STEP_SIZE_LOW) {
		glBegin(GL_TRIANGLES);

		for (Y = 0; Y < (cols - STEP_SIZE_LOW); Y += STEP_SIZE_LOW)
		{
			x = X;
			y = Height(&pHeightMap, X, Y);
			z = Y;

			glColor3f(1.0f, 1.0f, 1.0f);

			glVertex3f(x, y, z);

			x = X + STEP_SIZE_LOW;
			y = Height(&pHeightMap, X + STEP_SIZE_LOW, Y);
			z = Y;

			glColor3f(1.0f, 1.0f, 1.0f);

			glVertex3f(x, y, z);

			x = X;
			y = Height(&pHeightMap, X, Y + STEP_SIZE_LOW);
			z = Y + STEP_SIZE_LOW;

			glColor3f(1.0f, 1.0f, 1.0f);

			glVertex3f(x, y, z);

			x = X;
			y = Height(&pHeightMap, X, Y + STEP_SIZE_LOW);
			z = Y + STEP_SIZE_LOW;

			glColor3f(1.0f, 1.0f, 1.0f);

			glVertex3f(x, y, z);

			x = X + STEP_SIZE_LOW;
			y = Height(&pHeightMap, X + STEP_SIZE_LOW, Y);
			z = Y;

			glColor3f(1.0f, 1.0f, 1.0f);

			glVertex3f(x, y, z);

			x = X + STEP_SIZE_LOW;
			y = Height(&pHeightMap, X + STEP_SIZE_LOW, Y + STEP_SIZE_LOW);
			z = Y + STEP_SIZE_LOW;

			glColor3f(1.0f, 1.0f, 1.0f);

			glVertex3f(x, y, z);
		}

		glEnd();
	}

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);


	int viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	int yPoint = viewport[3] - mouseY;

	unsigned char pixel[3];
	glReadPixels(mouseX, yPoint, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, pixel);

	int triangleId =
		pixel[0] +
		pixel[1] * 256 +
		pixel[2] * 256 * 256;

	if (triangleId == 0x00000000) { // Background
	
	}
	else if (triangleId >= 0 && triangleId < numOfTriangles) {
		triangles[triangleId].picked = !triangles[triangleId].picked;
		pickedTriangles.push_back(triangleId);
	}

	picked = false;
	if (shouldFog) {
		fogScreen();
	}
}

int DrawGLScene(GLvoid)									// Here's Where We Do All The Drawing
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	// Clear The Screen And The Depth Buffer
	glLoadIdentity();									// Reset The Matrix

	//GLfloat ytrans = -walkbias - 0.25f;
	//GLfloat sceneroty = 360.0f - yrot;

	glRotatef(rotationAngle * 40, 0, 10, 0);
	glTranslatef(-10, yAxisTranslation * 10, 10);

	// 			 Position	      View		Up Vector
	gluLookAt(212, 60, 194, 186, 60, 171, 0, 1, 0);	// This Determines Where The Camera's Position And View Is

	glScalef(scaleValue, scaleValue * HEIGHT_RATIO, scaleValue);

	
	if (picked)
		pick(g_HeightMap);
	else {
		if (!highRes)
			renderMap(g_HeightMap);						// Render The Height Map
		else
			renderMapHigh(g_HeightMap);
	}

	return TRUE;										// Keep Going
}

GLvoid KillGLWindow(GLvoid)								// Properly Kill The Window
{
	if (hRC)											// Do We Have A Rendering Context?
	{
		if (!wglMakeCurrent(NULL, NULL))					// Are We Able To Release The DC And RC Contexts?
		{
			MessageBox(NULL, "Release Of DC And RC Failed.", "SHUTDOWN ERROR", MB_OK | MB_ICONINFORMATION);
		}

		if (!wglDeleteContext(hRC))						// Are We Able To Delete The RC?
		{
			MessageBox(NULL, "Release Rendering Context Failed.", "SHUTDOWN ERROR", MB_OK | MB_ICONINFORMATION);
		}
		hRC = NULL;										// Set RC To NULL
	}

	if (hDC && !ReleaseDC(hWnd, hDC))					// Are We Able To Release The DC
	{
		MessageBox(NULL, "Release Device Context Failed.", "SHUTDOWN ERROR", MB_OK | MB_ICONINFORMATION);
		hDC = NULL;										// Set DC To NULL
	}

	if (hWnd && !DestroyWindow(hWnd))					// Are We Able To Destroy The Window?
	{
		MessageBox(NULL, "Could Not Release hWnd.", "SHUTDOWN ERROR", MB_OK | MB_ICONINFORMATION);
		hWnd = NULL;										// Set hWnd To NULL
	}

	if (!UnregisterClass("OpenGL", hInstance))			// Are We Able To Unregister Class
	{
		MessageBox(NULL, "Could Not Unregister Class.", "SHUTDOWN ERROR", MB_OK | MB_ICONINFORMATION);
		hInstance = NULL;									// Set hInstance To NULL
	}
}

/*	This Code Creates Our OpenGL Window.  Parameters Are:					*
 *	title			- Title To Appear At The Top Of The Window				*
 *	width			- Width Of The GL Window								*
 *	height			- Height Of The GL Window								*
 *	bits			- Number Of Bits To Use For Color (8/16/24/32)			*/

BOOL CreateGLWindow(char* title, int width, int height, int bits)
{
	GLuint		PixelFormat;							// Holds The Results After Searching For A Match
	WNDCLASS	wc;										// Windows Class Structure
	DWORD		dwExStyle;								// Window Extended Style
	DWORD		dwStyle;								// Window Style
	RECT		WindowRect;								// Grabs Rectangle Upper Left / Lower Right Values
	WindowRect.left = (long)0;							// Set Left Value To 0
	WindowRect.right = (long)width;						// Set Right Value To Requested Width
	WindowRect.top = (long)0;								// Set Top Value To 0
	WindowRect.bottom = (long)height;						// Set Bottom Value To Requested Height

	hInstance = GetModuleHandle(NULL);		// Grab An Instance For Our Window
	wc.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;	// Redraw On Size, And Own DC For Window.
	wc.lpfnWndProc = (WNDPROC)WndProc;			// WndProc Handles Messages
	wc.cbClsExtra = 0;							// No Extra Window Data
	wc.cbWndExtra = 0;							// No Extra Window Data
	wc.hInstance = hInstance;					// Set The Instance
	wc.hIcon = LoadIcon(NULL, IDI_WINLOGO);	// Load The Default Icon
	wc.hCursor = LoadCursor(NULL, IDC_ARROW);	// Load The Arrow Pointer
	wc.hbrBackground = NULL;							// No Background Required For GL
	wc.lpszMenuName = NULL;							// We Don't Want A Menu
	wc.lpszClassName = "OpenGL";						// Set The Class Name

	if (!RegisterClass(&wc))							// Attempt To Register The Window Class
	{
		MessageBox(NULL, "Failed To Register The Window Class.", "ERROR", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;									// Return FALSE
	}

		dwExStyle = WS_EX_APPWINDOW | WS_EX_WINDOWEDGE;	// Window Extended Style
		dwStyle = WS_OVERLAPPEDWINDOW;					// Windows Style

	AdjustWindowRectEx(&WindowRect, dwStyle, FALSE, dwExStyle);	// Adjust Window To True Requested Size

	// Create The Window
	if (!(hWnd = CreateWindowEx(dwExStyle,				// Extended Style For The Window
		"OpenGL",				// Class Name
		title,					// Window Title
		dwStyle |				// Defined Window Style
		WS_CLIPSIBLINGS |		// Required Window Style
		WS_CLIPCHILDREN,		// Required Window Style
		0, 0,					// Window Position
		WindowRect.right - WindowRect.left,	// Calculate Window Width
		WindowRect.bottom - WindowRect.top,	// Calculate Window Height
		NULL,					// No Parent Window
		NULL,					// No Menu
		hInstance,				// Instance
		NULL)))					// Dont Pass Anything To WM_CREATE
	{
		KillGLWindow();									// Reset The Display
		MessageBox(NULL, "Window Creation Error.", "ERROR", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;									// Return FALSE
	}

	static	PIXELFORMATDESCRIPTOR pfd =					// pfd Tells Windows How We Want Things To Be
	{
		sizeof(PIXELFORMATDESCRIPTOR),					// Size Of This Pixel Format Descriptor
		1,												// Version Number
		PFD_DRAW_TO_WINDOW |							// Format Must Support Window
		PFD_SUPPORT_OPENGL |							// Format Must Support OpenGL
		PFD_DOUBLEBUFFER,								// Must Support Double Buffering
		PFD_TYPE_RGBA,									// Request An RGBA Format
		bits,											// Select Our Color Depth
		0, 0, 0, 0, 0, 0,								// Color Bits Ignored
		0,												// No Alpha Buffer
		0,												// Shift Bit Ignored
		0,												// No Accumulation Buffer
		0, 0, 0, 0,										// Accumulation Bits Ignored
		16,												// 16Bit Z-Buffer (Depth Buffer)  
		0,												// No Stencil Buffer
		0,												// No Auxiliary Buffer
		PFD_MAIN_PLANE,									// Main Drawing Layer
		0,												// Reserved
		0, 0, 0											// Layer Masks Ignored
	};

	if (!(hDC = GetDC(hWnd)))								// Did We Get A Device Context?
	{
		KillGLWindow();									// Reset The Display
		MessageBox(NULL, "Can't Create A GL Device Context.", "ERROR", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;									// Return FALSE
	}

	if (!(PixelFormat = ChoosePixelFormat(hDC, &pfd)))		// Did Windows Find A Matching Pixel Format?
	{
		KillGLWindow();									// Reset The Display
		MessageBox(NULL, "Can't Find A Suitable PixelFormat.", "ERROR", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;									// Return FALSE
	}

	if (!SetPixelFormat(hDC, PixelFormat, &pfd))			// Are We Able To Set The Pixel Format?
	{
		KillGLWindow();									// Reset The Display
		MessageBox(NULL, "Can't Set The PixelFormat.", "ERROR", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;									// Return FALSE
	}

	if (!(hRC = wglCreateContext(hDC)))					// Are We Able To Get A Rendering Context?
	{
		KillGLWindow();									// Reset The Display
		MessageBox(NULL, "Can't Create A GL Rendering Context.", "ERROR", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;									// Return FALSE
	}

	if (!wglMakeCurrent(hDC, hRC))						// Try To Activate The Rendering Context
	{
		KillGLWindow();									// Reset The Display
		MessageBox(NULL, "Can't Activate The GL Rendering Context.", "ERROR", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;									// Return FALSE
	}

	ShowWindow(hWnd, SW_SHOW);							// Show The Window
	SetForegroundWindow(hWnd);							// Slightly Higher Priority
	SetFocus(hWnd);										// Sets Keyboard Focus To The Window
	ReSizeGLScene(width, height);						// Set Up Our Perspective GL Screen

	if (!InitGL())										// Initialize Our Newly Created GL Window
	{
		KillGLWindow();									// Reset The Display
		MessageBox(NULL, "Initialization Failed.", "ERROR", MB_OK | MB_ICONEXCLAMATION);
		return FALSE;									// Return FALSE
	}

	return TRUE;										// Success
}

LRESULT CALLBACK WndProc(HWND	hWnd,				// Handle For This Window
	UINT	uMsg,				// Message For This Window
	WPARAM	wParam,				// Additional Message Information
	LPARAM	lParam)				// Additional Message Information
{
	switch (uMsg)										// Check For Windows Messages
	{
	case WM_ACTIVATE:								// Watch For Window Activate Message
	{
		if (!HIWORD(wParam))						// Check Minimization State
		{
			active = TRUE;							// Program Is Active
		}
		else
		{
			active = FALSE;							// Program Is No Longer Active
		}

		return 0;									// Return To The Message Loop
	}

	case WM_SYSCOMMAND:								// Intercept System Commands
	{
		switch (wParam)								// Check System Calls
		{
		case SC_SCREENSAVE:						// Screensaver Trying To Start?
		case SC_MONITORPOWER:					// Monitor Trying To Enter Powersave?
			return 0;								// Prevent From Happening
		}
		break;										// Exit
	}

	case WM_CLOSE:									// Did We Receive A Close Message?
	{
		PostQuitMessage(0);							// Send A Quit Message
		return 0;									// Jump Back
	}

	case WM_RBUTTONDOWN:							// Did We Receive A Right Mouse Click?
	{
		highRes = !highRes;
		if (!wasPaved) {
			transferPickingInfo();
		}
		else {
			transferPathToHighResolution();
		}
		return 0;									// Jump Back
	}

	case WM_LBUTTONDOWN:
		// Left mouse button was pressed
		if (highRes == false) {
			POINT cursorPos;
			GetCursorPos(&cursorPos);
			ScreenToClient(hWnd, &cursorPos);
			mouseX = cursorPos.x;
			mouseY = cursorPos.y;
			picked = TRUE;
	}
		break;

	case WM_KEYDOWN:								// Is A Key Being Held Down?
	{
		keys[wParam] = TRUE;						// If So, Mark It As TRUE
		return 0;									// Jump Back
	}

	case WM_KEYUP:									// Has A Key Been Released?
	{
		keys[wParam] = FALSE;						// If So, Mark It As FALSE
		return 0;									// Jump Back
	}

	case WM_SIZE:									// Resize The OpenGL Window
	{
		ReSizeGLScene(LOWORD(lParam), HIWORD(lParam));  // LoWord=Width, HiWord=Height
		return 0;									// Jump Back
	}
	}

	// Pass All Unhandled Messages To DefWindowProc
	return DefWindowProc(hWnd, uMsg, wParam, lParam);
}

void rotateOrTranslate() {
	if (keys[VK_F2])
		rotationAngle += 0.01f;
	if (keys[VK_F3])
		rotationAngle -= 0.01f;
	if (keys[VK_F4])
		yAxisTranslation += 0.01f;
	if (keys[VK_F5])
		yAxisTranslation -= 0.01f;
}

int WINAPI WinMain(HINSTANCE	hInstance,				// Instance
	HINSTANCE	hPrevInstance,			// Previous Instance
	LPSTR		lpCmdLine,				// Command Line Parameters
	int			nCmdShow)				// Window Show State
{
	MSG		msg;										// Windows Message Structure
	BOOL	done = FALSE;									// Bool Variable To Exit Loop

	// Create Our OpenGL Window
	if (!CreateGLWindow((char*)"Tomer's & Dolev's Project", 640, 480, 16))
	{
		return 0;										// Quit If Window Was Not Created
	}

	while (!done)										// Loop That Runs While done=FALSE
	{
		if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))		// Is There A Message Waiting?
		{
			if (msg.message == WM_QUIT)					// Have We Received A Quit Message?
			{
				done = TRUE;								// If So done=TRUE
			}
			else										// If Not, Deal With Window Messages
			{
				TranslateMessage(&msg);					// Translate The Message
				DispatchMessage(&msg);					// Dispatch The Message
			}
		}
		else											// If There Are No Messages
		{
			// Draw The Scene.  Watch For ESC Key And Quit Messages From DrawGLScene()
			if ((active && !DrawGLScene()) || keys[VK_ESCAPE])	// Active?  Was There A Quit Received?
			{
				done = TRUE;								// ESC or DrawGLScene Signalled A Quit
			}
			else if (active)							// Not Time To Quit, Update Screen
			{
				SwapBuffers(hDC);						// Swap Buffers (Double Buffering)
			}

			if (keys[VK_UP])							// Is the UP ARROW key Being Pressed?
				scaleValue += 0.001f;					// Increase the scale value to zoom in

			if (keys[VK_DOWN])							// Is the DOWN ARROW key Being Pressed?
			    scaleValue -= 0.001f;					// Decrease the scale value to zoom out
			
			if (keys[VK_F5] || keys[VK_F2] || keys[VK_F3] || keys[VK_F4]) {
				rotateOrTranslate();
			}

			if (keys[VK_F6]) {
				pickPathTriangles();
				wasPaved = true;
			}

			if (keys[VK_F7]) {
				clearPickedTriangles();
				wasPaved = false;
			}

			if (keys[VK_F8]) {
				shouldFog = !shouldFog;
				if (shouldFog)
					fogScreen();
				else {
					glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
					glDisable(GL_FOG);
				}
			}
		}
	}

	// Shutdown
	KillGLWindow();										// Kill The Window
	return (msg.wParam);								// Exit The Program
}