#ifndef MODEL
#define MODEL

#include <glm/glm.hpp>
#include <vector>
#include <map>

class Model
{

public:

	Model(void);
	~Model(void);

	int* indices;
	float* vertices;
	float* normals;
	float* colors;

	int numIndices;
	int numVertices;

	int* counterIndices;

	float maxCoor;

	std::vector<int>* markedVertices;
	std::map<int, bool>* markedVerticesMap;

	void setMarkedVerticesForGrabMode();
	void setMarkedVerticesForDefaultMode();

	void loadModel(char* path);
	void deleteObjects();

	int meshLoaded;
	int maxConnectedVertices;
	
	void updateVertices(float* newPoints);
	void makeVerticesConsistent();

	void markVertex(int index);
	void markVertices(std::vector<int>* indices);
	void unmarkAllVertices();

	void grabMarkedVertices(glm::vec4* vec);
};

#endif // MODEL
