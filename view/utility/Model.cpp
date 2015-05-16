#include "Model.h"

#include <iostream>
#include <fstream>

Model::Model(void)
{

	indices = NULL;
	vertices = NULL;
	normals = NULL;
	colors = NULL;

	markedVertices = NULL;
	markedVerticesMap = NULL;

	counterIndices = NULL;

	maxCoor = 1.0;

	meshLoaded = false;

}

Model::~Model(void)
{

	deleteObjects();

}

/*
 * delete all objects
 */
void Model::deleteObjects()
{

	if(indices!=NULL)
	{
		delete[] indices;
	}

	if(vertices!=NULL)
	{
		delete[] vertices;
	}

	if(normals!=NULL)
	{
		delete[] normals;
	}

	if(counterIndices!=NULL)
	{
		delete[] counterIndices;
	}

	if(markedVertices!=NULL)
	{
		delete markedVertices;
	}

	if(markedVerticesMap!=NULL)
	{
		delete markedVerticesMap;
	}

}

/*
 * load model from the file refered to path
 */
void Model::loadModel(char* path)
{

	deleteObjects();

	// get amount of indices and vertices
    numIndices = 12;
    numVertices = 8;

	// initiate arrays and vectors
	indices = new int[numIndices*3];
	vertices = new float[numVertices*3];
	normals = new float[numVertices*3];

	markedVertices = new std::vector<int>();
	markedVertices->reserve(numVertices);
	markedVerticesMap = new std::map<int,bool>();

    // write indices into array
    indices[0] = 3;
    indices[1] = 1;
    indices[2] = 0;

    indices[3] = 3;
    indices[4] = 2;
    indices[5] = 1;

    indices[6] = 2;
    indices[7] = 4;
    indices[8] = 1;

    indices[9] = 5;
    indices[10] = 4;
    indices[11] = 2;

    indices[12] = 0;
    indices[13] = 7;
    indices[14] = 6;

    indices[15] = 0;
    indices[16] = 6;
    indices[17] = 3;

    indices[18] = 4;
    indices[19] = 5;
    indices[20] = 7;

    indices[21] = 5;
    indices[22] = 6;
    indices[23] = 7;

    indices[24] = 0;
    indices[25] = 4;
    indices[26] = 7;

    indices[27] = 0;
    indices[28] = 1;
    indices[29] = 4;

    indices[30] = 2;
    indices[31] = 3;
    indices[32] = 6;

    indices[33] = 2;
    indices[34] = 6;
    indices[35] = 5;


	float tempMaxCoor = 0;


    // vertex 0
    vertices[ 0] = -1.0;
    vertices[ 1] = +1.0;
    vertices[ 2] = +1.0;

    // vertex 1
    vertices[ 3] = +1.0;
    vertices[ 4] = +1.0;
    vertices[ 5] = +1.0;

    // vertex 2
    vertices[ 6] = +1.0;
    vertices[ 7] = -1.0;
    vertices[ 8] = +1.0;

    // vertex 3
    vertices[ 9] = -1.0;
    vertices[10] = -1.0;
    vertices[11] = +1.0;

    // vertex 4
    vertices[12] = +1.0;
    vertices[13] = +1.0;
    vertices[14] = -1.0;

    // vertex 5
    vertices[15] = +1.0;
    vertices[16] = -1.0;
    vertices[17] = -1.0;

    // vertex 6
    vertices[18] = -1.0;
    vertices[19] = -1.0;
    vertices[20] = -1.0;

    // vertex 7
    vertices[21] = -1.0;
    vertices[22] = +1.0;
    vertices[23] = -1.0;


    // normal vertex 0
    normals[ 0] = -1.0;
    normals[ 1] = +1.0;
    normals[ 2] = +1.0;

    // normal vertex 1
    normals[ 3] = +1.0;
    normals[ 4] = +1.0;
    normals[ 5] = +1.0;

    // normal vertex 2
    normals[ 6] = +1.0;
    normals[ 7] = -1.0;
    normals[ 8] = +1.0;

    // normal vertex 3
    normals[ 9] = -1.0;
    normals[10] = -1.0;
    normals[11] = +1.0;

    // normal vertex 4
    normals[12] = +1.0;
    normals[13] = +1.0;
    normals[14] = -1.0;

    // normal vertex 5
    normals[15] = +1.0;
    normals[16] = -1.0;
    normals[17] = -1.0;

    // normal vertex 6
    normals[18] = -1.0;
    normals[19] = -1.0;
    normals[20] = -1.0;

    // normal vertex 7
    normals[21] = -1.0;
    normals[22] = +1.0;
    normals[23] = -1.0;


    tempMaxCoor = 1*2;

	// set max absolute value
	if(tempMaxCoor>0)
	{
		maxCoor = tempMaxCoor;
	}

	// calculate max amount of connected vertices 
    maxConnectedVertices = 10;

	// initiate color rgb array and color index array for vertices
	colors = new float[numVertices*3];
	counterIndices = new int[numVertices];
	for(int i=0;i<numVertices;i++)
	{

		colors[i*3+0] = .0;
		colors[i*3+1] = .0;
		colors[i*3+2] = 1.0;

		counterIndices[i] = i;
	}

	// declare mesh as loaded
	meshLoaded = true;

}

/*
 * update vertices by using the array newPoints
 */
void Model::updateVertices(float* newPoints)
{

}

/*
 * make vertices consistent by write vertex array data to open mesh and update normals
 */
void Model::makeVerticesConsistent()
{

}

/*
 * mark a vertex with index index
 */
void Model::markVertex(int index)
{

	std::vector<int>* tempVertices;
	std::map<int,bool>* tempVerticesMap;

	tempVertices = markedVertices;
	tempVerticesMap = markedVerticesMap;

	// test whether index is valid
	if(index<0 || this->numVertices<=index)
	{
		return;
	}
	if(tempVerticesMap->count(index)>0)
	{
		return;
	}

	if(markedVerticesMap->count(index)>0)
	{
		return;
	}

	// set color for marked vertex
	colors[index*3+0] = 1.0;
	colors[index*3+1] = 1.0;
	colors[index*3+2] = .0;


	// set marked vertex to map
	tempVerticesMap->insert(std::map<int, bool>::value_type(index, true));
	tempVertices->push_back(index);
}

/*
 * mark a set of vertices with indices indices
 */
void Model::markVertices(std::vector<int>* indices)
{

	std::vector<int>* tempVertices;
	std::map<int,bool>* tempVerticesMap;

	tempVertices = markedVertices;
	tempVerticesMap = markedVerticesMap;

	// if amount of new marked vertices is zero then return
	if(indices->size()<1)
	{
		return;
	}

	for(std::vector<int>::iterator it = indices->begin(); it != indices->end(); ++it) {

		// test whether index is valid
		if(*it<0 || this->numVertices<=*it)
		{
			continue;
		}
		if(tempVerticesMap->count(*it)>0)
		{
			continue;
		}

		if(markedVerticesMap->count(*it)>0)
		{
			continue;
		}

		// set color for marked vertex
		colors[*it*3+0] = 1.0;
		colors[*it*3+1] = 1.0;
		colors[*it*3+2] = .0;

		// set marked vertex to map
		tempVerticesMap->insert(std::map<int, bool>::value_type(*it, true));
		tempVertices->push_back(*it);
	}
}

/*
 * unmark all vertices
 */
void Model::unmarkAllVertices()
{

	std::vector<int>* tempVertices;
	std::map<int,bool>* tempVerticesMap;

	tempVertices = markedVertices;
	tempVerticesMap = markedVerticesMap;

	// for all marked vertices
	for(std::vector<int>::iterator it = tempVertices->begin(); it != tempVertices->end(); ++it) {

		// test whether index is valid
		if(*it<0 || this->numVertices<=*it)
		{
			continue;
		}
		
		// reset color
		colors[*it*3+0] = .0;
		colors[*it*3+1] = .0;
		colors[*it*3+2] = 1.0;

	}

	// clear vectors
	tempVertices->clear();
	tempVerticesMap->clear();
}

/*
 * grab marked vertices by adding vec to all marked vertices
 */
void Model::grabMarkedVertices(glm::vec4* vec)
{

	// for all marked vertices
	for(std::vector<int>::iterator it = markedVertices->begin(); it != markedVertices->end(); ++it) {

		if(*it<0 || this->numVertices<=*it)
		{
			continue;
		}

		// add vector vec to vertices
		vertices[*it*3+0] += vec->data[0];
		vertices[*it*3+1] += vec->data[1];
		vertices[*it*3+2] += vec->data[2];

	}

}

/*
 * set color for marked vertices (grap mode)
 */
void Model::setMarkedVerticesForGrabMode()
{

	// for all marked vertices
	for(std::vector<int>::iterator it = markedVertices->begin(); it != markedVertices->end(); ++it) {

		if(*it<0 || this->numVertices<=*it)
		{
			continue;
		}

		colors[*it*3+0] = 1.0;
		colors[*it*3+1] = .5;
		colors[*it*3+2] = .0;

	}

}

/*
 * set color for marked vertices (default mode)
 */
void Model::setMarkedVerticesForDefaultMode()
{

	// for all marked vertices
	for(std::vector<int>::iterator it = markedVertices->begin(); it != markedVertices->end(); ++it) {

		// for all marked vertices
		if(*it<0 || this->numVertices<=*it)
		{
			continue;
		}

		colors[*it*3+0] = 1.0;
		colors[*it*3+1] = 1.0;
		colors[*it*3+2] = .0;

	}

}
