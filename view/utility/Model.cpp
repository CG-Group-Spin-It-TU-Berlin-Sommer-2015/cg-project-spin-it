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

    /*

	// enable calcuation of face and vertex normals (so far only vertex normals are available)
	mesh.request_face_normals();
	mesh.request_vertex_normals();

	// read mesh with openMesh
	if ( !OpenMesh::IO::read_mesh(mesh, path) )
	{
		meshLoaded = false;
		return;
	}

	mesh.update_normals();

	// get amount of indices and vertices
	numIndices = mesh.n_faces();
	numVertices = mesh.n_vertices();

	// initiate arrays and vectors
	indices = new int[numIndices*3];
	vertices = new float[numVertices*3];
	normals = new float[numVertices*3];

	markedVertices = new std::vector<int>();
	markedVertices->reserve(numVertices);
	markedVerticesMap = new std::map<int,bool>();

	// write indices from openMesh mesh into array
	for (MyMesh::FaceIter f_it=mesh.faces_begin(); f_it!=mesh.faces_end(); ++f_it)
	{

		MyMesh::FaceVertexIter fv_it = mesh.fv_iter(f_it);		

		int counter = 0;
		for(; fv_it; ++fv_it) {
			
			indices[f_it->idx()*3+counter] = fv_it->idx();
			counter++;

		}

	}

	float tempMaxCoor = 0;

	// write vertices from openMesh mesh into array
	for (MyMesh::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it)
    {

		// get openMesh point
		Vec3f point = mesh.point(v_it);
		vertices[v_it->idx()*3+0] = point[0];
		vertices[v_it->idx()*3+1] = point[1];
		vertices[v_it->idx()*3+2] = point[2];

		// calcuate absolute max value according to x, y and z
		if(abs(point[0])>tempMaxCoor)
		{
			tempMaxCoor = abs(point[0]);
		}
		if(abs(point[1])>tempMaxCoor)
		{
			tempMaxCoor = abs(point[1]);
		}
		if(abs(point[2])>tempMaxCoor)
		{
			tempMaxCoor = abs(point[2]);
		}
		
		// calculate average
		Vec3f vec(0,0,0);
		float counter = 0;
	    for (MyMesh::VertexVertexIter vv_it=mesh.vv_iter( v_it ); vv_it; ++vv_it)
        {
			vec += mesh.normal( vv_it );
			counter++;
		}
		vec /=counter;

		// update normals(array)
		normals[v_it->idx()*3+0] = vec[0];
		normals[v_it->idx()*3+1] = vec[1];
		normals[v_it->idx()*3+2] = vec[2];
    }

	// set max absolute value
	if(tempMaxCoor>0)
	{
		maxCoor = tempMaxCoor;
	}

	// calculate max amount of connected vertices 
	maxConnectedVertices = 0;
	for (MyMesh::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it)
    {

		int counter = 0;
		for (MyMesh::VertexVertexIter vv_it=mesh.vv_iter( v_it ); vv_it; ++vv_it)
		{
			counter++;
		}

		if(counter>maxConnectedVertices)
		{
			maxConnectedVertices = counter;
		}
	}

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

    */

}

/*
 * update vertices by using the array newPoints
 */
void Model::updateVertices(float* newPoints)
{

    /*
	// update all vertices
	for (MyMesh::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it)
    {

		// update vertices(array)
		vertices[v_it->idx()*3+0] = newPoints[v_it->idx()*3+0];
		vertices[v_it->idx()*3+1] = newPoints[v_it->idx()*3+1];
		vertices[v_it->idx()*3+2] = newPoints[v_it->idx()*3+2];

		// update vertices(openMesh)
		Vec3f point;
		point[0] = newPoints[v_it->idx()*3+0];
		point[1] = newPoints[v_it->idx()*3+1];
		point[2] = newPoints[v_it->idx()*3+2];
		mesh.set_point(v_it,point);

    }

	// update vertex normals of openMesh mesh
	mesh.update_normals();

	float counter;

	// update face normals by using openMesh vertex normals
	for (MyMesh::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it)
    {
	
		// calculate average
		Vec3f vec(0,0,0);
		counter = 0;
	    for (MyMesh::VertexVertexIter vv_it=mesh.vv_iter( v_it ); vv_it; ++vv_it)
        {
			vec += mesh.normal( vv_it );
			counter++;
		}
		vec /=counter;

		// update normals(array)
		normals[v_it->idx()*3+0] = vec[0];
		normals[v_it->idx()*3+1] = vec[1];
		normals[v_it->idx()*3+2] = vec[2];
    }
    */

}

/*
 * make vertices consistent by write vertex array data to open mesh and update normals
 */
void Model::makeVerticesConsistent()
{

    /*
	// write vertices from array into openMesh
	for (MyMesh::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it)
    {

		Vec3f point;
		point[0] = vertices[v_it->idx()*3+0];
		point[1] = vertices[v_it->idx()*3+1];
		point[2] = vertices[v_it->idx()*3+2];

		mesh.set_point(v_it,point);

    }

	// update face and vertex normals
	mesh.update_normals();

	// update face normals by using openMesh vertex normals
	for (MyMesh::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it)
    {
	
		// calcuate average
		Vec3f vec(0,0,0);
	    for (MyMesh::VertexVertexIter vv_it=mesh.vv_iter( v_it ); vv_it; ++vv_it)
        {
			vec += mesh.normal( vv_it );
		}
		vec /=3.0;

		// update normals(array)
		normals[v_it->idx()*3+0] = vec[0];
		normals[v_it->idx()*3+1] = vec[1];
		normals[v_it->idx()*3+2] = vec[2];
    }
    */

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
