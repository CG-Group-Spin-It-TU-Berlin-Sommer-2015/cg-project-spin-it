#ifndef MODEL_H
#define MODEL_H

#include <stdio.h>
#include <eigen3/Eigen/Dense>

#include "view/utility/mesh.h"
#include "view/utility/extendedoctree.h"

using namespace Eigen;
using namespace std;

class Model
{
public:
    static float p;
    static QVector3D cp;

    static float w_c;
    static float w_I;

private:
    static Mesh* mesh;
    static float* mesh_volume;

public:

    static Mesh* modifiedMesh;
    static Mesh* shellMesh;
    static ExtendedOctree* octree;

    static void initialize(Mesh *mesh);
    static void hollow();

    static void initializeOctree(
            Mesh* originalMesh,
            GLint startDepth,
            GLint maximumDepth,
            GLint shellExtensionValue,
            QMatrix4x4 modelMatrix);

private:
    static float* calculateVolume(Mesh* mesh, float p);
    static MatrixXf eqMatrix(VectorXf b, MatrixXf S);
    static VectorXf eqVector(VectorXf b, MatrixXf S);
    static MatrixXf ineqMatrix(VectorXf b, QVector<int> J);
    static VectorXf ineqVector(VectorXf b, QVector<int> J);
    static VectorXf gradient(VectorXf b, MatrixXf S);
    static VectorXf optimize(VectorXf b, MatrixXf S);
    static float powell(VectorXf b, MatrixXf S, VectorXf d, float alpha, float delta, float beta);
    static float powellG1(float sigma, VectorXf b, MatrixXf S, VectorXf d, float alpha);
    static float powellG2(float sigma, VectorXf b, MatrixXf S, VectorXf d, float alpha);
    static float penalty(VectorXf b, MatrixXf S, float alpha);
    static float penaltyDirectionalGradient(VectorXf b, MatrixXf S, VectorXf d, float alpha);
};

#endif // MODEL_H
