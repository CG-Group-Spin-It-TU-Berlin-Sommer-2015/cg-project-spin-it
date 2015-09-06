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
    static float* mesh_volume;

public:

    static Mesh* mesh;
    static Mesh* shellMesh;
    static ExtendedOctree* octree;

    static QVector<int>* J;

public:
    static void initialize(Mesh *mesh);
    static void testSplitAndMerge();
    static void hollow();

    static void initializeOctree(
            Mesh* newModifiedMesh,
            GLint startDepth,
            GLint maximumDepth,
            GLint shellExtensionValue);

private:
    static float* calculateVolume(Mesh* mesh, float p);
    static MatrixXd eqMatrix(VectorXd b, MatrixXd S);
    static VectorXd eqVector(VectorXd b, MatrixXd S);
    static MatrixXd ineqMatrix(VectorXd b);
    static VectorXd ineqVector(VectorXd b);
    static VectorXd gradient(VectorXd b, MatrixXd S);
    static VectorXd optimize(VectorXd b, MatrixXd S);
    static double armijo(VectorXd b, MatrixXd S, VectorXd d, VectorXd beta, VectorXd gamma);
    static double powell(VectorXd b, MatrixXd S, VectorXd d, double d1, double d2, VectorXd beta, VectorXd gamma);
    static double powellG1(double sigma, VectorXd b, MatrixXd S, VectorXd d, VectorXd beta, VectorXd gamma);
    static double powellG2(double sigma, VectorXd b, MatrixXd S, VectorXd d, VectorXd beta, VectorXd gamma);
    static double penalty(VectorXd b, MatrixXd S, VectorXd beta, VectorXd gamma);
    static double penaltyDirectionalGradient(VectorXd b, MatrixXd S, VectorXd d, VectorXd beta, VectorXd gamma);
    static VectorXd test(VectorXd x);
    static double penaltyTest(VectorXd x, VectorXd beta, VectorXd gamma);
    static double penaltyTestDirectionalGradient(VectorXd x, VectorXd d, VectorXd beta, VectorXd gamma);
};

#endif // MODEL_H
