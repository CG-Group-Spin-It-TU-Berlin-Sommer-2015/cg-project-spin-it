#ifndef BETAOPTIMIZATION2_H
#define BETAOPTIMIZATION2_H

#include <stdio.h>
#include <eigen3/Eigen/Dense>

#include "view/utility/mesh.h"
#include "optimization/octree/extendedoctree.h"

#include <math.h>
#include <nlopt.h>

#define PI 3.14159265359

#define OPTIMIZATION_TYPE_YOYO 0
#define OPTIMIZATION_TYPE_TOP 1

typedef struct {
    int index;
} spin_it_constraint_data;

class BetaOptimization2
{

public:

    static Mesh* mesh;
    static Mesh* shellMesh;
    static ExtendedOctree octree;

public:

    static void testSimpleSplitAndMerge();
    static void testSplitAndMerge();

    static void initializeOctree(
            Mesh* newModifiedMesh,
            GLint startDepth,
            GLint maximumDepth,
            GLint shellExtensionValue);

    static void optimizeBetas(int optimizationType);
    static void optimizeBetasWithSplitAndMerge(int optimizationType);
    static void optimizeBetasBottomUp(GLint optimizationType);

private:

    static float mesh_volume[];
    static Eigen::VectorXd S_comp;
    static Eigen::MatrixXd S_mat;

private:


    static void optimizeBetasForYoyo(QVector<octree::cubeObject>* cubeVector);
    static void optimizeBetasForTop(QVector<octree::cubeObject>* cubeVector);

    static float* calculateVolume(Mesh* mesh, float p = 1.f);

    static void  setSForCompleteMesh();
    static void  setSMatrixForCubes(QVector<octree::cubeObject>* cubeVector);

    static void finishBetaOptimization();

    static double spinItContraints(unsigned n, const double *x, double *grad, void *data);
    static double spinItEnergyFunctionForYoyo(unsigned n, const double *x, double *grad, void *my_func_data);
    static double spinItEnergyFunctionForTop(unsigned n, const double *x, double *grad, void *my_func_data);

};

#endif // BETAOPTIMIZATION2_H
