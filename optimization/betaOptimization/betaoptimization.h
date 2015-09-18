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

class BetaOptimization
{

public:

    static Mesh* mesh;
    static Mesh* shellMesh;
    static ExtendedOctree octree;

public:

    static void doTopOptimization();
    static void doYoyoOptimization();

    static void testSimpleSplitAndMerge();
    static void testSplitAndMerge();

    static void initializeOctree(
            Mesh* newModifiedMesh,
            GLint startDepth,
            GLint maximumDepth,
            GLint shellExtensionValue);

    static void optimizeBetas(int optimizationType, bool withPhi);
    static void optimizeBetasWithSplitAndMerge(int optimizationType, bool withPhi);
    static void optimizeBetasBottomUp(GLint optimizationType, bool withPhi);

    static void resetPhi();

private:

    static double gamma_c_top;
    static double gamma_i_top;
    static double gamma_l_top;

    static double gamma_i_yoyo;
    static double gamma_l_yoyo;

    static SpMat L;

    static float mesh_volume[];
    static Eigen::VectorXd S_comp;
    static Eigen::MatrixXd S_mat;

private:

    static GLfloat getEpsilon(GLint depth);

    static void calcInertiaTensor(Eigen::VectorXd S_comp,Eigen::MatrixXd S_mat,QVector<octree::cubeObject>* cubeVector, double angle);

    static GLfloat phi;

    static void optimizeBetasForYoyo(QVector<octree::cubeObject>* cubeVector);
    static void optimizeBetasForTop(QVector<octree::cubeObject>* cubeVector);

    static void optimizeBetasForYoyoWithAngle(QVector<octree::cubeObject>* cubeVector);
    static void optimizeBetasForTopWithAngle(QVector<octree::cubeObject>* cubeVector);

    static float* calculateVolume(Mesh* mesh, float p = 1.f);

    static void  setSForCompleteMesh();
    static void  setSMatrixForCubes(QVector<octree::cubeObject>* cubeVector);

    static void finishBetaOptimization();

    static double spinItContraints(unsigned n, const double *x, double *grad, void *data);
    static double spinItEnergyFunctionForYoyo(unsigned n, const double *x, double *grad, void *my_func_data);
    static double spinItEnergyFunctionForTop(unsigned n, const double *x, double *grad, void *my_func_data);

    static double spinItContraintsWithAngle(unsigned n, const double *x, double *grad, void *data);
    static double spinItEnergyFunctionForYoyoWithAngle(unsigned n, const double *x, double *grad, void *my_func_data);
    static double spinItEnergyFunctionForTopWithAngle(unsigned n, const double *x, double *grad, void *my_func_data);


};

#endif // BETAOPTIMIZATION2_H
