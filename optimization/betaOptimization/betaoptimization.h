#ifndef BETAOPTIMIZATION2_H
#define BETAOPTIMIZATION2_H

#include <stdio.h>
#include <eigen3/Eigen/Dense>

#include "mesh/mesh.h"
#include "optimization/octree/extendedoctree.h"

#include <math.h>
#include <nlopt.h>

#define PI 3.14159265359


#define OPTIMIZATION_TYPE_TOP 0
#define OPTIMIZATION_TYPE_TIPPE_TOP 1
#define OPTIMIZATION_TYPE_YOYO 2


typedef struct {
    int index;
} spin_it_constraint_data;

class BetaOptimization
{

public:

    static Mesh* mesh;
    static Mesh* shellMesh;
    static ExtendedOctree octree;

    static double gamma_c_top;
    static double gamma_i_top;
    static double gamma_l_top;

    static double gamma_i_tippe_top;
    static double gamma_l_tippe_top;

    static double gamma_i_yoyo;
    static double gamma_l_yoyo;

    static GLfloat align_phi;

    static QVector3D com1;
    static QVector3D com2;

public:

    static void doTopOptimization();
    static void doTippeTopOptimization();
    static void doYoyoOptimization();

    static Mesh* getShellMesh(bool flipped = false);

    static void initializeOctree(
            Mesh* newModifiedMesh,
            GLint startDepth,
            GLint maximumDepth,
            GLint shellExtensionValue);

private:

    static SpMat L;

    static float mesh_volume[];
    static Eigen::VectorXd S_comp;
    static Eigen::MatrixXd S_mat;

    static Eigen::VectorXd S_inner_comp;

private:

    static GLfloat getEpsilon(GLint depth);

    static void showProperties(
            Eigen::VectorXd S_comp,
            Eigen::MatrixXd S_mat,
            QVector<octree::cubeObject>* cubeVector,
            double phi,
            bool useMinMaxValues=false);

    static GLfloat phi;

    static void executeBetasOptimization(QVector<octree::cubeObject>* cubeVector,GLint optimizationType);

    static float* calculateVolume(Mesh* mesh, float p = 1.f);

    static void  setSForCompleteMesh();
    static void  setSMatrixForCubes(QVector<octree::cubeObject>* cubeVector);

    static void finishBetaOptimization();

    static double spinItContraints(unsigned n, const double *x, double *grad, void *data);
    static double spinItEnergyFunction(unsigned n, const double *x, double *grad, void *my_func_data);

    static void setCheckMatrixForCubes();

    static void optimizeBetas(int optimizationType);
    static void optimizeBetasWithSplitAndMerge(int optimizationType);
    static void optimizeBetasBottomUp(GLint optimizationType);

    static void testSimpleSplitAndMerge();
    static void testSplitAndMerge();

    static void resetPhi();

    static double executePhiOptimization();
    static double phiEnergyFunction(unsigned n, const double *x, double *grad, void *my_func_data);


};

#endif // BETAOPTIMIZATION2_H
