#include "betaoptimization.h"

using namespace Eigen;
using namespace std;

#define OPT_ALGO 0

#if OPT_ALGO == 0

//Sequential Quadratic Programming(local and derivative-based)
#define USED_ALGO NLOPT_LD_SLSQP

#elif OPT_ALGO == 1

//Augmented Lagrangian algorithm (local and derivative-based)
#define USED_ALGO NLOPT_LD_AUGLAG_EQ

#elif OPT_ALGO == 2

//Augmented Lagrangian algorithm (local)
#define USED_ALGO NLOPT_AUGLAG_EQ

#elif OPT_ALGO == 3

//Constrained Optimization BY Linear Approximations (derivative-free and local)
#define USED_ALGO NLOPT_LN_COBYLA

#else

//Improved Stochastic Ranking Evolution Strategy (global and derivative-based)
#define USED_ALGO NLOPT_GN_ISRES

#endif

#define DEPTH_2_EPSILON_VALUE 1e-6
#define DEPTH_3_EPSILON_VALUE 1e-3
#define DEPTH_4_EPSILON_VALUE 0.8e-1
#define DEPTH_5_EPSILON_VALUE 0.5e-1
#define DEPTH_6_EPSILON_VALUE 0.4e-1
#define DEPTH_7_EPSILON_VALUE 0.3e-1
#define DEPTH_8_EPSILON_VALUE 0.2e-1

#define s_1  S(0)
#define s_x  S(1)
#define s_y  S(2)
#define s_z  S(3)
#define s_xy  S(4)
#define s_yz  S(5)
#define s_xz  S(6)
#define s_x2  S(7)
#define s_y2  S(8)
#define s_z2  S(9)

#define Smat_1(i) BetaOptimization::S_mat(0,i)
#define Smat_x(i) BetaOptimization::S_mat(1,i)
#define Smat_y(i) BetaOptimization::S_mat(2,i)
#define Smat_z(i) BetaOptimization::S_mat(3,i)
#define Smat_xy(i) BetaOptimization::S_mat(4,i)
#define Smat_yz(i) BetaOptimization::S_mat(5,i)
#define Smat_xz(i) BetaOptimization::S_mat(6,i)
#define Smat_x2(i) BetaOptimization::S_mat(7,i)
#define Smat_y2(i) BetaOptimization::S_mat(8,i)
#define Smat_z2(i) BetaOptimization::S_mat(9,i)

#define MAX(v1,v2) (v1>v2?v1:v2);
#define MIN(v1,v2) (v1<v2?v1:v2);

#define GAMMA_C_TOP 5e+4
#define GAMMA_I_TOP 1e+8
#define GAMMA_L_TOP 5e+3

#define GAMMA_I_YOYO 1e+8
#define GAMMA_L_YOYO 1e+4

#define MAX_TIME 120

double BetaOptimization::gamma_c_top = GAMMA_C_TOP/(GAMMA_C_TOP+GAMMA_I_TOP+GAMMA_L_TOP);
double BetaOptimization::gamma_i_top = GAMMA_I_TOP/(GAMMA_C_TOP+GAMMA_I_TOP+GAMMA_L_TOP);
double BetaOptimization::gamma_l_top = GAMMA_L_TOP/(GAMMA_C_TOP+GAMMA_I_TOP+GAMMA_L_TOP);

double BetaOptimization::gamma_i_yoyo = GAMMA_I_YOYO/(GAMMA_I_YOYO+GAMMA_L_YOYO);
double BetaOptimization::gamma_l_yoyo = GAMMA_L_YOYO/(GAMMA_I_YOYO+GAMMA_L_YOYO);

Mesh* BetaOptimization::mesh = NULL;

ExtendedOctree BetaOptimization::octree;

Mesh* BetaOptimization::shellMesh = NULL;

VectorXd BetaOptimization::S_comp;
MatrixXd BetaOptimization::S_mat;
SpMat BetaOptimization::L;

GLfloat BetaOptimization::phi = 0.f;

float BetaOptimization::mesh_volume[10];

void BetaOptimization::doTopOptimization()
{

    BetaOptimization::mesh->swapYZ();

    BetaOptimization::resetPhi();
    BetaOptimization::optimizeBetasBottomUp(OPTIMIZATION_TYPE_TOP,true);
    BetaOptimization::optimizeBetasWithSplitAndMerge(OPTIMIZATION_TYPE_TOP,true);

    //BetaOptimization::optimizeBetas(OPTIMIZATION_TYPE_TOP,true);

    BetaOptimization::mesh->swapYZ();

}

void BetaOptimization::doYoyoOptimization()
{


    BetaOptimization::mesh->swapYZ();

    BetaOptimization::resetPhi();

    //BetaOptimization::optimizeBetasBottomUp(OPTIMIZATION_TYPE_YOYO,true);
    //BetaOptimization::optimizeBetasWithSplitAndMerge(OPTIMIZATION_TYPE_YOYO,true);
    //BetaOptimization::optimizeBetas(OPTIMIZATION_TYPE_TOP,true);

    BetaOptimization::mesh->swapYZ();

}

/**
 * @brief BetaOptimization2::initializeOctree Initialize the octree for the optimation
 * @param newModifiedMesh the mesh for the optimation
 * @param startDepth the start depth
 * @param maximumDepth the maximum depth
 * @param shellExtensionValue the shell extension value
 */
void BetaOptimization::initializeOctree(
        Mesh* newModifiedMesh,
        GLint startDepth,
        GLint maximumDepth,
        GLint shellExtensionValue)
{

    if(BetaOptimization::mesh != NULL)
    {
        delete BetaOptimization::mesh;
    }

    BetaOptimization::mesh = newModifiedMesh;

    // initialize the octree
    // set point cloud
    BetaOptimization::octree.setMesh(mesh);
    BetaOptimization::octree.setStartMaxDepth(startDepth);
    BetaOptimization::octree.setOptimizationMaxDepth(maximumDepth);
    BetaOptimization::octree.quantizeSurface();
    BetaOptimization::octree.setupVectors();

    // calculate outer and inner cells
    BetaOptimization::octree.setupOctree();
    BetaOptimization::octree.setupOuterNodes();
    BetaOptimization::octree.setupInnerNodes();

    BetaOptimization::octree.printNumberOfLeafTypes();

    BetaOptimization::octree.adjustToBasicMaxDepth();

    BetaOptimization::octree.increaseShell(shellExtensionValue);

    BetaOptimization::octree.initiateMergeRoots();

    BetaOptimization::octree.makeDirty();

    // get mesh for inner shell (needed for view)
    BetaOptimization::octree.createInnerSurface();
    Mesh* newShellMesh = BetaOptimization::octree.getShellMesh();

    if(BetaOptimization::shellMesh != NULL)
    {
       delete BetaOptimization::shellMesh;
    }

    BetaOptimization::shellMesh = newShellMesh;

    BetaOptimization::mesh = newModifiedMesh;


}

/**
 * @brief spinItContraints
 * @param n
 * @param x
 * @param grad
 * @param data
 * @return
 */
double BetaOptimization::spinItContraints(unsigned n, const double *x, double *grad, void *data)
{
   spin_it_constraint_data *d = (spin_it_constraint_data *) data;
   int index = d->index;

   VectorXd betas(n);
   GLdouble phi = BetaOptimization::phi;
   for(unsigned int i=0;i<n;i++){
       betas(i) = x[i];
   }
   MatrixXd S = BetaOptimization::S_comp - BetaOptimization::S_mat*betas;

   double cosp = cos(phi);
   double sinp = sin(phi);

   switch(index)
   {
   case 0:

       // gradient for s_x constraint
       if(grad != NULL)
       {
           for(unsigned int i=0;i<n;i++){
               grad[i] = -Smat_x(i);
           }
       }

       return s_x;
   case 1:

       // gradient for s_y constraint
       if(grad != NULL)
       {
          for(unsigned int i=0;i<n;i++){
              grad[i] = -Smat_y(i);
          }
       }

       return s_y;
   case 2:

       // gradient for s_yz constraint
       if(grad != NULL)
       {
          for(unsigned int i=0;i<n;i++){
              grad[i] = -Smat_yz(i);
          }
       }

       return s_yz;
   case 3:

       // gradient for s_xz constraint
       if(grad != NULL)
       {
          for(unsigned int i=0;i<n;i++){
               grad[i] = -Smat_xz(i);
          }
       }

       return s_xz;
   case 4:

       // gradient for s_z constraint
       if(grad != NULL)
       {
          for(unsigned int i=0;i<n;i++){
               grad[i] = -Smat_z(i);
          }
       }

       return s_z;
   case 5:

       // gradient for Givens Rotation constraint
       if(grad != NULL)
       {
           for(unsigned int i=0;i<n;i++){
                grad[i] = 0;
                grad[i] += cosp*sinp*(Smat_y2(i)-Smat_x2(i));
                grad[i] += (pow(cosp,2)-pow(sinp,2))*(-Smat_xy(i));
           }
       }

       return cosp*sinp*(s_x2-s_y2)+(pow(cosp,2)-pow(sinp,2))*s_xy;
   }
   return -1;
}

/**
 * @brief spinItEnergyFunctionForYoyo
 * @param n
 * @param x
 * @param grad
 * @param my_func_data
 * @return
 */
double BetaOptimization::spinItEnergyFunctionForYoyo(unsigned n, const double *x, double *grad, void *my_func_data)
{

    (void)my_func_data;

    VectorXd betas(n);
    GLdouble phi = BetaOptimization::phi;
    for(unsigned int i=0;i<n;i++){
        betas(i) = x[i];
    }

    double cosp = cos(phi);
    double sinp = sin(phi);

    // current model volumes
    MatrixXd S = BetaOptimization::S_comp - BetaOptimization::S_mat*betas;

    // inertia tensor
    MatrixXd I(2,2);
    I(0,0) = s_y2+s_z2; I(0,1) = -s_xy;
    I(1,0) = -s_xy;     I(1,1) = s_x2+s_z2;

    // rotation matrix
    MatrixXd R(2,2);
    R(0,0) = +cosp;R(0,1) = -sinp;
    R(1,0) = +sinp;R(1,1) = +cosp;

    // Givens Rotation representation
    MatrixXd mat = R*I*R.transpose();

    double IX = mat(0,0);
    double IY = mat(1,1);
    double IZ = s_x2+s_y2;

    double gi = gamma_i_yoyo;
    double gl = gamma_l_yoyo;

    double regulator = gl*(betas.transpose()*(L*betas))(0,0);

    double f = gi*(pow(IX/IZ,2)+pow(IY/IZ,2))+regulator;

    //----------------------------------------

    // gradient for yoyo energy function
    if(grad != NULL)
    {
        //----------------------------------------
        for(unsigned int i=0;i<n;i++){

            // gradient depending on beta i
            double dIxb = 0;
            dIxb += pow(cosp,2)*(-Smat_y2(i)-Smat_z2(i));
            dIxb += pow(sinp,2)*(-Smat_x2(i)-Smat_z2(i));
            dIxb -= 2*cosp*sinp*Smat_xy(i);

            double dIyb = 0;
            dIyb += pow(sinp,2)*(-Smat_y2(i)-Smat_z2(i));
            dIyb += pow(cosp,2)*(-Smat_x2(i)-Smat_z2(i));
            dIyb -= 2*cosp*sinp*Smat_xy(i);

            double dIzb = -(Smat_x2(i)+Smat_y2(i));

            double dIxIz = (dIxb*IZ-IX*dIzb)/pow(IZ,2);
            double dIyIz = (dIyb*IZ-IY*dIzb)/pow(IZ,2);

            double dIxIz2 = 2*(IX/IZ)*dIxIz;
            double dIyIz2 = 2*(IY/IZ)*dIyIz;

            double dFirstPart = gi*(dIxIz2+dIyIz2);

            SpMat vec(n,1);
            std::vector<T> coefficients;
            coefficients.push_back(T(i,0,1));
            vec.setFromTriplets(coefficients.begin(), coefficients.end());
            double dThirdPart = gl*2*(betas.transpose()*(L*vec))(0,0);

            grad[i] = +dFirstPart+dThirdPart;
        }
    }
    //----------------------------------------

    return f;
}

/**
 * @brief spinItEnergyFunctionForTop
 * @param n
 * @param x
 * @param grad
 * @param my_func_data
 * @return
 */
double BetaOptimization::spinItEnergyFunctionForTop(unsigned n, const double *x, double *grad, void *my_func_data)
{

    (void)my_func_data;

    VectorXd betas(n);
    GLdouble phi = BetaOptimization::phi;
    for(unsigned int i=0;i<n;i++){
        betas(i) = x[i];
    }

    double cosp = cos(phi);
    double sinp = sin(phi);

    // current model volumes
    MatrixXd S = BetaOptimization::S_comp - BetaOptimization::S_mat*betas;

    // inertia tensor
    MatrixXd I(2,2);
    I(0,0) = s_y2+s_z2; I(0,1) = -s_xy;
    I(1,0) = -s_xy;     I(1,1) = s_x2+s_z2;

    MatrixXd diag110 = MatrixXd::Zero(2,2);
    diag110(0,0) = 1;
    diag110(1,1) = 1;

    // shifted inertia tensor
    MatrixXd I_com = I- (pow(s_z,2)/s_1) * diag110;

    // rotation matrix
    MatrixXd R(2,2);
    R(0,0) = +cosp;R(0,1) = -sinp;
    R(1,0) = +sinp;R(1,1) = +cosp;

    // Givens Rotation representation
    MatrixXd mat = R*I_com*R.transpose();

    double IX = mat(0,0);
    double IY = mat(1,1);
    double IZ = s_x2+s_y2;

    double gc = gamma_c_top;
    double gi = gamma_i_top;
    double gl = gamma_l_top;

    double M = s_1;
    double l = s_z/s_1;

    double regulator = gl*(betas.transpose()*(L*betas))(0,0);

    double f = gc*pow(l*M,2)+gi*(pow(IX/IZ,2)+pow(IY/IZ,2))+regulator;

    //----------------------------------------

    // gradient for top energy function constraint
    if(grad != NULL)
    {
        //----------------------------------------
        for(unsigned int i=0;i<n;i++){

            // gradient depending on beta i

            double d_sz_2_s1 = ((-2)*s_z*Smat_z(i)*s_1-pow(s_z,2)*(-Smat_1(i)))/pow(s_1,2);

            double dIxb = 0;
            dIxb += pow(cosp,2)*(-Smat_y2(i)-Smat_z2(i)-d_sz_2_s1);
            dIxb += pow(sinp,2)*(-Smat_x2(i)-Smat_z2(i)-d_sz_2_s1);
            dIxb -= 2*cosp*sinp*Smat_xy(i);

            double dIyb = 0;
            dIyb += pow(sinp,2)*(-Smat_y2(i)-Smat_z2(i)-d_sz_2_s1);
            dIyb += pow(cosp,2)*(-Smat_x2(i)-Smat_z2(i)-d_sz_2_s1);
            dIyb -= 2*cosp*sinp*Smat_xy(i);

            double dIzb = -(Smat_x2(i)+Smat_y2(i));

            double dIxIz = (dIxb*IZ-IX*dIzb)/pow(IZ,2);
            double dIyIz = (dIyb*IZ-IY*dIzb)/pow(IZ,2);

            double dIxIz2 = 2*(IX/IZ)*dIxIz;
            double dIyIz2 = 2*(IY/IZ)*dIyIz;

            double dFirstPart = -2*gc*s_z*Smat_z(i);
            double dSecondPart = gi*(dIxIz2+dIyIz2);

            SpMat vec(n,1);
            std::vector<T> coefficients;
            coefficients.push_back(T(i,0,1));
            vec.setFromTriplets(coefficients.begin(), coefficients.end());
            double dThirdPart = gl*2*(betas.transpose()*(L*vec))(0,0);

            grad[i] = dFirstPart+dSecondPart+dThirdPart;
        }
    }
    //----------------------------------------

    return f;
}

/**
 * @brief BetaOptimization2::optimizeBetasForYoyo
 * @param cubeVector
 */
void BetaOptimization::optimizeBetasForYoyo(QVector<octree::cubeObject>* cubeVector)
{

    double minf;

    int bs = cubeVector->size();
    //------------------------------

    double* lb_si = new double[bs];
    double* ub_si = new double[bs];
    double* betas = new double[bs];
    for(int i=0;i<bs;i++)
    {
        lb_si[i] = 0.0;
        ub_si[i] = 1.0;
        betas[i] = cubeVector->data()[i].beta;
    }

    //------------------------------
    nlopt_opt opt_spin_it;
    opt_spin_it = nlopt_create( USED_ALGO, bs);

    nlopt_set_lower_bounds(opt_spin_it, lb_si);
    nlopt_set_upper_bounds(opt_spin_it, ub_si);

    nlopt_set_min_objective(opt_spin_it, spinItEnergyFunctionForYoyo, NULL);

    double val1 = 1e-4;
    double val2 = 1e-8;

    nlopt_set_xtol_rel(opt_spin_it, val1);

    spin_it_constraint_data data_si[10] = { {0},{1},{2},{3},{4},{5} };
    nlopt_add_equality_constraint(opt_spin_it, spinItContraints, &data_si[0], val2);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraints, &data_si[1], val2);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraints, &data_si[2], val2);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraints, &data_si[3], val2);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraints, &data_si[4], val2);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraints, &data_si[5], val2);

    nlopt_set_maxtime(opt_spin_it,MAX_TIME);

    if (nlopt_optimize(opt_spin_it, betas, &minf) < 0)
    {
        cout << "optimization failed!" << endl;
    }
    else
    {
        cout << "optimization successful (found minimum " << minf << ")" << endl;
    }
    cout << "----------------------------------------" << endl;


    nlopt_destroy(opt_spin_it);

    delete lb_si;
    delete ub_si;
    delete betas;

    for (int i = 0; i < bs; i++) {
        octree::cubeObject* obj = &cubeVector->data()[i];
        obj->beta = betas[i];
    }

}

void BetaOptimization::calcInertiaTensor(VectorXd S_comp,MatrixXd S_mat,QVector<octree::cubeObject>* cubeVector, double phi)
{

    cout << "Inertia Properties:" << endl;
    cout << "----------" << endl;

    double cosp = cos(phi);
    double sinp = sin(phi);

    VectorXd betas;

    if(cubeVector != NULL){
        betas.resize(cubeVector->size());
        for (int i = 0; i < cubeVector->size(); i++) {
            betas(i) = cubeVector->data()[i].beta;
        }
    }

    // current model volumes
    MatrixXd S = cubeVector != NULL ? S_comp - S_mat*betas : S_comp;

    // inertia tensor
    MatrixXd I(3,3);
    I(0,0) = s_y2+s_z2; I(0,1) = -s_xy;     I(0,2) = -s_xz;
    I(1,0) = -s_xy;     I(1,1) = s_x2+s_z2; I(1,2) = -s_yz;
    I(2,0) = -s_xz;     I(2,1) = -s_yz;     I(2,2) = s_x2+s_y2;

    cout << "Inertia Matrix:" << endl;
    cout << I << endl;
    cout << "----------" << endl;

    // rotation matrix
    MatrixXd R(2,2);
    R(0,0) = +cosp;R(0,1) = -sinp;
    R(1,0) = +sinp;R(1,1) = +cosp;

    MatrixXd diag110 = MatrixXd::Zero(3,3);
    diag110(0,0) = 1;
    diag110(1,1) = 1;

    // shifted inertia tensor
    MatrixXd I_com = I - (pow(s_z,2)/s_1) * diag110;

    cout << "Shifted Inertia Matrix:" << endl;
    cout << I_com << endl;
    cout << "----------" << endl;

    MatrixXd I_2(2,2);
    I_2(0,0) = I_com(0,0);    I_2(0,1) = I_com(0,1);
    I_2(1,0) = I_com(1,0);    I_2(1,1) = I_com(1,1);

    // Givens Rotation representation
    MatrixXd mat = R*I_2*R.transpose();

    cout << "Rotated Component:" << endl;
    cout << mat << endl;
    cout << "----------------------------------------" << endl;

    // center of mass
    VectorXd c(3);
    c(0) = s_x;
    c(1) = s_y;
    c(2) = s_z;
    c =c/s_1;

    cout << "Center of Mass:" << endl;
    cout << c << endl;
    cout << "----------------------------------------" << endl;

    cout << "Mass:" << endl;
    cout << s_1 << endl;
    cout << "----------------------------------------" << endl;

    cout << "phi:" << endl;
    cout << phi << endl;
    cout << "----------------------------------------" << endl;

    if(cubeVector != NULL && cubeVector->size()<=20)
    {
        cout << "L:" << endl;
        cout << L.toDense() << endl;
        cout << "----------------------------------------" << endl;
        cout << "L*betas:" << endl;
        cout << (L*betas) << endl;
        cout << "----------------------------------------" << endl;
        cout << "betas.T*(L*betas):" << endl;
        cout << (betas.transpose()*(L*betas))(0,0) << endl;
        cout << "----------------------------------------" << endl;

    }

}

/**
 * @brief BetaOptimization2::optimizeBetasForTop
 * @param cubeVector
 */
void BetaOptimization::optimizeBetasForTop(QVector<octree::cubeObject>* cubeVector)
{

    double minf;

    int bs = cubeVector->size();
    //------------------------------

    double* lb_si = new double[bs];
    double* ub_si = new double[bs];
    double* betas = new double[bs];
    for(int i=0;i<bs;i++)
    {
        lb_si[i] = 0.0;
        ub_si[i] = 1.0;
        betas[i] = cubeVector->data()[i].beta;
    }

    //------------------------------
    nlopt_opt opt_spin_it;
    opt_spin_it = nlopt_create( USED_ALGO, bs);

    nlopt_set_lower_bounds(opt_spin_it, lb_si);
    nlopt_set_upper_bounds(opt_spin_it, ub_si);

    nlopt_set_min_objective(opt_spin_it, spinItEnergyFunctionForTop, NULL);


    double val1 = 1e-4;
    double val2 = 1e-8;


    nlopt_set_xtol_rel(opt_spin_it, val1);

    spin_it_constraint_data data_si[10] = { {0},{1},{2},{3},{5} };

    nlopt_add_equality_constraint(opt_spin_it, spinItContraints, &data_si[0], val2);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraints, &data_si[1], val2);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraints, &data_si[2], val2);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraints, &data_si[3], val2);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraints, &data_si[4], val2);

    nlopt_set_maxtime(opt_spin_it,MAX_TIME);

    if (nlopt_optimize(opt_spin_it, betas, &minf) < 0)
    {
        cout << "optimization failed!" << endl;
    }
    else
    {
        cout << "optimization successful (found minimum " << minf << ")" << endl;
    }
    cout << "----------------------------------------" << endl;


    nlopt_destroy(opt_spin_it);

    delete lb_si;
    delete ub_si;
    delete betas;

    cout << endl;

    for (int i = 0; i < bs; i++) {
        octree::cubeObject* obj = &cubeVector->data()[i];
        obj->beta = betas[i];
    }

    cout << endl;

}

/**
 * @brief BetaOptimization2::setSForCompleteMesh
 */
void  BetaOptimization::setSForCompleteMesh()
{
    BetaOptimization::S_comp.resize(10);

    float* s = calculateVolume(BetaOptimization::mesh);
    for (int i = 0; i < 10; i++) {
        BetaOptimization::S_comp(i) = s[i];
    }

}

/**
 * @brief BetaOptimization2::setSMatrixForCubes
 * @param cubeVector
 */
void  BetaOptimization::setSMatrixForCubes(QVector<octree::cubeObject>* cubeVector)
{

    BetaOptimization::S_mat.resize(10,cubeVector->size());

    for (int i = 0; i < cubeVector->size(); i++) {
        float* s = calculateVolume(cubeVector->at(i).mesh);
        for (int j = 0; j < 10; j++) {
            BetaOptimization::S_mat(j,i) = s[j];
        }
    }

}

GLfloat BetaOptimization::getEpsilon(GLint depth)
{

    switch(depth){
    case 2: return DEPTH_2_EPSILON_VALUE;
    case 3: return DEPTH_3_EPSILON_VALUE;
    case 4: return DEPTH_4_EPSILON_VALUE;
    case 5: return DEPTH_5_EPSILON_VALUE;
    case 6: return DEPTH_6_EPSILON_VALUE;
    case 7: return DEPTH_7_EPSILON_VALUE;
    default: return DEPTH_8_EPSILON_VALUE;
    }

    return 0.5;
}

/**
 * @brief BetaOptimization::optimizeBetasBottomUp
 * @param optimizationType
 * @param withPhi
 */
void BetaOptimization::optimizeBetasBottomUp(GLint optimizationType, bool withPhi)
{

    cout << "Beta Optimization Bottom Up" << endl;
    cout << "----------------------------------------" << endl;

    BetaOptimization::setSForCompleteMesh();

    calcInertiaTensor(S_comp,S_mat,NULL,BetaOptimization::phi);

    GLint depth = BetaOptimization::octree.getOptimizationMaxDepth();

    for (int i = 3; i < depth; i++) {

        QVector<octree::cubeObject>* cubeVector = BetaOptimization::octree.getCubesOfLowerDepth(i);

        cout << "number of cubes is " << cubeVector->size() << endl;

        BetaOptimization::setSMatrixForCubes(cubeVector);
        BetaOptimization::L = BetaOptimization::octree.getUniformLaplace(cubeVector);

        calcInertiaTensor(S_comp,S_mat,cubeVector,BetaOptimization::phi);

        switch(optimizationType)
        {
            case OPTIMIZATION_TYPE_YOYO:

            if(withPhi || i==3)
            {
                BetaOptimization::optimizeBetasForYoyoWithAngle(cubeVector);
            }
            else
            {
                BetaOptimization::optimizeBetasForYoyo(cubeVector);
            }

            break;
            case OPTIMIZATION_TYPE_TOP:

            if(withPhi || i==3)
            {
                BetaOptimization::optimizeBetasForTopWithAngle(cubeVector);
            }
            else
            {
                BetaOptimization::optimizeBetasForTop(cubeVector);
            }

            break;

        }

        calcInertiaTensor(S_comp,S_mat,cubeVector,BetaOptimization::phi);

        BetaOptimization::octree.updateBetaValuesWithPropagation();

        //BetaOptimization::octree.splitAndMerge(BetaOptimization::getEpsilon(i));

    }

    BetaOptimization::finishBetaOptimization();

}

/**
 * @brief BetaOptimization::optimizeBetas
 * @param optimizationType
 * @param withPhi
 */
void BetaOptimization::optimizeBetas(int optimizationType, bool withPhi)
{

    cout << "Beta Optimization" << endl;
    cout << "----------------------------------------" << endl;

    QVector<octree::cubeObject>* cubeVector = BetaOptimization::octree.getInnerCubes();

    cout << "number of cubes is " << cubeVector->size() << endl;
    cout << "----------------------------------------" << endl;

    BetaOptimization::setSForCompleteMesh();
    BetaOptimization::setSMatrixForCubes(cubeVector);
    BetaOptimization::L = BetaOptimization::octree.getUniformLaplace(cubeVector);

    calcInertiaTensor(S_comp,S_mat,cubeVector,BetaOptimization::phi);

    switch(optimizationType)
    {
        case OPTIMIZATION_TYPE_YOYO:

        if(withPhi)
        {
            BetaOptimization::optimizeBetasForYoyoWithAngle(cubeVector);
        }
        else
        {
            BetaOptimization::optimizeBetasForYoyo(cubeVector);
        }

        break;
        case OPTIMIZATION_TYPE_TOP:

        if(withPhi)
        {
            BetaOptimization::optimizeBetasForTopWithAngle(cubeVector);
        }
        else
        {
            BetaOptimization::optimizeBetasForTop(cubeVector);
        }

        break;

    }

    calcInertiaTensor(S_comp,S_mat,cubeVector,BetaOptimization::phi);

    BetaOptimization::octree.updateBetaValues();

    BetaOptimization::finishBetaOptimization();

}

/**
 * @brief BetaOptimization::optimizeBetasWithSplitAndMerge
 * @param optimizationType
 * @param withPhi
 */
void BetaOptimization::optimizeBetasWithSplitAndMerge(int optimizationType, bool withPhi)
{

    BetaOptimization::setSForCompleteMesh();

    bool notConverged =  true;

    cout << "Beta Optimization with Split and Merge Step" << endl;
    cout << "----------------------------------------" << endl;

    GLint counter=1;

    while(notConverged)
    {

        cout << "Split and Merge Step (" << counter << ")" << endl;
        cout << "----------------------------------------" << endl;
        counter++;

        QVector<octree::cubeObject>* cubeVector = BetaOptimization::octree.getInnerCubes();
        cout << "number of cubes is " << cubeVector->size() << endl;

        BetaOptimization::setSMatrixForCubes(cubeVector);
        BetaOptimization::L = BetaOptimization::octree.getUniformLaplace(cubeVector);

        calcInertiaTensor(S_comp,S_mat,cubeVector,BetaOptimization::phi);

        switch(optimizationType)
        {
            case OPTIMIZATION_TYPE_YOYO:

            if(withPhi)
            {
                BetaOptimization::optimizeBetasForYoyoWithAngle(cubeVector);
            }
            else
            {
                BetaOptimization::optimizeBetasForYoyo(cubeVector);
            }

            break;
            case OPTIMIZATION_TYPE_TOP:

            if(withPhi)
            {
                BetaOptimization::optimizeBetasForTopWithAngle(cubeVector);
            }
            else
            {
                BetaOptimization::optimizeBetasForTop(cubeVector);
            }

            break;

        }

        calcInertiaTensor(S_comp,S_mat,cubeVector,BetaOptimization::phi);

        BetaOptimization::octree.updateBetaValues();

        notConverged = BetaOptimization::octree.splitAndMerge(BetaOptimization::getEpsilon(BetaOptimization::octree.getOptimizationMaxDepth()));
    }

    BetaOptimization::finishBetaOptimization();

}

/**
 * @brief BetaOptimization2::testSimpleSplitAndMerge
 */
void BetaOptimization::testSimpleSplitAndMerge()
{

    QVector<octree::cubeObject>* cubeVector = NULL;

    cubeVector = BetaOptimization::octree.getCubesOfLowerDepth(3);

    for (int i = 0; i < cubeVector->size(); i++) {
        octree::cubeObject* obj = &cubeVector->data()[i];

        QVector<GLfloat>* vec = obj->mesh->getGeometry();

        bool above = false;
        bool under = false;

        for(int i=0;i<vec->size();i+=3)
        {
            //GLfloat x = vec->data()[i+0];
            GLfloat y = vec->data()[i+1];
            GLfloat z = vec->data()[i+2];

            if(z<3.5+y*2)
            {
                above = true;
            }
            else
            {
                under = true;
            }

        }

        obj->beta = above && !under ? 1.0 : (under && !above? 0.0 : 0.5);
    }

    BetaOptimization::octree.updateBetaValuesWithPropagation();

    BetaOptimization::finishBetaOptimization();

}

/**
 * @brief BetaOptimization2::testSplitAndMerge
 */
void BetaOptimization::testSplitAndMerge()
{

    QVector<octree::cubeObject>* cubeVector = NULL;

    bool not_converged = true;
    while (not_converged)
    {

        // get the inner cubes of the octree
        cubeVector = BetaOptimization::octree.getInnerCubes();

        for (int i = 0; i < cubeVector->size(); i++) {
            octree::cubeObject* obj = &cubeVector->data()[i];

            QVector<GLfloat>* vec = obj->mesh->getGeometry();

            bool above = false;
            bool under = false;

            for(int i=0;i<vec->size();i+=3)
            {
                //GLfloat x = vec->data()[i+0];
                GLfloat y = vec->data()[i+1];
                GLfloat z = vec->data()[i+2];

                if(z<3.5+y*2)
                {
                    above = true;
                }
                else
                {
                    under = true;
                }

            }

            obj->beta = above && !under ? 1.0 : (under && !above? 0.0 : 0.5);
        }

        // set new betas and clrea cubeVector
        BetaOptimization::octree.updateBetaValues();

        // do split and merge
        not_converged = BetaOptimization::octree.splitAndMerge(0.0001);
    }

    BetaOptimization::finishBetaOptimization();
}

/**
 * @brief BetaOptimization2::finishBetaOptimization2
 */
void BetaOptimization::finishBetaOptimization()
{

    BetaOptimization::octree.deleteNodeMeshes();

    // set each cube of the octree either to void (beta>0.5) or not void (beta<=0.5)
    BetaOptimization::octree.setVoids();

    BetaOptimization::octree.createInnerSurface();
    Mesh* newShellMesh = BetaOptimization::octree.getShellMesh();

    if(shellMesh != NULL)
    {
       delete shellMesh;
    }

    shellMesh = newShellMesh;

    BetaOptimization::octree.makeDirty();

}

/**
 * @brief BetaOptimization2::calculateVolume calculates volumne integrals of an object
 * @param mesh triangulated surface of object
 * @param p density of object (default 1)
 * @return
 */
float* BetaOptimization::calculateVolume(Mesh* mesh, float p)
{
    for (int i = 0; i < 10; i++) {
        BetaOptimization::mesh_volume[i] = 0;
    }
    QVector<float>* geometry = mesh->getGeometry();
    QVector<float>* normals = mesh->getSurfaceNormals();
    QVector<int>* indices = mesh->getIndices();

    for (int i = 0; i < indices->size(); i += 3) {
        QVector3D a;
        a.setX(geometry->at(3 * indices->at(i)));
        a.setY(geometry->at(3 * indices->at(i) + 1));
        a.setZ(geometry->at(3 * indices->at(i) + 2));

        QVector3D b;
        b.setX(geometry->at(3 * indices->at(i + 1)));
        b.setY(geometry->at(3 * indices->at(i + 1) + 1));
        b.setZ(geometry->at(3 * indices->at(i + 1) + 2));

        QVector3D c;
        c.setX(geometry->at(3 * indices->at(i + 2)));
        c.setY(geometry->at(3 * indices->at(i + 2) + 1));
        c.setZ(geometry->at(3 * indices->at(i + 2) + 2));

        QVector3D n;
        n.setX(normals->at(i));
        n.setY(normals->at(i + 1));
        n.setZ(normals->at(i + 2));

        QVector3D h1 = a + b + c;
        QVector3D h2 = a*a + b*(a + b);
        QVector3D h3 = h2 + c*h1;
        QVector3D h4 = a*a*a + b*h2 + c*h3;
        QVector3D h5 = h3 + a*(h1 + a);
        QVector3D h6 = h3 + b*(h1 + b);
        QVector3D h7 = h3 + c*(h1 + c);
        QVector3D h8 = QVector3D(a.y(),a.z(),a.x())*h5 + QVector3D(b.y(),b.z(),b.x())*h6 + QVector3D(c.y(),c.z(),c.x())*h7;

        QVector3D si;
        BetaOptimization::mesh_volume[0] += (n*h1).x();
        si = n*h3;
        BetaOptimization::mesh_volume[1] += si.x();
        BetaOptimization::mesh_volume[2] += si.y();
        BetaOptimization::mesh_volume[3] += si.z();
        si = n*h8;
        BetaOptimization::mesh_volume[4] += si.x();
        BetaOptimization::mesh_volume[5] += si.y();
        BetaOptimization::mesh_volume[6] += si.z();
        si = n*h4;
        BetaOptimization::mesh_volume[7] += si.x();
        BetaOptimization::mesh_volume[8] += si.y();
        BetaOptimization::mesh_volume[9] += si.z();
    }

    BetaOptimization::mesh_volume[0] *= (float) 1/6;
    for (int i = 1; i <= 3; i++) {
        BetaOptimization::mesh_volume[i] *= (float) 1/24;
    }
    for (int i = 4; i <= 6; i++) {
        BetaOptimization::mesh_volume[i] *= (float) 1/120;
    }
    for (int i = 7; i <= 9; i++) {
        BetaOptimization::mesh_volume[i] *= (float) 1/60;
    }

    for (int i = 0; i < 10; i++) {
        BetaOptimization::mesh_volume[i] *= p;
    }

    return BetaOptimization::mesh_volume;
}

/**
 * @brief spinItContraintsWithAngle
 * @param n
 * @param x
 * @param grad
 * @param data
 * @return
 */
double BetaOptimization::spinItContraintsWithAngle(unsigned n, const double *x, double *grad, void *data)
{
   spin_it_constraint_data *d = (spin_it_constraint_data *) data;
   int index = d->index;

   VectorXd betas(n-1);
   GLdouble phi = x[0];
   for(unsigned int i=1;i<n;i++){
       betas(i-1) = x[i];
   }
   MatrixXd S = BetaOptimization::S_comp - BetaOptimization::S_mat*betas;

   double cosp = cos(phi);
   double sinp = sin(phi);

   switch(index)
   {
   case 0:

       // gradient for s_x constraint
       if(grad != NULL)
       {
           grad[0] = 0;
           for(unsigned int i=0;i<n-1;i++){
               grad[i+1] = -Smat_x(i);
           }
       }

       return s_x;
   case 1:

       // gradient for s_y constraint
       if(grad != NULL)
       {
          grad[0] = 0;
          for(unsigned int i=0;i<n-1;i++){
              grad[i+1] = -Smat_y(i);
          }
       }

       return s_y;
   case 2:

       // gradient for s_yz constraint
       if(grad != NULL)
       {
          grad[0] = 0;
          for(unsigned int i=0;i<n-1;i++){
              grad[i+1] = -Smat_yz(i);
          }
       }

       return s_yz;
   case 3:

       // gradient for s_xz constraint
       if(grad != NULL)
       {
          grad[0] = 0;
          for(unsigned int i=0;i<n-1;i++){
               grad[i+1] = -Smat_xz(i);
          }
       }

       return s_xz;
   case 4:

       // gradient for s_z constraint
       if(grad != NULL)
       {
          grad[0] = 0;
          for(unsigned int i=0;i<n-1;i++){
               grad[i+1] = -Smat_z(i);
          }
       }

       return s_z;
   case 5:

       // gradient for Givens Rotation constraint
       if(grad != NULL)
       {
           grad[0] = 0;
           grad[0] += ( pow(cosp,2) - pow(sinp,2) )*(s_x2-s_y2);
           grad[0] += (-4*sinp*cosp)*s_xy;

           for(unsigned int i=0;i<n-1;i++){
                grad[i+1] = 0;
                grad[i+1] += cosp*sinp*(Smat_y2(i)-Smat_x2(i));
                grad[i+1] += (pow(cosp,2)-pow(sinp,2))*(-Smat_xy(i));
           }
       }

       return cosp*sinp*(s_x2-s_y2)+(pow(cosp,2)-pow(sinp,2))*s_xy;
   }
   return -1;
}

/**
 * @brief spinItEnergyFunctionForYoyoWithAngle
 * @param n
 * @param x
 * @param grad
 * @param my_func_data
 * @return
 */
double BetaOptimization::spinItEnergyFunctionForYoyoWithAngle(unsigned n, const double *x, double *grad, void *my_func_data)
{

    (void)my_func_data;

    VectorXd betas(n-1);
    GLdouble phi = x[0];
    for(unsigned int i=1;i<n;i++){
        betas(i-1) = x[i];
    }

    double cosp = cos(phi);
    double sinp = sin(phi);

    // current model volumes
    MatrixXd S = BetaOptimization::S_comp - BetaOptimization::S_mat*betas;

    // inertia tensor
    MatrixXd I(2,2);
    I(0,0) = s_y2+s_z2; I(0,1) = -s_xy;
    I(1,0) = -s_xy;     I(1,1) = s_x2+s_z2;

    // rotation matrix
    MatrixXd R(2,2);
    R(0,0) = +cosp;R(0,1) = -sinp;
    R(1,0) = +sinp;R(1,1) = +cosp;

    // Givens Rotation representation
    MatrixXd mat = R*I*R.transpose();

    double IX = mat(0,0);
    double IY = mat(1,1);
    double IZ = s_x2+s_y2;

    double gi = gamma_i_yoyo;
    double gl = gamma_l_yoyo;

    double regulator = gl*(betas.transpose()*(L*betas))(0,0);

    double f = gi*(pow(IX/IZ,2)+pow(IY/IZ,2))+regulator;

    //----------------------------------------

    // gradient for yoyo energy function
    if(grad != NULL)
    {

        // gradient depending on phi

        double dIxdphi = 0;
        dIxdphi += (-2*sinp*cosp)*(s_y2+s_z2);
        dIxdphi += (+2*sinp*cosp)*(s_x2+s_z2);
        dIxdphi += -2*(cosp-sinp)*(-s_xy);
        double dIxdphi2 = 2*IX*dIxdphi;

        double dIydphi = 0;
        dIydphi += (+2*sinp*cosp)*(s_y2+s_z2);
        dIydphi += (-2*sinp*cosp)*(s_x2+s_z2);
        dIydphi += +2*(cosp-sinp)*(-s_xy);
        double dIydphi2 = 2*IY*dIydphi;

        grad[0] = (gi/pow(IZ,2))*(dIxdphi2+dIydphi2);

        //----------------------------------------
        for(unsigned int i=0;i<n-1;i++){

            // gradient depending on beta i
            double dIxb = 0;
            dIxb += pow(cosp,2)*(-Smat_y2(i)-Smat_z2(i));
            dIxb += pow(sinp,2)*(-Smat_x2(i)-Smat_z2(i));
            dIxb -= 2*cosp*sinp*Smat_xy(i);

            double dIyb = 0;
            dIyb += pow(sinp,2)*(-Smat_y2(i)-Smat_z2(i));
            dIyb += pow(cosp,2)*(-Smat_x2(i)-Smat_z2(i));
            dIyb -= 2*cosp*sinp*Smat_xy(i);

            double dIzb = -(Smat_x2(i)+Smat_y2(i));

            double dIxIz = (dIxb*IZ-IX*dIzb)/pow(IZ,2);
            double dIyIz = (dIyb*IZ-IY*dIzb)/pow(IZ,2);

            double dIxIz2 = 2*(IX/IZ)*dIxIz;
            double dIyIz2 = 2*(IY/IZ)*dIyIz;

            double dFirstPart = gi*(dIxIz2+dIyIz2);

            SpMat vec(betas.size(),1);
            std::vector<T> coefficients;
            coefficients.push_back(T(i,0,1));
            vec.setFromTriplets(coefficients.begin(), coefficients.end());
            double dThirdPart = gl*2*(betas.transpose()*(L*vec))(0,0);


            grad[i+1] = +dFirstPart+dThirdPart;
        }
    }
    //----------------------------------------

    return f;
}

/**
 * @brief spinItEnergyFunctionForTopWithAngle
 * @param n
 * @param x
 * @param grad
 * @param my_func_data
 * @return
 */
double BetaOptimization::spinItEnergyFunctionForTopWithAngle(unsigned n, const double *x, double *grad, void *my_func_data)
{

    (void)my_func_data;

    VectorXd betas(n-1);
    GLdouble phi = x[0];
    for(unsigned int i=1;i<n;i++){
        betas(i-1) = x[i];
    }

    double cosp = cos(phi);
    double sinp = sin(phi);

    // current model volumes
    MatrixXd S = BetaOptimization::S_comp - BetaOptimization::S_mat*betas;

    // inertia tensor
    MatrixXd I(2,2);
    I(0,0) = s_y2+s_z2; I(0,1) = -s_xy;
    I(1,0) = -s_xy;     I(1,1) = s_x2+s_z2;

    MatrixXd diag110 = MatrixXd::Zero(2,2);
    diag110(0,0) = 1;
    diag110(1,1) = 1;

    // shifted inertia tensor
    MatrixXd I_com = I- (pow(s_z,2)/s_1) * diag110;

    // rotation matrix
    MatrixXd R(2,2);
    R(0,0) = +cosp;R(0,1) = -sinp;
    R(1,0) = +sinp;R(1,1) = +cosp;

    // Givens Rotation representation
    MatrixXd mat = R*I_com*R.transpose();

    double IX = mat(0,0);
    double IY = mat(1,1);
    double IZ = s_x2+s_y2;

    double gc = gamma_c_top;
    double gi = gamma_i_top;
    double gl = gamma_l_top;

    double M = s_1;
    double l = s_z/s_1;

    double regulator = gl*(betas.transpose()*(L*betas))(0,0);

    double f = gc*pow(l*M,2)+gi*(pow(IX/IZ,2)+pow(IY/IZ,2))+regulator;

    //----------------------------------------

    // gradient for top energy function constraint
    if(grad != NULL)
    {

        // gradient depending on phi

        double dIxdphi = 0;
        dIxdphi += (-2*sinp*cosp)*(s_y2+s_z2-pow(s_z,2)/s_1);
        dIxdphi += (+2*sinp*cosp)*(s_x2+s_z2-pow(s_z,2)/s_1);
        dIxdphi += -2*(cosp-sinp)*(-s_xy);
        double dIxdphi2 = 2*IX*dIxdphi;

        double dIydphi = 0;
        dIydphi += (+2*sinp*cosp)*(s_y2+s_z2-pow(s_z,2)/s_1);
        dIydphi += (-2*sinp*cosp)*(s_x2+s_z2-pow(s_z,2)/s_1);
        dIydphi += +2*(cosp-sinp)*(-s_xy);
        double dIydphi2 = 2*IY*dIydphi;

        grad[0] = (gc/pow(IZ,2))*(dIxdphi2+dIydphi2);

        //----------------------------------------
        for(unsigned int i=0;i<n-1;i++){

            // gradient depending on beta i

            double d_sz_2_s1 = ((-2)*s_z*Smat_z(i)*s_1-pow(s_z,2)*(-Smat_1(i)))/pow(s_1,2);

            double dIxb = 0;
            dIxb += pow(cosp,2)*(-Smat_y2(i)-Smat_z2(i)-d_sz_2_s1);
            dIxb += pow(sinp,2)*(-Smat_x2(i)-Smat_z2(i)-d_sz_2_s1);
            dIxb -= 2*cosp*sinp*Smat_xy(i);

            double dIyb = 0;
            dIyb += pow(sinp,2)*(-Smat_y2(i)-Smat_z2(i)-d_sz_2_s1);
            dIyb += pow(cosp,2)*(-Smat_x2(i)-Smat_z2(i)-d_sz_2_s1);
            dIyb -= 2*cosp*sinp*Smat_xy(i);

            double dIzb = -(Smat_x2(i)+Smat_y2(i));

            double dIxIz = (dIxb*IZ-IX*dIzb)/pow(IZ,2);
            double dIyIz = (dIyb*IZ-IY*dIzb)/pow(IZ,2);

            double dIxIz2 = 2*(IX/IZ)*dIxIz;
            double dIyIz2 = 2*(IY/IZ)*dIyIz;

            double dFirstPart = -2*gc*s_z*Smat_z(i);
            double dSecondPart = gi*(dIxIz2+dIyIz2);


            SpMat vec(betas.size(),1);
            std::vector<T> coefficients;
            coefficients.push_back(T(i,0,1));
            vec.setFromTriplets(coefficients.begin(), coefficients.end());
            double dThirdPart = gl*2*(betas.transpose()*(L*vec))(0,0);

            grad[i+1] = dFirstPart+dSecondPart+dThirdPart;
        }
    }
    //----------------------------------------

    return f;
}

/**
 * @brief BetaOptimization2::optimizeBetasForYoyoWithAngle
 * @param cubeVector
 */
void BetaOptimization::optimizeBetasForYoyoWithAngle(QVector<octree::cubeObject>* cubeVector)
{

    double minf;

    //------------------------------
    int bs = cubeVector->size()+1;
    //------------------------------
    double* lb_si = new double[bs];
    double* ub_si = new double[bs];
    double* betas = new double[bs];
    for(int i=1;i<bs;i++)
    {
        lb_si[i] = 0.0;
        ub_si[i] = 1.0;
        betas[i] = cubeVector->data()[i-1].beta;
    }

    lb_si[0] = -PI;
    ub_si[0] = +PI;
    betas[0] = BetaOptimization::phi;

    //------------------------------
    nlopt_opt opt_spin_it;
    opt_spin_it = nlopt_create( USED_ALGO, bs);

    nlopt_set_lower_bounds(opt_spin_it, lb_si);
    nlopt_set_upper_bounds(opt_spin_it, ub_si);

    nlopt_set_min_objective(opt_spin_it, spinItEnergyFunctionForYoyoWithAngle, NULL);

    double val1 = 1e-4;
    double val2 = 1e-8;

    nlopt_set_xtol_rel(opt_spin_it, val1);

    spin_it_constraint_data data_si[10] = { {0},{1},{2},{3},{4},{5} };
    nlopt_add_equality_constraint(opt_spin_it, spinItContraintsWithAngle, &data_si[0], val2);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraintsWithAngle, &data_si[1], val2);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraintsWithAngle, &data_si[2], val2);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraintsWithAngle, &data_si[3], val2);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraintsWithAngle, &data_si[4], val2);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraintsWithAngle, &data_si[5], val2);

    nlopt_set_maxtime(opt_spin_it,MAX_TIME);

    if (nlopt_optimize(opt_spin_it, betas, &minf) < 0)
    {
        cout << "optimization failed!" << endl;
    }
    else
    {
        cout << "optimization successful (found minimum " << minf << ")" << endl;
    }
    cout << "----------------------------------------" << endl;

    nlopt_destroy(opt_spin_it);

    delete lb_si;
    delete ub_si;
    delete betas;

    for (int i = 1; i < bs; i++) {
        octree::cubeObject* obj = &cubeVector->data()[i-1];
        obj->beta = betas[i];
    }
    BetaOptimization::phi = betas[0];

}

/**
 * @brief BetaOptimization2::optimizeBetasForTopWithAngle
 * @param cubeVector
 */
void BetaOptimization::optimizeBetasForTopWithAngle(QVector<octree::cubeObject>* cubeVector)
{

    double minf;

    //------------------------------
    int bs = cubeVector->size()+1;
    //------------------------------
    double* lb_si = new double[bs];
    double* ub_si = new double[bs];
    double* betas = new double[bs];
    for(int i=1;i<bs;i++)
    {
        lb_si[i] = 0.0;
        ub_si[i] = 1.0;
        betas[i] = cubeVector->data()[i-1].beta;
    }

    lb_si[0] = -PI;
    ub_si[0] = +PI;
    betas[0] = BetaOptimization::phi;

    //------------------------------
    nlopt_opt opt_spin_it;
    opt_spin_it = nlopt_create( USED_ALGO, bs);

    nlopt_set_lower_bounds(opt_spin_it, lb_si);
    nlopt_set_upper_bounds(opt_spin_it, ub_si);

    nlopt_set_min_objective(opt_spin_it, spinItEnergyFunctionForTopWithAngle, NULL);

    double val1 = 1e-4;
    double val2 = 1e-8;

    nlopt_set_xtol_rel(opt_spin_it, val1);

    spin_it_constraint_data data_si[10] = { {0},{1},{2},{3},{5} };

    nlopt_add_equality_constraint(opt_spin_it, spinItContraintsWithAngle, &data_si[0], val2);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraintsWithAngle, &data_si[1], val2);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraintsWithAngle, &data_si[2], val2);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraintsWithAngle, &data_si[3], val2);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraintsWithAngle, &data_si[4], val2);

    nlopt_set_maxtime(opt_spin_it,MAX_TIME);

    if (nlopt_optimize(opt_spin_it, betas, &minf) < 0)
    {
        cout << "optimization failed!" << endl;
    }
    else
    {
        cout << "optimization successful (found minimum " << minf << ")" << endl;
    }
    cout << "----------------------------------------" << endl;

    nlopt_destroy(opt_spin_it);

    delete lb_si;
    delete ub_si;
    delete betas;

    for (int i = 1; i < bs; i++) {
        octree::cubeObject* obj = &cubeVector->data()[i-1];
        obj->beta = betas[i];
    }
    BetaOptimization::phi = betas[0];

}

void BetaOptimization::resetPhi()
{
    BetaOptimization::phi = 0.f;
}
