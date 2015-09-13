#include "betaoptimization.h"

using namespace Eigen;
using namespace std;

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

#define Smat_1(i) BetaOptimization::S_mat(0,i-1)
#define Smat_x(i) BetaOptimization::S_mat(1,i-1)
#define Smat_y(i) BetaOptimization::S_mat(2,i-1)
#define Smat_z(i) BetaOptimization::S_mat(3,i-1)
#define Smat_xy(i) BetaOptimization::S_mat(4,i-1)
#define Smat_yz(i) BetaOptimization::S_mat(5,i-1)
#define Smat_xz(i) BetaOptimization::S_mat(6,i-1)
#define Smat_x2(i) BetaOptimization::S_mat(7,i-1)
#define Smat_y2(i) BetaOptimization::S_mat(8,i-1)
#define Smat_z2(i) BetaOptimization::S_mat(9,i-1)

Mesh* BetaOptimization::mesh = 0;

ExtendedOctree BetaOptimization::octree;

Mesh* BetaOptimization::shellMesh = NULL;

VectorXd BetaOptimization::S_ges;
MatrixXd BetaOptimization::S_mat;

float BetaOptimization::mesh_volume[10];

/**
 * @brief BetaOptimization::initializeOctree Initialize the octree for the optimation
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
    BetaOptimization::octree.setOuterNodes();
    BetaOptimization::octree.setInnerNodes();
    BetaOptimization::octree.increaseShell(shellExtensionValue);
    BetaOptimization::octree.setMergeNodes();
    BetaOptimization::octree.adjustToBasicMaxDepth();

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

typedef struct {
    int index;
} spin_it_constraint_data;

/**
 * @brief spinItContraints
 * @param n
 * @param x
 * @param grad
 * @param data
 * @return
 */
double spinItContraints(unsigned n, const double *x, double *grad, void *data)
{
   spin_it_constraint_data *d = (spin_it_constraint_data *) data;
   int index = d->index;

   VectorXd betas(n-1);
   GLdouble phi = x[0];
   for(unsigned int i=1;i<n;i++){
       betas(i-1) = x[i];
   }
   MatrixXd S = BetaOptimization::S_ges - BetaOptimization::S_mat*betas;

   double cosp = cos(phi);
   double sinp = sin(phi);

   switch(index)
   {
   case 0:

       // gradient for s_x constraint
       if(grad != NULL)
       {
           grad[0] = 0;
           for(unsigned int i=1;i<n;i++){
               grad[i] = -Smat_x(i);
           }
       }

       return s_x;
   case 1:

       // gradient for s_y constraint
       if(grad != NULL)
       {
          grad[0] = 0;
          for(unsigned int i=1;i<n;i++){
              grad[i] = -Smat_y(i);
          }
       }

       return s_y;
   case 2:

       // gradient for s_yz constraint
       if(grad != NULL)
       {
          grad[0] = 0;
          for(unsigned int i=1;i<n;i++){
              grad[i] = -Smat_yz(i);
          }
       }

       return s_yz;
   case 3:

       // gradient for s_xz constraint
       if(grad != NULL)
       {
          grad[0] = 0;
          for(unsigned int i=1;i<n;i++){
               grad[i] = -Smat_xz(i);
          }
       }

       return s_xz;
   case 4:

       // gradient for Givens Rotation constraint
       if(grad != NULL)
       {
           grad[0] = 0;
           grad[0] += ( pow(cosp,2) - pow(sinp,2) )*(s_x2-s_y2);
           grad[0] += (-4*sinp*cosp)*s_xy;

           for(unsigned int i=1;i<n;i++){
                grad[i] = 0;
                grad[i] += cosp*sinp*(Smat_y2(i)-Smat_x2(i));
                grad[i] += (pow(cosp,2)-pow(sinp,2))*(-Smat_xy(i));
           }
       }

       return cosp*sinp*(s_x2-s_y2)+(pow(cosp,2)-pow(sinp,2))*s_xy;
   case 5:

       // gradient for s_z constraint
       if(grad != NULL)
       {
          grad[0] = 0;
          for(unsigned int i=1;i<n;i++){
               grad[i] = -Smat_z(i);
          }
       }

       return s_z;
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
double spinItEnergyFunctionForYoyo(unsigned n, const double *x, double *grad, void *my_func_data)
{

    VectorXd betas(n-1);
    GLdouble phi = x[0];
    for(unsigned int i=1;i<n;i++){
        betas(i-1) = x[i];
    }

    double cosp = cos(phi);
    double sinp = sin(phi);

    // current model volumes
    MatrixXd S = BetaOptimization::S_ges - BetaOptimization::S_mat*betas;

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

    double p = 1;

    double f = p*(pow(IX/IZ,2)+pow(IY/IZ,2));

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

        grad[0] = (p/pow(IZ,2))*(dIxdphi2+dIydphi2);

        //----------------------------------------
        for(unsigned int i=1;i<n;i++){

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

            double dFirstPart = p*(dIxIz2+dIyIz2);

            grad[i] = +dFirstPart;
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
double spinItEnergyFunctionForTop(unsigned n, const double *x, double *grad, void *my_func_data)
{

    VectorXd betas(n-1);
    GLdouble phi = x[0];
    for(unsigned int i=1;i<n;i++){
        betas(i-1) = x[i];
    }

    double cosp = cos(phi);
    double sinp = sin(phi);

    // current model volumes
    MatrixXd S = BetaOptimization::S_ges - BetaOptimization::S_mat*betas;

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

    double p1 = 0.0001;
    double p2 = 1-p1;

    double M = s_1;
    double l = s_z/s_1;

    double f = p1*pow(l*M,2)+p2*(pow(IX/IZ,2)+pow(IY/IZ,2));

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

        grad[0] = (p1/pow(IZ,2))*(dIxdphi2+dIydphi2);

        //----------------------------------------
        for(unsigned int i=1;i<n;i++){

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

            double dFirstPart = -2*p1*s_z*Smat_z(i);
            double dSecondPart = p2*(dIxIz2+dIyIz2);

            grad[i] = dFirstPart+dSecondPart;
        }
    }
    //----------------------------------------

    return f;
}

/**
 * @brief BetaOptimization::optimizeBetasForYoyo
 * @param cubeVector
 */
void BetaOptimization::optimizeBetasForYoyo(QVector<octree::cubeObject>* cubeVector)
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
    betas[0] = 0.0;

    //------------------------------
    nlopt_opt opt_spin_it;
    opt_spin_it = nlopt_create( NLOPT_LD_SLSQP, bs);

    nlopt_set_lower_bounds(opt_spin_it, lb_si);
    nlopt_set_upper_bounds(opt_spin_it, ub_si);

    nlopt_set_min_objective(opt_spin_it, spinItEnergyFunctionForYoyo, NULL);

    nlopt_set_xtol_rel(opt_spin_it, 1e-4);

    spin_it_constraint_data data_si[10] = { {0},{1},{2},{3},{4},{5} };
    nlopt_add_equality_constraint(opt_spin_it, spinItContraints, &data_si[0], 1e-8);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraints, &data_si[1], 1e-8);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraints, &data_si[2], 1e-8);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraints, &data_si[3], 1e-8);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraints, &data_si[4], 1e-8);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraints, &data_si[5], 1e-8);

    if(cubeVector->size()>500){
        cout << "denied optimization" << endl << endl;
    }
    else
    {
        if (nlopt_optimize(opt_spin_it, betas, &minf) < 0) {
            cout << "optimizatio failed!" << endl << endl;
        }
        else {
            cout << "optimizatio successful (found minimum " << minf << ")" << endl << endl;
        }
    }

    nlopt_destroy(opt_spin_it);

    delete lb_si;
    delete ub_si;
    delete betas;

    for (int i = 1; i < bs; i++) {
        octree::cubeObject* obj = &cubeVector->data()[i-1];
        obj->beta = betas[i];
    }

}

/**
 * @brief BetaOptimization::optimizeBetasForTop
 * @param cubeVector
 */
void BetaOptimization::optimizeBetasForTop(QVector<octree::cubeObject>* cubeVector)
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
    betas[0] = 0.0;

    //------------------------------
    nlopt_opt opt_spin_it;
    opt_spin_it = nlopt_create( NLOPT_LD_SLSQP, bs);

    nlopt_set_lower_bounds(opt_spin_it, lb_si);
    nlopt_set_upper_bounds(opt_spin_it, ub_si);

    nlopt_set_min_objective(opt_spin_it, spinItEnergyFunctionForTop, NULL);

    nlopt_set_xtol_rel(opt_spin_it, 1e-4);

    spin_it_constraint_data data_si[10] = { {0},{1},{2},{3},{4} };

    nlopt_add_equality_constraint(opt_spin_it, spinItContraints, &data_si[0], 1e-8);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraints, &data_si[1], 1e-8);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraints, &data_si[2], 1e-8);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraints, &data_si[3], 1e-8);
    nlopt_add_equality_constraint(opt_spin_it, spinItContraints, &data_si[4], 1e-8);

    if(cubeVector->size()>500){
        cout << "denied optimization" << endl << endl;
    }
    else
    {
        if (nlopt_optimize(opt_spin_it, betas, &minf) < 0) {
            cout << "optimizatio failed!" << endl << endl;
        }
        else {
            cout << "optimizatio successful (found minimum " << minf << ")" << endl << endl;
        }
    }

    nlopt_destroy(opt_spin_it);

    delete lb_si;
    delete ub_si;
    delete betas;

    for (int i = 1; i < bs; i++) {
        octree::cubeObject* obj = &cubeVector->data()[i-1];
        obj->beta = betas[i];
    }

}

/**
 * @brief BetaOptimization::optimizeBetas
 * @param optimizationType
 */
void BetaOptimization::optimizeBetas(int optimizationType)
{

    QVector<octree::cubeObject>* cubeVector = NULL;

    cubeVector = BetaOptimization::octree.getInnerCubes();

    cout << "number of cubes is " << cubeVector->size() << endl;

    VectorXd b(cubeVector->size());
    BetaOptimization::S_ges.resize(10);
    BetaOptimization::S_mat.resize(10,cubeVector->size());

    float* s;

    s = calculateVolume(BetaOptimization::mesh);
    for (int i = 0; i < 10; i++) {
        BetaOptimization::S_ges(i) = s[i];
    }

    for (int i = 0; i < cubeVector->size(); i++) {
        b(i) = 0;
        float* s = calculateVolume(cubeVector->at(i).mesh);
        for (int j = 0; j < 10; j++) {
            BetaOptimization::S_mat(j,i) = s[j];
        }
    }

    switch(optimizationType)
    {
        case OPTIMIZATION_TYPE_YOYO:
            optimizeBetasForYoyo(cubeVector);
        break;
        case OPTIMIZATION_TYPE_TOP:
            optimizeBetasForTop(cubeVector);
        break;

    }

    BetaOptimization::octree.updateBetaValues();

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

    BetaOptimization::octree.setDirty();

}

/**
 * @brief BetaOptimization::testSimpleSplitAndMerge
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

        for(int i=0;i<vec->length();i+=3)
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

    BetaOptimization::octree.setDirty();

}

/**
 * @brief BetaOptimization::testSplitAndMerge
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

            for(int i=0;i<vec->length();i+=3)
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
        not_converged = BetaOptimization::octree.splitAndMerge(0.001);
    }

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

    BetaOptimization::octree.setDirty();

}

/**
 * @brief BetaOptimization::calculateVolume calculates volumne integrals of an object
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
