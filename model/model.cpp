#include "model.h"

float Model::p = 1.00;
QVector3D Model::cp;

float Model::w_c = 1;
float Model::w_I = 1;

Mesh* Model::mesh = 0;
float* Model::mesh_volume = 0;

double Model::angle = 0;

ExtendedOctree* Model::octree = 0;

QVector<int>* Model::J = 0;

Mesh* Model::shellMesh = NULL;

/**
 * @brief Model::initializeOctree Initialize the octree for the optimation
 * @param newModifiedMesh the mesh for the optimation
 * @param startDepth the start depth
 * @param maximumDepth the maximum depth
 * @param shellExtensionValue the shell extension value
 */
void Model::initializeOctree(
        Mesh* newModifiedMesh,
        GLint startDepth,
        GLint maximumDepth,
        GLint shellExtensionValue)
{

    if(mesh != NULL)
    {
        delete mesh;
    }

    mesh = newModifiedMesh;

    if(octree == NULL)
    {
        octree = new ExtendedOctree();
    }

    // initialize the octree
    // set point cloud
    octree->setMesh(mesh);
    octree->setStartMaxDepth(startDepth);
    octree->setOptimizationMaxDepth(maximumDepth);
    octree->quantizeSurface();
    octree->setupVectors();

    // calculate outer and inner cells
    octree->setupOctree();
    octree->setupOuterNodes();
    octree->setupInnerNodes();
    octree->increaseShell(shellExtensionValue);
    octree->adjustToBasicMaxDepth();

    // get mesh for inner shell (needed for view)
    octree->createInnerSurface();
    Mesh* newShellMesh = octree->getShellMesh();

    if(shellMesh != NULL)
    {
       delete shellMesh;
    }

    shellMesh = newShellMesh;

    mesh = newModifiedMesh;

    return;
}




void Model::hollow()
{
    mesh_volume = calculateVolume(Model::mesh, p);
    cout << "Volumnes:" << endl;
    for (int i = 0; i < 10; i++) {
        cout << mesh_volume[i] << endl;
    }

    QVector<octree::cubeObject>* cubeVector = NULL;

    bool not_converged = true;
    while (not_converged)
    {
        // get the inner cubes of the octree
        cubeVector = octree->getMergedCubes();

        VectorXd b(cubeVector->size());
        MatrixXd S(cubeVector->size(), 10);
        for (int i = 0; i < cubeVector->size(); i++) {
            b(i) = 0.5;
            float* s = calculateVolume(cubeVector->at(i).mesh, p);
            for (int j = 0; j < 10; j++) {
                S(i,j) = s[j];
            }
        }

        b = optimize(b,S);

        for (int i = 0; i < b.rows(); i++) {
            octree::cubeObject o = cubeVector->at(i);
            o.beta = b(i);
            cubeVector->replace(i, o);
        }

        // set new betas and clrea cubeVector
        octree->updateBetaValues();

        // do split and merge

        not_converged = octree->splitAndMerge(0.001);
    }

    octree->deleteNodeMeshes();

    // set each cube of the octree either to void (beta>0.5) or not void (beta<=0.5)
    octree->setVoids();

    octree->createInnerSurface();
    Mesh* newShellMesh = octree->getShellMesh();

    if(shellMesh != NULL)
    {
       delete shellMesh;
    }

    shellMesh = newShellMesh;

    Model::octree->makeDirty();

}

/**
 * @brief Model::calculateVolume calculates volumne integrals of an object
 * @param mesh triangulated surface of object
 * @param p density of object
 * @return
 */
float* Model::calculateVolume(Mesh* mesh, float p)
{
    float* s = new float[10];
    for (int i = 0; i < 10; i++) {
        s[i] = 0;
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
        s[0] += (n*h1).x();
        si = n*h3;
        s[1] += si.x();
        s[2] += si.y();
        s[3] += si.z();
        si = n*h8;
        s[4] += si.x();
        s[5] += si.y();
        s[6] += si.z();
        si = n*h4;
        s[7] += si.x();
        s[8] += si.y();
        s[9] += si.z();
    }

    s[0] *= (float) 1/6;
    for (int i = 1; i <= 3; i++) {
        s[i] *= (float) 1/24;
    }
    for (int i = 4; i <= 6; i++) {
        s[i] *= (float) 1/120;
    }
    for (int i = 7; i <= 9; i++) {
        s[i] *= (float) 1/60;
    }

    for (int i = 0; i < 10; i++) {
        s[i] *= p;
    }

    return s;
}

/**
 * @brief Model::eqMatrix Sets up the block matrix A of equality constraints Ax = b for quadratic auxilary problem
 * @param b current beta distribution
 * @param S internal mass distribution
 * @return H
 */
MatrixXd Model::eqMatrix(VectorXd b, MatrixXd S)
{
    MatrixXd H(5,b.rows());
    H.row(0) = S.col(1);
    H.row(1) = S.col(2);
    H.row(2) = cos(Model::angle)*sin(Model::angle)*(-S.col(7) + S.col(8)) - (cos(Model::angle)*cos(Model::angle) - sin(Model::angle)*sin(Model::angle)) * S.col(4);
    H.row(3) = S.col(5);
    H.row(4) = S.col(6);
    return H;
}

/**
 * @brief Model::eqVector Sets up the vector b of equality constraints Ax = b for quadratic auxilary problem
 * @param b current beta distribution
 * @param S internal mass distribution
 * @return h
 */
VectorXd Model::eqVector(VectorXd b, MatrixXd S)
{
    VectorXd h(5);
    h(0) = Model::mesh_volume[1] - b.dot(S.col(1));
    h(1) = Model::mesh_volume[2] - b.dot(S.col(2));
    h(2) = -cos(Model::angle)*sin(Model::angle)*((Model::mesh_volume[7] - b.dot(S.col(7))) - (Model::mesh_volume[8] - b.dot(S.col(8)))) - (cos(Model::angle)*cos(Model::angle) - sin(Model::angle)*sin(Model::angle)) * (mesh_volume[4] - b.dot(S.col(4)));
    h(3) = Model::mesh_volume[5] - b.dot(S.col(5));
    h(4) = Model::mesh_volume[6] - b.dot(S.col(6));
    return h;
}

/**
 * @brief Model::ineqMatrix Sets up the block matrix G of inequality constraints Gx <= r for quadratic auxilary problem
 * @param b current beta distribution
 * @param J index set of active inequality constraints
 * @return G
 */
MatrixXd Model::ineqMatrix(VectorXd b)
{
    MatrixXd G = MatrixXd::Zero(Model::J->size(), b.rows());
    for (int i = 0; i < Model::J->size(); i++) {
        if (Model::J->at(i) % 2 == 0) {
            G(i, Model::J->at(i) / 2) = -1;
        } else {
            G(i, Model::J->at(i) / 2) = 1;
        }
    }

    if (Model::J->size() == 0) {
        G.resize(0,0);
    }

    return G;
}

/**
 * @brief Model::ineqVector Sets up the vector r of inequality constraints Gx <= r for quadratic auxilary problem
 * @param b current beta distribution
 * @param J index set of active inequality constraints
 * @return g
 */
VectorXd Model::ineqVector(VectorXd b)
{
    VectorXd g(Model::J->size());
    for (int i = 0; i < Model::J->size(); i++) {
        if (Model::J->at(i) % 2 == 0) {
            g(i) = b(Model::J->at(i) / 2);
        }
        if (Model::J->at(i) % 2 == 1) {
            g(i) = 1 - b(Model::J->at(i) / 2);
        }
    }

    return g;
}

/**
 * @brief Model::gradient Calculates the gradient of objective function f
 * @param b current beta distribution
 * @param S internal mass distribution
 * @return grad
 */
VectorXd Model::gradient(VectorXd b, MatrixXd S)
{
    double Ia = (Model::mesh_volume[8] - b.dot(S.col(8))) + (Model::mesh_volume[9] - b.dot(S.col(9)));
    double Ib = (Model::mesh_volume[7] - b.dot(S.col(7))) + (Model::mesh_volume[9] - b.dot(S.col(9)));

    //shifted inertia tensor
    Ia -= ((Model::mesh_volume[3] - b.dot(S.col(3))) * (Model::mesh_volume[3] - b.dot(S.col(3)))) / (Model::mesh_volume[0] - b.dot(S.col(0)));
    Ib -= ((Model::mesh_volume[3] - b.dot(S.col(3))) * (Model::mesh_volume[3] - b.dot(S.col(3)))) / (Model::mesh_volume[0] - b.dot(S.col(0)));

    double Ix = cos(Model::angle)*cos(Model::angle)*Ia;
    Ix += sin(Model::angle)*sin(Model::angle)*Ib;
    Ix += 2*cos(Model::angle)*sin(Model::angle)*(Model::mesh_volume[4] - b.dot(S.col(4)));
    double Iy = cos(Model::angle)*cos(Model::angle)*Ib;
    Iy += sin(Model::angle)*sin(Model::angle)*Ia;
    Iy -= 2*cos(Model::angle)*sin(Model::angle)*(Model::mesh_volume[4] - b.dot(S.col(4)));
    double Iz = (Model::mesh_volume[7] - b.dot(S.col(7))) + (Model::mesh_volume[8] - b.dot(S.col(8)));

    VectorXd dIc = (2*(Model::mesh_volume[3] - b.dot(S.col(3)))*S.col(3))*(Model::mesh_volume[0] - b.dot(S.col(0)));
    dIc -= (Model::mesh_volume[3] - b.dot(S.col(3)))*(Model::mesh_volume[3] - b.dot(S.col(3)))*S.col(0);
    dIc /= (Model::mesh_volume[0] - b.dot(S.col(0)))*(Model::mesh_volume[0] - b.dot(S.col(0)));
    VectorXd dIx = cos(Model::angle)*cos(Model::angle)*(S.col(8) + S.col(9) + dIc);
    dIx += sin(Model::angle)*sin(Model::angle)*(S.col(7) + S.col(9) + dIc);
    dIx += 2*cos(Model::angle)*sin(Model::angle)*S.col(4);
    VectorXd dIy = cos(Model::angle)*cos(Model::angle)*(S.col(7) + S.col(9) + dIc);
    dIy += sin(Model::angle)*sin(Model::angle)*(S.col(8) + S.col(9) + dIc);
    dIy -= 2*cos(Model::angle)*sin(Model::angle)*S.col(4);
    VectorXd dIz = S.col(7) + S.col(8);

    VectorXd grad_top = -2 * Model::w_c * (Model::mesh_volume[3] - b.dot(S.col(3))) * S.col(3);
    VectorXd grad_yoyo = -2 * Model::w_I * ((Ix*Iz*dIx - Ix*Ix*dIz) / (Iz*Iz*Iz) + (Iy*Iz*dIy - Iy*Iy*dIz) / (Iz*Iz*Iz));
    VectorXd grad = grad_top + grad_yoyo;
    return grad;
}

/**
 * @brief Model::optimize Active set SQP strategy to optimize b
 * @param b start distribution
 * @param S internal mass distribution
 * @return b optimized distribution
 */
VectorXd Model::optimize(VectorXd b, MatrixXd S)
{   
    Matrix2d I;
    I(0,0) = (Model::mesh_volume[8] - b.dot(S.col(8))) + (Model::mesh_volume[9] - b.dot(S.col(9)));
    I(0,1) = -(Model::mesh_volume[4] - b.dot(S.col(4)));
    I(1,0) = -(Model::mesh_volume[4] - b.dot(S.col(4)));
    I(1,1) = (Model::mesh_volume[7] - b.dot(S.col(7))) + (Model::mesh_volume[9] - b.dot(S.col(9)));

    //shifted inertia tensor
    I(0,0) -= ((Model::mesh_volume[3] - b.dot(S.col(3))) * (Model::mesh_volume[3] - b.dot(S.col(3)))) / (Model::mesh_volume[0] - b.dot(S.col(0)));
    I(1,1) -= ((Model::mesh_volume[3] - b.dot(S.col(3))) * (Model::mesh_volume[3] - b.dot(S.col(3)))) / (Model::mesh_volume[0] - b.dot(S.col(0)));

    //rotated components
    Model::angle = M_PI / 4;
    if (I(0,0) != I(1,1)) {
        Model::angle = 0.5 * atan(2*I(0,1)/(I(0,0)-I(1,1)));
    }

    double Ia = (Model::mesh_volume[8] - b.dot(S.col(8))) + (Model::mesh_volume[9] - b.dot(S.col(9)));
    double Ib = (Model::mesh_volume[7] - b.dot(S.col(7))) + (Model::mesh_volume[9] - b.dot(S.col(9)));

    //shifted inertia tensor
    Ia -= ((Model::mesh_volume[3] - b.dot(S.col(3))) * (Model::mesh_volume[3] - b.dot(S.col(3)))) / (Model::mesh_volume[0] - b.dot(S.col(0)));
    Ib -= ((Model::mesh_volume[3] - b.dot(S.col(3))) * (Model::mesh_volume[3] - b.dot(S.col(3)))) / (Model::mesh_volume[0] - b.dot(S.col(0)));

    double Ix = cos(Model::angle)*cos(Model::angle)*Ia;
    Ix += sin(Model::angle)*sin(Model::angle)*Ib;
    Ix += 2*cos(Model::angle)*sin(Model::angle)*(Model::mesh_volume[4] - b.dot(S.col(4)));
    double Iy = cos(Model::angle)*cos(Model::angle)*Ib;
    Iy += sin(Model::angle)*sin(Model::angle)*Ia;
    Iy -= 2*cos(Model::angle)*sin(Model::angle)*(Model::mesh_volume[4] - b.dot(S.col(4)));
    double Iz = (Model::mesh_volume[7] - b.dot(S.col(7))) + (Model::mesh_volume[8] - b.dot(S.col(8)));

    VectorXd dIc = (2*(Model::mesh_volume[3] - b.dot(S.col(3)))*S.col(3))*(Model::mesh_volume[0] - b.dot(S.col(0)));
    dIc -= (Model::mesh_volume[3] - b.dot(S.col(3)))*(Model::mesh_volume[3] - b.dot(S.col(3)))*S.col(0);
    dIc /= (Model::mesh_volume[0] - b.dot(S.col(0)))*(Model::mesh_volume[0] - b.dot(S.col(0)));
    VectorXd dIx = cos(Model::angle)*cos(Model::angle)*(S.col(8) + S.col(9) + dIc);
    dIx += sin(Model::angle)*sin(Model::angle)*(S.col(7) + S.col(9) + dIc);
    dIx += 2*cos(Model::angle)*sin(Model::angle)*S.col(4);
    VectorXd dIy = cos(Model::angle)*cos(Model::angle)*(S.col(7) + S.col(9) + dIc);
    dIy += sin(Model::angle)*sin(Model::angle)*(S.col(8) + S.col(9) + dIc);
    dIy -= 2*cos(Model::angle)*sin(Model::angle)*S.col(4);
    VectorXd dIz = S.col(7) + S.col(8);

    MatrixXd A = 2 * Model::w_c * (S.col(3) * S.col(3).transpose());
    A += 2 * Model::w_I * ((((dIx*Iz + Ix*dIz)*dIx.transpose() - (2*Ix*dIx)*dIz.transpose()) * Iz * Iz * Iz - (Ix*Iz*dIx - Ix*Ix*dIz) * (3 * Iz * Iz * dIz.transpose())) / (Iz*Iz*Iz*Iz*Iz*Iz));
    A += 2 * Model::w_I * ((((dIy*Iz + Iy*dIz)*dIy.transpose() - (2*Iy*dIy)*dIz.transpose()) * Iz * Iz * Iz - (Iy*Iz*dIy - Iy*Iy*dIz) * (3 * Iz * Iz * dIz.transpose())) / (Iz*Iz*Iz*Iz*Iz*Iz));

    VectorXd d = VectorXd::Ones(1);
    VectorXd lambda = -VectorXd::Ones(1);
    VectorXd mu = -VectorXd::Ones(1);

    Model::J = new QVector<int>();
    for (int i = 0; i < 2*b.rows(); i++) {
        Model::J->push_back(i);
    }

//    double eps = 1000;

    VectorXd grad = gradient(b,S);

    //Setting up equality constraints
    MatrixXd H = eqMatrix(b, S);
    VectorXd h = eqVector(b, S);

    //Setting up active inequality constraints
    MatrixXd G = ineqMatrix(b);
    VectorXd g = ineqVector(b);

    MatrixXd L(A.rows() + H.rows() + G.rows(), A.cols() + H.rows() + G.rows());
    L.block(0, 0, A.rows(), A.cols()) = A;
    L.block(A.rows(), 0, H.rows(), H.cols()) = H;
    L.block(0, A.cols(), H.cols(), H.rows()) = H.transpose();
    L.block(A.rows() + H.rows(), 0, G.rows(), G.cols()) = G;
    L.block(0, A.cols() + H.rows(), G.cols(), G.rows()) = G.transpose();
    L.block(A.rows(), A.cols(), H.rows() + G.rows(), H.rows() + G.rows()).setZero();

    VectorXd c(grad.rows() + h.rows() + g.rows());
    c.block(0 , 0, grad.rows(), 1) = -grad;
    c.block(grad.rows(), 0, h.rows(), 1) = h;
    c.block(grad.rows() + h.rows(), 0, g.rows(), 1) = g;

    VectorXd x = L.fullPivHouseholderQr() .solve(c);
    d = x.block(0, 0, b.rows(), 1);
    lambda = x.block(b.rows(), 0, h.rows(), 1);
    mu = x.block(b.rows() + h.rows(), 0, g.rows(), 1);

    while (!(/*(d.array() == 0).all()*/d.norm() <= 1e-3 && (mu.array() >= 0).all())) {
        if (/*(d.array() == 0).all()*/d.norm() <= 1e-3 && !((mu.array() >= 0).all())) {
            //Inactivity step
            int j = 0;
            for (int i = 1; i < mu.rows(); i++) {
                if (mu(i) < mu(j)) {
                    j = i;
                }
            }
            Model::J->removeAt(j);

            MatrixXd G = ineqMatrix(b);
            VectorXd g = ineqVector(b);

            L.resize(A.rows() + H.rows() + G.rows(), A.cols() + H.rows() + G.rows());
            L.block(0, 0, A.rows(), A.cols()) = A;
            L.block(A.rows(), 0, H.rows(), H.cols()) = H;
            L.block(0, A.cols(), H.cols(), H.rows()) = H.transpose();
            L.block(A.rows() + H.rows(), 0, G.rows(), G.cols()) = G;
            L.block(0, A.cols() + H.rows(), G.cols(), G.rows()) = G.transpose();
            L.block(A.rows(), A.cols(), H.rows() + G.rows(), H.rows() + G.rows()).setZero();

            c.resize(grad.rows() + h.rows() + g.rows());
            c.block(0 , 0, grad.rows(), 1) = -grad;
            c.block(grad.rows(), 0, h.rows(), 1) = h;
            c.block(grad.rows() + h.rows(), 0, g.rows(), 1) = g;

            VectorXd z = L.fullPivHouseholderQr().solve(c);
            d = z.block(0, 0, b.rows(), 1);
            lambda = z.block(b.rows(), 0, h.rows(), 1);
            mu = z.block(b.rows() + h.rows(), 0, g.rows(), 1);
        }

//        VectorXd beta(lambda.rows());
//        VectorXd gamma(mu.rows());
//        //Powell step-size with merit function
//        for (int i = 0; i < lambda.rows(); i++) {
//            beta(i) = abs(lambda(i)) + eps;
//        }
//        for (int i = 0; i < mu.rows(); i++) {
//            gamma(i) = mu(i) + eps;
//        }

//        cout << "Penalty: " << Model::penalty(b, S, beta, gamma) << endl;
//        float Ix = (Model::mesh_volume[8] - b.dot(S.col(8))) + (Model::mesh_volume[9] - b.dot(S.col(9)));
//        float Iy = (Model::mesh_volume[7] - b.dot(S.col(7))) + (Model::mesh_volume[9] - b.dot(S.col(9)));
//        float Iz = (Model::mesh_volume[7] - b.dot(S.col(7))) + (Model::mesh_volume[8] - b.dot(S.col(8)));
//        float s1 = (Model::mesh_volume[3] - b.dot(S.col(3))) * (Model::mesh_volume[3] - b.dot(S.col(3)));
//        float s2 = (Ix * Ix) / (Iz * Iz) + (Iy * Iy) / (Iz * Iz);
//        cout << "Spinability: " << Model::w_c * s1 + Model::w_I * s2 << endl;

//        double sigma = 1;
//        sigma = powell(b, S, d, 0.1, 0.9, beta, gamma);

//        //BFGS Update
//        VectorXd s = (b + sigma * d) - b;
//        VectorXd y = gradient(b + sigma * d, S) - grad;
//        if (y.dot(s) > 0) {
//            A = A - ((A*s)*(A*s).transpose())/(s.dot(A*s)) + (y*y.transpose())/(y.dot(s));
//            SelfAdjointEigenSolver<MatrixXd> eigensolver2(A);
//            VectorXd eig = eigensolver2.eigenvalues();
//            double smallest = eig(0);
//            for (int i = 1; i < eig.rows(); i++) {
//                if (eig(i) < smallest) {
//                    smallest = eig(i);
//                }
//            }
//            cout << "Smallest Eigenvalue " << smallest << endl;
//        }

        b = b + /*sigma */ d;

        I(0,0) = (Model::mesh_volume[8] - b.dot(S.col(8))) + (Model::mesh_volume[9] - b.dot(S.col(9)));
        I(0,1) = -(Model::mesh_volume[4] - b.dot(S.col(4)));
        I(1,0) = -(Model::mesh_volume[4] - b.dot(S.col(4)));
        I(1,1) = (Model::mesh_volume[7] - b.dot(S.col(7))) + (Model::mesh_volume[9] - b.dot(S.col(9)));

        //shifted inertia tensor
        I(0,0) -= ((Model::mesh_volume[3] - b.dot(S.col(3))) * (Model::mesh_volume[3] - b.dot(S.col(3)))) / (Model::mesh_volume[0] - b.dot(S.col(0)));
        I(1,1) -= ((Model::mesh_volume[3] - b.dot(S.col(3))) * (Model::mesh_volume[3] - b.dot(S.col(3)))) / (Model::mesh_volume[0] - b.dot(S.col(0)));

        //rotated components
        Model::angle = M_PI / 4;
        if (I(0,0) != I(1,1)) {
            Model::angle = 0.5 * atan(2*I(0,1)/(I(0,0)-I(1,1)));
        }

        for (int i = 0; i < 2 * b.rows(); i++) {
            if (i % 2 == 0 && b(i/2) < 0 && !Model::J->contains(i)) {
                Model::J->push_back(i);
            }
            if (i % 2 == 1 && b(i/2) > 1 && !Model::J->contains(i)) {
                Model::J->push_back(i);
            }
        }
        QVector<int>* J1 = Model::J;

        VectorXd grad = gradient(b,S);

        //Setting up equality constraints
        MatrixXd H = eqMatrix(b, S);
        VectorXd h = eqVector(b, S);

        //Setting up active inequality constraints
        MatrixXd G = ineqMatrix(b);
        VectorXd g = ineqVector(b);

        Ia = (Model::mesh_volume[8] - b.dot(S.col(8))) + (Model::mesh_volume[9] - b.dot(S.col(9)));
        Ib = (Model::mesh_volume[7] - b.dot(S.col(7))) + (Model::mesh_volume[9] - b.dot(S.col(9)));

        //shifted inertia tensor
        Ia -= ((Model::mesh_volume[3] - b.dot(S.col(3))) * (Model::mesh_volume[3] - b.dot(S.col(3)))) / (Model::mesh_volume[0] - b.dot(S.col(0)));
        Ib -= ((Model::mesh_volume[3] - b.dot(S.col(3))) * (Model::mesh_volume[3] - b.dot(S.col(3)))) / (Model::mesh_volume[0] - b.dot(S.col(0)));

        Ix = cos(Model::angle)*cos(Model::angle)*Ia;
        Ix += sin(Model::angle)*sin(Model::angle)*Ib;
        Ix += 2*cos(Model::angle)*sin(Model::angle)*(Model::mesh_volume[4] - b.dot(S.col(4)));
        Iy = cos(Model::angle)*cos(Model::angle)*Ib;
        Iy += sin(Model::angle)*sin(Model::angle)*Ia;
        Iy -= 2*cos(Model::angle)*sin(Model::angle)*(Model::mesh_volume[4] - b.dot(S.col(4)));
        Iz = (Model::mesh_volume[7] - b.dot(S.col(7))) + (Model::mesh_volume[8] - b.dot(S.col(8)));

        dIc = (2*(Model::mesh_volume[3] - b.dot(S.col(3)))*S.col(3))*(Model::mesh_volume[0] - b.dot(S.col(0)));
        dIc -= (Model::mesh_volume[3] - b.dot(S.col(3)))*(Model::mesh_volume[3] - b.dot(S.col(3)))*S.col(0);
        dIc /= (Model::mesh_volume[0] - b.dot(S.col(0)))*(Model::mesh_volume[0] - b.dot(S.col(0)));
        dIx = cos(Model::angle)*cos(Model::angle)*(S.col(8) + S.col(9) + dIc);
        dIx += sin(Model::angle)*sin(Model::angle)*(S.col(7) + S.col(9) + dIc);
        dIx += 2*cos(Model::angle)*sin(Model::angle)*S.col(4);
        dIy = cos(Model::angle)*cos(Model::angle)*(S.col(7) + S.col(9) + dIc);
        dIy += sin(Model::angle)*sin(Model::angle)*(S.col(8) + S.col(9) + dIc);
        dIy -= 2*cos(Model::angle)*sin(Model::angle)*S.col(4);
        dIz = S.col(7) + S.col(8);

        A = 2 * Model::w_c * (S.col(3) * S.col(3).transpose());
        A += 2 * Model::w_I * ((((dIx*Iz + Ix*dIz)*dIx.transpose() - (2*Ix*dIx)*dIz.transpose()) * Iz * Iz * Iz - (Ix*Iz*dIx - Ix*Ix*dIz) * (3 * Iz * Iz * dIz.transpose())) / (Iz*Iz*Iz*Iz*Iz*Iz));
        A += 2 * Model::w_I * ((((dIy*Iz + Iy*dIz)*dIy.transpose() - (2*Iy*dIy)*dIz.transpose()) * Iz * Iz * Iz - (Iy*Iz*dIy - Iy*Iy*dIz) * (3 * Iz * Iz * dIz.transpose())) / (Iz*Iz*Iz*Iz*Iz*Iz));

        L.resize(A.rows() + H.rows() + G.rows(), A.cols() + H.rows() + G.rows());
        L.block(0, 0, A.rows(), A.cols()) = A;
        L.block(A.rows(), 0, H.rows(), H.cols()) = H;
        L.block(0, A.cols(), H.cols(), H.rows()) = H.transpose();
        L.block(A.rows() + H.rows(), 0, G.rows(), G.cols()) = G;
        L.block(0, A.cols() + H.rows(), G.cols(), G.rows()) = G.transpose();
        L.block(A.rows(), A.cols(), H.rows() + G.rows(), H.rows() + G.rows()).setZero();

        c.resize(grad.rows() + h.rows() + g.rows());
        c.block(0 , 0, grad.rows(), 1) = -grad;
        c.block(grad.rows(), 0, h.rows(), 1) = h;
        c.block(grad.rows() + h.rows(), 0, g.rows(), 1) = g;

        VectorXd x = L.fullPivHouseholderQr() .solve(c);
        d = x.block(0, 0, b.rows(), 1);
        lambda = x.block(b.rows(), 0, h.rows(), 1);
        mu = x.block(b.rows() + h.rows(), 0, g.rows(), 1);
    }

    delete Model::J;
    return b;
}

///**
// * @brief Model::armijo Armijo Stepsize strategy
// * @param b
// * @param S
// * @param d
// * @param alpha
// * @param delta
// * @param gamma
// * @return
// */
//double Model::armijo(VectorXd b, MatrixXd S, VectorXd d, VectorXd beta, VectorXd gamma)
//{
//    double sigma = -penaltyDirectionalGradient(b, S, d, beta, gamma) / d.dot(d);
//    cout << penaltyDirectionalGradient(b, S, d, beta, gamma) << endl;

//    while (!(penalty(b + sigma * d, S, beta, gamma) <= penalty(b, S, beta, gamma) + 0.1 * sigma * penaltyDirectionalGradient(b, S, d, beta, gamma))) {
//        sigma = 0.5 * sigma;
//    }

//    return sigma;
//}

///**
// * @brief Model::powell Calculates efficient step-size with powell strategy
// * @param b current beta distribution
// * @param S internal mass distribution
// * @param d descend direction
// * @param alpha scaling factor for merit function
// * @param delta bias for powell step-size
// * @param beta bias for powell step-size
// * @return sigma
// */
//double Model::powell(VectorXd b, MatrixXd S, VectorXd d, double d1, double d2, VectorXd beta, VectorXd gamma)
//{
//    double sigma = 1000;
//    double a1 = sigma;
//    double a2 = sigma;

//    cout << penalty(b, S, beta, gamma) << endl;
//    cout << penalty(b + sigma * d, S, beta, gamma) << endl;
//    cout << penaltyDirectionalGradient(b, S, d,beta, gamma) << endl;

//    double g1 = powellG1(sigma, b, S, d, beta, gamma);
//    double g2 = powellG2(sigma, b, S, d, beta, gamma);

//    if (g1 >= d1 && g2 <= d2) {
//        return sigma;
//    }
//    if (g1 >= d1) {
//        a1 = sigma;
//        int l = 1;
//        while (!(powellG1(a2, b, S, d, beta, gamma) < d1)) {
//            a2 = pow(2, l) * sigma;
//            l += 1;
//        }
//    } else {
//        if (g1 < d1) {
//            a2 = sigma;
//            int l = 1;
//            while (!(powellG1(a1, b, S, d, beta, gamma) >= d1 && powellG2(a1, b, S, d, beta, gamma) > d2)) {
////                cout << powellG1(a1, b, S, d, beta, gamma) << endl;
////                cout << powellG2(a1, b, S, d, beta, gamma) << endl;
//                a1 = pow(2, -l) * sigma;
//                l += 1;
//            }
//        }
//    }

//    while (!(g1 >= d1 && g2 <= d2)) {
//        sigma = 0.5 * (a1 + a2);

//        g1 = powellG1(sigma, b, S, d, beta, gamma);
//        g2 = powellG2(sigma, b, S, d, beta, gamma);

//        if (g1 >= d1) {
//            a1 = sigma;
//        } else {
//            a2 = sigma;
//        }
//    }

//    return sigma;
//}

///**
// * @brief Model::powellG1 Auxilary function for powell strategy. Calculates g1(x).
// * @param sigma current step-size
// * @param b current beta distribution
// * @param S internal mass distribution
// * @param d descend direction
// * @param alpha scaling factor for merit function
// * @return g
// */
//double Model::powellG1(double sigma, VectorXd b, MatrixXd S, VectorXd d, VectorXd beta, VectorXd gamma)
//{
//    double g = 1;
//    if (sigma > 0) {
//        g = (penalty(b + sigma * d, S, beta, gamma) - penalty(b, S, beta, gamma)) / (sigma * penaltyDirectionalGradient(b, S, d, beta, gamma));
//    }

//    return g;
//}

///**
// * @brief Model::powellG1 Auxilary function for powell strategy. Calculates g2(x).
// * @param sigma current step-size
// * @param b current beta distribution
// * @param S internal mass distribution
// * @param d descend direction
// * @param alpha scaling factor for merit function
// * @return
// */
//double Model::powellG2(double sigma, VectorXd b, MatrixXd S, VectorXd d, VectorXd beta, VectorXd gamma)
//{
//    double g2 = penaltyDirectionalGradient(b + sigma * d, S, d, beta, gamma) / penaltyDirectionalGradient(b, S, d, beta, gamma);
//    return g2;
//}

///**
// * @brief Model::penalty Returns value of merit function.
// * @param b current beta distribution
// * @param S internal mass distribution
// * @param alpha scaling factor for merit function
// * @return
// */
//double Model::penalty(VectorXd b, MatrixXd S, VectorXd beta, VectorXd gamma)
//{
//    double Ix = (Model::mesh_volume[8] - b.dot(S.col(8))) + (Model::mesh_volume[9] - b.dot(S.col(9)));
//    double Iy = (Model::mesh_volume[7] - b.dot(S.col(7))) + (Model::mesh_volume[9] - b.dot(S.col(9)));
//    double Iz = (Model::mesh_volume[7] - b.dot(S.col(7))) + (Model::mesh_volume[8] - b.dot(S.col(8)));
//    double s1 = (Model::mesh_volume[3] - b.dot(S.col(3))) * (Model::mesh_volume[3] - b.dot(S.col(3)));
//    double s2 = (Ix * Ix) / (Iz * Iz) + (Iy * Iy) / (Iz * Iz);

//    VectorXd h(beta.rows());
////    h(0) = abs(Model::mesh_volume[1] - b.dot(S.col(1)));
////    h(1) = abs(Model::mesh_volume[2] - b.dot(S.col(2)));
////    //h(2) = abs(Model::mesh_volume[4] - b.dot(S.col(4)));
////    h(2) = abs(Model::mesh_volume[5] - b.dot(S.col(5)));
////    h(3) = abs(Model::mesh_volume[6] - b.dot(S.col(6)));

//    VectorXd g(Model::J->size());
//    for (int i = 0; i < Model::J->size(); i++) {
//        if (Model::J->at(i) % 2 == 0) {
//            g(i) = (-b(Model::J->at(i) / 2) + abs(-b(Model::J->at(i) / 2))) / 2;
//        }
//        if (Model::J->at(i) % 2 == 1) {
//            g(i) = ((b(Model::J->at(i) / 2) - 1) + abs(b(Model::J->at(i) / 2) - 1)) / 2;
//        }
//    }

//    return Model::w_c * s1 + Model::w_I * s2 + beta.dot(h) + gamma.dot(g);
//}

///**
// * @brief Model::penaltyDirectionalGradient Returns value of directional derivate of merit function in direction d.
// * @param b current beta distribution
// * @param S internal mass distribution
// * @param d descent direction
// * @param alpha scaling factor for merit function
// * @return
// */
//double Model::penaltyDirectionalGradient(VectorXd b, MatrixXd S,  VectorXd d, VectorXd beta, VectorXd gamma)
//{
//    double result = gradient(b, S).dot(d);

////    double h = Model::mesh_volume[1] - b.dot(S.col(1));
////    if (h > 0) {
////       result += beta(0) * -S.col(1).dot(d);
////    } else {
////        if (h < 0) {
////            result -= beta(0) * -S.col(1).dot(d);
////        } else {
////            if (h == 0) {
////                result += beta(0) * (abs(-S.col(1).dot(d)));
////            }
////        }
////    }
////    h = Model::mesh_volume[2] - b.dot(S.col(2));
////    if (h > 0) {
////       result += beta(1) * -S.col(2).dot(d);
////    } else {
////        if (h < 0) {
////            result -= beta(1) * -S.col(2).dot(d);
////        } else {
////            if (h == 0) {
////                result += beta(1) * (abs(-S.col(2).dot(d)));
////            }
////        }
////    }
////    h = Model::mesh_volume[4] - b.dot(S.col(4));
////    if (h > 0) {
////       result += beta(2) * (-S.col(4).dot(d));
////    } else {
////        if (h < 0) {
////            result -= beta(2) * (-S.col(4).dot(d));
////        } else {
////            if (h == 0) {
////                result += beta(2) * (abs(-S.col(4).dot(d)));
////            }
////        }
////    }
////    h = Model::mesh_volume[5] - b.dot(S.col(5));
////    if (h > 0) {
////       result += beta(2) * (-S.col(5).dot(d));
////    } else {
////        if (h < 0) {
////            result -= beta(2) * (-S.col(5).dot(d));
////        } else {
////            if (h == 0) {
////                result += beta(2) * (abs(-S.col(5).dot(d)));
////            }
////        }
////    }
////    h = Model::mesh_volume[6] - b.dot(S.col(6));
////    if (h > 0) {
////       result += beta(3) * (-S.col(6).dot(d));
////    } else {
////        if (h < 0) {
////            result -= beta(3) * (-S.col(6).dot(d));
////        } else {
////            if (h == 0) {
////                result += beta(3) * (abs(-S.col(6).dot(d)));
////            }
////        }
////    }

//    double g = 0;
//    for (int i = 0; i < Model::J->size(); i++) {
//        if (Model::J->at(i) % 2 == 0) {
//            g = -b(Model::J->at(i) / 2);
//            if (g > 0 || (g == 0 && -d(Model::J->at(i) / 2) > 0)) {
//                result += gamma(i) * -d(Model::J->at(i) / 2);
//            }
//        } else {
//            g = b(Model::J->at(i) / 2) - 1;
//            if (g > 0 || (g == 0 && d(Model::J->at(i) / 2) > 0)) {
//                result += gamma(i) * d(Model::J->at(i) / 2);
//            }
//        }
//    }

//    return result;
//}
