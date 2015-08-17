#include "model.h"

float Model::p = 1.00;
QVector3D Model::cp;

float Model::w_c = 1;
float Model::w_I = 1;

Mesh* Model::mesh = 0;
float* Model::mesh_volume = 0;

ExtendedOctree* Model::octree = 0;


void Model::initialize(Mesh* mesh)
{
    Model::mesh = mesh;

    GLint depth = 3;
    Model::octree = new ExtendedOctree();
    octree->setMesh(mesh);
    octree->setStartDepth(depth);
    octree->setMaxDepth(depth+1);
    octree->quantizeSurface();
    octree->setupVectors();

    octree->setupOctree();
    octree->setShellNodeIndices();
    octree->setOuterNodes();
    octree->setInnerNodes();
    octree->setInnerNodeIndices();
    octree->adjustMaxDepth();
    octree->increaseShell(0);

    octree->setShellNodeIndices();
    octree->setInnerNodeIndices();

    octree->createInnerSurface();

    Model::hollow();

}

void Model::hollow()
{
    //QVector<float>* geometry = Model::mesh->getGeometry();
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
        cubeVector = octree->getInnerCubes();

        VectorXf b(cubeVector->size());
        MatrixXf S(cubeVector->size(), 10);
        for (int i = 0; i < cubeVector->size(); i++) {
            b(i) = cubeVector->at(i).beta;
            float* s = calculateVolume(cubeVector->at(i).mesh, p);
            for (int j = 0; j < 10; j++) {
                S(i,j) = s[j];
            }
        }

        //move center to (0,0,0)
        QVector3D c;
        c.setX(1/mesh_volume[0]*mesh_volume[1]);
        c.setY(1/mesh_volume[0]*mesh_volume[2]);
        c.setZ(1/mesh_volume[0]*mesh_volume[3]);

        cout << "Center of Mass: (" << c.x() << "," << c.y() << "," << c.z() << ")" << endl;

        /*QVector<GLfloat>* tmp =/ new QVector<GLfloat>();
        for (int i = 0; i < geometry->size(); i++) {
            tmp->push_back(0);
        }

        Mesh* working_copy = new Mesh(tmp, Model::mesh->getSurfaceNormals(), Model::mesh->getVertexNormals(), Model::mesh->getIndices());
        for (int i = 0; i < geometry->size(); i += 3) {
                working_copy->getGeometry()->replace(i, geometry->at(i) - cp.x());
                working_copy->getGeometry()->replace(i + 1, geometry->at(i + 1) - 0);
                working_copy->getGeometry()->replace(i + 2, geometry->at(i + 2) - 0);
        }

        mesh_volume = calculateVolume(working_copy, p);
        cout << "Volumnes:" << endl;
        for (int i = 0; i < 10; i++) {
            cout << mesh_volume[i] << endl;
        }

        c.setX(1/mesh_volume[0]*mesh_volume[1]);
        c.setY(1/mesh_volume[0]*mesh_volume[2]);
        c.setZ(1/mesh_volume[0]*mesh_volume[3]);

        cout << "Center of Mass: (" << c.x() << "," << c.y() << "," << c.z() << ")" << endl;
        cout << "Done" << endl;*/

        //b = optimize(b,S);

        for (int i = 0; i < b.rows(); i++) {
            octree::cubeObject o = cubeVector->at(i);
            o.beta = b(i);
            cubeVector->replace(i, o);
        }

        // set new betas and clrea cubeVector
        octree->updateBetas();

        // do split and merge
        not_converged = octree->splitAndMerge(0);
    }

    // set each cube of the octree either to void (beta>0.5) or not void (beta<=0.5)
    octree->setVoids();
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
MatrixXf Model::eqMatrix(VectorXf b, MatrixXf S)
{
    MatrixXf H(5, b.rows());
    H.row(0) = S.col(1);
    H.row(1) = S.col(2);
    H.row(2) = S.col(4);
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
VectorXf Model::eqVector(VectorXf b, MatrixXf S)
{
    VectorXf h(5);
    h(0) = Model::mesh_volume[1] - b.dot(S.col(1));
    h(1) = Model::mesh_volume[2] - b.dot(S.col(2));
    h(2) = Model::mesh_volume[4] - b.dot(S.col(4));
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
MatrixXf Model::ineqMatrix(VectorXf b, QVector<int> J)
{
    VectorXf zero = VectorXf::Zero(b.rows());
    MatrixXf G(J.size(), b.rows());
    for (int i = 0; i < J.size(); i++) {
        zero(J.at(i) / 2) = 1;
        G.row(i) = zero;
        zero(J.at(i) / 2) = 0;
    }
    return G;
}

/**
 * @brief Model::ineqVector Sets up the vector r of inequality constraints Gx <= r for quadratic auxilary problem
 * @param b current beta distribution
 * @param J index set of active inequality constraints
 * @return g
 */
VectorXf Model::ineqVector(VectorXf b, QVector<int> J)
{
    VectorXf g(J.size());
    for (int i = 0; i < J.size(); i++) {
        if (J.at(i) % 2 == 0) {
            g(i) = -b(J.at(i) / 2);
        }
        if (J.at(i) % 2 == 1) {
            g(i) = 1 - b(J.at(i) / 2);
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
VectorXf Model::gradient(VectorXf b, MatrixXf S)
{
    float u1 = Model::mesh_volume[8] - b.dot(S.col(8)) + Model::mesh_volume[9] - b.dot(S.col(9));
    float u2 = Model::mesh_volume[7] - b.dot(S.col(7)) + Model::mesh_volume[9] - b.dot(S.col(9));
    float v = Model::mesh_volume[7] - b.dot(S.col(7)) + Model::mesh_volume[8] - b.dot(S.col(8));
    VectorXf grad_top = -2 * Model::w_c * (Model::mesh_volume[3] - b.dot(S.col(3))) * S.col(3);
    VectorXf grad_yoyo = -2 * Model::w_I * (((S.col(8) + S.col(9))*u1*v + (S.col(7) + S.col(9))*u2*v - (u1*u1 + u2*u2)*(S.col(7)+S.col(8))) / (v * v * v));
    VectorXf grad = grad_top + grad_yoyo;
    cout << grad << endl << endl;
    return grad;
}

/**
 * @brief Model::optimize Active set SQP strategy to optimize b
 * @param b start distribution
 * @param S internal mass distribution
 * @return b optimized distribution
 */
VectorXf Model::optimize(VectorXf b, MatrixXf S)
{
    MatrixXf A(b.rows(), b.rows());
    A.setIdentity();

    VectorXf d = VectorXf::Ones(1);
    VectorXf lambda = -VectorXf::Ones(1);
    VectorXf mu = -VectorXf::Ones(1);

    float alpha = 0.1;
    float gamma = 0.01;

    while (!((d.array() == 0).all() && (mu.array() >= 0).all())) {
        VectorXf grad = gradient(b,S);

        //Setting up equality constraints
        MatrixXf H = eqMatrix(b, S);
        VectorXf h = eqVector(b, S);

        //Setting up active inequality constraints
        QVector<int> J;
        for (int i = 0; i < 2*b.rows(); i++) {
            if (i % 2 == 0) {
                if (b(i / 2) == 0) {
                    J.push_back(i);
                }
            }
            if (i % 2 == 1) {
                if (b(i / 2) == 1) {
                    J.push_back(i);
                }
            }
        }

        MatrixXf G = ineqMatrix(b, J);
        VectorXf g = ineqVector(b, J);

        MatrixXf L(A.rows() + H.rows() + G.rows(), A.cols() + H.cols() + G.cols());
        L.block(0, 0, A.rows(), A.cols()) = A;
        L.block(A.rows() + 1, 0, H.rows(), H.cols()) = H;
        L.block(0, A.cols() + 1, H.cols(), H.rows()) = H.transpose();
        L.block(A.rows() + H.rows() + 1, 0, G.rows(), G.cols()) = G;
        L.block(0, A.cols() + H.cols() + 1, G.cols(), G.rows()) = G.transpose();
        L.block(A.rows() + 1, A.cols() + 1, H.rows() + G.rows(), H.rows() + G.rows()).setZero();

        VectorXf c(grad.rows() + h.rows() + g.rows());
        c.block(0 , 0, grad.rows(), 1) = -grad;
        c.block(grad.rows() + 1, 0, h.rows(), 1) = h;
        c.block(grad.rows() + h.rows() + 1, 0, g.rows(), 1) = g;

        VectorXf x = L.fullPivHouseholderQr().solve(c);
        d = x.block(0, 0, b.rows(), 1);
        lambda = x.block(b.rows() + 1, 0, h.rows(), 1);
        mu = x.block(b.rows() + h.rows() + 1, 0, g.rows(), 1);

        if ((d.array() == 0).all() && !((mu.array() >= 0).all())) {
            //Inactivity step
            int j = 0;
            float lowest = mu(0);
            for (int i = 1; i < mu.rows(); i++) {
                if (mu(i) < mu(j)) {
                    j = i;
                    lowest = mu(i);
                }
            }
            J.removeAt(j);

            G = ineqMatrix(b, J);
            g = ineqVector(b, J);

            L(A.rows() + H.rows() + G.rows(), A.cols() + H.cols() + G.cols());
            L.block(0, 0, A.rows(), A.cols()) = A;
            L.block(A.rows() + 1, 0, H.rows(), H.cols()) = H;
            L.block(0, A.cols() + 1, H.cols(), H.rows()) = H.transpose();
            L.block(A.rows() + H.rows() + 1, 0, G.rows(), G.cols()) = G;
            L.block(0, A.cols() + H.cols() + 1, G.cols(), G.rows()) = G.transpose();
            L.block(A.rows() + 1, A.cols() + 1, H.rows() + G.rows(), H.rows() + G.rows()).setZero();

            c(grad.rows() + h.rows() + g.rows());
            c.block(0 , 0, grad.rows(), 1) = -grad;
            c.block(grad.rows() + 1, 0, h.rows(), 1) = h;
            c.block(grad.rows() + h.rows() + 1, 0, g.rows(), 1) = g;

            x = L.fullPivHouseholderQr().solve(c);
            d = x.block(0, 0, b.rows(), 1);
        }


        //Powell step-size with merit function
        for (int i = 0; i < lambda.rows(); i++) {
            if (lambda(i) + gamma > alpha) {
                alpha = lambda(i) + gamma;
            }
        }
        for (int i = 0; i < mu.rows(); i++) {
            if (mu(i) + gamma > alpha) {
                alpha = mu(i) + gamma;
            }
        }

        float sigma = powell(b, S, d, alpha, 0.25, 0.75);

        //BFGS Update
        VectorXf s = (b + sigma * d) - b;
        VectorXf y = gradient(b + sigma * d, S) - grad;
        A = A - (((A*s)*(A*s).transpose())/(s.dot(A*s)) + y*y.transpose()/(y.dot(s)));

        b = b + sigma * d;
    }


    return b;
}

/**
 * @brief Model::powell Calculates efficient step-size with powell strategy
 * @param b current beta distribution
 * @param S internal mass distribution
 * @param d descend direction
 * @param alpha scaling factor for merit function
 * @param delta bias for powell step-size
 * @param beta bias for powell step-size
 * @return sigma
 */
float Model::powell(VectorXf b, MatrixXf S, VectorXf d, float alpha, float delta, float beta)
{
    float sigma = 1;
    float a1 = sigma;
    float a2 = sigma;

    float g1 = powellG1(sigma, b, S, d, alpha);
    float g2 = powellG2(sigma, b, S, d, alpha);

    if (g1 >= delta && g2 <= beta) {
        return sigma;
    }
    if (g1 >= delta && g2 > beta) {
        a1 = sigma;
        int l = 1;
        while (!(powellG1(a2, b, S, d, alpha) < delta)) {
            a2 = pow(2, l) * sigma;
            l += 1;
        }
    } else {
        if (g1 < delta && g2 <= beta) {
            a2 = sigma;
            int l = 1;
            while (!(powellG1(a1, b, S, d, alpha) >= delta && powellG2(a1, b, S, d, alpha) > beta)) {
                a1 = pow(2, -l) * sigma;
                l += 1;
            }
        }
    }

    while (!(g1 >= delta && g2 <= beta)) {
        sigma = 0.5 * (a1 + a2);

        g1 = powellG1(sigma, b, S, d, alpha);
        g2 = powellG2(sigma, b, S, d, alpha);

        if (g1 >= delta && g2 > beta) {
            a1 = sigma;
        } else {
             if (g1 < delta && g2 <= beta) {
                 a2 = sigma;
             }
        }
    }

    return sigma;
}

/**
 * @brief Model::powellG1 Auxilary function for powell strategy. Calculates g1(x).
 * @param sigma current step-size
 * @param b current beta distribution
 * @param S internal mass distribution
 * @param d descend direction
 * @param alpha scaling factor for merit function
 * @return g
 */
float Model::powellG1(float sigma, VectorXf b, MatrixXf S, VectorXf d, float alpha)
{
    float g = 1;
    if (sigma > 0) {
        g = (penalty(b + sigma * d, S, alpha) - penalty(b, S, alpha)) / (sigma * penaltyDirectionalGradient(b, S, d, alpha));
    }
    return g;
}

/**
 * @brief Model::powellG1 Auxilary function for powell strategy. Calculates g2(x).
 * @param sigma current step-size
 * @param b current beta distribution
 * @param S internal mass distribution
 * @param d descend direction
 * @param alpha scaling factor for merit function
 * @return
 */
float Model::powellG2(float sigma, VectorXf b, MatrixXf S, VectorXf d, float alpha)
{
    return penaltyDirectionalGradient(b + sigma * d, S, d, alpha) / penaltyDirectionalGradient(b, S, d, alpha);
}

/**
 * @brief Model::penalty Returns value of merit function.
 * @param b current beta distribution
 * @param S internal mass distribution
 * @param alpha scaling factor for merit function
 * @return
 */
float Model::penalty(VectorXf b, MatrixXf S, float alpha)
{
    float Ix = (Model::mesh_volume[8] - b.dot(S.col(8))) + (Model::mesh_volume[9] - b.dot(S.col(9)));
    float Iy = (Model::mesh_volume[7] - b.dot(S.col(7))) + (Model::mesh_volume[9] - b.dot(S.col(9)));
    float Iz = (Model::mesh_volume[7] - b.dot(S.col(7))) + (Model::mesh_volume[8] - b.dot(S.col(8)));
    float s1 = (Model::mesh_volume[3] - b.dot(S.col(3))) * (Model::mesh_volume[3] - b.dot(S.col(3)));
    float s2 = (Ix * Ix) / (Iz * Iz) + (Iy * Iy) / (Iz * Iz);

    VectorXf h(5);
    h(0) = abs(Model::mesh_volume[1] - b.dot(S.col(1)));
    h(1) = abs(Model::mesh_volume[2] - b.dot(S.col(2)));
    h(2) = abs(Model::mesh_volume[4] - b.dot(S.col(4)));
    h(3) = abs(Model::mesh_volume[5] - b.dot(S.col(5)));
    h(4) = abs(Model::mesh_volume[6] - b.dot(S.col(6)));

    VectorXf g(2 * b.rows());
    for (int i = 0; i < g.rows(); i++) {
        if (i % 2 == 0) {
            g(i) = (-b(i/2) + abs(-b(i/2))) / 2;

        }
        if (i % 2 == 1) {
            g(i) = ((b(i/2) - 1) + abs(b(i/2) - 1)) / 2;
        }
    }

    VectorXf ones = VectorXf::Ones(h.rows());
    return Model::w_c * s1 + Model::w_I * s2 + alpha * (ones.dot(h)) + alpha * (ones.dot(g));
}

/**
 * @brief Model::penaltyDirectionalGradient Returns value of directional derivate of merit function in direction d.
 * @param b current beta distribution
 * @param S internal mass distribution
 * @param d descent direction
 * @param alpha scaling factor for merit function
 * @return
 */
float Model::penaltyDirectionalGradient(VectorXf b, MatrixXf S,  VectorXf d, float alpha)
{
    float result = gradient(b,S).dot(d);

    float h = Model::mesh_volume[1] - b.dot(S.col(1));
    if (h > 0) {
       result += alpha * -S.col(1).dot(d);
    } else {
        if (h < 0) {
            result -= alpha * -S.col(1).dot(d);
        } else {
            if (h == 0) {
                result += alpha * (abs(-S.col(1).dot(d)));
            }
        }
    }
    h = Model::mesh_volume[2] - b.dot(S.col(2));
    if (h > 0) {
       result += alpha * -S.col(2).dot(d);
    } else {
        if (h < 0) {
            result -= alpha * -S.col(2).dot(d);
        } else {
            if (h == 0) {
                result += alpha * (abs(-S.col(2).dot(d)));
            }
        }
    }
    h = Model::mesh_volume[4] - b.dot(S.col(4));
    if (h > 0) {
       result += alpha * (-S.col(4).dot(d));
    } else {
        if (h < 0) {
            result -= alpha * (-S.col(4).dot(d));
        } else {
            if (h == 0) {
                result += alpha * (abs(-S.col(4).dot(d)));
            }
        }
    }
    h = Model::mesh_volume[5] - b.dot(S.col(5));
    if (h > 0) {
       result += alpha * (-S.col(5).dot(d));
    } else {
        if (h < 0) {
            result -= alpha * (-S.col(5).dot(d));
        } else {
            if (h == 0) {
                result += alpha * (abs(-S.col(5).dot(d)));
            }
        }
    }
    h = Model::mesh_volume[6] - b.dot(S.col(6));
    if (h > 0) {
       result += alpha * (-S.col(6).dot(d));
    } else {
        if (h < 0) {
            result -= alpha * (-S.col(6).dot(d));
        } else {
            if (h == 0) {
                result += alpha * (abs(-S.col(6).dot(d)));
            }
        }
    }

    float g = 0;
    for (int i = 0; i < 2 * b.rows(); i++) {
        if (i % 2 == 0) {
            g = -b(i / 2);
            if (g > 0 || (g == 0 && -d(i/2) > 0)) {
                result += alpha * -d(i/2);
            }
        } else {
            g = b(i / 2) - 1;
            if (g > 0 || (g == 0 && d(i/2) > 0)) {
                result += alpha * d(i/2);
            }
        }
    }

    return result;
}
