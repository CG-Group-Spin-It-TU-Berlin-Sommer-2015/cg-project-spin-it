#include "model.h"

float Model::p = 1.07;
QVector3D Model::cp = null;

float* Model::mesh_volumne = 0;

float Model::spinability(float w_i, float w_c, float* volume)
{
    float M = volume[0];

    QVector3D c;
    c.setX(1/M*volume[1]);
    c.setY(1/M*volume[2]);
    c.setZ(1/M*volume[3]);

    float I[3][3] = {{volume[8] + volume[9], -volume[4], -volume[6]},
                   {-volume[4], volume[7] + volume[9], -volume[5]},
                   {-volume[6], -volume[5], volume[7] + volume[8]}};

    f_yoyo = w_i * ((I[1][1]/I[3][3])*(I[1][1]/I[3][3]) + (I[2][2]/I[3][3])*(I[2][2]/I[3][3]));
    f_top  = w_c * ((c-cp).length()*M)*((c-cp).length()*M) + f_yoyo;

    return f_top;
}

void Model::initialize(Mesh* mesh)
{
    mesh_volumne = calculateVolumne(mesh, p);
    cout << "Volumnes:" << endl;
    for (int i = 0; i < 10; i++) {
        cout << mesh_volumne[i] << endl;
    }
    Model::spinability(0,0,mesh_volumne);
}

float Model::calculateMass(float* volumne)
{
    return volumne[0];
}

float* Model::calculateVolumne(Mesh* mesh, float p)
{
    float* s = new float[10];
    for (int i = 0; i < 10; i++) {
        s[i] = 0;
    }
    QVector<float>* geometry = mesh->getGeometry();
    QVector<short>* indices = mesh->getIndices();

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
        c.setY(geometry->at(3 * indices->at(i + 2) + 2));

        QVector3D u = b - a;
        QVector3D v = c - a;
        QVector3D n = QVector3D::crossProduct(u, v);

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

