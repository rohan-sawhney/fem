#include "Fem.h"
#include <Eigen/SparseCholesky>

Fem::Fem(Mesh *mesh_):
mesh(mesh_),
gNi(7),
gNb(3),
phi(gNi),
dphi(gNi),
phib(gNb),
dphib(gNb)
{
    // set gauss weights
    double w1 = 31.0/480.0 + sqrt(15.0)/2400.0;
    double w2 = 31.0/480.0 - sqrt(15.0)/2400.0;
    gWi = {0.1125, w1, w1, w1, w2, w2, w2};
    gWb = {5.0/9.0, 8.0/9.0, 5.0/9.0};
}

void p1Triangle(double eta, double ata, VectorXd& phi, MatrixXd& dphi)
{
    phi.resize(3);
    phi(0) = eta;
    phi(1) = ata;
    phi(2) = 1.0 - eta - ata;
    
    dphi.resize(3, 2);
    dphi(0, 0) =  1.0;
    dphi(1, 0) =  0.0;
    dphi(2, 0) = -1.0;
    
    dphi(0, 1) =  0.0;
    dphi(1, 1) =  1.0;
    dphi(2, 1) = -1.0;
}

void p1Line(double eta, VectorXd& phib, VectorXd& dphib)
{
    phib.resize(2);
    phib(0) = 0.5*(1.0 - eta);
    phib(1) = 0.5*(1.0 + eta);
    
    dphib.resize(2);
    dphib(0) = -0.5;
    dphib(1) =  0.5;
}

void Fem::computeLinearBasis()
{
    // compute 2d basis functions using the 7 Point Gauss Rule on a triangle
    double b1 = 2.0/7.0 + sqrt(15.0)/21.0, a1 = 1 - 2*b1;
    double b2 = 2.0/7.0 - sqrt(15.0)/21.0, a2 = 1 - 2*b2;
    
    double eta = 1.0/3.0, ata = 1.0/3.0;
    p1Triangle(eta, ata, phi[0], dphi[0]);
    for (int i = 1; i <= 3; i++) {
        if (i == 1) eta = a1; else eta = b1;
        if (i == 2) ata = a1; else ata = b1;
        p1Triangle(eta, ata, phi[i], dphi[i]);
        
        if (i == 1) eta = a2; else eta = b2;
        if (i == 2) ata = a2; else ata = b2;
        p1Triangle(eta, ata, phi[i+3], dphi[i+3]);
    }
    
    // compute 1d basis functions at the gauss points
    vector<double> gPb = {-sqrt(3.0/5.0), 0.0, sqrt(3.0/5.0)};
    for (int i = 0; i < gNb; i++) p1Line(gPb[i], phib[i], dphib[i]);
}

void p2Triangle(double eta, double ata, VectorXd& phi, MatrixXd& dphi)
{
    phi.resize(6);
    phi(0) = 2.0*eta*(eta - 0.5);
    phi(1) = 2.0*ata*(ata - 0.5);
    phi(2) = 2.0*(1.0-eta-ata)*((1.0-eta-ata) - 0.5);
    phi(3) = 4.0*eta*ata;
    phi(4) = 4.0*ata*(1.0-eta-ata);
    phi(5) = 4.0*(1.0-eta-ata)*eta;
    
    dphi.resize(6, 2);
    dphi(0, 0) =  4.0*eta - 1.0;
    dphi(1, 0) =  0.0;
    dphi(2, 0) =  4.0*ata + 4.0*eta - 3.0;
    dphi(3, 0) =  4.0*ata;
    dphi(4, 0) = -4.0*ata;
    dphi(5, 0) = -4.0*(ata + 2.0*eta - 1.0);
    
    dphi(0, 1) =  0.0;
    dphi(1, 1) =  4.0*ata - 1.0;
    dphi(2, 1) =  4.0*eta + 4.0*ata - 3.0;
    dphi(3, 1) =  4.0*eta;
    dphi(4, 1) = -4.0*(eta + 2.0*ata - 1.0);
    dphi(5, 1) = -4.0*eta;
}

void p2Line(double eta, VectorXd& phib, VectorXd& dphib)
{
    phib.resize(3);
    phib(0) = 0.5*eta*(eta - 1.0);
    phib(1) = 0.5*eta*(eta + 1.0);
    phib(2) = 1.0 - eta*eta;
    
    dphib.resize(3);
    dphib(0) = eta - 0.5;
    dphib(1) = eta + 0.5;
    dphib(2) = -2.0*eta;
}

void Fem::computeQuadraticBasis()
{
    // compute 2d basis functions using the 7 Point Gauss Rule on a triangle
    double b1 = 2.0/7.0 + sqrt(15.0)/21.0, a1 = 1 - 2*b1;
    double b2 = 2.0/7.0 - sqrt(15.0)/21.0, a2 = 1 - 2*b2;
    
    double eta = 1.0/3.0, ata = 1.0/3.0;
    p2Triangle(eta, ata, phi[0], dphi[0]);
    for (int i = 1; i <= 3; i++) {
        if (i == 1) eta = a1; else eta = b1;
        if (i == 2) ata = a1; else ata = b1;
        p2Triangle(eta, ata, phi[i], dphi[i]);
        
        if (i == 1) eta = a2; else eta = b2;
        if (i == 2) ata = a2; else ata = b2;
        p2Triangle(eta, ata, phi[i+3], dphi[i+3]);
    }
    
    // compute 1d basis functions at the gauss points
    vector<double> gPb = {-sqrt(3.0/5.0), 0.0, sqrt(3.0/5.0)};
    for (int i = 0; i < gNb; i++) p2Line(gPb[i], phib[i], dphib[i]);
}

bool Fem::computeJacobian(int g, const vector<Vector2d>& x, Vector2d& xx,
                          Matrix2d& dsdx, double& det) const
{
    // compute transform matrix
    Matrix2d dxds = Matrix2d::Zero();
    for (int j = 0; j < eNi; j++) {
        xx += phi[g](j)*x[j];
        dxds += x[j]*dphi[g].row(j);
    }
    
    // compute inverse and determinant
    dsdx = dxds.inverse();
    det = dxds.determinant();
    if (det <= 0.0) {
        cerr << "Error: Negative Jacobian" << endl;
        return false;
    }
    
    return true;
}

void Fem::assemble()
{
    int N = (int)mesh->vertices.size();
    int offset = 0;
    if (eType == QUADRATIC) {
        offset = N;
        N += (int)mesh->edges.size();
    }
    
    // initialize global matrices
    F = VectorXd::Zero(N);
    A.resize(N, N);
    vector<Triplet<double>> triplets;
    
    // compute interior element matrices
    for (FaceCIter f = mesh->faces.begin(); f != mesh->faces.end(); f++) {
        if (!f->isBoundary()) {
            // compute element matrices
            vector<Vector2d> x(eNi);
            vector<int> index(eNi);
            MatrixXd ae = MatrixXd::Zero(eNi, eNi);
            VectorXd fe = VectorXd::Zero(eNi);
            f->vertexData(x, index, offset);
            computeElement(x, ae, fe);
            
            // insert element matrices into global matrices
            for (int i = 0; i < eNi; i++) {
                F(index[i]) += fe(i);
                for (int j = 0; j < eNi; j++) {
                    triplets.push_back(Triplet<double>(index[i], index[j], ae(i, j)));
                }
            }
        }
    }
            
    // compute boundary element matrices
    for (int b = 0; b < (int)mesh->boundaries.size(); b++) {
        HalfEdgeCIter he = mesh->boundaries[b];
        HalfEdgeCIter h = he;
        do {
            // compute element matrices
            vector<Vector2d> x(eNb);
            vector<int> index(eNb);
            MatrixXd ae = MatrixXd::Zero(eNb, eNb);
            VectorXd fe = VectorXd::Zero(eNb);
            h->vertexData(x, index, offset);
            computeBdyElement(h->edge->index, x, ae, fe);
            
            // insert element matrices into global matrices
            for (int i = 0; i < eNb; i++) {
                F(index[i]) += fe(i);
                for (int j = 0; j < eNb; j++) {
                    triplets.push_back(Triplet<double>(index[i], index[j], ae(i, j)));
                }
            }
            
            h = h->next;
        } while (h != he);
    }
    
    A.setFromTriplets(triplets.begin(), triplets.end());
}

void Fem::printNorms() const
{
    Vector3d norm = Vector3d::Zero();
    Vector3d dnorm = Vector3d::Zero();
    
    for (FaceCIter f = mesh->faces.begin(); f != mesh->faces.end(); f++) {
        if (!f->isBoundary()) {
            vector<Vector2d> x(eNi);
            vector<int> index(eNi);
            f->vertexData(x, index, eType == QUADRATIC ? (int)mesh->vertices.size() : 0);
            
            for (int i = 0; i < gNi; i++) {
                // compute transformed basis data
                Vector2d xx = Vector2d::Zero();
                Matrix2d dsdx;
                double det;
                if (!computeJacobian(i, x, xx, dsdx, det)) exit(0);
                
                // compute derivatives of basis functions on element
                vector<Vector2d> dphix(eNi);
                for (int j = 0; j < eNi; j++) {
                    dphix[j] = dphi[i].row(j)*dsdx;
                }
                
                // compute exact solution
                double u0; solution->exact(xx, u0);
                Vector2d dudx0; solution->dExact(xx, dudx0);
                
                // compute fem solution
                double uh = 0.0;
                Vector2d dudxh = Vector2d::Zero();
                for (int j = 0; j < eNi; j++) {
                    uh += phi[i](j)*u(index[j]);
                    dudxh += dphix[j]*u(index[j]);
                }
                
                // compute norms
                double w = det*gWi[i];
                norm(0) += u0*u0*w;
                norm(1) += uh*uh*w;
                norm(2) += (u0 - uh)*(u0 - uh)*w;
                dnorm(0) += dudx0.squaredNorm()*w;
                dnorm(1) += dudxh.squaredNorm()*w;
                dnorm(2) += (dudx0 - dudxh).squaredNorm()*w;
            }
        }
    }
    
    Vector3d hnorm;
    for (int i = 0; i < 3; i++) {
        hnorm(i) = sqrt(norm(i) + dnorm(i));
        norm(i) = sqrt(norm(i));
        dnorm(i) = sqrt(dnorm(i));
    }
    
    // print
    printf("\n                    L-2 norm        H-1 semi-norm       H-1  norm\n");
    printf("Exact Solution     %12.6e     %12.6e      %12.6e\n", norm(0), dnorm(0), hnorm(0));
    printf("FEM   Solution     %12.6e     %12.6e      %12.6e\n", norm(1), dnorm(1), hnorm(1));
    printf("Errors             %12.6e     %12.6e      %12.6e\n", norm(2), dnorm(2), hnorm(2));
}

const VectorXd& Fem::solve(int eType_, Solution *solution_)
{
    // compute basis
    eType = eType_;
    if (eType == LINEAR) {
        eNi = 3;
        eNb = 2;
        computeLinearBasis();
    
    } else {
        eNi = 6;
        eNb = 3;
        computeQuadraticBasis();
    }
    
    solution = solution_;
    
    // assemble
    assemble();
    
    // solve
    SimplicialCholesky<SparseMatrix<double>> solver(A);
    u = solver.solve(F);

    // print norms
    printNorms();

    return u;
}
