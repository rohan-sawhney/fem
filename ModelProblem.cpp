#include "ModelProblem.h"
#define BIG 1e50

ModelProblem::ModelProblem(Mesh *mesh_):
Fem(mesh_)
{
    
}

void ModelProblem::computeElement(const vector<Vector2d>& x, MatrixXd& ae, VectorXd& fe) const
{
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
        
        // compute weight, source term and coefficients
        double w = det*gWi[i];
        double f; modelData->fRhs(xx, f);
        double b; modelData->beta(xx, b);
        Matrix2d a; modelData->alpha(xx, a);
        
        // compute element matrices
        for (int j = 0; j < eNi; j++) {
            fe(j) += f*phi[i](j)*w;
            for (int k = 0; k < eNi; k++) {
                ae(j, k) += (dphix[j].dot(a*dphix[k]) + b*phi[i](j)*phi[i](k))*w;
            }
        }
    }
}

void ModelProblem::computeDirichletData(const vector<Vector2d>& x, MatrixXd& ae, VectorXd& fe) const
{
    for (int i = 0; i < eNb; i++) {
        double u0; solution->exact(x[i], u0);
        fe(i) += BIG*u0;
        ae(i, i) += BIG;
    }
}

void ModelProblem::computeNeumannData(const vector<Vector2d>& x, VectorXd& fe) const
{
    for (int i = 0; i < gNb; i++) {
        // compute vertex position and its derivative
        Vector2d xx = Vector2d::Zero();
        Vector2d dx = Vector2d::Zero();
        for (int j = 0; j < eNb; j++) {
            xx += phib[i](j)*x[j];
            dx += dphib[i](j)*x[j];
        }
        
        // compute normal derivative
        double dxds = dx.norm();
        Vector2d n = Vector2d(dx(1), -dx(0))/dxds;
        Vector2d dudx; solution->dExact(xx, dudx);
        double dudn = dudx.dot(n);
        
        // add neumann data to source term
        double w = dxds*gWb[i];
        for (int j = 0; j < eNb; j++) {
            fe(j) += phib[i](j)*dudn*w;
        }
    }
}

void ModelProblem::computeBdyElement(int index, const vector<Vector2d>& x, MatrixXd& ae, VectorXd& fe) const
{
    if (modelData->bdyType(index) == DIRICHLET) computeDirichletData(x, ae, fe);
    else computeNeumannData(x, fe);
}

const VectorXd& ModelProblem::solve(int eType_, Solution *solution_, ModelData *modelData_)
{
    modelData = modelData_;
    return Fem::solve(eType_, solution_);
}
