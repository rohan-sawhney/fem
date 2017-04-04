#ifndef FEM_H
#define FEM_H

#include "Types.h"
#include "Mesh.h"

#define LINEAR 0
#define QUADRATIC 1

struct Solution;

class Fem {
public:
    // constructor
    Fem(Mesh *mesh_);
    
    // solve
    const VectorXd& solve(int eType_, Solution *solution_);
    
protected:
    // computes linear basis
    void computeLinearBasis();
    
    // computes quadratic basis
    void computeQuadraticBasis();
    
    // computes jacobian
    bool computeJacobian(int g, const vector<Vector2d>& x, Vector2d& xx,
                         Matrix2d& dsdx, double& det) const;
    
    // computes element entries ae and ee
    virtual void computeElement(const vector<Vector2d>& x, MatrixXd& ae, VectorXd& fe) const = 0;
    
    // computes dirichlet boundary data
    virtual void computeDirichletData(const vector<Vector2d>& x, MatrixXd& ae, VectorXd& fe) const = 0;
    
    // computes neumann boundary data
    virtual void computeNeumannData(const vector<Vector2d>& x, VectorXd& fe) const = 0;
    
    // computes boundary element entries
    virtual void computeBdyElement(int index, const vector<Vector2d>& x,
                                   MatrixXd& ae, VectorXd& fe) const = 0;
    
    // assemble
    void assemble();
    
    // print norms
    void printNorms() const;
    
    // member variables
    Mesh *mesh;
    int gNi, gNb, eNi, eNb, eType;
    vector<double> gWi, gWb;
    vector<VectorXd> phi, phib, dphib;
    vector<MatrixXd> dphi;
    SparseMatrix<double> A;
    VectorXd F, u;
    Solution *solution;
};

struct Solution {
    // exact solution and derivative
    function<void(const Vector2d&, double&)> exact;
    function<void(const Vector2d&, Vector2d&)> dExact;
};

#endif
