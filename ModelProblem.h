#ifndef MODEL_2D_H
#define MODEL_2D_H

#include "Fem.h"
#define DIRICHLET 0
#define NEUMANN 1

struct ModelData;

// -Δu + Bu = f
class ModelProblem: Fem {
public:
    // constructor
    ModelProblem(Mesh *mesh_);
    
    // solve
    const VectorXd& solve(int eType_, Solution *solution_, ModelData *modelData_);
    
protected:
    // computes element entries ae and ee
    void computeElement(const vector<Vector2d>& x, MatrixXd& ae, VectorXd& fe) const;
    
    // computes dirichlet boundary data
    void computeDirichletData(const vector<Vector2d>& x, MatrixXd& ae, VectorXd& fe) const;
    
    // computes neumann boundary data
    void computeNeumannData(const vector<Vector2d>& x, VectorXd& fe) const;
    
    // computes boundary element entries
    void computeBdyElement(int index, const vector<Vector2d>& x, MatrixXd& ae, VectorXd& fe) const;
    
    // member variable
    ModelData *modelData;
};

struct ModelData {
    // boundary data type
    function<int(int)> bdyType;
    
    // rhs and coefficients
    function<void(const Vector2d&, double&)> fRhs;
    function<void(const Vector2d&, double&)> beta;
    function<void(const Vector2d&, Matrix2d&)> alpha;
};

#endif
