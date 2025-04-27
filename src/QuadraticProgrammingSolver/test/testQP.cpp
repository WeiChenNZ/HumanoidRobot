#include "SymbolicExpression.h"

using namespace std;
using namespace Eigen;

int main()
{
    // i = 0,1, dim = 3[x,y,z]
    //sumup((xi - 2)'*Q*(xi - 2) + (ui - 5)'*W*(ui - 5))
    //st. 1 <= ui <= 17
    //    A * xi = b

    //object
    VariablePtr x0_v = make_shared<DecisionVariable>("x0", 3);
    VariablePtr x1_v = make_shared<DecisionVariable>("x1", 3);
    VariablePtr u0_v = make_shared<DecisionVariable>("u0", 3);
    VariablePtr u1_v = make_shared<DecisionVariable>("u1", 3);
    ParameterPtr xr_p = make_shared<Parameter>("xr", MatrixXd::Constant(3,1,2.0));
    ParameterPtr ur_p = make_shared<Parameter>("ur", MatrixXd::Constant(3,1,5.0));

    //constraint
    ParameterPtr lu_p = make_shared<Parameter>("lu", MatrixXd::Constant(3,1,1.0));
    ParameterPtr uu_p = make_shared<Parameter>("uu", MatrixXd::Constant(3,1,17.0));
    ParameterPtr A_p = make_shared<Parameter>("A", MatrixXd::Constant(3,3,2.5));
    ParameterPtr b_p = make_shared<Parameter>("b", MatrixXd::Constant(3,1,3.5));

    //Symbol Expressions
    SymbolicExprPtr x0 = make_shared<DecisionVariableExpr>(x0_v);
    SymbolicExprPtr x1 = make_shared<DecisionVariableExpr>(x1_v);
    SymbolicExprPtr u0 = make_shared<DecisionVariableExpr>(u0_v);
    SymbolicExprPtr u1 = make_shared<DecisionVariableExpr>(u1_v);
    SymbolicExprPtr xr = make_shared<ParameterExpr>(xr_p);
    SymbolicExprPtr ur = make_shared<ParameterExpr>(ur_p);
    SymbolicExprPtr lu = make_shared<ParameterExpr>(lu_p);
    SymbolicExprPtr uu = make_shared<ParameterExpr>(uu_p);
    SymbolicExprPtr A = make_shared<ParameterExpr>(A_p);
    SymbolicExprPtr b = make_shared<ParameterExpr>(b_p);

    MatrixXd Q = MatrixXd::Identity(3,3);
    auto xr0 = quadForm(x0 - xr, Q);
    auto xr1 = quadForm(x1 - xr, Q);

    MatrixXd W = 3.0*MatrixXd::Identity(3,3);
    auto ur0 = quadForm(u0 - ur, W);
    auto ur1 = quadForm(u1 - ur, W);

    vector<SymbolicExprPtr> sum;
    sum.push_back(xr0 + ur0);
    sum.push_back(xr1 + ur1);

    auto object = sumup(sum);

    vector<Constraint> constraints;
    constraints.push_back(u0 >= lu);
    constraints.push_back(u0 <= uu);
    constraints.push_back(u1 >= lu);
    constraints.push_back(u1 <= uu);
    constraints.push_back(A*x0 == b);
    constraints.push_back(A*x1 == b);

    Problem problem(object, constraints, Problem::MINIMIZE);

    cout<<"Q = "<<endl<<problem.Q()<<endl;
    cout<<"p = "<<endl<<problem.p()<<endl;
    cout<<"A = "<<endl<<problem.A()<<endl;
    cout<<"l = "<<endl<<problem.l()<<endl;
    cout<<"u = "<<endl<<problem.u()<<endl;


    return 0;
}