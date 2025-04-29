#include "SymbolicExpression.h"

using namespace std;
using namespace Eigen;

int main()
{
    // // i = 0,1, dim = 3[x,y,z]
    // //sumup((xi - 2)'*Q*(xi - 2) + (ui - 5)'*W*(ui - 5))
    // //st. 1 <= ui <= 17
    // //    A * xi = b

    // //Symbol Expressions
    // SymbolicExprPtr x0 = make_shared<DecisionVariableExpr>("x0", 3);
    // SymbolicExprPtr x1 = make_shared<DecisionVariableExpr>("x1", 3);
    // SymbolicExprPtr u0 = make_shared<DecisionVariableExpr>("u0", 3);
    // SymbolicExprPtr u1 = make_shared<DecisionVariableExpr>("u1", 3);
    // SymbolicExprPtr A0 = make_shared<ParameterExpr>("A0", 4.*MatrixXd::Identity(3,3));
    // SymbolicExprPtr xr = make_shared<ParameterExpr>("xr", VectorXd::Constant(3, 2.0).matrix());
    // SymbolicExprPtr ur = make_shared<ParameterExpr>("ur", VectorXd::Constant(3, 5.0).matrix());
    // SymbolicExprPtr lu = make_shared<ParameterExpr>("lu", VectorXd::Constant(3,1.0).matrix());
    // SymbolicExprPtr uu = make_shared<ParameterExpr>("uu", VectorXd::Constant(3,17.0).matrix());
    // SymbolicExprPtr A = make_shared<ParameterExpr>("A", 1.5*MatrixXd::Identity(3,3));
    // SymbolicExprPtr b = make_shared<ParameterExpr>("b", VectorXd::Constant(3, 3.5).matrix());

    // MatrixXd Q = MatrixXd::Identity(3,3);
    // auto xr0 = quadForm(x0 - xr, Q);
    // auto xr1 = quadForm(x1 - xr, Q);

    // MatrixXd W = 3.0*MatrixXd::Identity(3,3);
    // auto ur0 = quadForm(u0 - ur, W);
    // auto ur1 = quadForm(u1 - ur, W);

    // vector<SymbolicExprPtr> sum;
    // sum.push_back(xr0 + ur0);
    // sum.push_back(xr1 + ur1);

    // auto object = sumup(sum);

    // vector<Constraint> constraints;
    // constraints.push_back(constraintRange(u0, lu, uu));
    // constraints.push_back(constraintRange(u1, lu, uu));
    // constraints.push_back(A*x0 == b);
    // constraints.push_back(A*x1 == b);

    // Problem problem(object, constraints, Problem::MINIMIZE);

    // cout<<"P = "<<endl<<problem.P()<<endl;
    // cout<<"q = "<<endl<<problem.q()<<endl;
    // cout<<"A = "<<endl<<problem.A()<<endl;
    // cout<<"l = "<<endl<<problem.l()<<endl;
    // cout<<"u = "<<endl<<problem.u()<<endl;

    // xr->updateValue(VectorXd::Constant(3, 9.).matrix());
    // ur->updateValue(VectorXd::Constant(3,10.).matrix());
    // lu->updateValue(VectorXd::Constant(3, 13.).matrix());
    // uu->updateValue(VectorXd::Constant(3,21.).matrix());
    // A->updateValue(6.6*MatrixXd::Identity(3,3));
    // b->updateValue(VectorXd::Constant(3, 7.7).matrix());
    // problem.updateObjectFunction(object);
    // problem.updateConstraints(constraints);
    // cout<<"================================================"<<endl;
    // cout<<"P = "<<endl<<problem.P()<<endl;
    // cout<<"q = "<<endl<<problem.q()<<endl;
    // cout<<"A = "<<endl<<problem.A()<<endl;
    // cout<<"l = "<<endl<<problem.l()<<endl;
    // cout<<"u = "<<endl<<problem.u()<<endl;

    //Symbol Expressions
    SymbolicExprPtr x0 = make_shared<DecisionVariableExpr>("x0", 2);
    SymbolicExprPtr u0 = make_shared<DecisionVariableExpr>("u0", 2);
    SymbolicExprPtr xr = make_shared<ParameterExpr>("xr", Vector2d(1,2));
    SymbolicExprPtr ur = make_shared<ParameterExpr>("ur", Vector2d(0,0));
    SymbolicExprPtr lu = make_shared<ParameterExpr>("lu", VectorXd::Constant(2,0.0).matrix());
    SymbolicExprPtr uu = make_shared<ParameterExpr>("uu", VectorXd::Constant(2,1.0).matrix());
    MatrixXd Am(2,2);
    Am<< 1,1,-1,2;
    Vector2d bm(2,2);
    SymbolicExprPtr A = make_shared<ParameterExpr>("A", Am);
    SymbolicExprPtr b = make_shared<ParameterExpr>("b", bm);

    MatrixXd W1 = 2. * MatrixXd::Identity(2,2);
    auto xr0 = quadForm(x0 - xr, W1);

    MatrixXd W2 = MatrixXd::Identity(2,2);
    auto ur0 = quadForm(u0 - ur, W2);

    vector<SymbolicExprPtr> sum;
    sum.push_back(xr0 + ur0);

    auto object = sumup(sum);

    vector<Constraint> constraints;
    constraints.push_back(constraintRange(u0, lu, uu));
    constraints.push_back(A*x0 <= b);

    Problem problem(object, constraints, Problem::MINIMIZE);

    cout<<"P = "<<endl<<problem.P()<<endl;
    cout<<"q = "<<endl<<problem.q()<<endl;
    cout<<"A = "<<endl<<problem.A()<<endl;
    cout<<"l = "<<endl<<problem.l()<<endl;
    cout<<"u = "<<endl<<problem.u()<<endl;

    VectorXd result;
    if(problem.solve())
        result = problem.getSolution();

    cout<<"result = "<<endl<<result<<endl;

    //solution
    //[0.4,1.6,0,0]

    return 0;
}