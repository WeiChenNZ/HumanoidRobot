#include "SymbolicExpression.h"

using namespace std;
using namespace Eigen;

int main()
{
    VariablePtr x = make_shared<DecisionVariable>("x", 3);
    SymbolicExprPtr X = make_shared<DecisionVariableExpr>(x);

    ParameterPtr zeroVect = make_shared<Parameter>("0", VectorXd::Zero(x->dim()));
    SymbolicExprPtr ZERO = make_shared<ParameterExpr>(zeroVect);

    auto c = X + ZERO;
    auto d = X*ZERO;

    auto e = dynamic_pointer_cast<DotExpr>(d);
    auto lhs = dynamic_pointer_cast<DecisionVariableExpr>(e->lhs());
    auto rhs = dynamic_pointer_cast<ParameterExpr>(e->rhs());
    cout<<lhs->variable()->name()<<endl
        <<rhs->parameter()->name()<<endl;

    return 0;
}