#include "SymbolicExpression.h"


using namespace std;
using namespace Eigen;

void Problem::collectVariables(SymbolicExprPtr expr)
{
    if(auto var = dynamic_pointer_cast<DecisionVariableExpr>(expr))
    {
        //if var is not in the variables
        if(find(variables_.begin(), variables_.end(), var->variable()) == variables_.end())
        {
            variables_.push_back(var->variable());
        }
    }
    else if(auto binary = dynamic_pointer_cast<BinaryExpr>(expr))
    {
        collectVariables(binary->lhs());
        collectVariables(binary->rhs());
    }
    else if(auto quard = dynamic_pointer_cast<QuadExpr>(expr))
    {
        collectVariables(quard->x());
    }
    else if(auto dot = dynamic_pointer_cast<DotExpr>(expr))
    {
        collectVariables(dot->lhs());
        collectVariables(dot->rhs());
    }
    else if(auto sumup = dynamic_pointer_cast<SumUpExpr>(expr))
    {
        for(auto& element: sumup->terms())
        {
            collectVariables(element);
        }
    }
}

void Problem::assignVaribalesIndeces()
{
    int offset = 0;
    for(const auto& v: variables_)
    {
        variableOffsets_[v->name()] = offset;
        offset += v->dim();
    }

    variablesDim_ = offset;
}

Eigen::MatrixXd Problem::buildMatrixQ()
{
    int n = totalVariablesDim();
    Q_ = Eigen::MatrixXd::Zero(n,n);

    auto quadBuildQ = [&](const QuadExpr* qf)
    {
        if(auto v = dynamic_pointer_cast<DecisionVariableExpr>(qf->x()))
        {
            int offset = variableOffset(v->variable()->name());
            Q_.block(offset, offset, qf->Q().rows(), qf->Q().cols()) += qf->Q();
        }
        else if(auto bin = dynamic_pointer_cast<BinaryExpr>(qf->x()))
        {
            if(auto v1 = dynamic_pointer_cast<DecisionVariableExpr>(bin->lhs()))
            {
                // (x-xr)'Q(x-xr)
                int offset = variableOffset(v1->variable()->name());
                Q_.block(offset, offset, qf->Q().rows(), qf->Q().cols()) += qf->Q();
            }
            else if(auto dot = dynamic_pointer_cast<DotExpr>(bin->lhs()))
            {
                // (Ax-xr)'Q(Ax-xr)
                if(auto var = dynamic_pointer_cast<DecisionVariableExpr>(dot->rhs()))
                {
                    auto param = dynamic_pointer_cast<ParameterExpr>(dot->lhs());
                    int offset = variableOffset(var->variable()->name());
                    Q_.block(offset, offset, qf->Q().rows(), qf->Q().cols()) += param->parameter()->value().transpose() * qf->Q() * param->parameter()->value();
                }
            }
        }
    };

    function<void(const SymbolicExprPtr&)> searchExpr = [&](const SymbolicExprPtr& expr)
    {
        if(auto e = dynamic_pointer_cast<QuadExpr>(expr))
        {
            quadBuildQ(e.get());
        }
        else if(auto sumup = dynamic_pointer_cast<SumUpExpr>(expr))
        {
            for(auto& e: sumup->terms())
            {
                searchExpr(e);
            }
        }
        else if(auto bin = dynamic_pointer_cast<BinaryExpr>(expr))
        {
            searchExpr(bin->lhs());
            searchExpr(bin->rhs());
        }
    };

    searchExpr(object_);
}

Eigen::VectorXd Problem::buildVectorP()
{
    int n = totalVariablesDim();
    p_ = Eigen::VectorXd::Zero(n);

    auto quadBuildP = [&](const QuadExpr* qf)
    {
        if(auto bin = dynamic_pointer_cast<BinaryExpr>(qf->x()))
        {
            if(auto xi = dynamic_pointer_cast<DecisionVariableExpr>(bin->lhs()))
            {
                // (x-xr)'Q(x-xr)
                if(auto xr = dynamic_pointer_cast<ParameterExpr>(bin->rhs()))
                {
                    int offset = variableOffset(xi->variable()->name());
                    p_.segment(offset, xi->variable()->dim()) -= 2.0 * qf->Q() * xr->parameter()->value();
                }
                
            }
            else if(auto dot = dynamic_pointer_cast<DotExpr>(bin->lhs()))
            {
                // (Ax-xr)'Q(Ax-xr)
                if(auto var = dynamic_pointer_cast<DecisionVariableExpr>(dot->rhs()))
                {
                    auto param = dynamic_pointer_cast<ParameterExpr>(dot->lhs());
                    auto xr = dynamic_pointer_cast<ParameterExpr>(bin->rhs());
                    int offset = variableOffset(var->variable()->name());
                    p_.segment(offset, var->variable()->dim()) -= 2.0* param->parameter()->value().transpose() * qf->Q() * xr->parameter()->value();
                }
            }
        }
    };

    function<void(const SymbolicExprPtr&)> searchExpr = [&](const SymbolicExprPtr& expr)
    {
        if(auto e = dynamic_pointer_cast<QuadExpr>(expr))
        {
            quadBuildP(e.get());
        }
        else if(auto sumup = dynamic_pointer_cast<SumUpExpr>(expr))
        {
            for(auto& e: sumup->terms())
            {
                searchExpr(e);
            }
        }
        else if(auto bin = dynamic_pointer_cast<BinaryExpr>(expr))
        {
            searchExpr(bin->lhs());
            searchExpr(bin->rhs());
        }
    };

    searchExpr(object_);
}

Eigen::MatrixXd Problem::buildMatrixA()
{
    int totalRows = 0;
    for(const auto& c: constraints_)
    {
        auto lhs = decodeLinearExpr(c.lhs());
        totalRows += lhs.bias.rows();
    }

    int n = totalVariablesDim();
    MatrixXd A(totalRows, n);

    int row = 0;
    for(const auto& c: constraints_)
    {
        auto lhs = decodeLinearExpr(c.lhs());
        int rows = lhs.bias.rows();
        A.block(row, 0, rows, n) = lhs.coeffs;
        row += rows;
    }

    return A;
}

Eigen::VectorXd Problem::buildVectorL()
{
    int totalRows = 0;
    for(const auto& c: constraints_)
    {
        auto lhs = decodeLinearExpr(c.lhs());
        totalRows += lhs.bias.rows();
    }

    VectorXd l(totalRows);
    int row = 0;

    for(const auto& c: constraints_)
    {
        auto lhs = decodeLinearExpr(c.lhs());
        auto rhs = decodeLinearExpr(c.rhs());
        int rows = lhs.bias.rows();

        switch (c.operation())
        {
            case Constraint::EQUAL:
                l.segment(row, rows) = rhs.bias;
                break;
            case Constraint::GEQ:
                l.segment(row, rows) = rhs.bias;
                break;
            case Constraint::LEQ:
                l.segment(row, rows) = VectorXd::Constant(rows, -numeric_limits<double>::infinity());
                break;    
        }
        row += rows;
    }

    return l;
}

Eigen::VectorXd Problem::buildVectorU()
{
    int totalRows = 0;
    for(const auto& c: constraints_)
    {
        auto lhs = decodeLinearExpr(c.lhs());
        totalRows += lhs.bias.rows();
    }

    VectorXd u(totalRows);
    int row = 0;

    for(const auto& c: constraints_)
    {
        auto lhs = decodeLinearExpr(c.lhs());
        auto rhs = decodeLinearExpr(c.rhs());
        int rows = lhs.bias.rows();

        switch (c.operation())
        {
            case Constraint::EQUAL:
                u.segment(row, rows) = rhs.bias;
                break;
            case Constraint::GEQ:
                u.segment(row, rows) = VectorXd::Constant(rows, numeric_limits<double>::infinity());
                break;
            case Constraint::LEQ:
                u.segment(row, rows) = rhs.bias;
                break;    
        }
        row += rows;
    }

    return u;  
}

LinearizedExpr Problem::decodeLinearExpr(const SymbolicExprPtr& expr)
{
    //decode a linear expression and get its coeffs and bias
    //eg. Ax+b --> coeffs = A, bias = b
    if(auto var = dynamic_pointer_cast<DecisionVariableExpr>(expr))
    {
        //case 1: f = x
        int offset = variableOffset(var->variable()->name());
        int dim = var->variable()->dim();
        MatrixXd coeffs = MatrixXd::Zero(dim, totalVariablesDim());
        coeffs.block(0,offset,dim,dim) = MatrixXd::Identity(dim,dim);
        return{coeffs, VectorXd::Zero(dim)};
    }
    else if(auto bin = dynamic_pointer_cast<BinaryExpr>(expr))
    {
        auto lhs = decodeLinearExpr(bin->lhs());
        auto rhs = decodeLinearExpr(bin->rhs());
        if(bin->operation() == BinaryExpr::ADD)
        {
            return {lhs.coeffs + rhs.coeffs, lhs.bias + rhs.bias};
        }
        else
        {
            return {lhs.coeffs - rhs.coeffs, lhs.bias - rhs.bias};
        }
    }
    else if(auto dot = dynamic_pointer_cast<DotExpr>(expr))
    {
        auto lhs = dot->lhs();
        auto rhs = dot->rhs();

        auto mat = dynamic_pointer_cast<ParameterExpr>(lhs);
        auto var = dynamic_pointer_cast<DecisionVariableExpr>(rhs);

        if(mat && var)
        {
            MatrixXd A = mat->parameter()->value();
            int offset = variableOffset(var->variable()->name());
            int dim = var->variable()->dim();

            MatrixXd coeffs = MatrixXd::Zero(A.rows(), totalVariablesDim());
            coeffs.block(0,offset, A.rows(), dim) = A;

            return {coeffs, VectorXd::Zero(A.rows())};
        }

    }

    throw runtime_error("Unsupported expression in linear funciton!");
}


//updata object function at the runtime
void Problem::updateObjectFunction(SymbolicExprPtr object)
{
    object_ = object;
    //assume that the decision variables are not changed
    //so only need to update Q,P matrices instead of update indeces and sizes
    buildMatrixQ();
    buildVectorP();
}

//update constraits at the runtime
void Problem::updateConstraints(std::vector<Constraint>& constraints)
{
    constraints_ = constraints;
    //assume that the decision variables are not changed
    //so only need to update A,l,u matrices instead of update indeces and sizes
    buildMatrixA();
    buildVectorL();
    buildVectorU();
}

