#include "SymbolicExpression.h"
#include <stdlib.h>
#include "OsqpEigen/OsqpEigen.h"



using namespace std;
using namespace Eigen;

void Problem::collectVariables(SymbolicExprPtr expr)
{
    // static int offset = 0;

    if(auto var = dynamic_pointer_cast<DecisionVariableExpr>(expr))
    {
        //if var is not in the variables
        if(find(variables_.begin(), variables_.end(), var->name()) == variables_.end())
        {
            variables_.push_back(var->name());
            variablesWithDim_[var->name()] = var->dim();
            // variableOffsets_[var->name()] = offset;
            // offset += var->dim();
            // variablesDim_ += var->dim();
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

void Problem::sortVariables(vector<string>& v)
{
    auto extractNumber = [](string& s)
    {
        string num;
        for(char c: s)
        {
            if(isdigit(c)) num += c;
        }
        return stoi(num);
    };
    sort(v.begin(), v.end(), [&](string& a, string& b){if(a[0] != b[0]) return a[0] > b[0]; return extractNumber(a) < extractNumber(b);});
}

void Problem::setVariablesIndeces(void)
{
    //sort all variables so that they could be arranged like
    //x0,x1,x2...xn, u0,u1,u2...un
    sortVariables(variables_);

    for(auto c:variables_)
        cout<<c<<" ";
    cout<<endl;

    int offset = 0;
    for(auto s: variables_)
    {
        variableOffsets_[s] = offset;
        offset += variablesWithDim_[s];
    }
    variablesDim_ = offset;
}


void Problem::buildMatrixP()
{
    int n = totalVariablesDim();
    P_ = Eigen::MatrixXd::Zero(n,n);

    auto quadBuildP = [&](const QuadExpr* qf)
    {
        if(auto v = dynamic_pointer_cast<DecisionVariableExpr>(qf->x()))
        {
            //x'Wx
            int offset = variableOffset(v->name());
            P_.block(offset, offset, qf->W().rows(), qf->W().cols()) += qf->W();
        }
        else if(auto dot = dynamic_pointer_cast<DotExpr>(qf->x()))
        {
            //(Ax)'W(Ax)
            auto var = dynamic_pointer_cast<DecisionVariableExpr>(dot->rhs());
            auto param = dynamic_pointer_cast<ParameterExpr>(dot->lhs());
            if(var && param)
            {
                int offset = variableOffset(var->name());
                P_.block(offset, offset, qf->W().rows(), qf->W().rows()) += param->value().transpose() * qf->W() * param->value();
            }
        }
        else if(auto bin = dynamic_pointer_cast<BinaryExpr>(qf->x()))
        {
            //case 1
            //(xi - xr)
            if(auto v1 = dynamic_pointer_cast<DecisionVariableExpr>(bin->lhs()))
            {
                // (x-xr)'Q(x-xr)
                int offset = variableOffset(v1->name());
                P_.block(offset, offset, qf->W().rows(), qf->W().cols()) += qf->W();
            }
            //case 2
            //(Axi - xr)
            else if(auto dot = dynamic_pointer_cast<DotExpr>(bin->lhs()))
            {
                // (Ax-xr)'Q(Ax-xr)
                if(auto var = dynamic_pointer_cast<DecisionVariableExpr>(dot->rhs()))
                {
                    auto param = dynamic_pointer_cast<ParameterExpr>(dot->lhs());
                    int offset = variableOffset(var->name());
                    P_.block(offset, offset, qf->W().rows(), qf->W().cols()) += param->value().transpose() * qf->W() * param->value();
                }
            }
            else if(auto subBin = dynamic_pointer_cast<BinaryExpr>(bin->lhs()))
            {
                //case 3
                //(xi - xj - xr)
                if(auto vi = dynamic_pointer_cast<DecisionVariableExpr>(subBin->lhs()))
                {
                    if(auto vj = dynamic_pointer_cast<DecisionVariableExpr>(subBin->rhs()))
                    {
                        int offset1 = variableOffset(vi->name());
                        int offset2 = variableOffset(vj->name());
                        P_.block(offset1, offset1, qf->W().rows(), qf->W().cols()) += qf->W();
                        P_.block(offset2, offset2, qf->W().rows(), qf->W().cols()) += qf->W();

                        if(subBin->operation() == BinaryExpr::ADD)
                        {
                            P_.block(offset1, offset2, qf->W().rows(), qf->W().cols()) += qf->W();
                            P_.block(offset2, offset1, qf->W().rows(), qf->W().cols()) += qf->W();
                        }
                        else
                        {
                            P_.block(offset1, offset2, qf->W().rows(), qf->W().cols()) -= qf->W();
                            P_.block(offset2, offset1, qf->W().rows(), qf->W().cols()) -= qf->W();
                        }
                    }
                }
                //case 4
                //(Axi - Axj - xr)
                else if(auto dotl = dynamic_pointer_cast<DotExpr>(subBin->lhs()))
                {
                    if(auto dotr = dynamic_pointer_cast<DotExpr>(subBin->rhs()))
                    {
                        auto vi = dynamic_pointer_cast<DecisionVariableExpr>(dotl->rhs());
                        auto vj = dynamic_pointer_cast<DecisionVariableExpr>(dotr->rhs());
                        auto parami = dynamic_pointer_cast<ParameterExpr>(dotl->lhs());
                        auto paramj = dynamic_pointer_cast<ParameterExpr>(dotr->lhs());
                        int offset1 = variableOffset(vi->name());
                        int offset2 = variableOffset(vj->name());
                        //need to be tested
                        P_.block(offset1, offset1, qf->W().rows(), qf->W().cols()) += parami->value().transpose() * qf->W() * parami->value();
                        P_.block(offset2, offset2, qf->W().rows(), qf->W().cols()) += paramj->value().transpose() * qf->W() * paramj->value();
                        if(subBin->operation() == BinaryExpr::ADD)
                        {
                            P_.block(offset1, offset2, qf->W().rows(), qf->W().cols()) += parami->value().transpose() * qf->W() * paramj->value();
                            P_.block(offset2, offset1, qf->W().rows(), qf->W().cols()) += paramj->value().transpose() * qf->W() * parami->value();
                        }
                        else
                        {
                            P_.block(offset1, offset2, qf->W().rows(), qf->W().cols()) -= parami->value().transpose() * qf->W() * paramj->value();
                            P_.block(offset2, offset1, qf->W().rows(), qf->W().cols()) -= paramj->value().transpose() * qf->W() * parami->value();
                        }
                    }
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

    //to match the OSQP object form: 0.5x'Px + q'x so I need to multiply P_ by 2
    P_ *= 2.0;
}

void Problem::buildVectorQ()
{
    int n = totalVariablesDim();
    q_ = Eigen::VectorXd::Zero(n);

    auto quadBuildP = [&](const QuadExpr* qf)
    {
        if(auto bin = dynamic_pointer_cast<BinaryExpr>(qf->x()))
        {
            if(auto xi = dynamic_pointer_cast<DecisionVariableExpr>(bin->lhs()))
            {
                // (x-xr)'Q(x-xr)
                if(auto xr = dynamic_pointer_cast<ParameterExpr>(bin->rhs()))
                {
                    int offset = variableOffset(xi->name());
                    if(bin->operation() == BinaryExpr::SUB)
                        q_.segment(offset, xi->dim()) -= 2.0 * qf->W() * xr->value();
                    else
                        q_.segment(offset, xi->dim()) += 2.0 * qf->W() * xr->value();
                }
                
            }
            else if(auto dot = dynamic_pointer_cast<DotExpr>(bin->lhs()))
            {
                // (Ax-xr)'Q(Ax-xr)
                if(auto var = dynamic_pointer_cast<DecisionVariableExpr>(dot->rhs()))
                {
                    auto param = dynamic_pointer_cast<ParameterExpr>(dot->lhs());
                    auto xr = dynamic_pointer_cast<ParameterExpr>(bin->rhs());
                    int offset = variableOffset(var->name());
                    if(bin->operation() == BinaryExpr::SUB)
                        q_.segment(offset, var->dim()) -= 2.0* param->value().transpose() * qf->W() * xr->value();
                    else
                        q_.segment(offset, var->dim()) += 2.0* param->value().transpose() * qf->W() * xr->value();
                }
            }
            else if(auto subBin = dynamic_pointer_cast<BinaryExpr>(bin->lhs()))
            {
                //(xi - xj - xr)
                if(auto xi = dynamic_pointer_cast<DecisionVariableExpr>(subBin->lhs()))
                {
                    if(auto xj = dynamic_pointer_cast<DecisionVariableExpr>(subBin->rhs()))
                    {
                        auto xr = dynamic_pointer_cast<ParameterExpr>(bin->rhs());
                        int offset1 = variableOffset(xi->name());
                        int offset2 = variableOffset(xj->name());
                        if(bin->operation() == BinaryExpr::SUB)
                            q_.segment(offset1, xi->dim()) -= 2.0 * qf->W() * xr->value();
                        else
                            q_.segment(offset1, xi->dim()) += 2.0 * qf->W() * xr->value();

                        if(bin->operation() == subBin->operation())
                            q_.segment(offset2, xj->dim()) += 2.0 * qf->W() * xr->value();
                        else
                            q_.segment(offset2, xj->dim()) += 2.0 * qf->W() * xr->value();
                    }
                }
                //(Axi - Bxj - xr)
                else if(auto dotl = dynamic_pointer_cast<DotExpr>(subBin->lhs()))
                {
                    if(auto dotr = dynamic_pointer_cast<DotExpr>(subBin->rhs()))
                    {
                        auto xi = dynamic_pointer_cast<DecisionVariableExpr>(dotl->rhs());
                        auto xj = dynamic_pointer_cast<DecisionVariableExpr>(dotr->rhs());
                        auto A = dynamic_pointer_cast<ParameterExpr>(dotl->lhs());
                        auto B = dynamic_pointer_cast<ParameterExpr>(dotr->lhs());
                        auto xr = dynamic_pointer_cast<ParameterExpr>(bin->rhs());
                        int offset1 = variableOffset(xi->name());
                        int offset2 = variableOffset(xj->name());

                        if(bin->operation() == BinaryExpr::SUB)
                            q_.segment(offset1, xi->dim()) -= 2.0 * A->value().transpose() * qf->W() * xr->value();
                        else
                            q_.segment(offset1, xi->dim()) += 2.0 * A->value().transpose() * qf->W() * xr->value();
                        
                        if(bin->operation() == subBin->operation())
                            q_.segment(offset2, xj->dim()) += 2.0 * B->value().transpose() * qf->W() * xr->value();
                        else
                            q_.segment(offset2, xj->dim()) += 2.0 * B->value().transpose() * qf->W() * xr->value();
                    }
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

void Problem::buildMatrixA()
{
    int totalRows = 0;
    for(const auto& c: constraints_)
    {
        auto lhs = decodeLinearExpr(c.lhs());
        totalRows += lhs.bias.rows();
    }

    int n = totalVariablesDim();
    A_ = MatrixXd::Zero(totalRows, n);

    int row = 0;
    for(const auto& c: constraints_)
    {
        if(c.operation() != Constraint::RANGE)
        {
            auto lhs = decodeLinearExpr(c.lhs());
            int rows = lhs.bias.rows();
            A_.block(row, 0, rows, n) = lhs.coeffs;
            row += rows;
        }
        else
        {
            auto middle = decodeLinearExpr(c.middle());
            int rows = middle.bias.rows();
            A_.block(row, 0, rows, n) = middle.coeffs;
            row += rows;
        }   
    }
}

void Problem::buildVectorL()
{
    int totalRows = 0;
    for(const auto& c: constraints_)
    {
        auto lhs = decodeLinearExpr(c.lhs());
        totalRows += lhs.bias.rows();
    }

    l_ = VectorXd::Zero(totalRows);
    int row = 0;

    for(const auto& c: constraints_)
    {
        auto lhs = decodeLinearExpr(c.lhs());
        auto rhs = decodeLinearExpr(c.rhs());
        int rows = lhs.bias.rows();

        switch (c.operation())
        {
            case Constraint::EQUAL:
                l_.segment(row, rows) = rhs.bias;
                break;
            case Constraint::GEQ:
                l_.segment(row, rows) = rhs.bias;
                break;
            case Constraint::LEQ:
                l_.segment(row, rows) = VectorXd::Constant(rows, -numeric_limits<double>::infinity());
                break;  
            case Constraint::RANGE:
                l_.segment(row, rows) = lhs.bias;
                break;  
        }
        row += rows;
    }

}

void Problem::buildVectorU()
{
    int totalRows = 0;
    for(const auto& c: constraints_)
    {
        auto lhs = decodeLinearExpr(c.lhs());
        totalRows += lhs.bias.rows();
    }

    u_ = VectorXd::Zero(totalRows);
    int row = 0;

    for(const auto& c: constraints_)
    {
        auto lhs = decodeLinearExpr(c.lhs());
        auto rhs = decodeLinearExpr(c.rhs());
        int rows = lhs.bias.rows();

        switch (c.operation())
        {
            case Constraint::EQUAL:
                u_.segment(row, rows) = rhs.bias;
                break;
            case Constraint::GEQ:
                u_.segment(row, rows) = VectorXd::Constant(rows, numeric_limits<double>::infinity());
                break;
            case Constraint::LEQ:
                u_.segment(row, rows) = rhs.bias;
                break;   
            case Constraint::RANGE:
                u_.segment(row, rows) = rhs.bias;
                break;   
        }
        row += rows;
    }

}

LinearizedExpr Problem::decodeLinearExpr(const SymbolicExprPtr& expr)
{
    //decode a linear expression and get its coeffs and bias
    //eg. Ax+b --> coeffs = A, bias = b
    if(auto var = dynamic_pointer_cast<DecisionVariableExpr>(expr))
    {
        //case 1: f = x
        int offset = variableOffset(var->name());
        int dim = var->dim();
        MatrixXd coeffs = MatrixXd::Zero(dim, totalVariablesDim());
        coeffs.block(0,offset,dim,dim) = MatrixXd::Identity(dim,dim);
        return{coeffs, VectorXd::Zero(dim)};
    }
    else if(auto bin = dynamic_pointer_cast<BinaryExpr>(expr))
    {
        //support Axi + Bui +...
        //do not support Axi + b, becuase b should be in the rhs of the constrait
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
            MatrixXd A = mat->value();
            int offset = variableOffset(var->name());
            int dim = var->dim();

            MatrixXd coeffs = MatrixXd::Zero(A.rows(), totalVariablesDim());
            coeffs.block(0,offset, A.rows(), dim) = A;

            return {coeffs, VectorXd::Zero(A.rows())};
        }
    }
    else if(auto param = dynamic_pointer_cast<ParameterExpr>(expr))
    {
        // b / l/ u
        VectorXd bias = VectorXd::Zero(param->value().rows());
        bias = param->value().col(0);//param is a MatrixXd(n,1), so get the 0's column
        return {MatrixXd::Zero(bias.rows(), totalVariablesDim()), bias};
    }

    throw runtime_error("Unsupported expression in linear funciton!");
}


//updata object function at the runtime
void Problem::updateObjectFunction(SymbolicExprPtr object)
{
    object_ = object;
    //assume that the decision variables are not changed
    //so only need to update Q,P matrices instead of update indeces and sizes
    buildMatrixP();
    buildVectorQ();
}

//update constraits at the runtime
void Problem::updateConstraints(vector<Constraint>& constraints)
{
    constraints_ = constraints;
    //assume that the decision variables are not changed
    //so only need to update Q,P matrices instead of update indeces and sizes
    buildMatrixA();
    buildVectorL();
    buildVectorU();
}


bool Problem::solve()
{
    int n = P_.rows();
    int m = A_.rows();

    OsqpEigen::Solver solver;

    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    solver.data()->setNumberOfVariables(n);
    solver.data()->setNumberOfConstraints(m);

    SparseMatrix<double> Psparse = P_.sparseView();
    SparseMatrix<double> Asparse = A_.sparseView();


    solver.data()->setHessianMatrix(Psparse);
    solver.data()->setGradient(q_);
    solver.data()->setLinearConstraintsMatrix(Asparse);
    solver.data()->setLowerBound(l_);
    solver.data()->setUpperBound(u_);

    solver.initSolver();

    if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;

    solution_ = solver.getSolution();

    return true;
}


            