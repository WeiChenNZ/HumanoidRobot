#pragma once

#include <iostream>
#include "Eigen/Core"
#include <memory>
#include <unordered_map>

//in QP formular, desision variables are the unknown variables that needs to be optimized
//eg. x1,x2,x3...xn  
//    u0,u1,u2...un
class DecisionVariable{
    public:
        DecisionVariable(std::string name, int dim):name_(std::move(name)), dim_(dim){}
        const std::string& name() const {return name_;}
        int dim() const {return dim_;}

    private:
        std::string name_;
        int dim_;
};

using VariablePtr = std::shared_ptr<DecisionVariable>;


//Parameters
class Parameter{
    public:
        Parameter(std::string name, Eigen::MatrixXd value):name_(std::move(name)), value_(value){}
        const std::string name() const {return name_;}
        const Eigen::MatrixXd value() const {return value_;}

    private:
        std::string name_;
        Eigen::MatrixXd value_;
};

using ParameterPtr = std::shared_ptr<Parameter>;

//Symbolic Expression Base Class
class SymbolicExpression{
    public:
        virtual ~SymbolicExpression() = default;
};

using SymbolicExprPtr = std::shared_ptr<SymbolicExpression>;

//Constant
class ConstantExpr: public SymbolicExpression{
    public:
        explicit ConstantExpr(Eigen::MatrixXd& value):value_(value) {}
        const Eigen::MatrixXd& value() const {return value_;}

    private:
        Eigen::MatrixXd value_;
};

//Decision Variable expression
class DecisionVariableExpr: public SymbolicExpression{
    public:
        DecisionVariableExpr(VariablePtr var):var_(std::move(var)){}
        VariablePtr variable() const {return var_;}
    private:
        VariablePtr var_;
};

//Parameter expression
class ParameterExpr: public SymbolicExpression{
    public:
        ParameterExpr(ParameterPtr param):param_(std::move(param)){}
        ParameterPtr parameter() const {return param_;}
    private:
        ParameterPtr param_;
};

//Binary expression, +,-
//a+b, a-b 
class BinaryExpr: public SymbolicExpression{
    public:
        enum TYPE{ADD, SUB};
        BinaryExpr(SymbolicExprPtr a, SymbolicExprPtr b, TYPE op):a_(a), b_(b), op_(op){}
        SymbolicExprPtr lhs() const {return a_;}
        SymbolicExprPtr rhs() const {return b_;}
        TYPE operation() const {return op_;}
    private:
        SymbolicExprPtr a_;
        SymbolicExprPtr b_;
        TYPE op_;
};

inline SymbolicExprPtr operator+(SymbolicExprPtr a, SymbolicExprPtr b)
{
    return std::make_shared<BinaryExpr>(a, b, BinaryExpr::ADD);
}

inline SymbolicExprPtr operator-(SymbolicExprPtr a, SymbolicExprPtr b)
{
    return std::make_shared<BinaryExpr>(a, b, BinaryExpr::SUB);
}

//Sumup expression
class SumUpExpr: public SymbolicExpression{
    public:
        explicit SumUpExpr(const std::vector<SymbolicExprPtr>& terms) :terms_(terms){}
        const std::vector<SymbolicExprPtr>& terms() const {return terms_;}

    private:
        std::vector<SymbolicExprPtr> terms_;
};

inline SymbolicExprPtr sumup(std::vector<SymbolicExprPtr>& terms)
{
    return std::make_shared<SumUpExpr>(terms);
}

//multipule
// a * b
class DotExpr: public SymbolicExpression{
    public:
        DotExpr(SymbolicExprPtr a, SymbolicExprPtr b): a_(a), b_(b){}
        SymbolicExprPtr lhs() const {return a_;}
        SymbolicExprPtr rhs() const {return b_;}
    private:
        SymbolicExprPtr a_;
        SymbolicExprPtr b_;
};

inline SymbolicExprPtr dot(SymbolicExprPtr a, SymbolicExprPtr b)
{
    return std::make_shared<DotExpr>(a, b);
}

inline SymbolicExprPtr operator*(SymbolicExprPtr a, SymbolicExprPtr b)
{
    return std::make_shared<DotExpr>(a,b);
}

//Quadratic form
// x'*Q*x
class QuadExpr: public SymbolicExpression{
    public:
        QuadExpr(SymbolicExprPtr x, Eigen::MatrixXd& Q): x_(x), Q_(Q){}
        SymbolicExprPtr x() const {return x_;}
        const Eigen::MatrixXd& Q() const {return Q_;}

    private:
        SymbolicExprPtr x_;
        Eigen::MatrixXd Q_;
};

inline SymbolicExprPtr quadForm(SymbolicExprPtr x, Eigen::MatrixXd& Q)
{
    if(auto var = std::dynamic_pointer_cast<DecisionVariableExpr>(x))
    {
        //x = xi
        //convert to binary form xi-0
        ParameterPtr zero = std::make_shared<Parameter>("0", Eigen::VectorXd::Zero(var->variable()->dim()));
        SymbolicExprPtr ZERO = std::make_shared<ParameterExpr>(zero);
        return std::make_shared<QuadExpr>(var - ZERO, Q); 
    }
    else
    {
        // binary form
        //x = xi - xr
        return std::make_shared<QuadExpr>(x, Q); 
    }
}

///////////////////////// End of Expression Definition //////////////////////////////

//Constraint
class Constraint{
    public:
        enum TYPE{EQUAL, GEQ, LEQ};
        Constraint(SymbolicExprPtr lhs, SymbolicExprPtr rhs, TYPE op): lhs_(lhs), rhs_(rhs), op_(op){}
        SymbolicExprPtr lhs() const {return lhs_;}
        SymbolicExprPtr rhs() const {return rhs_;}
        TYPE operation() const {return op_;}
    private:
        SymbolicExprPtr lhs_;
        SymbolicExprPtr rhs_;
        TYPE op_;
};

inline Constraint operator==(SymbolicExprPtr lhs, SymbolicExprPtr rhs)
{
    return Constraint(lhs, rhs, Constraint::EQUAL);
}

inline Constraint operator>=(SymbolicExprPtr lhs, SymbolicExprPtr rhs)
{
    return Constraint(lhs, rhs, Constraint::GEQ);
}

inline Constraint operator<=(SymbolicExprPtr lhs, SymbolicExprPtr rhs)
{
    return Constraint(lhs, rhs, Constraint::LEQ);
}

//Linearized expression Ax+b
//define a expression Ax >= l, then lhs = Ax + 0, where coeffs = A, bias = 0
// rhs = 0x + l, where coeffs = 0, bias = l
struct LinearizedExpr
{
    Eigen::MatrixXd coeffs; //A
    Eigen::VectorXd bias;   //b
};

//Quadratic Programming Problem
//TODO: support (xi - xk - c)' Q (xi - xk -c), where xi,xk are decision variables, c is a constant

class Problem{
    public:
        enum OptimizationType{MINIMIZE, MAXIMIZE};

        Problem(SymbolicExprPtr object, std::vector<Constraint>& constraints, OptimizationType type)
        :object_(object), constraints_(constraints), type_(type)
        {
            collectVariables(object_);
            for(auto c: constraints_)
            {
                collectVariables(c.lhs());
                collectVariables(c.rhs());
            }
            assignVaribalesIndeces();

            //build Q,p, A, l, u
            buildMatrixQ();
            buildVectorP();
            buildMatrixA();
            buildVectorL();
            buildVectorU();
        }

        const Eigen::MatrixXd& Q() const {return Q_;}
        const Eigen::VectorXd& p() const {return p_;}
        const Eigen::MatrixXd& A() const {return A_;}
        const Eigen::VectorXd& l() const {return l_;}
        const Eigen::VectorXd& u() const {return u_;}

        void updateObjectFunction(SymbolicExprPtr);
        void updateConstraints(std::vector<Constraint>&);

        const std::vector<VariablePtr>& variables() const {return variables_;}

        int totalVariablesDim() const {return variablesDim_;}

        int variableOffset(const std::string& name) const {return variableOffsets_.at(name);}

    private:
        SymbolicExprPtr object_;
        std::vector<Constraint> constraints_;
        OptimizationType type_;
        std::vector<VariablePtr> variables_;
        int variablesDim_;
        std::unordered_map<std::string, int> variableOffsets_;

        //QP matrics for Q, p, A, l, u
        // x'Qx + p'x
        // l <= Ax <= u
        Eigen::MatrixXd Q_;
        Eigen::VectorXd p_;
        Eigen::MatrixXd A_;
        Eigen::VectorXd l_;        
        Eigen::VectorXd u_;

        void buildMatrixQ(void);
        void buildMatrixA(void);
        void buildVectorP(void);
        void buildVectorL(void);
        void buildVectorU(void);

        void collectVariables(SymbolicExprPtr);
        void assignVaribalesIndeces(void);

        LinearizedExpr decodeLinearExpr(const SymbolicExprPtr&);
};