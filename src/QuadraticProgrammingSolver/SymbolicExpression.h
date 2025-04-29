#pragma once

#include <iostream>
#include "Eigen/Core"
#include <memory>
#include <unordered_map>


//Symbolic Expression Base Class
class SymbolicExpression{
    public:
        virtual void updateValue(Eigen::MatrixXd) {};
        virtual ~SymbolicExpression() = default;
};

using SymbolicExprPtr = std::shared_ptr<SymbolicExpression>;


//Decision Variable expression
class DecisionVariableExpr: public SymbolicExpression{
    public:
        DecisionVariableExpr(std::string name, int dim):name_(std::move(name)), dim_(dim){}
        std::string name() const {return name_;}
        int dim() const {return dim_;}

    private:
        std::string name_;
        int dim_;
};

//Parameter expression
class ParameterExpr: public SymbolicExpression{
    public:
        ParameterExpr(std::string name, Eigen::MatrixXd value):name_(std::move(name)), value_(std::move(value)){}
        std::string name() const {return name_;}
        const Eigen::MatrixXd& value() const {return value_;}

        void updateValue(Eigen::MatrixXd value) { value_ = value;}

    private:
        std::string name_;
        Eigen::MatrixXd value_;
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

// inline SymbolicExprPtr dot(SymbolicExprPtr a, SymbolicExprPtr b)
// {
//     return std::make_shared<DotExpr>(a, b);
// }

inline SymbolicExprPtr operator*(SymbolicExprPtr a, SymbolicExprPtr b)
{
    return std::make_shared<DotExpr>(a,b);
}

//Quadratic form
// x'*W*x
class QuadExpr: public SymbolicExpression{
    public:
        QuadExpr(SymbolicExprPtr x, Eigen::MatrixXd& W): x_(x), W_(W){}
        SymbolicExprPtr x() const {return x_;}
        const Eigen::MatrixXd& W() const {return W_;}

    private:
        SymbolicExprPtr x_;
        Eigen::MatrixXd W_;
};

inline SymbolicExprPtr quadForm(SymbolicExprPtr x, Eigen::MatrixXd& Q)
{
    return std::make_shared<QuadExpr>(x, Q); 
}

///////////////////////// End of Expression Definition //////////////////////////////

//Constraint
class Constraint{
    public:
        enum TYPE{EQUAL, GEQ, LEQ, RANGE};
        Constraint(SymbolicExprPtr lhs, SymbolicExprPtr rhs, TYPE op): lhs_(lhs), rhs_(rhs), op_(op){}
        Constraint(SymbolicExprPtr middleExpr, SymbolicExprPtr lhs, SymbolicExprPtr rhs, TYPE op):middleExpr_(middleExpr), lhs_(lhs), rhs_(rhs), op_(op){}
        SymbolicExprPtr lhs() const {return lhs_;}
        SymbolicExprPtr rhs() const {return rhs_;}
        SymbolicExprPtr middle() const {return middleExpr_;}
        TYPE operation() const {return op_;}
    private:
        SymbolicExprPtr lhs_;
        SymbolicExprPtr rhs_;
        SymbolicExprPtr middleExpr_;
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

inline Constraint constraintRange(SymbolicExprPtr expr, SymbolicExprPtr lhs, SymbolicExprPtr rhs)
{
    return Constraint(expr, lhs, rhs, Constraint::RANGE);
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

            setVariablesIndeces();

            //build Q,p, A, l, u
            buildMatrixP();
            buildVectorQ();
            buildMatrixA();
            buildVectorL();
            buildVectorU();
        }

        const Eigen::MatrixXd& P() const {return P_;}
        const Eigen::VectorXd& q() const {return q_;}
        const Eigen::MatrixXd& A() const {return A_;}
        const Eigen::VectorXd& l() const {return l_;}
        const Eigen::VectorXd& u() const {return u_;}

        void updateObjectFunction(SymbolicExprPtr);
        void updateConstraints(std::vector<Constraint>&);

        const std::vector<std::string>& variables() const {return variables_;}

        int totalVariablesDim() const {return variablesDim_;}

        int variableOffset(const std::string& name) const {return variableOffsets_.at(name);}

        bool solve(void);

        const Eigen::VectorXd& getSolution(void) const { return solution_;}

    private:
        SymbolicExprPtr object_;
        std::vector<Constraint> constraints_;
        OptimizationType type_;
        std::vector<std::string> variables_;
        std::unordered_map<std::string, int> variablesWithDim_;
        int variablesDim_ = 0;
        std::unordered_map<std::string, int> variableOffsets_;

        //QP matrics for Q, p, A, l, u
        // x'Qx + p'x
        // l <= Ax <= u
        Eigen::MatrixXd P_;
        Eigen::VectorXd q_;
        Eigen::MatrixXd A_;
        Eigen::VectorXd l_;        
        Eigen::VectorXd u_;

        Eigen::VectorXd solution_;

        void buildMatrixP(void);
        void buildMatrixA(void);
        void buildVectorQ(void);
        void buildVectorL(void);
        void buildVectorU(void);

        void collectVariables(SymbolicExprPtr);
        void sortVariables(std::vector<std::string>&);
        void setVariablesIndeces(void);

        LinearizedExpr decodeLinearExpr(const SymbolicExprPtr&);
};