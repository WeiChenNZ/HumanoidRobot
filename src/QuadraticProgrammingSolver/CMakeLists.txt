cmake_minimum_required(VERSION 3.10)

project(QP_Solver)


add_executable(SymbolicExptrTest test/testQP.cpp SymbolicExpression.cpp)
target_link_libraries(SymbolicExptrTest PRIVATE OsqpEigen::OsqpEigen)