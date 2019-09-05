 //######################################################################
 //
// GraspIt!
 // Copyright (C) 2002-2009  Columbia University in the City of New York.
 // All rights reserved.
//
 // GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
 // (at your option) any later version.
 //
// GraspIt! is distributed in the hope that it will be useful,
 // but WITHOUT ANY WARRANTY; without even the implied warranty of
 // MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 // GNU General Public License for more details.
//
 // You should have received a copy of the GNU General Public License
 // along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
 //
 // Author(s): Matei T. Ciocarlie
 //
 // $Id: matrix.h,v 1.21 2010/08/03 17:17:09 cmatei Exp $
 //
 //######################################################################
 
 #ifndef _matrix_h_
 #define _matrix_h_
 
 #include <assert.h>
 #include <iostream>
 #include <vector>
 #include <list>
 #include <map>
 #include <memory>
 #include <cstdio>
 
 class transf;
 class mat3;
 
 
 class Matrix{
 private:
         double *mData;
         void initialize(int m, int n);
         void setFromColMajor(const double *M);
         void setFromRowMajor(const double *M);
 
         mutable int mSequentialI;
         mutable int mSequentialJ;
 protected:
         int mRows, mCols;
 public:
         enum Type{DENSE, SPARSE};
         Matrix(int m, int n);
         Matrix(const Matrix &M);
         Matrix(const double *M, int m, int n, bool colMajor);
 
         virtual ~Matrix();
 
         virtual void resize(int m, int n);
 
         virtual Type getType() const {return DENSE;}
         virtual double& elem(int m, int n);
         virtual const double& elem(int m, int n) const;
         virtual std::auto_ptr<double> getDataCopy() const;
         virtual void getData(std::vector<double> *data) const;
         virtual double* getDataPointer();
   
         int rows() const {return mRows;}
         int cols() const {return mCols;}
         Matrix getColumn(int c) const;
         Matrix getRow(int r) const;
         Matrix getSubMatrix(int startRow, int startCol, int rows, int cols) const;
 
         virtual void setAllElements(double val);
         virtual void copySubBlock(int startRow, int startCol, int rows, int cols, 
                                                   const Matrix &m, int startMRow, int startMCol);
         void copySubMatrix(int startRow, int startCol, const Matrix &m){
                 copySubBlock(startRow, startCol, m.rows(), m.cols(), m, 0, 0);}
         void copyMatrix(const Matrix &m){copySubMatrix(0, 0, m);}
 
         friend std::ostream& operator<<(std::ostream &os, const Matrix &m);
         void print(FILE *fp = stderr) const;
 
         virtual void sequentialReset() const;
         virtual bool nextSequentialElement(int &i, int &j, double &val) const;
 
         virtual double getDefault() const {return 0.0;}
 
         virtual int numElements() const {return mRows * mCols;}
         int rank() const;
         double fnorm() const;
         double absMax() const;
         double elementSum() const;
         void swapRows(int r1, int r2);
         void swapCols(int c1, int c2);
         virtual void transpose();
         void eye();
         void multiply(double s);
 
         Matrix transposed() const;
 
         static Matrix EYE(int m, int n);
         static Matrix NEGEYE(int m, int n);
 
         static Matrix PERMUTATION(int n, int *jpvt);
         static Matrix TRANSFORM(const transf &t);
         static Matrix ROTATION(const mat3 &rot);
         static Matrix ROTATION2D(double theta);
         static Matrix MAX_VECTOR(int rows);
         static Matrix MIN_VECTOR(int rows);
 
         template <class MatrixType>
         static MatrixType ZEROES(int m, int n);
 
         template <class MatrixType>
         static MatrixType BLOCKDIAG(const Matrix &M1, const Matrix &M2);
         template <class MatrixType>
         static MatrixType BLOCKCOLUMN(const Matrix &M1, const Matrix &M2);
         template <class MatrixType>
         static MatrixType BLOCKROW(const Matrix &M1, const Matrix &M2);
 
         template <class MatrixType>
         static MatrixType BLOCKDIAG(std::list<Matrix*> *blocks);
         template <class MatrixType>
         static MatrixType BLOCKCOLUMN(std::list<Matrix*> *blocks);
         template <class MatrixType>
         static MatrixType BLOCKROW(std::list<Matrix*> *blocks);
 
         static const double EPS;
 };
 
 class SparseMatrix : public Matrix
 {
 private:
         double mDefaultValue;
         double mFoo;
 
         std::map<int, double> mSparseData;
         

         int key(int m, int n) const{
                 return n*mRows + m;
         }
         void reverseKey(int k, int &m, int &n) const {
                 m = k % mRows;
                 n = k / mRows;
                 assert( key(m,n) == k );
         }
 
         mutable std::map<int, double>::const_iterator mSequentialIt;
 public:
         SparseMatrix(int m, int n, double defaultValue=0.0) : Matrix(0,0){
                 //since we call the super constructor with 0 size, mData will be NULL
                 mRows = m;
                 mCols = n;
                 mDefaultValue = defaultValue;
                 mFoo = 0.0;
         }
         SparseMatrix(const SparseMatrix &SM);
         ~SparseMatrix() {
                 //so that the super destructor does not delete mData
                 mRows = 0;
         }
 
         virtual void resize(int m, int n);
 
         virtual Type getType() const {return SPARSE;}
         virtual double getDefault() const {return mDefaultValue;}
 
         virtual int numElements() const {
                 return mSparseData.size();
         }
 
         virtual double& elem(int m, int n);
         virtual const double& elem(int m, int n) const;
         virtual std::auto_ptr<double> getDataCopy() const;
         virtual void getData(std::vector<double> *data) const;
         virtual double* getDataPointer();
         virtual void transpose();
 
         void copySubBlock(int startRow, int startCol, int rows, int cols, 
                                       const Matrix &m, int startMRow, int startMCol);
 
         virtual void setAllElements(double val){
                 mDefaultValue = val;
                 mSparseData.clear();
         }
 
         static SparseMatrix EYE(int m, int n);
         static SparseMatrix NEGEYE(int m, int n);
 
         virtual void sequentialReset() const;
         virtual bool nextSequentialElement(int &i, int &j, double &val) const;
 };
 
 void matrixMultiply(const Matrix &L, const Matrix &R, Matrix &M);
 void matrixAdd(const Matrix &L, const Matrix &R, Matrix &M);
 bool matrixEqual(const Matrix &R, const Matrix &L);
 int triangularSolve(Matrix &A, Matrix &B);
 int linearSolveMPInv(Matrix &A, Matrix &B, Matrix &X);
 int linearSolveSVD(Matrix &A, Matrix &B, Matrix &X);
 int underDeterminedSolveQR(Matrix &A, Matrix &B, Matrix &X);
 int matrixInverse(const Matrix &A, Matrix &AInv);
 int QPSolver(const Matrix &Q, 
                         const Matrix &Eq, const Matrix &b,
                          const Matrix &InEq, const Matrix &ib,
                          const Matrix &lowerBounds, const Matrix &upperBounds,
                          Matrix &sol, double *objVal);
 int factorizedQPSolver(const Matrix &Qf, 
                                                                   const Matrix &Eq, const Matrix &b,
                                                                  const Matrix &InEq, const Matrix &ib,                 
                                                                   const Matrix &lowerBounds, const Matrix &upperBounds,
                                                                   Matrix &sol, double *objVal);
 void testQP();
 int LPSolver(const Matrix &Q,
                          const Matrix &Eq, const Matrix &b,
                          const Matrix &InEq, const Matrix &ib,
                          const Matrix &lowerBounds, const Matrix &upperBounds,
                          Matrix &sol, double *objVal);
 
 template <class MatrixType>
 MatrixType Matrix::ZEROES(int m, int n)
 {
         assert( m>0 && n>0 );
         MatrixType Z(m,n);
         Z.setAllElements(0.0);
         return Z;
 }
 
template <class MatrixType>
 MatrixType Matrix::BLOCKROW(std::list<Matrix*> *blocks)
 {
         int numCols = 0;
         std::list<Matrix*>::iterator it;
         int numRows = 0;
         for(it=blocks->begin(); it!=blocks->end(); it++) {
                 numCols += (*it)->cols();
                 if ( (*it)->cols() ) {
                         if (numRows == 0) {
                                 numRows = (*it)->rows();
                         } else {
                                 assert((*it)->rows() == numRows);
                         }
                 }
         }
         if (!numCols) return MatrixType(0, 0);
         MatrixType C(numRows, numCols);
         numCols  = 0;
         for(it=blocks->begin(); it!=blocks->end(); it++) {
                 if (!(*it)->cols()) continue;
                 C.copySubMatrix(0, numCols, *(*it));
                 numCols += (*it)->cols();
         }
         return C;
 }
 
 
 template <class MatrixType>
 MatrixType Matrix::BLOCKDIAG(std::list<Matrix*> *blocks)
 {
         int numRows=0, numCols=0;
         std::list<Matrix*>::iterator it;
         for(it=blocks->begin(); it!=blocks->end(); it++) {
                 numRows += (*it)->rows();
                 numCols += (*it)->cols();
         }
         if (!numRows || !numCols) return MatrixType(numRows, numCols);
         MatrixType B(numRows, numCols);
         B.setAllElements(0.0);
         numRows = numCols = 0;
         for(it=blocks->begin(); it!=blocks->end(); it++) {
                 if (!(*it)->rows() || !(*it)->cols()) continue;
                 B.copySubMatrix(numRows, numCols, *(*it));
                 numRows += (*it)->rows();
                 numCols += (*it)->cols();
         }
         return B;
 }
 
 template <class MatrixType>
 MatrixType Matrix::BLOCKCOLUMN(std::list<Matrix*> *blocks)
 {
         int numRows = 0;
         std::list<Matrix*>::iterator it;
         int numCols = 0;
         for(it=blocks->begin(); it!=blocks->end(); it++) {
                 numRows += (*it)->rows();
                 if ( (*it)->rows() ) {
                         if (numCols == 0) {
                                 numCols = (*it)->cols();
                         } else {
                                 assert((*it)->cols() == numCols);
                         }
                 }
         }
         if (!numRows) return MatrixType(0, 0);
         MatrixType C(numRows, numCols);
         numRows  = 0;
         for(it=blocks->begin(); it!=blocks->end(); it++) {
                 if (!(*it)->rows()) continue;
                 C.copySubMatrix(numRows, 0, *(*it));
                 numRows += (*it)->rows();
         }
         return C;
 }
 
 template <class MatrixType>
 MatrixType Matrix::BLOCKCOLUMN(const Matrix &M1, const Matrix &M2)
 {
         if ( M1.rows() && M2.rows() ) {
                  assert(M1.cols() == M2.cols());
         }
         MatrixType M(M1.rows() + M2.rows(), std::max(M1.cols(), M2.cols()));
         M.copySubMatrix(0, 0, M1);
         M.copySubMatrix(M1.rows(), 0, M2);
         return M;
 }
 
 template <class MatrixType>
 MatrixType Matrix::BLOCKROW(const Matrix &M1, const Matrix &M2)
 {
         assert(M1.rows() == M2.rows());
         MatrixType M(M1.rows(), M1.cols() + M2.cols());
         M.copySubMatrix(0, 0, M1);
         M.copySubMatrix(0, M1.cols(), M2);
         return M;
 }
 
 template <class MatrixType>
 MatrixType Matrix::BLOCKDIAG(const Matrix &M1, const Matrix &M2)
 {
         MatrixType M( M1.rows() + M2.rows(), M1.cols() + M2.cols() );
         M.setAllElements(0.0);
        M.copySubMatrix(0, 0, M1);
         M.copySubMatrix(M1.rows(), M1.cols(), M2);
        return M;
 }
 
 
 #endif
