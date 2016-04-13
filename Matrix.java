package fyp;
//Encapsulates a 4x4 matrix
public class Matrix {
//create a 2d array
  protected int numRows;
  protected int numCols;
  public double[][] elements;
  //Constructs new 4x4 matrix, initializes to it to zeros
  Matrix() {
    numRows= 4; numCols= 4;
    elements= new double[numRows][numCols];
    for(int r=0; r<numRows; r++)
      for(int c=0; c<numCols; c++) {
        elements[r][c]= 0.0;
      }
  }

  Matrix(int rows, int cols) {
    numRows= rows; numCols= cols;
    elements= new double[numRows][numCols];
    for(int r=0; r<numRows; r++)
      for(int c=0; c<numCols; c++) {
        elements[r][c]= 0.0;
      }
  }

  // Copy constructor
  Matrix(Matrix m) {
    numRows= m.numRows;
    numCols= m.numCols;
    elements= new double[numRows][numCols];
    for(int r=0; r<numRows; r++)
      for(int c=0; c<numCols; c++)
        this.elements[r][c]= m.elements[r][c];
  }

  public int getNumRows() {
    return numRows;
  }

  public int getNumCols() {
    return numCols;
  }

// a new matrix which is equal to the difference of this - m
  public Matrix subtractFrom (Matrix m) {
    if(numRows!=m.numRows || numCols!=m.numCols) {
      System.out.println("dimensions bad in addTo()");
      System.exit(1);
    }
    Matrix ret= new Matrix(numRows, numCols);

    for(int r=0; r<numRows; r++)
      for(int c=0; c<numCols; c++)
        ret.elements[r][c]= this.elements[r][c] - m.elements[r][c];

    return ret;
  }


// a new matrix which is equal to the add of this + m
  public Matrix plusFrom (Matrix m) {
    if(numRows!=m.numRows || numCols!=m.numCols) {
      System.out.println("dimensions bad in addTo()");
      System.exit(1);
    }
    Matrix ret= new Matrix(numRows, numCols);

    for(int r=0; r<numRows; r++)
      for(int c=0; c<numCols; c++)
        ret.elements[r][c]= this.elements[r][c] + m.elements[r][c];
    return ret;
  }
     
//a new matrix which is equal to the product of this*m
  public Matrix multiply (Matrix m) {
    if(numCols!=m.numRows) {
      System.out.println("dimensions bad in multiply()");
      System.exit(1);
    }

    Matrix ret= new Matrix(numRows, m.numCols);

    for(int r=0; r<numRows; r++)
      for(int c=0; c<m.numCols; c++) {
    	  for(int k=0;k<numCols; k++) {
    		  ret.elements[r][c] += this.elements[r][k] * m.elements[k][c];
    	  }
      }

    return ret;
  }

//Scalar multiplication- multiplies each element by a scalar
  public Matrix multiply (double s) {
    Matrix ret= new Matrix(numRows, numCols);
    for(int i=0; i<numRows; i++)
      for(int j=0; j<numCols; j++)
    	  ret.elements[i][j]= elements[i][j]*s;
    return ret;
  }

  //the transposed matrix with dimensions numCols x numRows
  public Matrix transpose() {
    Matrix ret= new Matrix(numCols, numRows);

    for(int r=0; r<numRows; r++)
      for(int c=0;c<numCols; c++)
    	  ret.elements[c][r]= elements[r][c];
    return ret;
  }

//true if and only if all elements of the matrix equal val
  public boolean equals(double val) {
    for(int r=0; r<numRows; r++)
      for(int c=0; c<numCols; c++)
    	  if(Math.abs(elements[r][c]-val) > .0001) return false;
    return true;
  }

//the dot product (scalar product)
  public double dot(Matrix m) {
    if(numRows!=m.numRows || numCols!=m.numCols) {
      System.out.println("dimensions bad in dot()");
      System.exit(1);
    }
    
    double sum= 0;
    for(int r=0; r<numRows; r++)
      for(int c=0; c<numCols; c++)
        sum += this.elements[r][c] * m.elements[r][c];
    
    return sum;
  }

// an MxN matrix which is the matrix's pseudoinverse.
  public Matrix pseudoInverse() {
    int r,c;
    int k=1;
    Matrix ak= new Matrix(numRows, 1);
    Matrix dk, ck, bk, R_plus;

    for(r=0; r<numRows; r++)
      ak.elements[r][0]= this.elements[r][0];

    if(!ak.equals(0.0)) {
      R_plus= ak.transpose().multiply( 1.0/( ak.dot(ak) ) );
    }
    else {
      R_plus= new Matrix(1, numCols);
    }

    while(k< this.numCols) {

      for(r=0; r<numRows; r++)
    	  ak.elements[r][0]= this.elements[r][k];

      dk= R_plus.multiply(ak);
      Matrix T= new Matrix(numRows, k);
      
      for(r=0; r<numRows; r++)
    	  for(c=0; c<k; c++)
    		  T.elements[r][c]= this.elements[r][c];

      ck= ak.subtractFrom( T.multiply(dk) );

      if( !ck.equals(0.0) ) {
    	  bk= ck.transpose().multiply( 1.0/(ck.dot(ck)) );
      }
      else {
    	  bk= dk.transpose().multiply( 1.0/( 1.0 + dk.dot(dk) ) ).multiply(R_plus);
      }

      Matrix N= R_plus.subtractFrom( dk.multiply(bk) );
      R_plus= new Matrix(N.numRows+1, N.numCols);

      for(r=0; r< N.numRows; r++)
    	  for(c=0; c< N.numCols; c++)
    		  R_plus.elements[r][c]= N.elements[r][c];
      for(c=0; c<N.numCols; c++)
    	  R_plus.elements[R_plus.numRows-1][c]= bk.elements[0][c];
      
      k++;
    }
    return R_plus;
  }
  
  public static void setGoal(Matrix goal, int goalX, int goalY) {
      goal.elements[0][0]= goalX;
      goal.elements[1][0]= goalY;
  }
}