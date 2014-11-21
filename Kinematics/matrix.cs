//------------------------------------------------------------------------------
//  This file is part of the KUKA Education Tutorials Code Samples
//
//  <copyright file="Matrix.cs.cs" company="KUKA Roboter GmbH">
//     Copyright (c) KUKA Roboter GmbH.  All rights reserved.
//  </copyright>
//
//------------------------------------------------------------------------------
using System;
using System.Collections.Generic;
using System.Text;

namespace KUKA.Tutorials.Transformation
{
    /// <summary>
    /// the matrix class
    /// </summary>
    public class Matrix
    {
        float[,] _values;
        /// <summary>
        /// the values of the matrix
        /// </summary>
        public float[,] Values
        {
            get { return _values; }
            set { _values = value; }
        }


        /// <summary>
        /// constructor: creates a n x n matrix with all values 0
        /// </summary>
        /// <param name="n">the size of the matrix</param>
        public Matrix(int n)
        {
            _values = new float[n, n];
        }


        /// <summary>
        /// constructor: creates a new matrix using the parameter values
        /// </summary>
        /// <param name="values">the values for the matrix</param>
        public Matrix(float[,] values)
        {
            if (values.GetLength(0) == values.GetLength(1))
            {
                _values = values;
            }
            else throw new NotSupportedException("The matrix must be symmetric");
        }


        /// <summary>
        /// addition operator for matrices
        /// </summary>
        /// <param name="mat1">first matric for addition</param>
        /// <param name="mat2">second matric for addition</param>
        /// <returns>the resulting matrix after addition</returns>
        public static Matrix operator +(Matrix mat1, Matrix mat2)
        {
            Matrix mat = new Matrix(mat1.Values.GetLength(0));
            for (int i = 0; i < mat1.Values.GetLength(0); i++)
            {
                for (int j = 0; j < mat1.Values.GetLength(1); i++)
                {
                    mat.Values[i, j] = mat1.Values[i, j] + mat2.Values[i, j];
                }
            }
            return mat;
        }


        /// <summary>
        /// division operator of matrices
        /// </summary>
        /// <param name="mat1">first matrix for division</param>
        /// <param name="mat2">second matrix for division</param>
        /// <returns>the resulting matrix after the division</returns>
        public static Matrix operator /(Matrix mat1, Matrix mat2)
        {
            return mat1.times(mat2.invertSpezial());
        }


        /// <summary>
        /// subtraction operator for matrices
        /// </summary>
        /// <param name="mat1">first matric for subtraction</param>
        /// <param name="mat2">second matric for subtraction</param>
        /// <returns>the resulting matrix after subtraction</returns>
        public static Matrix operator -(Matrix mat1, Matrix mat2)
        {
            Matrix mat = new Matrix(mat1.Values.GetLength(0));
            for (int i = 0; i < mat1.Values.GetLength(0); i++)
            {
                for (int j = 0; j < mat1.Values.GetLength(1); j++)
                {
                    mat.Values[i, j] = mat1.Values[i, j] - mat2.Values[i, j];
                }
            }
            return mat;
        }


        /// <summary>
        /// multiplication operator for matrices
        /// </summary>
        /// <param name="mat1">first matric for multiplication</param>
        /// <param name="mat2">second matric for multiplication</param>
        /// <returns>the resulting matrix after multiplication</returns>
        public static Matrix operator *(Matrix mat1, Matrix mat2)
        {
            return mat1.times(mat2);
        }


        /// <summary>
        /// simple matrix multiplication
        /// </summary>
        /// <param name="mat1">the matrix to multiply with the current one</param>
        /// <returns>the result of the matrix multiplication</returns>
        public Matrix times(Matrix mat1)
        {
            if ((_values.GetLength(0) != mat1.Values.GetLength(1)) ||
                (_values.GetLength(1) != mat1.Values.GetLength(0)))
                throw new NotSupportedException("The length of the matrices is not identical!");
            float[,] ret = new float[mat1.Values.GetUpperBound(0) + 1, mat1.Values.GetUpperBound(1) + 1];

            for (int i = 0; i < _values.GetLength(0); i++)
            {
                for (int j = 0; j < _values.GetLength(1); j++)
                {
                    for (int k = 0; k < _values.GetLength(0); k++)
                    {
                        ret[i, j] += _values[i, k] * mat1.Values[k, j];
                    }
                }
            }
            return new Matrix(ret);
        }


        /// <summary>
        /// simple multiplication between matrix and vector
        /// </summary>
        /// <param name="vector">the vector to be multiplied with the current matrix</param>
        /// <returns>the result of the multiplication</returns>
        public float[] times(float[] vector)
        {
            if (vector.GetLength(0) != _values.GetLength(1))
                throw new NotSupportedException("the length of the matrice must be identical with the length of the vector!");
            float[] ret = new float[_values.GetLength(0)];
            for (int i = 0; i < _values.GetLength(1); i++)
            {
                for (int j = 0; j < _values.GetLength(0); j++)
                {
                    ret[i] += _values[i, j] * vector[j];
                }
            }
            return ret;
        }


        /// <summary>
        /// transposes the the current matrix
        /// </summary>
        /// <returns>the transposed matrix</returns>
        public Matrix transpose()
        {
            if (_values.GetLength(0) != Values.GetLength(1))
                throw new NotSupportedException("the matrix must be symmetric!");
            float[,] ret = new float[_values.GetLength(0), _values.GetLength(0)];
            for (int i = 0; i < _values.GetLength(0); i++)
            {
                for (int j = 0; j < _values.GetLength(1); j++)
                {
                    ret[i, j] = _values[j, i];
                }
            }
            return new Matrix(ret);
        }


        /// <summary>
        /// inverts the current matrix
        /// WARNING: works only correct on translational matrices
        /// </summary>
        /// <returns>the inverted matrix</returns>
        public Matrix invertSpezial()
        {
            if (_values.GetLength(0) != Values.GetLength(1))
                throw new NotSupportedException("the matrix must be symmetric!");
            float[,] ret = new float[_values.GetLength(0), _values.GetLength(0)];
            float[,] temp = new float[_values.GetLength(0) - 1, _values.GetLength(1) - 1];
            float[] vec = new float[_values.GetLength(0) - 1];

            //transpose the rotational part
            for (int i = 0; i < _values.GetLength(0); i++)
            {
                for (int j = 0; j < _values.GetLength(1); j++)
                {
                    if (i == _values.GetLength(0) - 1 || j == _values.GetLength(1) - 1)
                    {
                        if (i != _values.GetLength(0) - 1 && j == _values.GetLength(1) - 1) vec[i] = _values[i, j];
                    }
                    else
                    {
                        ret[i, j] = _values[j, i];
                        temp[i, j] = -_values[j, i];
                    }
                }
            }
            Matrix tempM = new Matrix(temp);
            float[] vecNew = tempM.times(vec);

            //negate the transposed rotational part and multiply it with the translational vector
            ret[_values.GetLength(0) - 1, _values.GetLength(1) - 1] = 1;

            for (int i = 0; i < _values.GetLength(0) - 1; i++)
            {
                ret[i, _values.GetLength(1) - 1] = vecNew[i];
            }

            ret[_values.GetLength(0) - 1, _values.GetLength(1) - 1] = 1;


            return new Matrix(ret);
        }


        /// <summary>
        /// conferts the matrix into a string
        /// </summary>
        /// <returns>the string representation of the matrix</returns>
        public override string ToString()
        {
            string ret = "";
            for (int i = 0; i < _values.GetLength(0); i++)
            {
                for (int j = 0; j < _values.GetLength(1); j++)
                {
                    ret += _values[i, j] + "\t\t";
                }
                ret += "\n";
            }
            ret += "\n\n";
            return ret;
        }


        /// <summary>
        /// calculates the rotational matrix Rz(yaw)*Ry(pitch)*Rx(roll)
        /// </summary>
        /// <param name="roll">the roll Winkel</param>
        /// <param name="pitch">the pitch Winkel</param>
        /// <param name="yaw">the yaw Winkel</param>
        /// <returns>the rotational matrix</returns>
        public static float[,] GetRotationMatrix(float alpha, float beta, float gamma)
        {
            float[,] rotation = new float[4, 4];
            rotation[0, 0] = (float)(Math.Cos(alpha) * Math.Cos(beta));
            rotation[0, 1] = (float)(Math.Cos(alpha) * Math.Sin(beta) * Math.Sin(gamma)
                    - Math.Sin(alpha) * Math.Cos(gamma));
            rotation[0, 2] = (float)(Math.Cos(alpha) * Math.Sin(beta) * Math.Cos(gamma)
                    + Math.Sin(alpha) * Math.Sin(gamma));
            rotation[0, 3] = 0;
            rotation[1, 0] = (float)(Math.Sin(alpha) * Math.Cos(beta));
            rotation[1, 1] = (float)(Math.Sin(alpha) * Math.Sin(beta) * Math.Sin(gamma)
                    + Math.Cos(alpha) * Math.Cos(gamma));
            rotation[1, 2] = (float)(Math.Sin(alpha) * Math.Sin(beta) * Math.Cos(gamma)
                    - Math.Cos(alpha) * Math.Sin(gamma));
            rotation[1, 3] = 0;
            rotation[2, 0] = (float)(-Math.Sin(beta));
            rotation[2, 1] = (float)(Math.Cos(beta) * Math.Sin(gamma));
            rotation[2, 2] = (float)(Math.Cos(beta) * Math.Cos(gamma));
            rotation[2, 3] = 0;
            rotation[3, 0] = 0;
            rotation[3, 1] = 0;
            rotation[3, 2] = 0;
            rotation[3, 3] = 1;
            return rotation;
        }


        /// <summary>
        /// calculates the rotation matrix around the x-axis
        /// </summary>
        /// <param name="angle">the angle for the rotation</param>
        /// <returns>the rotation matrix</returns>
        public static Matrix getRotationMatrixX(float angle)
        {
            float[,] rotation = new float[4, 4];
            rotation[0, 0] = 1;
            rotation[0, 1] = 0;
            rotation[0, 2] = 0;
            rotation[0, 3] = 0;
            rotation[1, 0] = 0;
            rotation[1, 1] = (float)Math.Cos(angle);
            rotation[1, 2] = -(float)Math.Sin(angle);
            rotation[1, 3] = 0;
            rotation[2, 0] = 0;
            rotation[2, 1] = (float)Math.Sin(angle);
            rotation[2, 2] = (float)Math.Cos(angle);
            rotation[2, 3] = 0;
            rotation[3, 0] = 0;
            rotation[3, 1] = 0;
            rotation[3, 2] = 0;
            rotation[3, 3] = 1;
            return new Matrix(rotation);
        }


        /// <summary>
        /// calculates the rotation matrix around the y-axis
        /// </summary>
        /// <param name="angle">the angle for the rotation</param>
        /// <returns>the rotation matrix</returns>
        public static Matrix getRotationMatrixY(float angle)
        {
            float[,] rotation = new float[4, 4];
            rotation[0, 0] = (float)Math.Cos(angle);
            rotation[0, 1] = 0;
            rotation[0, 2] = (float)Math.Sin(angle);
            rotation[0, 3] = 0;
            rotation[1, 0] = 0;
            rotation[1, 1] = 1;
            rotation[1, 2] = 0;
            rotation[1, 3] = 0;
            rotation[2, 0] = -(float)Math.Sin(angle);
            rotation[2, 1] = 0;
            rotation[2, 2] = (float)Math.Cos(angle);
            rotation[2, 3] = 0;
            rotation[3, 0] = 0;
            rotation[3, 1] = 0;
            rotation[3, 2] = 0;
            rotation[3, 3] = 1;
            return new Matrix(rotation);
        }


        /// <summary>
        /// calculates the rotation matrix around the z-axis
        /// </summary>
        /// <param name="angle">the angle for the rotation</param>
        /// <returns>the rotation matrix</returns>
        public static Matrix getRotationMatrixZ(float angle)
        {
            float[,] rotation = new float[4, 4];
            rotation[0, 0] = (float)Math.Cos(angle);
            rotation[0, 1] = -(float)Math.Sin(angle);
            rotation[0, 2] = 0;
            rotation[0, 3] = 0;
            rotation[1, 0] = (float)Math.Sin(angle);
            rotation[1, 1] = (float)Math.Cos(angle);
            rotation[1, 2] = 0;
            rotation[1, 3] = 0;
            rotation[2, 0] = 0;
            rotation[2, 1] = 0;
            rotation[2, 2] = 1;
            rotation[2, 3] = 0;
            rotation[3, 0] = 0;
            rotation[3, 1] = 0;
            rotation[3, 2] = 0;
            rotation[3, 3] = 1;
            return new Matrix(rotation);
        }


        /// <summary>
        /// calculates all the translation matrices from the coordinate system specified by lowIndex to the coordinate 
        /// system indecated by highIndex
        /// </summary>
        /// <param name="lowIndex">the start for calculating the translation matrices</param>
        /// <param name="highIndex">the end for calculation the translation matrices</param>
        /// <param name="theta">the joint angles of DH</param>
        /// <param name="alpha">the alpha values of DH</param>
        /// <param name="a">the a values of DH</param>
        /// <param name="d">the d values of DH</param>
        /// <returns></returns>
        public static Matrix[] GetTranslationMatrixes(int lowIndex, int highIndex, float[] theta, float[] alpha, float[] a, float[] d)
        {
            Matrix[] matrices = new Matrix[highIndex - lowIndex];
            float[,] values;
            for (int i = lowIndex + 1; i < highIndex + 1; i++)
            {
                values = new float[4, 4];
                //fill the values for each translation matrix
                values[0, 0] = (float)(Math.Cos(theta[i]));
                values[0, 1] = -(float)(Math.Sin(theta[i]));
                values[0, 2] = 0;
                values[0, 3] = a[i - 1];
                values[1, 0] = (float)(Math.Cos(alpha[i - 1]) * Math.Sin(theta[i]));
                values[1, 1] = (float)(Math.Cos(alpha[i - 1]) * Math.Cos(theta[i]));
                values[1, 2] = -(float)(Math.Sin(alpha[i - 1]));
                values[1, 3] = -(float)(d[i] * Math.Sin(alpha[i - 1]));
                values[2, 0] = (float)(Math.Sin(alpha[i - 1]) * Math.Sin(theta[i]));
                values[2, 1] = (float)(Math.Sin(alpha[i - 1]) * Math.Cos(theta[i]));
                values[2, 2] = (float)(Math.Cos(alpha[i - 1]));
                values[2, 3] = (float)(d[i] * Math.Cos(alpha[i - 1]));
                values[3, 0] = 0;
                values[3, 1] = 0;
                values[3, 2] = 0;
                values[3, 3] = 1;

                matrices[i - (lowIndex + 1)] = new Matrix(values);
            }

            //multiply the translation matrices
            Matrix T6 = matrices[highIndex - lowIndex - 1];

            Matrix[] ret = new Matrix[highIndex - lowIndex];
            ret[ret.Length - 1] = matrices[matrices.Length - 1];

            Matrix temp = null;
            for (int i = 1; i < highIndex - lowIndex; i++)
            {
                temp = matrices[highIndex - lowIndex - 1 - i];
                T6 = temp.times(T6);
                ret[highIndex - lowIndex - 1 - i] = T6;
            }
            return ret;
        }


        /// <summary>
        /// calculates the translation matrix from the coordinate system specified by lowIndex to the coordinate 
        /// system indecated by highIndex
        /// </summary>
        /// <param name="lowIndex">the start coordinate system</param>
        /// <param name="highIndex">the destination coordinate system</param>
        /// <param name="theta">the joint angles of DH</param>
        /// <param name="alpha">the alpha values of DH</param>
        /// <param name="a">the a values of DH</param>
        /// <param name="d">the d values of DH</param>
        /// <returns></returns>
        public static Matrix GetTransformationMatrix(int lowIndex, int highIndex, float[] theta, float[] alpha, float[] a, float[] d)
        {
            Matrix[] matrices = new Matrix[highIndex];
            float[,] values;
            for (int i = lowIndex + 1; i < highIndex + 1; i++)
            {
                values = new float[4, 4];
                //fill the values
                values[0, 0] = (float)(Math.Cos(theta[i]));
                values[0, 1] = -(float)(Math.Sin(theta[i]));
                values[0, 2] = 0;
                values[0, 3] = a[i - 1];
                values[1, 0] = (float)(Math.Cos(alpha[i - 1]) * Math.Sin(theta[i]));
                values[1, 1] = (float)(Math.Cos(alpha[i - 1]) * Math.Cos(theta[i]));
                values[1, 2] = -(float)(Math.Sin(alpha[i - 1]));
                values[1, 3] = -(float)(d[i] * Math.Sin(alpha[i - 1]));
                values[2, 0] = (float)(Math.Sin(alpha[i - 1]) * Math.Sin(theta[i]));
                values[2, 1] = (float)(Math.Sin(alpha[i - 1]) * Math.Cos(theta[i]));
                values[2, 2] = (float)(Math.Cos(alpha[i - 1]));
                values[2, 3] = (float)(d[i] * Math.Cos(alpha[i - 1]));
                values[3, 0] = 0;
                values[3, 1] = 0;
                values[3, 2] = 0;
                values[3, 3] = 1;
                matrices[i - (lowIndex + 1)] = new Matrix(values);
            }

            //multiply the matrices
            Matrix T6 = matrices[0];
            Matrix temp = null;
            for (int i = lowIndex + 1; i < highIndex; i++)
            {
                temp = matrices[i - lowIndex];
                T6 = T6.times(temp);
            }
            return T6;
        }

        public float GetDet()
        {
            float[,] val = this.Values;
            return val[0, 0] * val[1, 1] * val[2, 2] + val[0, 1] * val[1, 2] * val[2, 0] + val[0, 2] * val[1, 0] * val[2, 1]
                - val[0, 0] * val[1, 2] * val[2, 1] - val[0, 1] * val[1, 0] * val[2, 2] - val[0, 2] * val[1, 1] * val[2, 0];
        }
    }
}
