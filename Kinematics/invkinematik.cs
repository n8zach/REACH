//------------------------------------------------------------------------------
//  This file is part of the KUKA Education Tutorials Code Samples
//
//  <copyright file="invKinematik.cs" company="KUKA Roboter GmbH">
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
    /// processes the inverse kinematics
    /// </summary>
    public class InvKinematik
    {
        /// <summary>
        /// the solutions for the different joint angles [solutionnr.][angle]
        /// </summary>  
        private float[][] _theta = new float[8][];

        /// <summary>
        /// the best solution
        /// </summary>
        private float[] _bestSolution = new float[7];

        /// <summary>
        /// the current joint angles
        /// </summary>
        private float[] currentJointAngles;

        public float[] BestSolution
        {
            get { return _bestSolution; }
            private set { _bestSolution = value; }
        }

        public InvKinematik(float[] currentJointAngles)
        {
            this.currentJointAngles = currentJointAngles;
        }

        /// <summary>
        /// Calculates solutions from a given position and orientation of the TCP
        /// also finds the best solution given the current angles.
        /// </summary>
        /// <param name="x">x-value of the TCP</param>
        /// <param name="y">y-value of the TCP</param>
        /// <param name="z">z-value of the TCP</param>
        /// <param name="roll">roll-angle of the TCP</param>
        /// <param name="pitch">pitch-angle of the TCP</param>
        /// <param name="yaw">yaw-angle of the TCP</param>
        /// <param name="state">the state including the DH parameter</param>
        public void Calculate(float x, float y, float z, float roll, float pitch, float yaw, TransformationState state)
        {
            for (int i = 0; i < _theta.GetLength(0); i++)
            {
                _theta[i] = new float[6];

            }

            //calculation of the rotationmatrix for the orientation of the TCP
            Matrix rot0_7 = Matrix.getRotationMatrixZ(yaw).times(Matrix.getRotationMatrixY(pitch).times(Matrix.getRotationMatrixX(roll)));

            //the normal of the TCP in coordinate system 0
            float[] n = new float[] { rot0_7.Values[0, 2], rot0_7.Values[1, 2], rot0_7.Values[2, 2], 1 };

            #region calculate intersection point of the wrist

            float[] p04K0 = new float[4];
            p04K0[0] = (float)(Math.Round(x - state.D[7] * n[0], 5));
            p04K0[1] = (float)(Math.Round(y - state.D[7] * n[1], 5));
            p04K0[2] = (float)(Math.Round(z - state.D[7] * n[2], 5));
            p04K0[3] = 1;

            #endregion

            #region Theta1 - 3

            #region Angle 1
            /*
             * Theta1:
             */

            //Solution 1 (NOTICE: ATAN2 - definition may differ):
            _theta[0][0] = _theta[1][0] = _theta[2][0] = _theta[3][0] = 
                (float)(Atan2(-p04K0[2], p04K0[0]));
            
            //Solution 2 (NOTICE: ATAN2 - definition may differ):
            _theta[4][0] = _theta[5][0] = _theta[6][0] = _theta[7][0] = 
                (float)(Atan2(-p04K0[2], p04K0[0]) + Math.PI);

            #endregion

            #region Angle 3
            /*
             * Theta3:
             */

            //For Angles[0][0]

            Matrix T01 = Matrix.GetTransformationMatrix(0, 1, new float[] {0, _theta[0][0] + state.Theta[1] }, state.Alpha, state.A, state.D);

            //p14 = p04 - p01
            float[] p14K0 = new float[4];
            p14K0[0] = (float)(p04K0[0] - T01.Values[0, 3]);
            p14K0[1] = (float)(p04K0[1] - T01.Values[1, 3]);
            p14K0[2] = (float)(p04K0[2] - T01.Values[2, 3]);
            p14K0[3] = 1;

            //the square of the value of vector p14K0
            float p14BetQuad = (float)(p14K0[0] * p14K0[0] + p14K0[1] * p14K0[1] + p14K0[2] * p14K0[2]);

            if (Math.Round(p14BetQuad, 3) > Math.Round(Math.Pow(Math.Abs(state.D[4]) + Math.Abs(state.A[2]), 2),3))
                throw new Exception("No possible solution exists for the following pose:" +
                    " Position: " + x + ", " + y + ", " + z + " Orientation: " + roll + ", " + pitch + ", " + yaw);

            //the fraction for the arccos
            float help3 = (float)(((state.A[2] * state.A[2]) + (state.D[4] * state.D[4]) - p14BetQuad)
                / (2 * Math.Abs(state.A[2]) * Math.Abs(state.D[4])));

            //round to avoid problems near -1 and 1
            float phi;

            if (help3 > 1)
                phi = 0;
            else if (help3 < -1)
                phi = (float)Math.PI;
            else
                phi = (float)(Math.Acos(help3));

            //Solution 1:
            _theta[0][2] = _theta[1][2] = (float)(Math.PI - phi); //for Solution1 of Theta1
            _theta[4][2] = _theta[5][2] = -_theta[0][2];          //for Solution2 of Theta1

            //Solution 2:
            _theta[2][2] = _theta[3][2] = (float)(Math.PI + phi); //for Solution1 of Theta1   
            _theta[6][2] = _theta[7][2] = -_theta[2][2];          //for Solution2 of Theta1

            #endregion

            #region Angle 2
            /*
             * Theta2
             */

            //Rotationmatrix to get from coordinate system 0 to coordinate system 1
            T01.Values[0, 3] = 0;
            T01.Values[1, 3] = 0;
            T01.Values[2, 3] = 0;
            float[] p14K1 = T01.transpose().times(p14K0);

            //the fraction for the arccos
            float help2 = (float)((state.A[2] * state.A[2] + p14BetQuad - state.D[4] * state.D[4]) /
                (2 * Math.Abs(state.A[2]) * Math.Sqrt(p14BetQuad)));

            //float beta1 = (float)(Math.Atan2(-p14K1[2], -p14K1[0]));
            float beta1 = (float)(Atan2(p14K1[0], p14K1[2]));

            //round to avoid problems near -1 and 1
            float beta2;

            if (help2 > 1)
                beta2 = 0;
            else if (help2 < -1)
                beta2 = (float)Math.PI;
            else
                beta2 = (float)(Math.Acos(help2));




            //Solutions

            //for Solution 1 of Theta1 and Solution 1 of Theta3
            _theta[0][1] = _theta[1][1] = -(beta1 + beta2);
            //for Solution 1 of Theta1 and Solution 2 of Theta3
            _theta[2][1] = _theta[3][1] = -(beta1 - beta2);
            //for Solution 2 of Theta1 and Solution 1 of Theta3
            _theta[4][1] = _theta[5][1] = -_theta[0][1];
            //for Solution 2 of Theta1 and Solution 2 of Theta3
            _theta[6][1] = _theta[7][1] = -_theta[2][1];

            #endregion

            #endregion

            #region Angles 4 - 6

            #region Angles 5

            /*
             * Theta5:
             */

            //Translationmatrices to get from coordinate system 0 to coordinate system 4
            //for Solution 1 of Angles 3
            Matrix A0_4S1 = Matrix.GetTransformationMatrix(0, 4, new float[] {state.Theta[0], _theta[0][0] + state.Theta[1], 
                _theta[0][1] + state.Theta[2], _theta[0][2] + state.Theta[3], state.Theta[4]}, state.Alpha, state.A, 
                state.D);
            //for Solution 2 of Angles 3
            Matrix A0_4S2 = Matrix.GetTransformationMatrix(0, 4, new float[] {state.Theta[0], _theta[2][0] + state.Theta[1], 
                _theta[2][1] + state.Theta[2], _theta[2][2] + state.Theta[3], state.Theta[4]}, state.Alpha, state.A,
                state.D);

            //for Solution 1 of Angles 3
            float[] z4K0S1 = new float[] { A0_4S1.Values[0, 2], A0_4S1.Values[1, 2], A0_4S1.Values[2, 2], 0 };

            //for Solution 2 of Angles 3
            float[] z4K0S2 = new float[] { A0_4S2.Values[0, 2], A0_4S2.Values[1, 2], A0_4S2.Values[2, 2], 0 };

            float help5S1 = (float)((z4K0S1[0] * n[0]) + (z4K0S1[1] * n[1]) + (z4K0S1[2] * n[2]));
            float help5S2 = (float)((z4K0S2[0] * n[0]) + (z4K0S2[1] * n[1]) + (z4K0S2[2] * n[2]));

            //for solution 1 of Angles 1 and solution 1 of Theta3
            //solution 1
            _theta[0][4] = (float)Math.Acos(help5S1);
            //solution 2
            _theta[1][4] = -_theta[0][4];

            //for solution 1 of Angles 1 and solution 2 of Theta3
            //solution 1
            _theta[2][4] = (float)Math.Acos(help5S2);
            //solution 2
            _theta[3][4] = -_theta[2][4];

            //for solutuin 2 of Angles 1 and solution 1 of Theta3
            //solution 1
            _theta[4][4] = _theta[0][4];
            //solution 2
            _theta[5][4] = -_theta[4][4];
            //for solutuin 2 of Angles 1 and solution 2 of Theta3
            //solution 1
            _theta[6][4] = _theta[2][4];
            //solution 2
            _theta[7][4] = -_theta[6][4];
            

            #endregion

            #region Angles 4 + 6
            /*
             * Angles 4 + 6
             */

            //Translationmatrix to get from coordinate system 4 to coordinate system 0
            //for solution 1 of Theta3
            Matrix A4_0S1 = A0_4S1.transpose();
            //for solution 2 of Theta3
            Matrix A4_0S2 = A0_4S2.transpose();

            //Translationmatrix to get from coordinate system 7 to coordinate system 4
            //for solution 1 of Theta3
            Matrix A7_4S1 = A4_0S1.times(rot0_7);
            //for solution 2 of Theta3
            Matrix A7_4S2 = A4_0S2.times(rot0_7);

            //Angles 4 solutions
            _theta[0][3] = (float)(Atan2(A7_4S1.Values[1, 2], A7_4S1.Values[0, 2]));
            _theta[2][3] = (float)(Atan2(A7_4S2.Values[1, 2], A7_4S2.Values[0, 2]));
            _theta[4][3] = (float)(_theta[0][3] + Math.PI);
            _theta[6][3] = (float)(_theta[2][3] + Math.PI);

            _theta[1][3] = _theta[4][3];
            _theta[3][3] = _theta[6][3];
            _theta[5][3] = _theta[0][3];
            _theta[7][3] = _theta[2][3];


            //Angles 6 solutions
            _theta[0][5] = (float)(Atan2(-A7_4S1.Values[2, 1], A7_4S1.Values[2, 0]));
            _theta[2][5] = (float)(Atan2(-A7_4S2.Values[2, 1], A7_4S2.Values[2, 0]));
            _theta[4][5] = _theta[0][5];
            _theta[6][5] = _theta[2][5];

            _theta[1][5] = _theta[0][5] + (float)Math.PI;
            _theta[3][5] = _theta[2][5] + (float)Math.PI;
            _theta[5][5] = _theta[4][5] + (float)Math.PI;
            _theta[7][5] = _theta[6][5] + (float)Math.PI;

            #endregion

            #region Singularity Angles 4 + 6

            //singular position if Theta5 = 0: add the rotation of joint 4 to the one of joint 6. 
            //the half of this rotation is set for each joint
            //float help46;
            //for (int i = 0; i < _theta.GetLength(0); i++)
            //{
            //    if (Math.Round(Theta[i][4], 2) == 0)
            //    {
            //        help46 = (_theta[i][3] + _theta[i][5]) % (float)(2f * Math.PI);
            //        //solution 1 of Angles 4 + 6
            //        _theta[i][3] = _theta[i][5] = help46 / 2f;
            //        //solution 2 of Angles 4 + 6
            //        if (i % 2 == 1)
            //        {
            //            _theta[i][3] = _theta[i][5] += (float)Math.PI;
            //        }
            //    }
            //}
            

            #endregion

            #endregion

            #region normalize the thetas values

            for (int i = 0; i < _theta.GetLength(0); i++)
            {
                for (int j = 0; j < _theta[i].GetLength(0); j++)
                {
                    _theta[i][j] = (float)(_theta[i][j] % (Math.PI * 2));
                    if (_theta[i][j] < -(float)Math.PI)
                        _theta[i][j] += (float)(2f * Math.PI);
                    else if (_theta[i][j] > (float)Math.PI)
                        _theta[i][j] -= (float)(2f * Math.PI);
                }
            }

            #endregion

            #region find the best angle

            //sort the solutions in a way, that the solution nearest to the given joint is first
            int min_deviationID = 0;
            float min_deviation = float.MaxValue;
            int k = 0;
            if (currentJointAngles != null && currentJointAngles.Length > 0)
            {
                float deviation;
                float jointangle;
                for (int i = 0; i < _theta.GetLength(0); i++)
                {
                    deviation = 0;
                    k = 0;
                    for (int j = 0; j < _theta[i].Length; j++)
                    {
                        //if the drive is set locked, use the next input value
                        if (!state.IsUseable[k])
                            k++;

                        jointangle = currentJointAngles[k] % (float)(Math.PI * 2f);
                        if (jointangle > Math.PI)
                            jointangle -= (float)(Math.PI * 2f);
                        if (jointangle < -Math.PI)
                            jointangle += (float)(Math.PI * 2f);
                        deviation += Math.Abs(_theta[i][j] - jointangle);
                        k++;
                    }

                    //find the minimum deviation
                    if (deviation < min_deviation)
                    {
                        min_deviation = deviation;
                        min_deviationID = i;
                    }
                }
            }
            float[] help;
            help = _theta[0];
            _theta[0] = _theta[min_deviationID];
            _theta[min_deviationID] = help;

            _bestSolution = new float[_theta[0].Length + 1];

            k = 0;
            for (int i = 0; i < _bestSolution.Length; i++)
            {
                //if the drive is useable, insert it into the response
                if (state.IsUseable[i])
                {
                    _bestSolution[i] = (float)(_theta[0][k]);
                    k++;
                }
                //else add zero
                else
                    _bestSolution[i] = 0;
            }

            currentJointAngles = _bestSolution;
            #endregion

        }


        /// <summary>
        /// improved atan2 implementation
        /// </summary>
        /// <param name="y">x-value</param>
        /// <param name="x">y-value</param>
        /// <returns>atan2 value</returns>
        double Atan2(double y, double x)
        {
            if (Math.Round(y, 5) == 0 && Math.Round(x, 5) == 0)
                return 0;
            else
                return Math.Atan2(y, x);
        }
    }
}