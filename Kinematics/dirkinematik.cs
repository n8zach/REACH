//------------------------------------------------------------------------------
//  This file is part of the KUKA Education Tutorials Code Samples
//
//  <copyright file="dirKinematik.cs" company="KUKA Roboter GmbH">
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
    /// processes the direct kinematic
    /// </summary>
    public class DirKinematik
    {
        private float _x, _y, _z, _roll, _pitch, _yaw;
        
        /// <summary>
        /// the x-value of the position
        /// </summary>
        public float X
        {
            get { return _x; }
            private set { _x = value; }
        }

        /// <summary>
        /// the y-value of the position
        /// </summary>
        public float Y
        {
            get { return _y; }
            private set { _y = value; }
        }

        /// <summary>
        /// the z-value of the position
        /// </summary>
        public float Z
        {
            get { return _z; }
            private set { _z = value; }
        }

        /// <summary>
        /// the roll-angle (Euler) (rotation around z-Axis)
        /// </summary>
        public float Roll
        {
            get { return _roll; }
            private set { _roll = value; }
        }

        /// <summary>
        /// the pitch-angle (Euler) (rotation around y-Axis)
        /// </summary>
        public float Pitch
        {
            get { return _pitch; }
            private set { _pitch = value; }
        }

        /// <summary>
        /// the yaw-angle (Euler) (rotation around x-Axis)
        /// </summary>
        public float Yaw
        {
            get { return _yaw; }
            private set { _yaw = value; }
        }



        /// <summary>
        /// calculates the position and orientation of the TCP from the joint angles
        /// </summary>
        /// <param name="state">the internal state including the joint angles</param>
	    public DirKinematik()
        {
        }
        
        //takes 7 angles in
        public void Calculate(TransformationState state, float[] angles)
        {
            //Calculate the correct angle (combine DH Offset und the real angle)
            float[] theta = new float[8];

            int j = 0;
            for (int i = 0; i < state.IsUseable.Length; i++)
            {
                //combine only if the drive can be used
                if (state.IsUseable[i])
                {
                    theta[j + 1] = (float)(Math.Round(state.Theta[j + 1] + angles[i], 5));
                    j++;
                }
            }

            theta[j + 1] = state.Theta[j + 1];

            		    //calculate the translation matrix from robot coordinate system to flange coordinate system
            Matrix T6 = Matrix.GetTransformationMatrix(0, 7, theta, state.Alpha, state.A, state.D); 

            #region Position of the TCP

            //read the position of the TCP
		    _x = T6.Values[0, 3];
		    _y = T6.Values[1, 3];
		    _z = T6.Values[2, 3];

            #endregion


            #region Orientation of the TCP (in Z-Y-X Eulerangles)
            
            Pitch = (float)(Math.Atan2(-T6.Values[2, 0], 
                Math.Sqrt(T6.Values[0, 0] * T6.Values[0, 0] + T6.Values[1, 0] * T6.Values[1, 0])));

            //special cases
            if (Pitch == (float)(Math.PI / 2))
            {
                Roll = 0;
                Yaw = (float)(Math.Atan2(T6.Values[0,1], T6.Values[1,1]));
            }
            else if (Pitch == -(float)(Math.PI / 2))
            {
                Roll = 0;
                Yaw = (float)(-Math.Atan2(T6.Values[0,1], T6.Values[1,1]));
            }
            //default case
            else
            {
                Roll = (float)(Math.Atan2(T6.Values[2, 1], T6.Values[2, 2])); 
                Yaw = (float)(Math.Atan2(T6.Values[1, 0], T6.Values[0, 0]));
            }
           
            #endregion

            //if a wrong solution exists: send an error
            if (double.IsNaN(_x) || double.IsNaN(_y) || double.IsNaN(_z)
                || double.IsNaN(_roll) || double.IsNaN(_pitch) || double.IsNaN(_yaw))
            {
                throw new ArithmeticException("No possible solution exists for the passed joint values");
            }

            //adjust the results
            if (Math.Round(_roll, 5) <= -Math.Round(Math.PI, 5)) _roll += (float)(Math.PI * 2f);
            else if (Math.Round(_roll, 5) > Math.Round(Math.PI, 5)) _roll -= (float)(Math.PI * 2f);
            if (Math.Round(_pitch, 5) <= -Math.Round(Math.PI, 5)) _pitch += (float)(Math.PI * 2f);
            else if (Math.Round(_pitch, 5) > Math.Round(Math.PI, 5)) _pitch -= (float)(Math.PI * 2f);
            if (Math.Round(_yaw, 5) <= -Math.Round(Math.PI, 5)) _yaw += (float)(Math.PI * 2f);
            else if (Math.Round(_yaw, 5) > Math.Round(Math.PI, 5)) _yaw -= (float)(Math.PI * 2f);
        }
    }
}
