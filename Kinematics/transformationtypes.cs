//------------------------------------------------------------------------------
//  This file is part of the KUKA Education Tutorials Code Samples
//
//  <copyright file="TransformationTypes.cs" company="KUKA Roboter GmbH">
//     Copyright (c) KUKA Roboter GmbH.  All rights reserved.
//  </copyright>
//
//------------------------------------------------------------------------------
using System;


namespace KUKA.Tutorials.Transformation
{
    /// <summary>
    /// Transformation State
    /// </summary>
    public class TransformationState
    {
        private float[] _d;
        /// <summary>
        /// Denavit-Hartenberg: the axis distance measured along Z
        /// </summary>
        public float[] D
        {
            get { return _d; }
            //set { _d = value; }
        }

        private float[] _a;
        /// <summary>
        /// Denavit-Hartenberg: the axis distance measured along X
        /// </summary>
        public float[] A
        {
            get { return _a; }
            //set { _a = value; }
        }

        private float[] _alpha;
        /// <summary>
        /// Denavit-Hartenberg: the angle difference measured along X
        /// </summary>
        public float[] Alpha
        {
            get { return _alpha; }
            //set { _alpha = value; }
        }

        private float[] _theta;
        /// <summary>
        /// Denavit-Hartenberg: the angle difference measured along Z
        /// </summary>
        public float[] Theta
        {
            get { return _theta; }
            //set { _theta = value; }
        }

        private Boolean[] _isUseable;
        /// <summary>
        /// indicates whether the joint is useable (free or limited) or locked
        /// </summary>
        public Boolean[] IsUseable
        {
            get { return _isUseable; }
            //set { _isUseable = value; }
        }

        /// <summary>
        /// Constructor
        /// </summary>
        public TransformationState()
        {
            _a = new float[8];
            _a[0] = 0;
            _a[1] = 0;
            _a[2] = 0.27f;
            _a[3] = 0;
            _a[4] = 0;
            _a[5] = 0;
            _a[6] = 0;
            _a[7] = 0;

            _alpha = new float[8];
            _alpha[0] = -(float)Math.PI / 2f;
            _alpha[1] = (float)Math.PI / 2f;
            _alpha[2] = 0;
            _alpha[3] = (float)Math.PI / 2f;
            _alpha[4] = -(float)Math.PI / 2f;
            _alpha[5] = (float)Math.PI / 2f;
            _alpha[6] = 0;
            _alpha[7] = 0;

            _d = new float[8];
            _d[0] = 0;
            _d[1] = 0.215f;
            _d[2] = 0;
            _d[3] = 0;
            _d[4] = 0.258f;
            _d[5] = 0;
            _d[6] = 0;
            _d[7] = 0.05f;

            _theta = new float[8];
            _theta[0] = 0;
            _theta[1] = (float)Math.PI;
            _theta[2] = (float)Math.PI / 2f;
            _theta[3] = (float)Math.PI / 2f;
            _theta[4] = 0;
            _theta[5] = 0;
            _theta[6] = 0;
            _theta[7] = (float)Math.PI;

            _isUseable = new bool[7];
            _isUseable[0] = true;
            _isUseable[1] = true;
            _isUseable[2] = false;
            _isUseable[3] = true;
            _isUseable[4] = true;
            _isUseable[5] = true;
            _isUseable[6] = true;
        }
    }

}
