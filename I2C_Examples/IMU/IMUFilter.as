package {
	
	/**
	 * @author Aaron Berk
	 * 
	 * @section LICENSE
	 *
	 * Copyright (c) 2010 ARM Limited
	 *
	 * Permission is hereby granted, free of charge, to any person obtaining a copy
	 * of this software and associated documentation files (the "Software"), to deal
	 * in the Software without restriction, including without limitation the rights
	 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	 * copies of the Software, and to permit persons to whom the Software is
	 * furnished to do so, subject to the following conditions:
	 *
	 * The above copyright notice and this permission notice shall be included in
	 * all copies or substantial portions of the Software.
	 *
	 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	 * THE SOFTWARE.
	 *
	 * @section DESCRIPTION
	 *
	 * IMU orientation filter developed by Sebastian Madgwick.
	 *
	 * Find more details about his paper here:
	 *
	 * http://code.google.com/p/imumargalgorithm30042010sohm/ 
	 * 
	 * 
	 * ported to AS3 by Jeff Hoefs
	 * 4/6/11
	 */	
	
	public class IMUFilter {
		
		private var firstUpdate:int;
		
		//Quaternion orientation of earth frame relative to auxiliary frame.
		private var AEq_1:Number;
		private var AEq_2:Number;
		private var AEq_3:Number;
		private var AEq_4:Number;
		
		//Estimated orientation quaternion elements with initial conditions.
		private var SEq_1:Number;
		private var SEq_2:Number;
		private var SEq_3:Number;
		private var SEq_4:Number;
		
		//Sampling period
		private var deltat:Number;
		
		//Gyroscope measurement error (in degrees per second).
		private var gyroMeasError:Number;
		
		//Compute beta (filter tuning constant..
		private var beta:Number;
		
		private var phi:Number;
		private var theta:Number;
		private var psi:Number;
		
		public function IMUFilter(rate:Number, gyroscopeMeasurementError:Number) {
			
			firstUpdate = 0;
			
			//Quaternion orientation of earth frame relative to auxiliary frame.
			AEq_1 = 1;
			AEq_2 = 0;
			AEq_3 = 0;
			AEq_4 = 0;
			
			//Estimated orientation quaternion elements with initial conditions.
			SEq_1 = 1;
			SEq_2 = 0;
			SEq_3 = 0;
			SEq_4 = 0;
			
			//Sampling period (typical value is ~0.1s).
			deltat = rate;
			
			//Gyroscope measurement error (in degrees per second).
			gyroMeasError = gyroscopeMeasurementError;
			
			//Compute beta.
			beta = Math.sqrt(3.0 / 4.0) * (Math.PI * (gyroMeasError / 180.0));
			
		}
		
		public function updateFilter(w_x:Number, w_y:Number, w_z:Number, a_x:Number, a_y:Number, a_z:Number):void {
			
			
			//Local system variables.
			
			//Vector norm.
			var norm:Number;
			//Quaternion rate from gyroscope elements.
			var SEqDot_omega_1:Number;
			var SEqDot_omega_2:Number;
			var SEqDot_omega_3:Number;
			var SEqDot_omega_4:Number;
			//Objective function elements.
			var f_1:Number;
			var f_2:Number;
			var f_3:Number;
			//Objective function Jacobian elements.
			var J_11or24:Number;
			var J_12or23:Number;
			var J_13or22:Number;
			var J_14or21:Number;
			var J_32:Number;
			var J_33:Number;
			//Objective function gradient elements.
			var nablaf_1:Number;
			var nablaf_2:Number;
			var nablaf_3:Number;
			var nablaf_4:Number;
			
			//Auxiliary variables to avoid reapeated calculations.
			var halfSEq_1:Number = 0.5 * SEq_1;
			var halfSEq_2:Number = 0.5 * SEq_2;
			var halfSEq_3:Number = 0.5 * SEq_3;
			var halfSEq_4:Number = 0.5 * SEq_4;
			var twoSEq_1:Number = 2.0 * SEq_1;
			var twoSEq_2:Number = 2.0 * SEq_2;
			var twoSEq_3:Number = 2.0 * SEq_3;
			
			//Compute the quaternion rate measured by gyroscopes.
			SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
			SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
			SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
			SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
			
			//Normalise the accelerometer measurement.
			norm = Math.sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
			a_x /= norm;
			a_y /= norm;
			a_z /= norm;
			
			//Compute the objective function and Jacobian.
			f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
			f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
			f_3 = 1.0 - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
			//J_11 negated in matrix multiplication.
			J_11or24 = twoSEq_3;
			J_12or23 = 2 * SEq_4;
			//J_12 negated in matrix multiplication
			J_13or22 = twoSEq_1;
			J_14or21 = twoSEq_2;
			//Negated in matrix multiplication.
			J_32 = 2 * J_14or21;
			//Negated in matrix multiplication.
			J_33 = 2 * J_11or24;
			
			//Compute the gradient (matrix multiplication).
			nablaf_1 = J_14or21 * f_2 - J_11or24 * f_1;
			nablaf_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
			nablaf_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
			nablaf_4 = J_14or21 * f_1 + J_11or24 * f_2;
			
			//Normalise the gradient.
			norm = Math.sqrt(nablaf_1 * nablaf_1 + nablaf_2 * nablaf_2 + nablaf_3 * nablaf_3 + nablaf_4 * nablaf_4);
			nablaf_1 /= norm;
			nablaf_2 /= norm;
			nablaf_3 /= norm;
			nablaf_4 /= norm;
			
			//Compute then integrate the estimated quaternion rate.
			SEq_1 += (SEqDot_omega_1 - (beta * nablaf_1)) * deltat;
			SEq_2 += (SEqDot_omega_2 - (beta * nablaf_2)) * deltat;
			SEq_3 += (SEqDot_omega_3 - (beta * nablaf_3)) * deltat;
			SEq_4 += (SEqDot_omega_4 - (beta * nablaf_4)) * deltat;
			
			//Normalise quaternion
			norm = Math.sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
			SEq_1 /= norm;
			SEq_2 /= norm;
			SEq_3 /= norm;
			SEq_4 /= norm;
			
			if (firstUpdate == 0) {
				//Store orientation of auxiliary frame.
				AEq_1 = SEq_1;
				AEq_2 = SEq_2;
				AEq_3 = SEq_3;
				AEq_4 = SEq_4;
				firstUpdate = 1;
			}
			
		}
		
		public function computeEuler():void {
			
			//Quaternion describing orientation of sensor relative to earth.
			var ESq_1:Number;
			var ESq_2:Number;
			var ESq_3:Number;
			var ESq_4:Number;
			//Quaternion describing orientation of sensor relative to auxiliary frame.
			var ASq_1:Number;
			var ASq_2:Number;
			var ASq_3:Number;
			var ASq_4:Number;    
			
			//Compute the quaternion conjugate.
			ESq_1 =  SEq_1;
			ESq_2 = -SEq_2;
			ESq_3 = -SEq_3;
			ESq_4 = -SEq_4;
			
			//Compute the quaternion product.
			ASq_1 = ESq_1 * AEq_1 - ESq_2 * AEq_2 - ESq_3 * AEq_3 - ESq_4 * AEq_4;
			ASq_2 = ESq_1 * AEq_2 + ESq_2 * AEq_1 + ESq_3 * AEq_4 - ESq_4 * AEq_3;
			ASq_3 = ESq_1 * AEq_3 - ESq_2 * AEq_4 + ESq_3 * AEq_1 + ESq_4 * AEq_2;
			ASq_4 = ESq_1 * AEq_4 + ESq_2 * AEq_3 - ESq_3 * AEq_2 + ESq_4 * AEq_1;
			
			//Compute the Euler angles from the quaternion.
			
			phi = Math.atan2(2 * ASq_3 * ASq_4 - 2 * ASq_1 * ASq_2, 2 * ASq_1 * ASq_1 + 2 * ASq_4 * ASq_4 - 1);
			theta = Math.asin(2 * ASq_2 * ASq_3 - 2 * ASq_1 * ASq_3);
			psi   = Math.atan2(2 * ASq_2 * ASq_3 - 2 * ASq_1 * ASq_4, 2 * ASq_1 * ASq_1 + 2 * ASq_2 * ASq_2 - 1);
			
		}
		
		public function getRoll():Number {
			return phi;
		}
		
		public function getPitch():Number {
			return theta;
		}
		
		public function getYaw():Number {
			return psi;
		}
		
		public function setGyroError(gyroscopeMeasurementError:Number):void {
			gyroMeasError = gyroscopeMeasurementError;
			reset();
		}
		
		public function getGyroError():Number {
			return gyroMeasError;
		}
		
		public function reset():void {
			
			firstUpdate = 0;
			
			//Quaternion orientation of earth frame relative to auxiliary frame.
			AEq_1 = 1;
			AEq_2 = 0;
			AEq_3 = 0;
			AEq_4 = 0;
			
			//Estimated orientation quaternion elements with initial conditions.
			SEq_1 = 1;
			SEq_2 = 0;
			SEq_3 = 0;
			SEq_4 = 0;
			
			//Compute beta.
			beta = Math.sqrt(3.0 / 4.0) * (Math.PI * (gyroMeasError / 180.0));
		}
	}
}
