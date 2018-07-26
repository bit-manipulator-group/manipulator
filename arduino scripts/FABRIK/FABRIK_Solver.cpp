#include "FABRIK_Solver.h"
#include<math.h>
#define PI 3.1415926535

Vector3d::Vector3d()
{
	x = 0;
	y = 0;
	z = 0;
}

Vector3d::Vector3d(double in_x, double in_y, double in_z)
{
	x = in_x;
	y = in_y;
	z = in_z;
}

Vector3d::Vector3d(double in[3])
{
	x = in[0];
	y = in[1];
	z = in[2];
}

double Vector3d::Dot(Vector3d vector2)
{
	return x * vector2.x + y * vector2.y + z * vector2.z;
}

Vector3d Vector3d::Cross(Vector3d vector2)
{
	Vector3d ans = Vector3d(y*vector2.z - z * vector2.y, z*vector2.x - x * vector2.z, x*vector2.y - y * vector2.x);
	return ans;
}

Vector3d Vector3d::Rotate(Vector3d rotationAxis, double rotationAngle)
{
	double valCos = cos(rotationAngle);
	double oneSubCos = 1 - valCos;
	double valSin = sin(rotationAngle);
	double rx = (valCos + rotationAxis.x * rotationAxis.x * oneSubCos) * x + (rotationAxis.x * rotationAxis.y * oneSubCos - rotationAxis.z * valSin) * y + (rotationAxis.x * rotationAxis.z * oneSubCos + rotationAxis.y * valSin) * z;
	double ry = (rotationAxis.y * rotationAxis.x * oneSubCos + rotationAxis.z * valSin) * x + (valCos + rotationAxis.y * rotationAxis.y * oneSubCos) * y + (rotationAxis.y * rotationAxis.z * oneSubCos - rotationAxis.x * valSin) * z;
	double rz = (rotationAxis.z * rotationAxis.x * oneSubCos - rotationAxis.y * valSin) * x + (rotationAxis.z * rotationAxis.y * oneSubCos + rotationAxis.x * valSin) * y + (valCos + rotationAxis.z * rotationAxis.z * oneSubCos) * z;
	return Vector3d(rx, ry, rz);
}

double Vector3d::GetValue(int indexAxis)
{
	switch (indexAxis)
	{
	case 0:
		return x;
	case 1:
		return y;
	case 2:
		return z;
	default:
		return 0;
	}
}

Vector3d Vector3d::operator+(Vector3d vector2)
{
	Vector3d ans = Vector3d(x + vector2.x, y + vector2.y, +z + vector2.z);
	return ans;
}

Vector3d Vector3d::operator-(Vector3d vector2)
{
	Vector3d ans = Vector3d(x - vector2.x, y - vector2.y, z - vector2.z);
	return ans;
}

Vector3d Vector3d::operator*(double coffi)
{
	Vector3d ans = Vector3d(coffi*x, coffi*y, coffi*z);
	return ans;
}

Vector3d Vector3d::operator/(double coffi)
{
	Vector3d ans = Vector3d(x / coffi, y / coffi, z / coffi);
	return ans;
}

Vector3d& Vector3d::operator=(double in[3])
{
	Vector3d ans = Vector3d(in[0], in[1], in[2]);
	x = in[0];
	y = in[1];
	z = in[2];
	return ans;
}

Vector3d& Vector3d::operator=(Vector3d vector2)
{
	Vector3d ans = Vector3d(vector2.x, vector2.y, vector2.z);
	x = vector2.x;
	y = vector2.y;
	z = vector2.z;
	return ans;
}

Vector3d Vector3d::Normalize()
{
	double mod = Mod();
	Vector3d ans = Vector3d(x / mod, y / mod, z / mod);
	return ans;
}
double Vector3d::Mod()
{
	return sqrt(x * x + y * y + z * z);
}

Vector3d operator*(double inputDouble, Vector3d inputVector3d)
{
	Vector3d ans = Vector3d(inputVector3d.x*inputDouble, inputVector3d.y*inputDouble, inputVector3d.z*inputDouble);
	return ans;
}

void FABRIK_Solver::OrientationCorrection(Vector3d inputN, Vector3d inputN_1, Vector3d inputOri[3], Vector3d outputOri[3], double linkLength, int jointIndex, int jointIndexNext, double* angleOutput)
{
	//note that constrain should be carefully considered
	Vector3d outOriTemp[3];
	Vector3d outPosTemp;
	double sign = 1;
	double cosValue;

	if (jointIndexNext > jointIndex)
	{
		sign = 1;
	}
	else
	{
		sign = -1;
	}

	switch(TypeOfJoints[jointIndex])
	{
	case 1:
	{
		outOriTemp[0] = (inputOri[1].Cross(-sign * (inputN - inputN_1))).Normalize();
		if (outOriTemp[0].Dot(inputOri[0]) < 0)
			outOriTemp[0] = -1 * outOriTemp[0];
		outOriTemp[1] = inputOri[1];
		outOriTemp[2] = outOriTemp[0].Cross(outOriTemp[1]);
		outPosTemp = inputN + sign * linkLength * outOriTemp[2];

		double mod1 = sqrt(inputOri[0].Dot(inputOri[0]));
		double mod2 = sqrt(outOriTemp[0].Dot(outOriTemp[0]));
		cosValue = (inputOri[0].Dot(outOriTemp[0])) / (mod1 * mod2);
		if (cosValue > 1)
		{
			cosValue = 1;
		}
		else
		{
			if (cosValue < -1)
			{
				cosValue = -1;
			}
		}
		double angleLocal = acos(cosValue);

		if (inputOri[0].Dot(outOriTemp[2])>0)
			angleLocal = -angleLocal;

		if (angleLocal > maxAngleJoint)//if angleLocal>0
		{
			outOriTemp[2] = cos(maxAngleJoint) * inputOri[2] - sin(maxAngleJoint) * inputOri[0];
			outOriTemp[0] = sin(maxAngleJoint) * inputOri[2] + cos(maxAngleJoint) * inputOri[0];
			outPosTemp = inputN + sign * linkLength * outOriTemp[2];
			angleLocal = maxAngleJoint;
		}
		else if (angleLocal < -maxAngleJoint)//if angleLocal<0
		{
			outOriTemp[2] = cos(maxAngleJoint) * inputOri[2] + sin(maxAngleJoint) * inputOri[0];
			outOriTemp[0] = cos(maxAngleJoint) * inputOri[0] - sin(maxAngleJoint) * inputOri[2];
			outPosTemp = inputN + sign * linkLength * outOriTemp[2];
			angleLocal = -maxAngleJoint;
		}

		*angleOutput = angleLocal;
		outputOri[0] = outOriTemp[0];
		outputOri[2] = outOriTemp[2];
		outputOri[1] = outOriTemp[1];
		inputN_1 = outPosTemp;

		break;
	}
	case 2:
	{
		outOriTemp[1] = ((-sign * (inputN - inputN_1)).Cross(inputOri[0])).Normalize();
		if (outOriTemp[1].Dot(inputOri[1]) < 0)
			outOriTemp[1] = -1 * outOriTemp[1];
		outOriTemp[0] = inputOri[0];
		outOriTemp[2] = outOriTemp[0].Cross(outOriTemp[1]);
		outPosTemp = inputN + sign * linkLength * outOriTemp[2];

		double mod1 = sqrt(inputOri[1].Dot(inputOri[1]));
		double mod2 = sqrt(outOriTemp[1].Dot(outOriTemp[1]));
		cosValue = inputOri[1].Dot(outOriTemp[1]) / (mod1 * mod2);
		if (cosValue > 1)
		{
			cosValue = 1;
		}
		else
		{
			if (cosValue < -1)
			{
				cosValue = -1;
			}
		}
		double angleLocal = acos(cosValue);

		if (inputOri[1].Dot(outOriTemp[2])>0)
			angleLocal = -angleLocal;

		if (angleLocal > maxAngleJoint)//if angleLocal>0
		{
			outOriTemp[2] = cos(maxAngleJoint) * inputOri[2] - sin(maxAngleJoint) * inputOri[1];
			outOriTemp[1] = sin(maxAngleJoint) * inputOri[2] + cos(maxAngleJoint) * inputOri[1];
			outPosTemp = inputN + sign * linkLength * outOriTemp[2];
			angleLocal = maxAngleJoint;
		}
		else if (angleLocal < -maxAngleJoint)//if angleLocal<0
		{
			outOriTemp[2] = cos(maxAngleJoint) * inputOri[2] + sin(maxAngleJoint) * inputOri[1];
			outOriTemp[1] = cos(maxAngleJoint) * inputOri[1] - sin(maxAngleJoint) * inputOri[2];
			outPosTemp = inputN + sign * linkLength * outOriTemp[2];
			angleLocal = -maxAngleJoint;
		}

		*angleOutput = angleLocal;
		outputOri[0] = outOriTemp[0];
		outputOri[2] = outOriTemp[2];
		outputOri[1] = outOriTemp[1];
		inputN_1 = outPosTemp;
		break;
	}
	case 3:
	{
		Vector3d delta = (sign * (inputN - inputN_1).Normalize()).Cross(inputOri[2]);
		outOriTemp[2] = inputOri[2];
		if (delta.Dot(delta) < minOffset || jointIndex > jointIndexNext)
		{
			//if the offset is very small
			outOriTemp[1] = outputOri[1];
			outOriTemp[0] = outOriTemp[1].Cross(outOriTemp[2]);
		}
		else
		{
			//if the offset is large
			//two type of joint should be dealt with differently
			switch (TypeOfJoints[jointIndexNext])
			{
				case 2://type II
				{
					outOriTemp[0] = delta.Normalize();
					if (outOriTemp[0].Dot(inputOri[0]) < 0)
					{
						outOriTemp[0] = -1 * outOriTemp[0];
					//if (outOriTemp[0].Dot(inputOri[1]) < 0)
					//{
					//    outOriTemp[0] = -1*outOriTemp[0];
					//}
					//else
					//{
					//    outOriTemp[0] = (inputOri[0] + inputOri[1]).Normalize();
					//}
					}
					outOriTemp[1] = outOriTemp[2].Cross(outOriTemp[0]);
					break;
				}
				case 1://type I
				{
					outOriTemp[1] = delta.Normalize();
					if (outOriTemp[1].Dot(inputOri[1]) < 0)
					{
						outOriTemp[1] = -1 * outOriTemp[1];
					//if (outOriTemp[1].Dot(inputOri[0]) < 0)
					//{
					//    outOriTemp[1] = (inputOri[1] - inputOri[0]).Normalize();
					//}
					//else
					//{
					//    outOriTemp[1] = (inputOri[0] + inputOri[1]).Normalize();
					//}
					}
					outOriTemp[0] = outOriTemp[1].Cross(outOriTemp[2]);
					break;
				}
			}
		}

		double mod1 = sqrt(inputOri[0].Dot(inputOri[0]));
		double mod2 = sqrt(outOriTemp[0].Dot(outOriTemp[0]));
		cosValue = inputOri[0].Dot(outOriTemp[0]) / (mod1 * mod2);
		if (cosValue > 1)
		{
			cosValue = 1;
		}
		else
		{
			if (cosValue < -1)
			{
				cosValue = -1;
			}
		}
		double angleLocal = acos(cosValue);

		if (inputOri[0].Dot(outOriTemp[1])>0)
			angleLocal = -angleLocal;

		if (angleLocal > maxAngleJoint)
		{
			outOriTemp[1] = cos(maxAngleJoint) * inputOri[1] - sin(maxAngleJoint) * inputOri[0];
			outOriTemp[0] = sin(maxAngleJoint) * inputOri[1] + cos(maxAngleJoint) * inputOri[0];
			angleLocal = maxAngleJoint;
		}
		else if (angleLocal < -maxAngleJoint)//if angleLocal<0
		{
			outOriTemp[1] = cos(maxAngleJoint) * inputOri[1] + sin(maxAngleJoint) * inputOri[0];
			outOriTemp[0] = cos(maxAngleJoint) * inputOri[0] - sin(maxAngleJoint) * inputOri[1];
			angleLocal = -maxAngleJoint;
		}

		*angleOutput = angleLocal;
		outputOri[0] = outOriTemp[0];
		outputOri[1] = outOriTemp[1];
		outputOri[2] = outOriTemp[2];
		inputN_1 = inputN + sign * linkLength * inputOri[2];
		break;
	}
	}
}

FABRIK_Solver::FABRIK_Solver(Vector3d InitialPositionJoints[7], Vector3d InitialTargetPosition, Vector3d InitialTargetOrientation[3])
{
	for (int i = 0;i<NumberJoints;i++)
	{
		PositionJoints[i] = InitialPositionJoints[i];
		if (i>0)
		{
			LengthLink[i - 1] = sqrt((PositionJoints[i] - PositionJoints[i - 1]).Dot(PositionJoints[i] - PositionJoints[i - 1]));
		}
	}
	double tempval[] = { 0.0, 0.0, 0.0 };
	PositionBase = tempval;
	for (int i = 0; i <= 6; i++)
	{
		tempval[0] = 1;
		tempval[1] = 0;
		tempval[2] = 0;
		OrientationLink[i][0] = tempval;
		tempval[0] = 0;
		tempval[1] = 1;
		tempval[2] = 0;
		OrientationLink[i][1] = tempval;
		tempval[0] = 0;
		tempval[1] = 0;
		tempval[2] = 1;
		OrientationLink[i][2] = tempval;
	}
	PositionTarget = InitialTargetPosition;
	OrientationLink[7][0] = InitialTargetOrientation[0];
	OrientationLink[7][1] = InitialTargetOrientation[1];
	OrientationLink[7][2] = InitialTargetOrientation[2];
	OrientationTarget[0] = InitialTargetOrientation[0];
	OrientationTarget[1] = InitialTargetOrientation[1];
	OrientationTarget[2] = InitialTargetOrientation[2];
	//initial statement
}

int  FABRIK_Solver::Solve()
{
	int iterationNumber = 0;
	double totalLength = 0;
	for (int i = 0;i<NumberLinks;i++)
	{
		totalLength += LengthLink[i];
	}
	Vector3d targetbaseVec = (PositionTarget - PositionJoints[0]);
	double targetbaseLength = sqrt(targetbaseVec.Dot(targetbaseVec));
	if (targetbaseLength >= totalLength)
	{
		//means the target can not be reached
		//forward reaching
		//angle tolorence value should be considered
		for (int i = 0;i<NumberJoints - 1;i++)
		{
			Vector3d targetJointVec = PositionTarget - PositionJoints[i];
			double targetJointLength = sqrt(targetJointVec.Dot(targetJointVec));
			double lambda = LengthLink[i] / targetbaseLength;
			PositionJoints[i + 1] = (1 - lambda)*PositionJoints[i] + lambda * PositionTarget;
			OrientationCorrection(PositionJoints[i], PositionJoints[i + 1], OrientationLink[i], OrientationLink[i + 1], LengthLink[i], i, i + 1, &AngleLinks[i]);
			//I need to insert orientation correction operation
		}
		for (int i = NumberJoints - 2;i >= 0;i--)
		{
			Vector3d deltavector = PositionJoints[i + 1] - PositionJoints[i];
			double deltalength = sqrt(deltavector.Dot(deltavector));
			double lambda = LengthLink[i] / deltalength;
			PositionJoints[i] = (1 - lambda)*PositionJoints[i + 1] + lambda * PositionJoints[i];
			OrientationCorrection(PositionJoints[i + 1], PositionJoints[i], OrientationLink[i + 2], OrientationLink[i + 1], LengthLink[i], i + 1, i, &AngleLinks[i + 1]);
		}

		return 1;
	}
	else
	{
		//means the target can be reached
		PositionJoints[0] = PositionBase;
		//continue if error is not small enough and iteraion number is below max number
		double diffLength = (PositionJoints[6] - PositionTarget).Mod();
		double diffOri1 = (OrientationLink[7][0].Cross(OrientationTarget[0])).Mod();
		double diffOri2 = (OrientationLink[7][1].Cross(OrientationTarget[1])).Mod();
		while (((diffLength > tolorenceValue || diffOri1 > tolorenceValueOri || diffOri2 > tolorenceValueOri) && iterationNumber<MaxIteration)||iterationNumber==0)
		{
			//forward reaching
			PositionJoints[6] = PositionTarget;
			for (int i = NumberJoints - 2;i >= 0;i--)
			{
				Vector3d deltavector = PositionJoints[i + 1] - PositionJoints[i];
				double deltalength = sqrt(deltavector.Dot(deltavector));
				double lambda = LengthLink[i] / deltalength;
				PositionJoints[i] = (1 - lambda)*PositionJoints[i + 1] + lambda * PositionJoints[i];
				OrientationCorrection(PositionJoints[i + 1], PositionJoints[i], OrientationLink[i + 2], OrientationLink[i + 1], LengthLink[i], i + 1, i, &AngleLinks[i + 1]);
				//I need to insert orientation operation
			}
			PositionJoints[0] = PositionBase;
			for (int i = 0;i<NumberJoints - 1;i++)
			{
				Vector3d deltavector = PositionJoints[i + 1] - PositionJoints[i];
				double deltalength = sqrt(deltavector.Dot(deltavector));
				double lambda = LengthLink[i] / deltalength;
				PositionJoints[i + 1] = (1 - lambda)*PositionJoints[i] + lambda * PositionJoints[i + 1];
				OrientationCorrection(PositionJoints[i], PositionJoints[i + 1], OrientationLink[i], OrientationLink[i + 1], LengthLink[i], i, i + 1, &AngleLinks[i]);
				//I need to insert orientation operation
			}
			iterationNumber++;
			diffLength = (PositionTarget - PositionJoints[6]).Mod();
		}
		if (diffLength > tolorenceValue)
			return 2;
		else
			return 3;
	}
}

void  FABRIK_Solver::UpdateTarget(Vector3d InitialTargetPosition, Vector3d InitialTargetOrientation[3])//update position and orientation of the target
{
	PositionTarget = InitialTargetPosition;

	OrientationTarget[0] = InitialTargetOrientation[0];
	OrientationTarget[1] = InitialTargetOrientation[1];
	OrientationTarget[2] = InitialTargetOrientation[2];
	OrientationLink[7][0] = InitialTargetOrientation[0];
	OrientationLink[7][1] = InitialTargetOrientation[1];
	OrientationLink[7][2] = InitialTargetOrientation[2];
}

double FABRIK_Solver::GetAngle(int indexJoint, int sign)
{
	if (indexJoint >= 0 && indexJoint <= 6)
	{
		if (sign >= 0)
		{
			return AngleLinks[indexJoint];
		}
		else
		{
			return -AngleLinks[indexJoint];
		}
	}
	else
	{
		return 0;
	}
}
double FABRIK_Solver::GetPosition(int indexJoint, int indexAxis)
{
	if (indexJoint >= 0 && indexJoint <= 6 && indexAxis >= 0 && indexAxis <= 2)
	{
		return PositionJoints[indexJoint].GetValue(indexAxis);
	}
	else
	{
		return 0;
	}
}