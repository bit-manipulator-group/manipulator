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

Vector3d Vector3d::operator=(double* in)
{
	Vector3d ans = Vector3d(in[0], in[1], in[2]);
	x = in[0];
	y = in[1];
	z = in[2];
	return ans;
}

Vector3d Vector3d::operator=(Vector3d vector2)
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
void Vector3d::SetValue(double in_x, double in_y, double in_z)
{
	x = in_x;
	y = in_y;
	z = in_z;
}

Vector3d operator*(double inputDouble, Vector3d inputVector3d)
{
	Vector3d ans = Vector3d(inputVector3d.x*inputDouble, inputVector3d.y*inputDouble, inputVector3d.z*inputDouble);
	return ans;
}

 void FABRIK_Solver::OrientationCorrection(int correctionIndex, int castTargetIndex, int jointTypeIndex, Vector3d vectorToBeCasted, int auxCorr1, int auxCorr2)
{
	int axisFixed = -1;
	int axisCorr1 = -1;
	int axisCorr2 = -1;

	int axisLeftX = -1;
	int axisRighX = -1;
	switch (TypeOfJoints[jointTypeIndex])
	{
	case 1:
	{
		axisFixed = 1;
		axisCorr1 = 2;
		axisCorr2 = 0;
		break;
	}
	case 2:
	{
		axisFixed = 0;
		axisCorr1 = 2;
		axisCorr2 = 1;
		break;
	}
	case 3:
	{
		axisFixed = 2;
		axisCorr1 = auxCorr1;
		axisCorr2 = auxCorr2;
		break;
	}
	}

	if ((OrientationLink[castTargetIndex + 1][axisFixed].Cross(vectorToBeCasted)).Mod() > minOffset)
	{
		OrientationLink[correctionIndex + 1][axisCorr1] = ((OrientationLink[castTargetIndex + 1][axisFixed].Cross(vectorToBeCasted)).Cross(OrientationLink[castTargetIndex + 1][axisFixed])).Normalize();
		OrientationLink[correctionIndex + 1][axisFixed] = OrientationLink[castTargetIndex + 1][axisFixed];
		GetFrameAxisIndex(axisCorr2, &axisLeftX, &axisRighX);
		OrientationLink[correctionIndex + 1][axisCorr2] = OrientationLink[correctionIndex + 1][axisLeftX].Cross(OrientationLink[correctionIndex + 1][axisRighX]);
	}
	else
	{
		if ((OrientationLink[castTargetIndex + 1][axisFixed].Cross(OrientationLink[correctionIndex + 1][axisCorr1])).Mod() > minOffset)
		{
			OrientationLink[correctionIndex + 1][axisCorr1] = ((OrientationLink[castTargetIndex + 1][axisFixed].Cross(OrientationLink[correctionIndex + 1][axisCorr1])).Cross(OrientationLink[castTargetIndex + 1][axisFixed])).Normalize();
			OrientationLink[correctionIndex + 1][axisFixed] = OrientationLink[castTargetIndex + 1][axisFixed];
			GetFrameAxisIndex(axisCorr2, &axisLeftX, &axisRighX);
			OrientationLink[correctionIndex + 1][axisCorr2] = OrientationLink[correctionIndex + 1][axisLeftX].Cross(OrientationLink[correctionIndex + 1][axisRighX]);
		}
		else
		{
			OrientationLink[correctionIndex + 1][axisCorr2] = ((OrientationLink[castTargetIndex + 1][axisFixed].Cross(OrientationLink[correctionIndex + 1][axisCorr2])).Cross(OrientationLink[castTargetIndex + 1][axisFixed])).Normalize();
			OrientationLink[correctionIndex + 1][axisFixed] = OrientationLink[castTargetIndex + 1][axisFixed];
			GetFrameAxisIndex(axisCorr1, &axisLeftX, &axisRighX);
			OrientationLink[correctionIndex + 1][axisCorr1] = OrientationLink[correctionIndex + 1][axisLeftX].Cross(OrientationLink[correctionIndex + 1][axisRighX]);
		}
	}
}

FABRIK_Solver::FABRIK_Solver()
{
	SetLinkLengths(35, 62, 54, 65, 62, 35);
	SetInitialAngles(0, 0, 0, 0, 0, 0, 0);

	OrientationTarget[0] = { 1, 0, 0 };
	OrientationTarget[1] = { 0, 1, 0 };
	OrientationTarget[2] = { 0, 0, 1 };
	PositionTarget = { 0, 20, 280 };

	OrientationBase[0] = { 1, 0, 0 };
	OrientationBase[1] = { 0, 1, 0 };
	OrientationBase[2] = { 0, 0, 1 };
	PositionBase = { 0, 0, 0 };

	OrientationLink[0][0] = { 1, 0, 0 };
	OrientationLink[0][1] = { 0, 1, 0 };
	OrientationLink[0][2] = { 0, 0, 1 };

	OrientationLink[1][0] = OrientationBase[0];
	OrientationLink[1][1] = OrientationBase[1];
	OrientationLink[1][2] = OrientationBase[2];
	PositionJoints[0] = PositionBase;

	UpdateOrisAndPos();
}

int  FABRIK_Solver::Solve()
{
	int iterationNumber = 0;
	double totalLength = 0;
	for (int i = 0; i < NumberLinks; i++)
	{
		totalLength += LengthLink[i];
	}
	Vector3d targetbaseVec = (PositionTarget - PositionJoints[0]);
	double targetbaseLength = targetbaseVec.Mod();

	Vector3d deltavector = { 0,0,0 };
	double deltalength;
	double lambda;
	double anglethislink = 0;

	//means the target can not be reached
	//statement like this is not very necessary
	if (targetbaseLength >= totalLength)
		deltavector = totalLength * targetbaseVec.Normalize();
	else
		deltavector = PositionTarget;

	double diffLength = (PositionJoints[6] - PositionTarget).Mod();
	double diffOri1 = (OrientationLink[7][0].Cross(OrientationTarget[0])).Mod();
	double diffOri2 = (OrientationLink[7][1].Cross(OrientationTarget[1])).Mod();
	while (((diffLength > tolorenceValue || diffOri1 > tolorenceValueOri || diffOri2 > tolorenceValueOri) && iterationNumber < MaxIteration) || iterationNumber == 0)
	{
		if (iterationNumber == 0)
		{
			AngleLinks[1] = AngleLinks[1] + 0.03;
			UpdateOrisAndPos();
		}

		//forward reaching
		PositionJoints[6] = deltavector;
		OrientationLink[7][0] = OrientationTarget[0];
		OrientationLink[7][1] = OrientationTarget[1];
		OrientationLink[7][2] = OrientationTarget[2];
		for (int i = NumberJoints - 2; i >= 0; i--)
		{
			//set orientation of the ith joint, and set position of the ith joint
			deltalength = (PositionJoints[i + 1] - PositionJoints[i]).Mod();
			lambda = LengthLink[i] / deltalength;
			PositionJoints[i] = (1 - lambda) * PositionJoints[i + 1] + lambda * PositionJoints[i];
			switch (TypeOfJoints[i + 1])
			{
			case 1:
			{
				OrientationCorrection(i, i + 1, i + 1, PositionJoints[i + 1] - PositionJoints[i], -1, -1);
				AngleCal(i + 1, i + 2, 0, 2, 0, i + 1, i + 2, 2, 0, &anglethislink);
				break;
			}
			case 2:
			{
				OrientationCorrection(i, i + 1, i + 1, PositionJoints[i + 1] - PositionJoints[i], -1, -1);
				AngleCal(i + 1, i + 2, 2, 1, 2, i + 1, i + 2, 1, 2, &anglethislink);
				break;
			}
			case 3:
			{
				PositionJoints[i] = PositionJoints[i + 1] - LengthLink[i] * OrientationLink[i + 2][2];
				OrientationCorrection(i, i + 1, i + 1, OrientationLink[i + 1][0], 0, 1);
				AngleCal(i + 1, i + 2, 1, 0, 0, i + 1, i + 2, 0, 1, &anglethislink);
				break;
			}
			}

			AngleLinks[i + 1] = anglethislink;
			PositionJoints[i] = PositionJoints[i + 1] - LengthLink[i] * OrientationLink[i + 1][2];
		}

		//forward reaching
		PositionJoints[0] = PositionBase;
		OrientationLink[1][2] = { 0, 0, 1 };
		for (int i = 0; i <= NumberJoints - 2; i++)
		{
			deltalength = (PositionJoints[i] - PositionJoints[i + 1]).Mod();
			lambda = LengthLink[i] / deltalength;
			PositionJoints[i + 1] = (1 - lambda) * PositionJoints[i] + lambda * PositionJoints[i + 1];

			//set the orientation of the ith joint, and the position of the i+1th joint
			//this depend on the position of the i+1th joint
			//and type of the ith joint
			switch (TypeOfJoints[i])
			{
			case 1:
			{
				OrientationCorrection(i, i, i, PositionJoints[i + 1] - PositionJoints[i], -1, -1);
				AngleCal(i, i + 1, 0, 2, 0, i + 1, i, 0, 2, &anglethislink);
				break;
			}
			case 2:
			{
				OrientationCorrection(i, i, i, PositionJoints[i + 1] - PositionJoints[i], -1, -1);
				AngleCal(i, i + 1, 2, 1, 2, i + 1, i, 2, 1, &anglethislink);
				break;
			}
			case 3:
			{
				//try to pass the orientation of the i-1th joint
				//should try to maintainorientation of the
				switch (TypeOfJoints[i + 1])
				{
				case 1:
				{
					OrientationCorrection(i, i, i, OrientationLink[i + 2][1], 1, 0);
					break;
				}
				case 2:
				{
					OrientationCorrection(i, i, i, OrientationLink[i + 2][0], 0, 1);
					break;
				}
				case 3:
				{
					OrientationCorrection(i, i, i, OrientationLink[i + 1][0], 0, 1);
					break;
				}
				}
				AngleCal(i, i + 1, 1, 0, 0, i + 1, i, 1, 0, &anglethislink);
				break;
			}
			}

			//confine the orientatioin of the i+1th joint
			//this depend on type of the i+1th joint
			switch (TypeOfJoints[i + 1])
			{
			case 1:
			{
				OrientationLink[i + 2][1] = OrientationLink[i + 1][1];
				break;
			}
			case 2:
			{
				OrientationLink[i + 2][0] = OrientationLink[i + 1][0];
				break;
			}
			case 3:
			{
				OrientationLink[i + 2][2] = OrientationLink[i + 1][2];
				break;
			}
			}
			AngleLinks[i] = anglethislink;
			PositionJoints[i + 1] = PositionJoints[i] + LengthLink[i] * OrientationLink[i + 1][2];
		}

		OrientationLink[7][1] = ((OrientationLink[7][2].Cross(OrientationLink[7][1])).Cross(OrientationLink[7][2])).Normalize();
		OrientationLink[7][0] = OrientationLink[7][1].Cross(OrientationLink[7][2]);
		AngleCal(6, 7, 1, 0, 0, 7, 6, 1, 0, &anglethislink);
		AngleLinks[6] = anglethislink;

		iterationNumber++;
		diffLength = (PositionTarget - PositionJoints[6]).Mod();
		diffOri1 = (OrientationLink[7][0].Cross(OrientationTarget[0])).Mod();
		diffOri2 = (OrientationLink[7][1].Cross(OrientationTarget[1])).Mod();
		UpdateOrisAndPos();
	}
	if (diffLength > tolorenceValue)
		return 0;
	else
		return 1;
	//return 1;//return value 1 means the target is obviously can't be reached
}

void  FABRIK_Solver::UpdateTarget(Vector3d InitialTargetPosition, Vector3d InitialTargetOrientation[3])//update position and orientation of the target
{
	OrientationTarget[0] = InitialTargetOrientation[0];
	OrientationTarget[1] = InitialTargetOrientation[1];
	OrientationTarget[2] = InitialTargetOrientation[2];
	PositionTarget = InitialTargetPosition;
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
void FABRIK_Solver::GetFrameAxisIndex(int axisTarget, int* axisLeftX, int* axisRighX)
{
	switch (axisTarget)
	{
	case 0:
	{
		*axisLeftX = 1;
		*axisRighX = 2;
		break;
	}
	case 1:
	{
		*axisLeftX = 2;
		*axisRighX = 0;
		break;
	}
	case 2:
	{
		*axisLeftX = 0;
		*axisRighX = 1;
		break;
	}
	}
}
void FABRIK_Solver::AngleCal(int lowerIndex, int higherIndex, int lowerAxis, int higherAxis, int angleAXis, int correctionIndex, int originIndex, int axisIndex1, int axisIndex2, double* anglethislink)
{
	double mod1 = OrientationLink[lowerIndex][angleAXis].Mod();
	double mod2 = OrientationLink[higherIndex][angleAXis].Mod();
	double cosValue = (OrientationLink[lowerIndex][angleAXis].Dot(OrientationLink[higherIndex][angleAXis])) / (mod1 * mod2);
	if (cosValue > 1)
		cosValue = 1;
	else if (cosValue < -1)
		cosValue = -1;
	*anglethislink = acos(cosValue);
	if (OrientationLink[lowerIndex][lowerAxis].Dot(OrientationLink[higherIndex][higherAxis]) < 0)
		*anglethislink = -*anglethislink;

	if (*anglethislink > maxAngleJoint)
	{
		OrientationLink[correctionIndex][axisIndex1] = cos(maxAngleJoint) * OrientationLink[originIndex][axisIndex1] - sin(maxAngleJoint) * OrientationLink[originIndex][axisIndex2];
		OrientationLink[correctionIndex][axisIndex2] = sin(maxAngleJoint) * OrientationLink[originIndex][axisIndex1] + cos(maxAngleJoint) * OrientationLink[originIndex][axisIndex2];
		*anglethislink = maxAngleJoint;
	}
	else if (*anglethislink < -maxAngleJoint)
	{
		OrientationLink[correctionIndex][axisIndex1] = cos(maxAngleJoint) * OrientationLink[originIndex][axisIndex1] + sin(maxAngleJoint) * OrientationLink[originIndex][axisIndex2];
		OrientationLink[correctionIndex][axisIndex2] = cos(maxAngleJoint) * OrientationLink[originIndex][axisIndex2] - sin(maxAngleJoint) * OrientationLink[originIndex][axisIndex1];
		*anglethislink = -maxAngleJoint;
	}
}
void FABRIK_Solver::SetInitialAngles(double angle0, double angle1, double angle2, double angle3, double angle4, double angle5, double angle6)
{
	AngleLinks[0] = angle0;
	AngleLinks[1] = angle1;
	AngleLinks[2] = angle2;
	AngleLinks[3] = angle3;
	AngleLinks[4] = angle4;
	AngleLinks[5] = angle5;
	AngleLinks[6] = angle6;
}
void FABRIK_Solver::SetLinkLengths(double length0, double length1, double length2, double length3, double length4, double length5)
{
	LengthLink[0] = length0;
	LengthLink[1] = length1;
	LengthLink[2] = length2;
	LengthLink[3] = length3;
	LengthLink[4] = length4;
	LengthLink[5] = length5;
}
void FABRIK_Solver::UpdateOrisAndPos()
{
	//get orientations from angles
	for (int i = 1; i <= 7; i++)
	{
		int rotationAxisIndex = -1;
		switch (TypeOfJoints[i - 1])
		{
		case 1:
		{
			rotationAxisIndex = 1;
			break;
		}
		case 2:
		{
			rotationAxisIndex = 0;
			break;
		}
		case 3:
		{
			rotationAxisIndex = 2;
			break;
		}
		default:
		{
			break;
		}

		}
		OrientationLink[i][0] = (OrientationLink[i - 1][0].Rotate(OrientationLink[i - 1][rotationAxisIndex], AngleLinks[i - 1])).Normalize();
		OrientationLink[i][1] = (OrientationLink[i - 1][1].Rotate(OrientationLink[i - 1][rotationAxisIndex], AngleLinks[i - 1])).Normalize();
		OrientationLink[i][2] = (OrientationLink[i - 1][2].Rotate(OrientationLink[i - 1][rotationAxisIndex], AngleLinks[i - 1])).Normalize();
	}

	//get positions from orientations
	for (int i = 0; i <= 6; i++)
	{
		if (i == 0)
		{
			PositionJoints[i] = { 0, 0, 0 };
		}
		else
		{
			PositionJoints[i] = PositionJoints[i - 1] + LengthLink[i - 1] * OrientationLink[i][2];
		}
	}
}