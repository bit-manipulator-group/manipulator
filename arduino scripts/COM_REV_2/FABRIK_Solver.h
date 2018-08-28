#define PI 3.1415926535
class Vector3d
{
public:
	double x;
	double y;
	double z;

	Vector3d();
	Vector3d(double in_x, double in_y, double in_z);
	Vector3d(double in[3]);
	void SetValue(double in_x, double in_y, double in_z);

	double Dot(Vector3d vector2);
	Vector3d Cross(Vector3d vector2);
	Vector3d Rotate(Vector3d rotationAxis, double rotationAngle);
	
	Vector3d operator+(Vector3d vector2);
	Vector3d operator-(Vector3d vector2);
	Vector3d operator*(double coffi);
	Vector3d operator/(double coffi);
	Vector3d operator=(double* in);
	Vector3d operator=(Vector3d vector2);
	Vector3d Normalize();
	double Mod();
	double GetValue(int indexAxis);
};

class FABRIK_Solver
{
private:
	int NumberJoints = 7;
	int NumberLinks = 6;
	Vector3d PositionJoints[7];
	enum JointType { XRotation = 1, YRotation = 2, AxisRotation = 3 };//this is not used yet
	int TypeOfJoints[7] = { 3,1,1,1,2,2,3 };
	Vector3d OrientationLink[8][3];
	double LengthLink[6];
	double AngleLinks[7];
	double minOffset = 0.05;
	Vector3d OrientationBase[3];
	Vector3d PositionBase;
	Vector3d OrientationTarget[3];
	Vector3d PositionTarget;
	double tolorenceValue = 0.1;
	double tolorenceValueOri = 0.1;
	double maxAngleJoint = PI / 2;
	int MaxIteration = 7;
	void OrientationCorrection(int correctionIndex, int castTargetIndex, int jointTypeIndex, Vector3d vectorToBeCasted, int auxCorr1, int auxCorr2);
	void GetFrameAxisIndex(int axisTarget, int* axisLeftX, int* axisRighX);
	void AngleCal(int lowerIndex, int higherIndex, int lowerAxis, int higherAxis, int angleAXis, int correctionIndex, int originIndex, int axisIndex1, int axisIndex2, double* anglethislink);
public:
	void SetInitialAngles(double angle0, double angle1, double angle2, double angle3, double angle4, double angle5, double angle6);
	void SetLinkLengths(double length0, double length1, double length2, double length3, double length4, double length5);
	void UpdateOrisAndPos();
	FABRIK_Solver();
	int Solve();
	void UpdateTarget(Vector3d InitialTargetPosition, Vector3d InitialTargetOrientation[3]);//update position and orientation of the target
	double GetAngle(int indexJoint, int sign);
	double GetPosition(int indexJoint, int indexAxis);
};
