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
    double Dot(Vector3d vector2);
    Vector3d Cross(Vector3d vector2);
    Vector3d Rotate(Vector3d rotationAxis, double rotationAngle);
    double GetValue(int indexAxis);
    Vector3d operator+(Vector3d vector2);
    Vector3d operator-(Vector3d vector2);
    Vector3d operator*(double coffi);
    Vector3d operator/(double coffi);
    Vector3d& operator=(double* in);
    Vector3d& operator=(Vector3d vector2);
    Vector3d Normalize();
    double Mod();
};

class FABRIK_Solver
{
  private:
    int NumberJoints = 7;
    int NumberLinks = 6;
    Vector3d PositionJoints[7];
    enum JointType { XRotation = 1, YRotation = 2, AxisRotation = 3 };
    int TypeOfJoints[7] = { 3, 1, 1, 1, 2, 2, 3 };
    Vector3d OrientationLink[8][3];
    double LengthLink[6];
    double AngleLinks[7];
    double minOffset = 0.05;
    Vector3d OrientationBase[3];
    Vector3d PositionBase;
    Vector3d OrientationTarget[3];
    Vector3d PositionTarget;
    double tolorenceValue = 1;
    double tolorenceValueOri = 0.1;
    double maxAngleJoint = PI / 2;
    int MaxIteration = 10;
    void OrientationCorrection(Vector3d& inputN, Vector3d& inputN_1, Vector3d inputOri[3], Vector3d outputOri[3], double linkLength, int jointIndex, int jointIndexNext, double* angleOutput);
  public:
    FABRIK_Solver(Vector3d InitialPositionJoints[7], Vector3d InitialTargetPosition, Vector3d InitialTargetOrientation[3]);
    FABRIK_Solver();
    int Solve();
    void UpdateTarget(Vector3d InitialTargetPosition, Vector3d InitialTargetOrientation[3]);//update position and orientation of the target
    double GetAngle(int indexJoint, int sign);
    double GetPosition(int indexJoint, int indexAxis);
};
