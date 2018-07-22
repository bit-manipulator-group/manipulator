#include<math.h>
#define PI 3.1415926535
class Vector3d
{
    //class to deal with vector operation
    public:
        double x;
        double y;
        double z;
        Vector3d()
        {
            x=0;
            y=0;
            z=0;
        }

        Vector3d(double in_x, double in_y, double in_z)
        {
            x=in_x;
            y=in_y;
            z=in_z;
        }

        Vector3d(double in[3])
        {
            x=in[0];
            y=in[1];
            z=in[2];
        }

        double dot(Vector3d vector2)
        {
            return x*vector2.x+y*vector2.y+z*vector2.z;
        }

        Vector3d cross(Vector3d vector2)
        {
            Vector3d ans=Vector3d(y*vector2.z-vector2.y*z,z*vector2.z-x*vector2.z,x*vector2.y-vector2.x*y);
            return ans;
        }

        Vector3d operator+(Vector3d t)
        {
            Vector3d ans=Vector3d(x+t.x, y+t.y, z+t.z);
            return ans;           
        }

        Vector3d operator-(Vector3d t)
        {
            Vector3d ans=Vector3d(x-t.z, y-t.y, z-t.z);
            return ans;
        }

        Vector3d operator*(double coffi)
        {
            Vector3d ans=Vector3d(coffi*x,coffi*y,coffi*z);
            return ans;
        }

        Vector3d operator/(double coffi)
        {
            Vector3d ans=Vector3d(x/coffi,y/coffi,z/coffi);
            return ans;
        }

        Vector3d operator=(double in[3])
        {
            Vector3d ans=Vector3d(in[0],in[1],in[2]);
            return ans;
        }
        Vector3d operator=(Vector3d in)
        {
            Vector3d ans=Vector3d(in.x,in.y,in.z);
            return ans;
        }
        Vector3d normalize()
        {
            double mod=sqrt(x*x+y*y*z*z);//normlize the vector
            x=x/mod;
            y=y/mod;
            z=z/mod;
        }
};

Vector3d operator*(double inputDouble, Vector3d inputVector3d)
{
    Vector3d ans=Vector3d(inputVector3d.x*inputDouble, inputVector3d.y*inputDouble, inputVector3d.z*inputDouble);
    return ans;
}

class FABRIK_Solver
{
    private:
    //enum {type1:1,type2:2,rotation:3};
    enum JointType {XRotation = 1, YRotation = 2, AxisRotation = 3};//This is not used yet
    int NumberJoints = 7;//number of joints
    int NumberLinks = 6;//number of links

    Vector3d PositionJoints[7];//position of joints
    int TypeOfJoints[7]={3,1,1,1,2,2,3};//XRotation = 1, YRotation = 2, AxisRotation = 3
    Vector3d OrientationLink[8][3];//orientation of links, I use 3 vector to describe orientation of one link
    //orientation is expressed : P1, P2, F<--->0, 1, 2
    //index 0 means base, while index 7 means target
    double LengthLink[6];//length of links
    double AngleLinks[7];
    double minOffset;

    //Vector3d OrientationBase[3];//orientation of the base, should be set as some constant
    Vector3d PositionBase;//position of the base
    //Vector3d OrientationTargrt[3];//orientation of the target
    Vector3d PositionTarget;//position of the target

    double tolorenceValue = 3;

    //corrention of the orientation
    void OrientationCorrection(Vector3d inputN, Vector3d inputN_1, Vector3d inputOri[3],Vector3d outputOri[3], double linkLength, int jointIndex, int jointIndexNext, double* angleOutput)
    {
        //note that constrain should be carefully considered
        Vector3d outOriTemp[3];
        Vector3d outPosTemp;
        switch (TypeOfJoints[jointIndex])
        {
            case 1:
            {
                outOriTemp[0]=(inputOri[1].cross(inputN-inputN_1)).normalize();
                outPosTemp=inputN+linkLength*outOriTemp[0].cross(inputOri[1]);
                outOriTemp[2]=(inputN-outPosTemp).normalize();
                outOriTemp[1]=inputOri[1];

                double mod1=sqrt(inputOri[0].dot(inputOri[0]));
                double mod2=sqrt(outOriTemp[0].dot(outOriTemp[0]));
                double dotans=inputOri[0].dot(outOriTemp[0]);
                double angleLocal=acos(dotans/(mod1*mod2));
                
                if(inputOri[0].dot(outOriTemp[2])>0)
                    angleLocal=-angleLocal;

                if(angleLocal>PI/4 )//if angleLocal>0
                {
                    outOriTemp[2]=(inputOri[2]-inputOri[0]).normalize();
                    outOriTemp[0]=(inputOri[2]+inputOri[0]).normalize();
                    outPosTemp=inputN+linkLength*outOriTemp[2];
                    angleLocal=PI/4;
                }
                else if(angleLocal<-PI/4)//if angleLocal<0
                {
                    outOriTemp[2]=(inputOri[2]+inputOri[0]).normalize();
                    outOriTemp[0]=(inputOri[2]-inputOri[0]).normalize();
                    outPosTemp=inputN+linkLength*outOriTemp[2];
                    angleLocal=-PI/4;
                }

                *angleOutput=angleLocal;
                outputOri[0]=outOriTemp[0];   
                outputOri[2]=outOriTemp[2];
                outputOri[1]=outOriTemp[1];
                inputN_1=outPosTemp;

                break;
            }
            case 2:
            {
                outOriTemp[1]=(inputOri[0].cross(inputN-inputN_1)).normalize();
                outPosTemp=inputN+linkLength*outOriTemp[1].cross(inputOri[0]);
                outOriTemp[2]=(inputN-outPosTemp).normalize();
                outOriTemp[0]=inputOri[0];

                double mod1=sqrt(inputOri[1].dot(inputOri[1]));
                double mod2=sqrt(outOriTemp[1].dot(outOriTemp[1]));
                double dotans=inputOri[1].dot(outOriTemp[1]);
                double angleLocal=acos(dotans/(mod1*mod2));
                
                if(inputOri[1].dot(outOriTemp[2])>0)
                    angleLocal=-angleLocal;

                if(angleLocal>PI/4 )//if angleLocal>0
                {
                    outOriTemp[2]=(inputOri[2]-inputOri[1]).normalize();
                    outOriTemp[1]=(inputOri[2]+inputOri[1]).normalize();
                    outPosTemp=inputN+linkLength*outOriTemp[2];
                    angleLocal=PI/4;
                }
                else if(angleLocal<-PI/4)//if angleLocal<0
                {
                    outOriTemp[2]=(inputOri[2]+inputOri[1]).normalize();
                    outOriTemp[1]=(inputOri[2]-inputOri[1]).normalize();
                    outPosTemp=inputN+linkLength*outOriTemp[2];
                    angleLocal=-PI/4;
                }

                *angleOutput=angleLocal;
                outputOri[0]=outOriTemp[0];   
                outputOri[2]=outOriTemp[2];
                outputOri[1]=outOriTemp[1];
                inputN_1=outPosTemp;
                break;
            }
            case 3:
            {
                Vector3d delta=((inputN-inputN_1).normalize()).cross(inputOri[2]);
                outOriTemp[2]=inputOri[2];
                if(delta.dot(delta)<minOffset)
                {
                    //if the offset is very small
                    outOriTemp[1]=outputOri[1];
                    outOriTemp[0]=outputOri[0];
                }
                else
                {
                    //if the offset is large
                    //two type of joint should be dealt with differently
                    switch (TypeOfJoints[jointIndexNext])
                    {
                        case 2://type II
                        {
                            outOriTemp[0]=delta.normalize();
                            if(outOriTemp[0].dot(inputOri[0])<0)
                            {
                                if(outOriTemp[0].dot(inputOri[1])<0)
                                {
                                    outOriTemp[0]=(inputOri[0]-inputOri[1]).normalize();
                                }
                                else
                                {
                                    outOriTemp[0]=(inputOri[0]+inputOri[1]).normalize();
                                }
                            }
                            outOriTemp[1]=outOriTemp[2].cross(outOriTemp[0]);
                        }
                        case 1://type I
                        {
                            outOriTemp[1]=delta.normalize();
                            if(outOriTemp[1].dot(inputOri[1])<0)
                            {
                                if(outOriTemp[1].dot(inputOri[0])<0)
                                {
                                    outOriTemp[1]=(inputOri[1]-inputOri[0]).normalize();
                                }
                                else
                                {
                                    outOriTemp[1]=(inputOri[0]+inputOri[1]).normalize();
                                }
                            }
                            outOriTemp[0]=outOriTemp[1].cross(outOriTemp[2]);
                        }
                    }
                    

                    
                    
                }

                double mod1=sqrt(inputOri[0].dot(inputOri[0]));
                double mod2=sqrt(outOriTemp[0].dot(outOriTemp[0]));
                double dotans=inputOri[0].dot(outOriTemp[0]);
                double angleLocal=acos(dotans/(mod1*mod2));

                if(inputOri[0].dot(outOriTemp[1])>0)
                    angleLocal=-angleLocal;

                if(angleLocal>PI/4)
                {
                    outOriTemp[1]=(inputOri[1]-inputOri[0]).normalize();
                    outOriTemp[0]=(inputOri[1]+inputOri[0]).normalize();
                    angleLocal=PI/4;
                }
                else if(angleLocal<-PI/4)//if angleLocal<0
                {
                    outOriTemp[1]=(inputOri[1]+inputOri[0]).normalize();
                    outOriTemp[0]=(inputOri[1]-inputOri[0]).normalize();
                    angleLocal=-PI/4;
                }

                *angleOutput=angleLocal;
                outputOri[0]=outOriTemp[0];
                outputOri[1]=outOriTemp[1];
                inputN_1=inputN+linkLength*inputOri[2];
                break;
            }
        }
    }

    public:
    //the constructor of the class
    FABRIK_Solver(Vector3d InitialPositionJoints[7], Vector3d InitialTargetPosition, Vector3d InitialTargetOrientation[3])
    {
        for(int i=0;i<NumberJoints;i++)
        {
            PositionJoints[i]=InitialPositionJoints[i];
            if(i>0)
            {
                LengthLink[i-1]=sqrt((PositionJoints[i]-PositionJoints[i-1]).dot(PositionJoints[i]-PositionJoints[i-1]));
            }
        }
        PositionBase = {0,0,0};
        OrientationLink[0][2] = {0,0,-1};
        OrientationLink[0][1] = {1,0,0};
        OrientationLink[0][0] = {0,1,0};
        PositionTarget = InitialTargetPosition;
        OrientationLink[7][0] = InitialTargetOrientation[0];
        OrientationLink[7][1] = InitialTargetOrientation[1];
        OrientationLink[7][2] = InitialTargetOrientation[2];
        //initial statement
    }

    int Solve()//solve the IK problem
    {
        int MaxIteration =10;
        int iterationNumber = 0;
        double totalLength = 0;
        for(int i=0;i<NumberLinks;i++)
        {
            totalLength += LengthLink[i];
        }
        Vector3d targetbaseVec=(PositionTarget-PositionJoints[0]);
        double targetbaseLength=sqrt(targetbaseVec.dot(targetbaseVec));
        if(targetbaseLength >= totalLength)
        {
            //means the target can not be reached
            for(int i=0;i<NumberJoints-1;i++)
            {
                Vector3d targetJointVec=PositionTarget-PositionJoints[i];
                double targetJointLength=sqrt(targetJointVec.dot(targetJointVec));
                double lambda = LengthLink[i]/targetbaseLength;
                PositionJoints[i+1]=(1-lambda)*PositionJoints[i]+lambda*PositionTarget;
                OrientationCorrection(PositionJoints[i],PositionJoints[i+1],OrientationLink[i],OrientationLink[i+1],LengthLink[i],i,i+1,&AngleLinks[i]);
                //I need to insert orientation correction operation
            }
            return 1;
        }
        else
        {
            //means the target can be reached
            PositionJoints[0]=PositionBase;
            //continue if error is not small enough and iteraion number is below max number
            while((PositionJoints[6]-PositionTarget).dot(PositionJoints[6]-PositionTarget) >tolorenceValue && iterationNumber<MaxIteration)
            {
                //forward reaching
                PositionJoints[6]=PositionTarget;
                for(int i=5;i>=0;i--)
                {
                    Vector3d deltavector=PositionJoints[i+1]-PositionJoints[i];
                    double deltalength=sqrt(deltavector.dot(deltavector));
                    double lambda=LengthLink[i]/deltalength;
                    PositionJoints[i]=(1-lambda)*PositionJoints[i+1]+lambda*PositionJoints[i];
                    OrientationCorrection(PositionJoints[i+1],PositionJoints[i],OrientationLink[i+1],OrientationLink[i],LengthLink[i],i+1,i,&AngleLinks[i]);
                    //I need to insert orientation operation
                }
                PositionJoints[0]=PositionBase;
                for(int i=0;i<NumberJoints-1;i++)
                {
                    Vector3d deltavector=PositionJoints[i+1]-PositionJoints[i];
                    double deltalength=sqrt(deltavector.dot(deltavector));
                    double lambda=LengthLink[i]/deltalength;
                    PositionJoints[i+1]=(1-lambda)*PositionJoints[i]+lambda*PositionJoints[i+1];
                    OrientationCorrection(PositionJoints[i],PositionJoints[i+1],OrientationLink[i],OrientationLink[i+1],LengthLink[i],i,i+1,&AngleLinks[i]);
                    //I need to insert orientation operation
                }
            }
            if((PositionJoints[6]-PositionTarget).dot(PositionJoints[6]-PositionTarget) > tolorenceValue)
                return 2;
            else
                return 3; 
        }
    }

    void UpdateTarget(Vector3d InitialTargetPosition, Vector3d InitialTargetOrientation[3])//update position and orientation of the target
    {
        PositionTarget = InitialTargetPosition;
        OrientationLink[7][0] = InitialTargetOrientation[0];
        OrientationLink[7][1] = InitialTargetOrientation[1];
        OrientationLink[7][2] = InitialTargetOrientation[2];
    }

    void CalculateAngles()//get angles of joints
    {
        int refIndex = 0;
        for(int i=0;i<NumberJoints;i++)
        {
            double mod1=OrientationLink[i][refIndex].dot(OrientationLink[i][refIndex]);
            double mod2=OrientationLink[i+1][refIndex].dot(OrientationLink[i+1][refIndex]);
            double dotans=OrientationLink[i][refIndex].dot(OrientationLink[i+1][refIndex]);
            AngleLinks[i]=acos(dotans/(mod1*mod2));
            if(OrientationLink[i][2].dot(OrientationLink[i+1][refIndex])<0)
                AngleLinks[i]=-AngleLinks[i];
        }
    }
    void CalculateAngles(int sign)//get angles of joints
    {
        int refIndex = 0;
        for(int i=0;i<NumberJoints;i++)
        {
            double mod1=OrientationLink[i][refIndex].dot(OrientationLink[i][refIndex]);
            double mod2=OrientationLink[i+1][refIndex].dot(OrientationLink[i+1][refIndex]);
            double dotans=OrientationLink[i][refIndex].dot(OrientationLink[i+1][refIndex]);
            AngleLinks[i]=acos(dotans/(mod1*mod2));
            if(OrientationLink[i][2].dot(OrientationLink[i+1][refIndex])<0)
                AngleLinks[i]=-AngleLinks[i];
            if(sign<0)
                AngleLinks[i]=-AngleLinks[i];
        }
    }
};

Vector3d InitialPositionJoints[7];
Vector3d InitialTargetPosition;
Vector3d InitialTargetOrientation[3];

FABRIK_Solver mySolver=FABRIK_Solver(InitialPositionJoints, InitialTargetPosition, InitialTargetOrientation);

void setup() {
    InitialPositionJoints[0]={0,0,0};
    InitialPositionJoints[1]={0,0,35};
    InitialPositionJoints[2]={0,0,97};
    InitialPositionJoints[3]={0,0,151};
    InitialPositionJoints[4]={0,0,216};
    InitialPositionJoints[5]={0,0,278};
    InitialPositionJoints[6]={0,0,313};

    InitialTargetPosition={50,50,280};
    InitialTargetOrientation[0]= {1,0,0};
    InitialTargetOrientation[1]= {0,1,0};
    InitialTargetOrientation[0]= {0,0,1};
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}