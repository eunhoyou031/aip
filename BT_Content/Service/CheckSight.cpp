//내 시야에 대한 적기 위치와 적기 시야에 따른 내 위치를 통하여 각각의 시야안에 상대방이 존재하는지 업데이트

#include "CheckSight.h"

namespace Action
{
	PortsList CheckSight::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
		};
	}

	NodeStatus CheckSight::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
		
		//내 시야 정보
		Vector3 MyLocation = (*BB)->MyLocation_Cartesian;
		Vector3 TargetLocation = (*BB)->TargetLocaion_Cartesian;
		EulerAngle MyRotation = (*BB)->MyRotation_EDegree;

		MyRotation = MyRotation / 57.2958;
		Quaternion QU = MyRotation.toQuaternion();

		//쿼터니언을 이용하여 전방벡터(ForwardVector)를 생성
		Vector3 ForwardVector;

		ForwardVector.Y = 2 * (QU.X*QU.Z + QU.W * QU.Y);
		ForwardVector.Z = -2 * (QU.Y*QU.Z - QU.W *  QU.X);
		ForwardVector.X = 1 - 2 * (QU.X*QU.X + QU.Y * QU.Y);


		Vector3 TV = TargetLocation - MyLocation;
		float Distance = MyLocation.distance(TargetLocation);

		TV = TV / Distance;

		float Los_Radian = acos(ForwardVector.dot(TV));
		float Los_Degree = Los_Radian * 57.2958;
		
		(*BB)->Los_Degree = Los_Degree;
		


		if (Los_Degree <= 95.739166)
		{
			(*BB)->EnemyInSight = true;
		}
		else
		{
			(*BB)->EnemyInSight = false;
		}

		//적기 시야 정보
		Vector3 MyLocation2 = (*BB)->TargetLocaion_Cartesian;
		Vector3 TargetLocation2 = (*BB)->MyLocation_Cartesian;
		EulerAngle MyRotation2 = (*BB)->TargetRotation_EDegree;

		MyRotation2 = MyRotation2 / 57.2958;
		Quaternion QU2 = MyRotation2.toQuaternion();

		//쿼터니언을 이용하여 전방벡터(ForwardVector)를 생성
		Vector3 ForwardVector2;

		ForwardVector2.Y = 2 * (QU2.X*QU2.Z + QU2.W * QU2.Y);
		ForwardVector2.Z = -2 * (QU2.Y*QU2.Z - QU2.W *  QU2.X);
		ForwardVector2.X = 1 - 2 * (QU2.X*QU2.X + QU2.Y * QU2.Y);


		Vector3 TV2 = TargetLocation2 - MyLocation2;
		float Distance2 = MyLocation2.distance(TargetLocation2);

		TV2 = TV2 / Distance2;

		float Los_Radian2 = acos(ForwardVector2.dot(TV2));
		float Los_Degree2 = Los_Radian2 * 57.2958;

		(*BB)->Los_Degree_Target = Los_Degree2;

		if (Los_Degree2 <= 95.739166)
		{
			(*BB)->EnemyInSight_Target = true;
		}
		else
		{
			(*BB)->EnemyInSight_Target = false;
		}


		
		return NodeStatus::SUCCESS;
	}

}