#include "Task_HeadOnBFM.h"

namespace Action
{
    PortsList Task_HeadOnBFM::providedPorts()
    {
        return {
            InputPort<CPPBlackBoard*>("BB")
        };
    }

    NodeStatus Task_HeadOnBFM::tick()
    {
        Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

        float distance = (*BB)->Distance;
        float mySpeed = (*BB)->MySpeed_MS;
        float targetSpeed = (*BB)->TargetSpeed_MS;

        // ��Ż â Ȯ��
        if (IsEscapeWindowOpen(BB.value()) && distance < 3704.0f) // 2nm �̳�
        {
            // ���� �ߴ��ϰ� ��Ż
            Vector3 myLocation = (*BB)->MyLocation_Cartesian;
            Vector3 myForward = (*BB)->MyForwardVector;
            (*BB)->VP_Cartesian = myLocation + myForward * 5000.0f;
            (*BB)->Throttle = 1.0f;
            return NodeStatus::SUCCESS;
        }

        // ���� �� ���� ���� �Ǵ�
        if (ShouldInitiateLeadTurn(BB.value()))
        {
            (*BB)->VP_Cartesian = CalculateLeadTurn(BB.value());
        }
        else
        {
            // ���� �� �ɼ� ����
            if (mySpeed > targetSpeed + 20.0f) // �ӵ� ���� ��
            {
                // �����̽� �� (��� �Ʒ���)
                (*BB)->VP_Cartesian = CalculateSliceTurn(BB.value());
            }
            else if (mySpeed < targetSpeed - 20.0f) // �ӵ� ���� ��
            {
                // ���� �⵿
                (*BB)->VP_Cartesian = CalculateVerticalManeuver(BB.value());
            }
            else
            {
                // ���� ��ȸ
                (*BB)->VP_Cartesian = CalculateLevelTurn(BB.value());
            }
        }

        // �ڳ� �ӵ� ����
        if (mySpeed < 120.0f)
        {
            (*BB)->Throttle = 1.0f;
        }
        else if (mySpeed > 150.0f)
        {
            (*BB)->Throttle = 0.6f;
        }
        else
        {
            (*BB)->Throttle = 0.8f;
        }

        return NodeStatus::SUCCESS;
    }

    Vector3 Task_HeadOnBFM::CalculateSliceTurn(CPPBlackBoard* BB)
    {
        // �����̽� �� - ����� 10�� ���� �Ʒ��� �Ͽ� ���� ���� ��ȸ
        Vector3 myLocation = BB->MyLocation_Cartesian;
        Vector3 targetLocation = BB->TargetLocaion_Cartesian;
        Vector3 myForward = BB->MyForwardVector;
        Vector3 myRight = BB->MyRightVector;

        // ���� �������� ��ȸ ���� ����
        Vector3 toTarget = (targetLocation - myLocation);
        float dotRight = toTarget.dot(myRight);

        Vector3 sliceDirection;
        if (dotRight > 0)
        {
            sliceDirection = myRight;
        }
        else
        {
            sliceDirection = myRight * -1.0f;
        }

        // ���� ���ϰ����� �����̽�
        Vector3 slicePoint = myLocation + sliceDirection * 1500.0f + myForward * 2000.0f;
        slicePoint.Z = myLocation.Z - 300.0f; // �ణ �ϰ�

        return slicePoint;
    }

    Vector3 Task_HeadOnBFM::CalculateLevelTurn(CPPBlackBoard* BB)
    {
        // ���� ��ȸ - ���� �þ� ���� ����
        Vector3 myLocation = BB->MyLocation_Cartesian;
        Vector3 targetLocation = BB->TargetLocaion_Cartesian;
        Vector3 myRight = BB->MyRightVector;

        Vector3 toTarget = (targetLocation - myLocation);
        float dotRight = toTarget.dot(myRight);

        Vector3 turnDirection = (dotRight > 0) ? myRight : myRight * -1.0f;

        Vector3 levelTurnPoint = myLocation + turnDirection * 1200.0f;
        levelTurnPoint.Z = myLocation.Z; // ���� ����

        return levelTurnPoint;
    }

    Vector3 Task_HeadOnBFM::CalculateVerticalManeuver(CPPBlackBoard* BB)
    {
        // ���� �⵿ - �ӵ��� ����� ����
        Vector3 myLocation = BB->MyLocation_Cartesian;
        Vector3 myForward = BB->MyForwardVector;
        Vector3 myUp = BB->MyUpVector;

        float mySpeed = BB->MySpeed_MS;

        if (mySpeed > 140.0f) // ����� �ӵ��� ���� ��
        {
            // ���� ���
            Vector3 verticalPoint = myLocation + myUp * 1000.0f + myForward * 500.0f;
            return verticalPoint;
        }
        else
        {
            // �ӵ��� �����ϸ� ���� ��ȸ�� ��ü
            return CalculateLevelTurn(BB);
        }
    }

    Vector3 Task_HeadOnBFM::CalculateLeadTurn(CPPBlackBoard* BB)
    {
        // ���� �� - ������ 3/9 ������ ������ ���� �ޱ� ���� ����
        Vector3 myLocation = BB->MyLocation_Cartesian;
        Vector3 targetLocation = BB->TargetLocaion_Cartesian;
        Vector3 targetForward = BB->TargetForwardVector;
        Vector3 myRight = BB->MyRightVector;

        // ������ �̷� ��ġ ����
        float targetSpeed = BB->TargetSpeed_MS;
        float distance = BB->Distance;
        float closingRate = 200.0f; // �뷫���� ������
        float timeToMerge = distance / closingRate;

        Vector3 predictedTargetPos = targetLocation + targetForward * targetSpeed * timeToMerge;

        // ���� ��ġ�� ���� ��
        Vector3 toPredict = (predictedTargetPos - myLocation);
        Vector3 leadTurnPoint = myLocation + toPredict * 1500.0f;

        return leadTurnPoint;
    }

    bool Task_HeadOnBFM::ShouldInitiateLeadTurn(CPPBlackBoard* BB)
    {
        // ���Ⱑ 30�� ���� ��ġ�� �ְ� �ü� ���ӵ��� ������ �� ���� �� ����
        float distance = BB->Distance;
        float los = BB->Los_Degree;

        // �Ÿ��� ��������� LOS�� 30�� �̳��� ��
        return (distance < 5556.0f && los < 30.0f); // 3nm �̳�, 30�� �̳�
    }

    bool Task_HeadOnBFM::IsEscapeWindowOpen(CPPBlackBoard* BB)
    {
        // ��Ż â ���� �Ǵ�
        float distance = BB->Distance;
        float mySpeed = BB->MySpeed_MS;
        float targetSpeed = BB->TargetSpeed_MS;
        float angleOff = BB->MyAngleOff_Degree;

        // �Ÿ��� �ְ�, �ӵ� ������ ������, ���� �ޱ� ������ ��
        return (distance > 1852.0f && mySpeed > targetSpeed + 10.0f && angleOff > 90.0f);
    }
}