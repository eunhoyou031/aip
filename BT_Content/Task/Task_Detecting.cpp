#include "Task_Detecting.h"

namespace Action
{
    PortsList Task_Detecting::providedPorts()
    {
        return {
            InputPort<CPPBlackBoard*>("BB")
        };
    }

    NodeStatus Task_Detecting::tick()
    {
        Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

        bool enemyInSight = (*BB)->EnemyInSight;
        float distance = (*BB)->Distance;

        if (!enemyInSight)
        {
            // ���⸦ �þ߿��� ã�� ���� ��� - ���� ����
            (*BB)->VP_Cartesian = CalculateSearchPattern(BB.value());
        }
        else if (distance > 18520.0f) // 10nm �̻� BVR ��Ȳ
        {
            // ���ðŸ� �� ��� - ���� ������
            (*BB)->VP_Cartesian = CalculateSternConversion(BB.value());
        }
        else
        {
            // �Ϲ� ��� �ڽ�
            (*BB)->VP_Cartesian = CalculateInterceptCourse(BB.value());
        }

        // Ž�� �ÿ��� ���� ����Ʋ
        (*BB)->Throttle = 0.8f;

        return NodeStatus::SUCCESS;
    }

    Vector3 Task_Detecting::CalculateSearchPattern(CPPBlackBoard* BB)
    {
        // ���⸦ ã�� ���� ���� ���� ����
        Vector3 myLocation = BB->MyLocation_Cartesian;
        Vector3 myForward = BB->MyForwardVector;
        Vector3 myRight = BB->MyRightVector;

        // �ð� ��� S�� ���� ����
        double time = BB->RunningTime;
        float patternPhase = fmod(time, 20.0); // 20�� �ֱ�

        Vector3 searchDirection;
        if (patternPhase < 10.0)
        {
            // ���������� ����
            searchDirection = myRight;
        }
        else
        {
            // �������� ����
            searchDirection = myRight * -1.0f;
        }

        Vector3 searchPoint = myLocation + myForward * 3000.0f + searchDirection * 1500.0f;

        return searchPoint;
    }

    Vector3 Task_Detecting::CalculateInterceptCourse(CPPBlackBoard* BB)
    {
        // �Ϲ����� ��� �ڽ� - ����� ����
        Vector3 targetLocation = BB->TargetLocaion_Cartesian;
        Vector3 targetForward = BB->TargetForwardVector;
        float targetSpeed = BB->TargetSpeed_MS;
        float mySpeed = BB->MySpeed_MS;
        float distance = BB->Distance;

        // ���� ��� �ð� ���
        float interceptTime = distance / (mySpeed + targetSpeed * 0.5f);

        // ������ ���� ��ġ
        Vector3 interceptPoint = targetLocation + targetForward * targetSpeed * interceptTime;

        return interceptPoint;
    }

    Vector3 Task_Detecting::CalculateSternConversion(CPPBlackBoard* BB)
    {
        // ���� ������ ��� - ���� �Ĺ����� ����
        Vector3 myLocation = BB->MyLocation_Cartesian;
        Vector3 targetLocation = BB->TargetLocaion_Cartesian;
        Vector3 targetForward = BB->TargetForwardVector;
        Vector3 targetRight = BB->TargetRightVector;
        float aspectAngle = BB->MyAspectAngle_Degree;

        // ������Ʈ �ޱ��� �ݴ������� �ɼ�
        Vector3 offsetDirection;
        if (aspectAngle > 180.0f) // ���� ������Ʈ
        {
            offsetDirection = targetRight;
        }
        else // ���� ������Ʈ
        {
            offsetDirection = targetRight * -1.0f;
        }

        // ���� �������� �̵��Ͽ� �ʹ� �� Ȯ��
        Vector3 sternPoint = targetLocation + offsetDirection * 2000.0f - targetForward * 3000.0f;

        return sternPoint;
    }
}