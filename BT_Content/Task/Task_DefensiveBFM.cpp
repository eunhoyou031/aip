#include "Task_DefensiveBFM.h"

namespace Action
{
    PortsList Task_DefensiveBFM::providedPorts()
    {
        return {
            InputPort<CPPBlackBoard*>("BB")
        };
    }

    NodeStatus Task_DefensiveBFM::tick()
    {
        Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

        float distance = (*BB)->Distance;
        float los = (*BB)->Los_Degree;

        // �̻��� ���� ��Ȳ Ȯ��
        if (IsUnderMissileAttack(BB.value()))
        {
            // �̻��� ȸ�� - �� ��ġ�� �̵�
            (*BB)->VP_Cartesian = CalculateBeamManeuver(BB.value());
        }
        else if (distance < 914.4f) // 3000 feet �̳� ���� ���
        {
            // ���� ��� ¡ŷ
            (*BB)->VP_Cartesian = CalculateJinkManeuver(BB.value());
        }
        else
        {
            // �⺻ ��� ��ȸ
            (*BB)->VP_Cartesian = CalculateDefensiveTurn(BB.value());
        }

        // ��� �� �ִ� ����Ʋ
        (*BB)->Throttle = 1.0f;

        return NodeStatus::SUCCESS;
    }

    Vector3 Task_DefensiveBFM::CalculateDefensiveTurn(CPPBlackBoard* BB)
    {
        // ���⿡�� BFM ���� ���� - ��º��͸� ���⿡�� ���� �ִ� ��ȸ
        Vector3 myLocation = BB->MyLocation_Cartesian;
        Vector3 targetLocation = BB->TargetLocaion_Cartesian;
        Vector3 myForward = BB->MyForwardVector;
        Vector3 myRight = BB->MyRightVector;

        // ���� ���� ����
        Vector3 toTarget = targetLocation - myLocation;

        // ���� ������ ���� ���� ��ȸ ���� ����
        float dotRight = toTarget.dot(myRight);

        Vector3 turnDirection;
        if (dotRight > 0)
        {
            // ���������� ��ȸ
            turnDirection = myRight;
        }
        else
        {
            // �������� ��ȸ
            turnDirection = myRight * -1.0f;
        }

        // �ִ� ��ȸ���� ���� ������ ��ȸ
        Vector3 defensivePoint = myLocation + turnDirection * 1000.0f;
        defensivePoint.Z = myLocation.Z; // ���� ��ȸ ����

        return defensivePoint;
    }

    Vector3 Task_DefensiveBFM::CalculateBeamManeuver(CPPBlackBoard* BB)
    {
        // �̻����� 3/9 ����(��) ��ġ�� ���� �⵿
        Vector3 myLocation = BB->MyLocation_Cartesian;
        Vector3 targetLocation = BB->TargetLocaion_Cartesian;
        Vector3 myRight = BB->MyRightVector;

        // ���⿡ ���� 90�� �������� �⵿
        Vector3 beamDirection = myRight;
        Vector3 beamPoint = myLocation + beamDirection * 2000.0f;

        return beamPoint;
    }

    Vector3 Task_DefensiveBFM::CalculateJinkManeuver(CPPBlackBoard* BB)
    {
        // ���� �� ���� �ұ�Ģ �⵿
        Vector3 myLocation = BB->MyLocation_Cartesian;
        Vector3 myForward = BB->MyForwardVector;
        Vector3 myUp = BB->MyUpVector;
        Vector3 myRight = BB->MyRightVector;

        // �ð� ��� �ұ�Ģ �⵿ ����
        double time = BB->RunningTime;
        int pattern = ((int)(time * 2.0)) % 4;

        Vector3 jinkDirection;
        switch (pattern)
        {
        case 0: // ����
            jinkDirection = myUp;
            break;
        case 1: // ����������
            jinkDirection = myRight;
            break;
        case 2: // �Ʒ���
            jinkDirection = myUp * -1.0f;
            break;
        case 3: // ��������
            jinkDirection = myRight * -1.0f;
            break;
        }

        Vector3 jinkPoint = myLocation + jinkDirection * 500.0f + myForward * 1000.0f;

        return jinkPoint;
    }

    bool Task_DefensiveBFM::IsUnderMissileAttack(CPPBlackBoard* BB)
    {
        // ������ �̻��� ���� �Ǵ� ����
        // �����δ� RWR(���̴� ��� ���ű�) ��ȣ�� �̻��� Ž�� ������ �ʿ�
        float los = BB->Los_Degree;
        float distance = BB->Distance;

        // ���Ⱑ ���� ���ϰ� �ְ� ���� �Ÿ� �̳��� �̻��� ���� ���ɼ� ����
        return (distance > 1852.0f && distance < 18520.0f && los < 30.0f);
    }
}