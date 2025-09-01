#include "Task_OffensiveBFM.h"

namespace Action
{
    PortsList Task_OffensiveBFM::providedPorts()
    {
        return {
            InputPort<CPPBlackBoard*>("BB")
        };
    }

    NodeStatus Task_OffensiveBFM::tick()
    {
        Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

        float distance = (*BB)->Distance;
        float aspectAngle = (*BB)->MyAspectAngle_Degree;

        // ��Ʈ�� ������ Ȯ�� �� ����
        if (!IsInTurnCircle(BB.value()))
        {
            // ������ �� ��Ŭ �ٱ��� ���� �� - ��Ʈ�� ������� �̵�
            (*BB)->VP_Cartesian = CalculateEntryWindow(BB.value());
        }
        else
        {
            // ������ �� ��Ŭ �ȿ� ���� ��
            if (distance > 914.4f) // 3000 feet = 914.4m
            {
                // 3000��Ʈ �̻󿡼��� ���� ����
                (*BB)->VP_Cartesian = CalculateLagPursuit(BB.value());
            }
            else
            {
                // 3000��Ʈ �̳������� ���� ���� (���� ���)
                (*BB)->VP_Cartesian = CalculateLeadPursuit(BB.value());
            }
        }

        // ����Ʋ ���� (�ڳ� �ӵ� ����)
        if ((*BB)->MySpeed_MS < 130.0f) // 450 KCAS ? 230 m/s, �ڳʼӵ����� ������
        {
            (*BB)->Throttle = 1.0f;
        }
        else if ((*BB)->MySpeed_MS > 140.0f) // �ڳʼӵ����� ������
        {
            (*BB)->Throttle = 0.7f;
        }
        else
        {
            (*BB)->Throttle = 0.85f;
        }

        return NodeStatus::SUCCESS;
    }

    Vector3 Task_OffensiveBFM::CalculateEntryWindow(CPPBlackBoard* BB)
    {
        // ���Ⱑ ��ȸ�� ������ ����(��Ʈ�� ������)���� �̵�
        Vector3 targetLocation = BB->TargetLocaion_Cartesian;
        Vector3 targetForward = BB->TargetForwardVector;

        // ���� �Ĺ� 30�� �������� �̵�
        Vector3 entryPoint = targetLocation - targetForward * 1852.0f; // 1nm ��

        return entryPoint;
    }

    Vector3 Task_OffensiveBFM::CalculateLagPursuit(CPPBlackBoard* BB)
    {
        // ���� ���� - ������ ������ ����
        Vector3 targetLocation = BB->TargetLocaion_Cartesian;
        Vector3 targetForward = BB->TargetForwardVector;

        // ���� ���� ���� ������ ����
        Vector3 lagPoint = targetLocation - targetForward * 500.0f;

        return lagPoint;
    }

    Vector3 Task_OffensiveBFM::CalculateLeadPursuit(CPPBlackBoard* BB)
    {
        // ���� ���� - ������ ������ ���� (���� ��ݿ�)
        Vector3 targetLocation = BB->TargetLocaion_Cartesian;
        Vector3 targetForward = BB->TargetForwardVector;
        float targetSpeed = BB->TargetSpeed_MS;

        // �ð� ���� (Time of Flight)
        float tof = BB->Distance / 1000.0f; // ������ TOF ���

        // ���� ����Ʈ ���
        Vector3 leadPoint = targetLocation + targetForward * targetSpeed * tof;

        return leadPoint;
    }

    bool Task_OffensiveBFM::IsInTurnCircle(CPPBlackBoard* BB)
    {
        float distance = BB->Distance;
        float targetSpeed = BB->TargetSpeed_MS;

        // ������ �� ��Ŭ �ݰ� ���� (������ ���)
        float estimatedTurnRadius = (targetSpeed * targetSpeed) / (9.81f * 7.0f); // 7G ����

        // �Ÿ��� �� ��Ŭ �ݰ��� 2�躸�� ������ �� ��Ŭ �ȿ� �ִٰ� �Ǵ�
        return distance < (estimatedTurnRadius * 2.0f);
    }
}