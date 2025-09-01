#include "Task_WeaponEngagement.h"

namespace Action
{
    PortsList Task_WeaponEngagement::providedPorts()
    {
        return {
            InputPort<CPPBlackBoard*>("BB")
        };
    }

    NodeStatus Task_WeaponEngagement::tick()
    {
        Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

        float distance = (*BB)->Distance;
        float aspectAngle = (*BB)->MyAspectAngle_Degree;

        // ���� ��� �켱����: �̻��� -> ����
        if (IsInMissileRange(BB.value()))
        {
            (*BB)->VP_Cartesian = CalculateMissileEngagement(BB.value());
        }
        else if (IsInGunRange(BB.value()) && HasValidGunSolution(BB.value()))
        {
            (*BB)->VP_Cartesian = CalculateGunEngagement(BB.value());
        }
        else
        {
            // ���� ��� �Ұ� - ��ġ ����
            Vector3 targetLocation = (*BB)->TargetLocaion_Cartesian;
            (*BB)->VP_Cartesian = targetLocation;
        }

        // ���� �� �ִ� ����Ʋ
        (*BB)->Throttle = 1.0f;

        return NodeStatus::SUCCESS;
    }

    Vector3 Task_WeaponEngagement::CalculateMissileEngagement(CPPBlackBoard* BB)
    {
        // �̻��� ����� ���� ��ġ ���
        Vector3 targetLocation = BB->TargetLocaion_Cartesian;
        Vector3 targetForward = BB->TargetForwardVector;
        float targetSpeed = BB->TargetSpeed_MS;
        float distance = BB->Distance;

        // Pure Pursuit for missile engagement
        // �̻��� ���� �ð� ����
        float missileSpeed = 800.0f; // �뷫���� �̻��� �ӵ� m/s
        float missileFlightTime = distance / missileSpeed;

        // ǥ���� ���� ��ġ
        Vector3 leadPoint = targetLocation + targetForward * targetSpeed * missileFlightTime;

        return leadPoint;
    }

    Vector3 Task_WeaponEngagement::CalculateGunEngagement(CPPBlackBoard* BB)
    {
        // ���� ����� ���� Lead Pursuit ���
        Vector3 targetLocation = BB->TargetLocaion_Cartesian;
        Vector3 targetForward = BB->TargetForwardVector;
        Vector3 targetRight = BB->TargetRightVector;
        float targetSpeed = BB->TargetSpeed_MS;
        float distance = BB->Distance;

        // ���� źȯ �ӵ� (�뷫 1000 m/s)
        float bulletSpeed = 1000.0f;
        float timeOfFlight = distance / bulletSpeed;

        // Lead ���
        Vector3 leadVector = targetForward * targetSpeed * timeOfFlight;

        // ���Ⱑ ��ȸ ���̶�� �߰� lead �ʿ�
        Vector3 turnLead = targetRight * targetSpeed * 0.1f; // ������ ��ȸ ����

        Vector3 gunAimPoint = targetLocation + leadVector + turnLead;

        return gunAimPoint;
    }

    bool Task_WeaponEngagement::IsInMissileRange(CPPBlackBoard* BB)
    {
        float distance = BB->Distance;
        float aspectAngle = BB->MyAspectAngle_Degree;

        // AIM-9M ���̵���δ� �⺻ ����
        float rMin = 926.0f;   // 0.5nm
        float rMax = 18520.0f; // 10nm

        // ������Ʈ�� ���� ��Ÿ� ����
        if (aspectAngle < 60.0f) // �Ĺ� ����
        {
            rMax *= 1.2f; // �Ĺ濡�� �� �ָ� ��� ����
        }
        else if (aspectAngle > 120.0f) // ���� ����
        {
            rMax *= 0.8f; // ���鿡���� ��Ÿ� ����
        }

        return (distance >= rMin && distance <= rMax);
    }

    bool Task_WeaponEngagement::IsInGunRange(CPPBlackBoard* BB)
    {
        float distance = BB->Distance;
        float aspectAngle = BB->MyAspectAngle_Degree;

        // M61 ����� ��ȿ ��Ÿ�
        float gunRange = 762.0f; // 2500 feet

        if (aspectAngle > 120.0f) // ���� ������Ʈ
        {
            gunRange *= 1.6f; // 4000 feet���� Ȯ��
        }

        return (distance <= gunRange);
    }

    bool Task_WeaponEngagement::HasValidGunSolution(CPPBlackBoard* BB)
    {
        float angleOff = BB->MyAngleOff_Degree;
        float los = BB->Los_Degree;

        // ���� ��� ����
        // 1. �ޱ� ������ 45�� �̳�
        // 2. LOS�� ������ ���� ��
        // 3. Lead Pursuit ����

        return (angleOff <= 45.0f && los <= 30.0f);
    }
}