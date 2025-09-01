#include "BFM_ModeDetermination.h"

namespace Action
{
    PortsList BFMModeDetermination::providedPorts()
    {
        return {
            InputPort<CPPBlackBoard*>("BB")
        };
    }

    NodeStatus BFMModeDetermination::tick()
    {
        Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");

        float distance = (*BB)->Distance;
        float aspectAngle = (*BB)->MyAspectAngle_Degree;
        float angleOff = (*BB)->MyAngleOff_Degree;
        bool enemyInSight = (*BB)->EnemyInSight;

        // ���ðŸ� �� ��Ȳ (BVR - Beyond Visual Range)
        if (distance > 18520.0f) // 10nm = 18520m
        {
            (*BB)->BFM = DETECTING;
            return NodeStatus::SUCCESS;
        }

        // �þ߿� ���� ��� Ž�� ���
        if (!enemyInSight)
        {
            (*BB)->BFM = DETECTING;
            return NodeStatus::SUCCESS;
        }

        // Head-on BFM �Ǵ� (���� ������Ʈ, ���� �ޱ� ����)
        if (aspectAngle > 120.0f && angleOff > 120.0f)
        {
            (*BB)->BFM = HABFM;
            return NodeStatus::SUCCESS;
        }

        // Offensive BFM �Ǵ� (���� ������Ʈ)
        if (aspectAngle < 60.0f)
        {
            (*BB)->BFM = OBFM;
            return NodeStatus::SUCCESS;
        }

        // Defensive BFM �Ǵ� (���Ⱑ �� �Ĺ濡 �ִ� ���)
        if (aspectAngle > 120.0f && angleOff < 60.0f)
        {
            (*BB)->BFM = DBFM;
            return NodeStatus::SUCCESS;
        }

        // �⺻���� NONE
        (*BB)->BFM = NONE;
        return NodeStatus::SUCCESS;
    }
}