using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

// Agent - 빨강은 피하고 초록색은 먹는다

public class GoodFoodAgent : Agent
{
    // 변수
    // 연결할 오브젝트, 움직임 관련된 변수, 보상설정(잘했을 때, 못 했을 때), 움직일 수 있도록
    public Transform target;    // 초록큐브(먹어야 하는 것)
    public Transform badTarget; // 빨강큐브(피해야 하는 것)

    // 움직임 설정
    public float forceMultiplier = 10f;
    private Rigidbody rBody;

    // 보상 설정
    public float goodReward = 1f;   // 초록 큐브에 닿았을 때
    public float badPenalty = -1f;  // 빨강 큐브에 닿았을 때

    public override void Initialize()
    {
        rBody = GetComponent<Rigidbody>();
    }

    // 무작위 위치를 뽑으나 target, badtarget이 3m 이상 떨어진 곳이 나올때까지 계속 뽑음
    // RandomSpot -> 리턴이 있는 함수 (리턴값 어떤 것을 줄 지 고민), 파라미터 값(벡터값 변수로 2개)
    // 반복문 사용, 랜덤뽑기 범위는 -4~4. 반복문은 두 개의 스팟이 거리가 3이상(Vector3.Distance)이면 반복문 종료
    // 리턴으로 spot을 돌려줍니다.
    private Vector3 RandomSpot(Vector3 avoid1, Vector3 avoid2)
    {
        Vector3 spot = Vector3.zero;

        for(int i = 0; i < 30; i++)
        {
            spot = new Vector3(Random.Range(-4f, 4f), 0.5f, Random.Range(-4f, 4f));

            if (Vector3.Distance(spot, avoid1) > 3f && Vector3.Distance(spot, avoid2) > 3f) break;
        }

        return spot;
    }

    // 에피소드 시작 정의 (한 판 시작)
    // 공 떨어졌을 때 판의 가운데로 되돌린다
    // 타켓, 배드타켓의 위치를 재정의
    // 공 -> 초록큐브 -> 빨간큐브 순으로 공에서 떨어진 자리에 놓는다
    public override void OnEpisodeBegin()
    {
        // 공 떨어졌을 때 판의 가운데로 되돌린다 -> rollerballAgent
        if(transform.localPosition.y < 0)
        {
            rBody.angularVelocity = Vector3.zero;   // 회전속도
            rBody.linearVelocity = Vector3.zero;    // 이동 속도
            transform.localPosition = new Vector3(0, 0.5f, 0);
        }
        target.localPosition = RandomSpot(transform.localPosition, transform.localPosition);    // 이 시점에는 피해야할 물체가 공 하나뿐임
        badTarget.localPosition = RandomSpot(transform.localPosition, target.localPosition); // 공위치, 초록큐브 위치를 피해서 새로운 위치를 지정해야함
    }

    // Agent 동작
    // 관찰 -> 행동 -> 보상 -> 학습
    public override void CollectObservations(VectorSensor sensor)   // 관측값을 담을 바구니
    {
        // 어떤 관측정보를 전달해야할지
        // 공(내위치), 초록큐브위치, 빨간큐브위치 -> Vector3 -> 3개 * 3 = 9개
        // 내 속도 x, 내 속도 z -> float -> 1개 * 2 = 2개
        // 총합 11개의 관측, Space size = 11 입력

        sensor.AddObservation(transform.localPosition);
        sensor.AddObservation(target.localPosition);
        sensor.AddObservation(badTarget.localPosition);
        
        sensor.AddObservation(rBody.linearVelocity.x);  // 이동속도 (내 속도의 x값)
        sensor.AddObservation(rBody.linearVelocity.z);  // 이동속도 (내 속도의 z값)


    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // 행동 (공의 이동, x와 z값을 사용하여 힘을 더해준다)

        Vector3 controlSignal = Vector3.zero;
        controlSignal.x = actions.ContinuousActions[0]; // -1~1
        controlSignal.z = actions.ContinuousActions[1]; // -1~1
        rBody.AddForce(controlSignal * forceMultiplier);

        // 보상
        float distanceToGood = Vector3.Distance(transform.localPosition, target.localPosition);
        float distanceToBad = Vector3.Distance(transform.localPosition, badTarget.localPosition);

        // 조건문 완성하기
        // 각 조건문에 어울리는 AddReward와 EndEpisode 함수 호출

        // AddReward를 사용한 이유는 나중에 시간 벌점, 거리 기반 보상이 추가가 된다고 해도 서로 덮어쓰지 않게 하기 위해서

        // AddReward() 이번 스텝에 보상 x를 더함
        // SetReward 이번 스텝에 지금까지 쌓인 보상을 지우고 x로 덮어씀

        // Academy.Instance ML agent를 관리하는 academy 객체에 접근
        // StatsRecorder 파이썬 학습기의 통계를 보내주는 기록기
        // "GoodFood/GoodRate" GoodFood 는 그룹(탭) 이름, GoodRate 그래프 이름

        if (distanceToGood < 1.42f)  // 초록큐브에 가까워졌을 때
        {
            AddReward(goodReward);
            Academy.Instance.StatsRecorder.Add("GoodFood/GoodRate", 1f);    // Add는 평균 방식으로 집계
            EndEpisode();
        }
        else if (distanceToBad < 1.42f)  // 빨간큐브에 가까워졌을 때
        {
            AddReward(badPenalty);
            Academy.Instance.StatsRecorder.Add("GoodFood/GoodRate", 0f);
            EndEpisode();
        }
        else if (transform.localPosition.y < 0) // 바닥에서 떨어졌을 때
        {
            Academy.Instance.StatsRecorder.Add("GoodFood/GoodRate", 0f);
            EndEpisode();
        }
    }
    
    // 수동 모드 테스트를 위한 함수
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // -1~1 -> GetAxis
        var continuousActionsout = actionsOut.ContinuousActions;

        continuousActionsout[0] = Input.GetAxis("Horizontal");
        continuousActionsout[1] = Input.GetAxis("Vertical");
    }
}
