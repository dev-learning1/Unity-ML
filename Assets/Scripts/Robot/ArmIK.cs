using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ArmIK : MonoBehaviour
{
    #region Inspector 

    [Header("관절 (비워두면 이름으로 자동 검색)")]
    public ArticulationBody[] joints = new ArticulationBody[0];
    static readonly string[] JointNames =
        { "shoulder_link", "arm_link", "elbow_link", "forearm_link", "wrist_link", "hand_link" };
    public Transform tip;                       // AttachPoint
    public ArticulationBody closingFinger;      // right_gripper

    [Header("관절 회전 방향 (처음 한 번 자동 확인 후 저장)")]
    public bool signsCalibrated = false;
    public float[] jointSigns = new float[0];

    [Header("자세")]
    [Tooltip("그리퍼가 항상 수직 아래를 보도록")]
    public bool pointDown = true;
    [Tooltip("자세 오차 1 rad를 위치 오차 몇 m로 볼지")]
    public float orientationWeight = 0.2f;

    [Header("이동")]
    public float jointSpeed = 60f;         // 관절 이동 속도 (도/초)
    public float minMoveTime = 0.3f;
    public float linearStep = 0.01f;        // 직선 경로를 몇 m 간격으로 나눌지
    public int correctionPasses = 3;        // 도착 후 처짐 보정 횟수
    public float positionTolerance = 0.001f;    // 보정을 멈추는 위치 오차 기준. 실제로 그리퍼 끝이 목표와 1mm 이내 이면 더 이상 보정하지 않는다.

    [Header("테스트: IKTarget을 끌면 따라가기 (IKTarget의 Rotation Y = 손목 각도)")]
    public Transform testTarget;
    public bool followTestTarget = false;

    public bool Ready { get; private set; } // IK 준비 완료 여부. 관절 찾기, 모델 만들기, 회전 방향 확인이 끝나면 True
    public bool IsMoving { get; private set; }  // 팔이 이동중인지. MoveTo/MoveLinear 시작 때 treu. 보정이 끝났을 때 false
    public bool LastSolveOk { get; private set; } = true;   // 마지막 이동의 계산 성공 여부. 목표에 닿는 자세를 못 찾았으면 false
    public float PositionError { get; private set; }   // 마지막 이동 후 실제 위치 오차 (m)
    public float AngleError { get; private set; }      // 마지막 이동 후 수직 오차 (도)
    public float YawError { get; private set; }        // 마지막 이동 후 손목 각도 오차 (도)

    #endregion

    #region 계산용 모델 변수 

    private int n;  // 관절 개수
    private Transform baseT;    // 계산의 출발점 base link (움직이지 않는 받침)
    private Quaternion[] relRot0;   // 시작할 때 각 링크가 부모 링크에 대해 돌아가 있던 방향
    private Vector3[] pivotInParent, anchorPos, axisLocal;  // 각 관절의 회전 중심 위치(부모링크 기준), 각 관절의 회전 중심 위치(자기 링크 기준), 각 관절의 도는 축(자기 링크 기준)
    private float[] theta0, lower, upper;   // 시작할 때 각 관절 각도(도) 계산시 "여기서 몇도 더 돌았나", 각 관절이 돌 수 있는 최소 각도, 각 관절이 돌 수 있는 최대 각도
    private bool[] limited; // 각도 범위 제한이 있는 관절인지
    private Vector3 tipLocal, dirLocal, closeLocal; // 그리퍼 끝(AttachPoint)의 위치 (hand link 기준), 그리퍼가 향하는 방향. (hand link 기준) 아래를 보는지 확인할 대 사용, 손가락이 닫히는 축 (hand link 기준) 손목 각도를 확인할 때 사용
    private float? activeYaw;   // 목표 손목 각도 (null 이면 손목 각도가 자유). 보정때 도 이 값을 사용
    
    // 이동 관리용
    private Vector3 lastFollowPos;  // 테스트 시 마지막으로 따라간 KITarget의 위치
    private float lastFollowYaw;    // 마지막으로 따라간 IKTarget의 회전 Y
    private bool firstMove = true;  // 첫 이동인지. 첫 이동때만 모델 예측과 실제위치를 비교해서 회전 방향 오류를 경고

    #endregion

    #region 1. 로봇 구조를 읽고, 계산용 모델 만들기

    // 시작순서 관절 찾기 -> 계산용 모델 만들기 -> (처음이면) 회전 방향 확인
    // 끝나면 Ready = true가 되어 다른 스크립트 IK를 쓸 수 있는 상태됨
    IEnumerator Start()
    {
        AutoAssign();
        BuildModel();

        if (!signsCalibrated || jointSigns == null || jointSigns.Length != n)
            yield return CalibrateSigns();

        Ready = true;
    }

    // 관절, 그리퍼 끝(AttachPoint), 오른쪽 손가락을 이름으로 찾아서 넣기
    // 인스펙터에 값이 있다면 그대로 그 값을 씀
    void AutoAssign()
    {
        var bodies = GetComponentsInChildren<ArticulationBody>();
        if (joints == null || joints.Length == 0)
        {
            joints = new ArticulationBody[JointNames.Length];
            for (int i = 0; i < JointNames.Length; i++)
                foreach (var b in bodies)
                    if (b.name == JointNames[i]) joints[i] = b;
        }
        if (closingFinger == null)
            foreach (var b in bodies)
                if (b.name == "right_gripper") closingFinger = b;
        if (tip == null)
            foreach (var t in GetComponentsInChildren<Transform>())
                if (t.name == "AttachPoint") tip = t;

        for (int i = 0; i < joints.Length; i++)
            if (joints[i] == null) Debug.LogError($"[ArmIK] {i}번 관절을 찾지 못했습니다. Inspector에서 직접 넣어 주세요.");
        if (tip == null) Debug.LogError("[ArmIK] AttachPoint를 찾지 못했습니다.");
        if (closingFinger == null) Debug.LogError("[ArmIK] right_gripper를 찾지 못했습니다.");
    }

    // 각 관절의 회전중심, 회전축, 처음 각도, 각도 범위를 읽어서 저장 (URDF 정보)
    // 이후 계산은 진짜 로봇이 아니라 이 저장된 값으로 한다
    void BuildModel()
    {
        // 1. 관절 6개 빈칸(배열) 만들기
        n = joints.Length;
        baseT = joints[0].transform.parent;   // base_link (1번 관절의 부모) -> 계산의 출발점
        relRot0 = new Quaternion[n];
        pivotInParent = new Vector3[n];
        anchorPos = new Vector3[n];
        axisLocal = new Vector3[n];
        theta0 = new float[n];
        lower = new float[n]; upper = new float[n]; limited = new bool[n];

        // 2. 관절 정보를 채움
        // 월드 기준값은 팔이 움직이면 바뀐다. 변하지 않는 부모기준으로 바꿔서 값을 저장

        // 부모 기준 값을 사용하는 이유
        // 부모 기준 값이 있으면 어깨 각도가 몇도든 다음순서로 손끝위치를 계산으로 만들수있다
        // 자기 기준이 나오는 이유 = 관절의 회전 중심. 자신을 기준으로 알려준다
        for (int j = 0; j < n; j++)
        {
            Transform c = joints[j].transform, p = c.parent;    // c는 이 관절의 링크, p는 부모 링크
            Quaternion inv = Quaternion.Inverse(p.rotation);
            relRot0[j] = inv * c.rotation;  // 곱하면 "부모 기준" 값이 됨. relRot 은 부모 기준으로 이 링크가 돌아가 있는 방향
            anchorPos[j] = joints[j].anchorPosition;    // 자기 링크 기준. 회전 중심 위치
            axisLocal[j] = joints[j].anchorRotation * Vector3.right;    // 회전축 AritculatrionBody는 앵커의 x축으로 돈다
            pivotInParent[j] = inv * (c.position - p.position) + relRot0[j] * anchorPos[j]; // 회전 중심 위치(부모 기준)
            theta0[j] = joints[j].jointPosition[0] * Mathf.Rad2Deg; // 지금 관절 각도 (라디안 -> 도)

            var d = joints[j].xDrive;
            limited[j] = joints[j].twistLock == ArticulationDofLock.LimitedMotion;  // 각도 제한이 있는 관절인지
            lower[j] = d.lowerLimit; upper[j] = d.upperLimit;
        }

        // 3. 그리퍼 끝 정보를 채우기 (관절이 아니므로 한번만 실행)
        // 마지막 링크(hand link) 기준으로 바꿔서 저장. 손이 어디로 움지여도 손에 대한 위치 같도록
        Transform hand = joints[n - 1].transform;
        Quaternion handInv = Quaternion.Inverse(hand.rotation);
        tipLocal = handInv * (tip.position - hand.position);    // 곱하면 "손 기준" 값이 된다. 그리퍼 끝(AttachPoint)의 위치
        dirLocal = (tipLocal - anchorPos[n - 1]).normalized;   // 손목 회전 중심에서 그리퍼 끝을 향하는 방향
        closeLocal = handInv * ActualCloseAxis();              // 손가락이 닫히는 축 (손 기준)
    }

    // 관절을 4도씩 살짝 움직여봄. 모델과 같은 방향(+1)으로 도는지 반대인지(-1) 확인
    // 그리퍼에 + 넣으면 열리는지 닫히는지 알아야 함
    // 팔 관절도 +를 넣었을 때 어는 방향으로 도는지 알아야 함
    // 처음 한번 직접 코드로 돌려보고, 확인하는 것이 calibrationSigns 함수
    // 결과값은 jointSigns 에 저장. 저장됐으면 다음부터는 이 과정을 건너뛴다
    IEnumerator CalibrateSigns()
    {
        // 1. 모든 관절을 4도씩 살짝 돌린다
        jointSigns = new float[n];
        for (int j = 0; j < n; j++)
        {
            float mid = (lower[j] + upper[j]) * 0.5f;
            SetTarget(j, theta0[j] + (theta0[j] <= mid ? 4f : -4f));
        }
        yield return new WaitForSeconds(0.3f);  // 실제로 돌때까지 0.3초 기다린다

        // 2. 관절마다 +방향이 어느쪽인지를 확인
        // 몇도 돌았는지, 회전축은 알지만, 그 축을 기준으로 어느쪽으로 돌았는지를 모른다
        // 그래서 +쪽으로 돌았다면 혹은 -쪽으로 돌았다면 -> 두가지 경우의 수를 계산해보고. 실제 모습과 더 가까운 쪽을 고른다
        for (int j = 0; j < n; j++)
        {
            Transform c = joints[j].transform;

            // 실제모습: 이 링크가 지금 부모에 대해 돌아가 있는 방향
            Quaternion actual = Quaternion.Inverse(c.parent.rotation) * c.rotation;

            // 실제로 몇도 움직였다 (약 4도)
            float moved = joints[j].jointPosition[0] * Mathf.Rad2Deg - theta0[j];

            // 상상한 두개와 실제의 차이. Quaternion.Angle = 두 회전이 몇 도 다른지
            // relRot0[j] * Quaternion.AngleAxis(moved, axisLocal[j])
            // 상상 1. 처음 모습에서 +방향으로 4도 돌았다
            float ePlus = Quaternion.Angle(relRot0[j] * Quaternion.AngleAxis(moved, axisLocal[j]), actual);
            float eMinus = Quaternion.Angle(relRot0[j] * Quaternion.AngleAxis(-moved, axisLocal[j]), actual);

            // 차이가 작은 쪽이 진짜 방향
            jointSigns[j] = ePlus <= eMinus ? 1f : -1f;
            SetTarget(j, theta0[j]);    // 원래 각도로 돌려줌
        }
        yield return new WaitForSeconds(0.3f);

        signsCalibrated = true;
        Debug.Log($"[ArmIK] 회전 방향 확인 완료: {string.Join(", ", jointSigns)}\n" +
                  "Play 중에 ArmIK 컴포넌트 메뉴(점 3개)에서 Copy Component, Play 끄고 Paste Component Values 하면 다음부터 움찔거림이 없습니다.");
    }

    #endregion

    #region 2. 목표 받기 - 위치 + 아래 보기 + 손목 각도

    // 테스트 모드
    // IKTarget 을 2mm 이상 옮기거나 1도 이상 돌렸다면 MoveTo로 로봇 움직임
    // followTestTarget 가 true일 때만 동작
    void Update()
    {
        if (!Ready || !followTestTarget || testTarget == null || IsMoving) return;

        float yaw = testTarget.eulerAngles.y;   // 목표 손목 각도 (IKTarget 의 Rotation Y를 가져옴)

        // 지난번에 따라간 뒤로(로컬이 움직인 뒤로) 충분히 움직였을 때만 새로 이동한다
        // 위치 2mm 넘게 이동했거나 회전 1도 넘게 돌았을 때
        if ((testTarget.position - lastFollowPos).sqrMagnitude > 0.002f * 0.002f ||
            Mathf.Abs(Mathf.DeltaAngle(yaw, lastFollowYaw)) > 1f)
        {
            lastFollowPos = testTarget.position;
            lastFollowYaw = yaw;
            StartCoroutine(MoveTo(testTarget.position, yaw));   // 우리가 원하는 위치와 각도로 이동
        }
    }

    // 목표 위치로 이동(관절들이 한번에 돌아서 가므로 경로는 곡선)
    // yawDeg = 손목 각도, null 이면 자유
    // 계산(Solve) -> 움직이기 (Drive) -> 멈출때까지 기다리기(Settle) -> 보정(Correct)
    public IEnumerator MoveTo(Vector3 goal, float? yawDeg = null)
    {
        // [미션] 빈칸을 채워 MoveTo를 완성하세요

        // 1. 이동 중임을 표시하세요 (IsMoving)
        IsMoving = true;

        // 2. 목표 손목 각도(yawDeg)를 activeYaw에 기억하세요 (아래 Solve가 이 값을 씁니다)
        activeYaw = yawDeg;

        // 3.목표 각도 계산을 통해 실제로 움직이기
        float[] current = CurrentTargets();          // 지금 관절 각도 6개 = 출발 자세
        float[] solution = new float[n];             // 계산 결과를 담을 빈 칸 = 도착 자세

        LastSolveOk = Solve(goal, activeYaw, current, solution);
        if (!LastSolveOk) Debug.LogWarning($"[ArmIK] {goal} (손목 {yawDeg}도) 에 정확히 닿는 자세가 없습니다. 가장 가까운 자세로 이동합니다.");

        yield return Drive(solution, MoveTime(current, solution), true);   // 이동 시간 = 가장 많이 도는 관절 기준

        // 4. Settle을 호출해서 관절이 다 따라올 때까지 기다리세요 (코루틴 사용법 생각해보기) 
        yield return Settle();  // Settle 함수가 실행되고, 끝날 때까지 이 줄에서 기다린다

        // 5. Correct로 목표 위치(goal)에 맞게 보정하세요  (코루틴 사용법 생각해보기) 
        yield return Correct(goal);

        // 6. 이동이 끝났음을 표시하세요 (IsMoving)
        IsMoving = false;
    }

    // 목표까지 직선으로 이동. 길을 1cm(linearStep) 간격 점으로 나눠서 점마다 각도를 미리 계산
    // 계산이 끝나면 점들을 차례로 따라가고, 마지막에 기다리기와 보정을 진행
    public IEnumerator MoveLinear(Vector3 goal, float speed, float? yawDeg = null)
    {
        IsMoving = true;
        activeYaw = yawDeg;

        // 1. 출발위치 구하기
        float[] startAngles = CurrentTargets();

        // FK로 출발 자세의 그리퍼 끝 위치를 계산한다
        Forward(startAngles, out Vector3 startPos, out _, out _, null, null);   

        // 2. 직선을 몇개의 점으로 나눌지 정하기
        float distance = Vector3.Distance(startPos, goal);
        int pointCount = Mathf.Max(1, Mathf.CeilToInt(distance / linearStep));  // 1cm 간격일 때 점 몇개인지
        // 점 하나 지나가는 시간 = 전체 시간(거리/속도) / 점 개수
        float pointTime = Mathf.Max(Time.fixedDeltaTime, distance / Mathf.Max(speed, 0.001f) / pointCount);

        // 3. 점마다 관절 각도를 계산해서 목록에 담기 
        List<float[]> pathAngles = new List<float[]>();
        float[] prevAngles = startAngles;   // 첫 점은 출발 자세에서 부터 계산
        bool allOk = true;
        for (int i = 1; i <= pointCount; i++)
        {
            Vector3 point = Vector3.Lerp(startPos, goal, (float)i / pointCount);
            float[] pointAngles = new float[n];
            allOk &= Solve(point, activeYaw, prevAngles, pointAngles);  // allOk & Solve() 결과를 allOk 에 대입
            pathAngles.Add(pointAngles);
            prevAngles = pointAngles;
        }
        LastSolveOk = allOk;
        if (!allOk) Debug.LogWarning($"[ArmIK] 직선 경로 일부가 닿을 수 없는 범위입니다. (목표 {goal}, 손목 {yawDeg}도)");

        // 4. 실제로 움직이기 (점들을 차례로 따라가기)
        foreach (float[] pointAngles in pathAngles)
        {
            yield return Drive(pointAngles, pointTime, false);  // false로 전달해서 일정한 속도
        }
        yield return Settle();  // 모든 관절이 0.5 이내로 따라올때까지 기다린다
        yield return Correct(goal); // 확인과 보상

        IsMoving = false;   // 이동 끝
    }

    #endregion

    #region 3. 머릿속 계산 - FK로 예측하고 고치기

    // FK(순기구학) 관절 각도를 넣으면 그리퍼 끝 위치, 그리퍼 방향, 손가락 닫히는 축을 계산한다
    // FK = 관절을 이 각도로 돌리면 손끝이 어디에 오나 계산하는 것
    // angles 은 관절 6개를 이 각도로 돌린다면?
    // tipPos 손끝 위치 (그리퍼가 향하는 방향)
    // closeAxis 손가락이 닫히는 방향(손목 각도 확인)
    // jointAxes, jointPivots 각 관절의 회전축과 회전 중심. (Solve 에서만 사용. 다른 곳에서 null 넣어서 건너뜀)

    // 계산법 (받침 base_link)에서 시작해 관절을 하나씩 돌리며 쌓아 올린다
    // 위치 + 회전 + 거리 = "거기서 그 방향으로 그만큼 간 곳"
    // 회전 * 회전 = "돌린 상태에서 또 돌리기"
    void Forward(float[] angles, out Vector3 tipPos, out Vector3 tipDir, out Vector3 closeAxis, Vector3[] jointAxes, Vector3[] jointPivots)
    {
        // 출발 : 받침의 위치와 방향
        Vector3 linkPos = baseT.position;
        Quaternion linkRot = baseT.rotation;

        // 관철 차례대로
        for (int j = 0; j < n; j++)
        {
            // 이 관절의 회전중심 : 부모에서 설계도만큼 간 곳
            Vector3 pivotWorld = linkPos + linkRot * pivotInParent[j];

            // 이 링크의 방향: 부모 방향 -> 처음 꺾음 -> 이번에 더 돌린만큼 jointSigns로 실제 회전 방향에 맞춤
            float turn = jointSigns[j] * (angles[j] - theta0[j]);
            Quaternion childRot = linkRot * relRot0[j] * Quaternion.AngleAxis(turn, axisLocal[j]);

            // Solve가 요청했을때만 실행
            // 이 관절의 회전축과 회전 중심을 저장
            if (jointAxes != null)
            {
                jointAxes[j] = jointSigns[j] * (childRot * axisLocal[j]);
                jointPivots[j] = pivotWorld;
            }

            // 이 링크의 위치를 구하고, 다음 바퀴에서는 이 링크가 부모가 된다
            linkPos = pivotWorld - childRot * anchorPos[j];
            linkRot = childRot;
        }

        tipPos = linkPos + linkRot * tipLocal;  // 손끝 위치
        tipDir = linkRot * dirLocal;    // 그리퍼 방향
        closeAxis = linkRot * closeLocal;   // 손가락이 닫히는 방향
    }

    // 손가락이 닫히는 축이 목표 손목 각도와 몇 도 어긋났는지 계산
    // 손가락은 앞뒤가 같으므로(180도 대칭) 가까운 쪽을 기준으로 잼
    // static인 이유는 ArmIK의 변수를 쓰지 않고, 파라미터로 받은 값만으로 계산하는 함수
    static Vector3 YawErrorVector(Vector3 closeAxis, float yawDeg, out float errorDeg)
    {
        // 1. 목표 방향: 월드 z축(앞)을 수직축(위)로 yawDeg 만큼 돌린 방향
        Vector3 targetDir = Quaternion.AngleAxis(yawDeg, Vector3.up) * Vector3.forward;

        // 2. 지금 방향 = 손가락 닫히는 축을 바닥에 비춘 그림자. 위아래 성분을 버리고 길이 1로
        // projectOnPlane(벡터, 면의 법선) : 벡터를 그 면에 눕힌 그림자를 구한다
        Vector3 currentDir = Vector3.ProjectOnPlane(closeAxis, Vector3.up).normalized;

        // 3. 180도 대칭 처리 (손가락 앞뒤가 똑같아서 0도 180도 가 같은 자세)
        // 두 방향이 반대로 보고있으면 Dot < 0, 목표를 뒤집어서 가까운 쪽으로 맞춘다
        // Vector3.Dot(a,b) 두 방향이 같은 쪽이면 +, 직각이면 0, 반대면 -
        if (Vector3.Dot(currentDir, targetDir) < 0f) targetDir = -targetDir;

        // 4. 몇도 어긋났는지
        errorDeg = Vector3.Angle(currentDir, targetDir);

        // 5. 어느 쪽으로 돌려야 하나
        // 부호 = 돌릴방향, 길이 = 어긋난 정도
        // Vector3.Cross(a,b) a를 b쪽으로 돌릴때의 회전축을 구한다. 구 방향 모두에 수직인 방향
        return Vector3.Cross(currentDir, targetDir);
    }

    // IK  목표와의 오차 확인해 각도를 조금 고치기를 최대 300번 반복한다
    bool Solve(Vector3 goal, float? yaw, float[] startAngles, float[] result)
    {
        float[] angles = (float[])startAngles.Clone();
        Vector3[] jointAxes = new Vector3[n];
        Vector3[] jointPivots = new Vector3[n];

        bool useYaw = pointDown && yaw.HasValue;
        int errorCount = 3 + (pointDown ? 2 : 0) + (useYaw ? 1 : 0);

        float rotWeight = orientationWeight;

        float[,] jacobian = new float[errorCount, n];
        float[] errors = new float[errorCount];
        float[,] matrixA = new float[errorCount, errorCount];
        const float damping = 0.02f;
        float yawErrorDeg = 0f;

        for (int iter = 0; iter < 300; iter++)
        {
            Forward(angles, out Vector3 tipPos, out Vector3 tipDir, out Vector3 closeAxis, jointAxes, jointPivots);

            Vector3 posError = goal - tipPos;
            Vector3 downError = Vector3.Cross(tipDir, Vector3.down);
            Vector3 yawError = useYaw ? YawErrorVector(closeAxis, yaw.Value, out yawErrorDeg) : Vector3.zero;

            if (posError.magnitude < 0.0005f &&
                (!pointDown || Vector3.Angle(tipDir, Vector3.down) < 0.5f) &&
                (!useYaw || yawErrorDeg < 0.5f)) break;

            errors[0] = posError.x; errors[1] = posError.y; errors[2] = posError.z;
            if (pointDown) { errors[3] = rotWeight * downError.x; errors[4] = rotWeight * downError.z; }
            if (useYaw) errors[5] = rotWeight * yawError.y;

            for (int j = 0; j < n; j++)
            {
                Vector3 tipMove = Vector3.Cross(jointAxes[j], tipPos - jointPivots[j]);
                jacobian[0, j] = tipMove.x; jacobian[1, j] = tipMove.y; jacobian[2, j] = tipMove.z;
                if (pointDown) { jacobian[3, j] = rotWeight * jointAxes[j].x; jacobian[4, j] = rotWeight * jointAxes[j].z; }
                if (useYaw) jacobian[5, j] = rotWeight * jointAxes[j].y;
            }

            for (int row = 0; row < errorCount; row++)
            {
                for (int col = 0; col < errorCount; col++)
                {
                    float sum = 0f;
                    for (int k = 0; k < n; k++) sum += jacobian[row, k] * jacobian[col, k];
                    matrixA[row, col] = sum + (row == col ? damping * damping : 0f);
                }
            }
            float[] solved = SolveLinear(matrixA, errors, errorCount);
            if (solved == null) break;

            for (int j = 0; j < n; j++)
            {
                float deltaRad = 0f;
                for (int row = 0; row < errorCount; row++) deltaRad += jacobian[row, j] * solved[row];
                angles[j] += Mathf.Clamp(deltaRad * Mathf.Rad2Deg, -10f, 10f);
                if (limited[j]) angles[j] = Mathf.Clamp(angles[j], lower[j], upper[j]);
            }
        }

        System.Array.Copy(angles, result, n);

        Forward(angles, out Vector3 finalPos, out Vector3 finalDir, out Vector3 finalClose, null, null);
        float finalYawDeg = 0f;
        if (useYaw) YawErrorVector(finalClose, yaw.Value, out finalYawDeg);
        return (goal - finalPos).magnitude < 0.002f &&
               (!pointDown || Vector3.Angle(finalDir, Vector3.down) < 3f) &&
               (!useYaw || finalYawDeg < 3f);
    }

    // 가우스 소거법으로 연립방정식 A x = b 를 푼다 (Solve에서 각도 고칠 양을 구할 때 사용)
    // 풀 수 없으면 null을 돌려준다
    static float[] SolveLinear(float[,] A, float[] b, int n)
    {
        var M = (float[,])A.Clone();
        var v = (float[])b.Clone();
        for (int col = 0; col < n; col++)
        {
            int piv = col;
            for (int r = col + 1; r < n; r++)
                if (Mathf.Abs(M[r, col]) > Mathf.Abs(M[piv, col])) piv = r;
            if (Mathf.Abs(M[piv, col]) < 1e-9f) return null;
            if (piv != col)
            {
                for (int c = 0; c < n; c++) { float t = M[col, c]; M[col, c] = M[piv, c]; M[piv, c] = t; }
                float tv = v[col]; v[col] = v[piv]; v[piv] = tv;
            }
            for (int r = col + 1; r < n; r++)
            {
                float f = M[r, col] / M[col, col];
                for (int c = col; c < n; c++) M[r, c] -= f * M[col, c];
                v[r] -= f * v[col];
            }
        }
        var x = new float[n];
        for (int r = n - 1; r >= 0; r--)
        {
            float s = v[r];
            for (int c = r + 1; c < n; c++) s -= M[r, c] * x[c];
            x[r] = s / M[r, r];
        }
        return x;
    }

    #endregion

    #region 4. 실제로 움직이기 - xDrive target 바꾸고 기다리기

    float MoveTime(float[] a, float[] b)
    {
        float max = 0f;
        for (int j = 0; j < n; j++) max = Mathf.Max(max, Mathf.Abs(b[j] - a[j]));
        return Mathf.Max(minMoveTime, max / jointSpeed);
    }

    IEnumerator Drive(float[] goal, float duration, bool smooth)
    {
        float[] start = CurrentTargets();
        float t = 0f;
        while (t < 1f)
        {
            t += Time.fixedDeltaTime / duration;
            float s = smooth ? Mathf.SmoothStep(0f, 1f, t) : Mathf.Clamp01(t);
            for (int j = 0; j < n; j++) SetTarget(j, Mathf.Lerp(start[j], goal[j], s));
            yield return new WaitForFixedUpdate();
        }
    }

    IEnumerator Settle()
    {
        float wait = 0f;
        while (wait < 1f)
        {
            bool done = true;
            for (int j = 0; j < n; j++)
                if (Mathf.Abs(joints[j].jointPosition[0] * Mathf.Rad2Deg - joints[j].xDrive.target) > 0.5f)
                { done = false; break; }
            if (done) break;
            wait += Time.fixedDeltaTime;
            yield return new WaitForFixedUpdate();
        }
    }

    // 지금 관절들의 xDrive Target 값을 읽어온다
    float[] CurrentTargets()
    {
        float[] targets = new float[n];
        for (int j = 0; j < n; j++)
        {
            targets[j] = joints[j].xDrive.target;
        }
        return targets;
    }

    // 파라미터로 전달받은 j번 관절의 xDrive target을 deg(도)로 바꾼다
    void SetTarget(int j, float deg)
    {
        var d = joints[j].xDrive;
        d.target = deg;
        joints[j].xDrive = d;
    }

    #endregion

    #region 5. 확인과 보정 - 처진 만큼 다시 보정
    IEnumerator Correct(Vector3 goal)
    {
        Vector3 aimPos = goal;

        for (int pass = 0; pass <= correctionPasses; pass++)
        {
            Vector3 posError = goal - tip.position;

            if (pass == 0 && firstMove)
            {
                firstMove = false;
                if (posError.magnitude > 0.02f)
                    Debug.LogWarning("[ArmIK] 모델 예측과 실제 위치 차이가 큽니다. Joint Signs 값이 틀렸을 수 있습니다. Signs Calibrated 체크를 끄고 다시 확인하세요.");
            }

            if (posError.magnitude < positionTolerance || pass == correctionPasses) break;

            aimPos += posError;

            float[] currentAngles = CurrentTargets();
            float[] solution = new float[n];
            Solve(aimPos, activeYaw, currentAngles, solution);
            yield return Drive(solution, 0.2f, true);
            yield return Settle();
        }

        PositionError = (goal - tip.position).magnitude;
        AngleError = Vector3.Angle(ActualDir(), Vector3.down);
        YawError = 0f;
        if (activeYaw.HasValue)
        {
            YawErrorVector(ActualCloseAxis(), activeYaw.Value, out float yawErrorDeg);
            YawError = yawErrorDeg;
        }
    }

    Vector3 ActualDir()
    {
        Vector3 wristPivot = joints[n - 1].transform.TransformPoint(anchorPos[n - 1]);
        return (tip.position - wristPivot).normalized;
    }

    // 실제 손가락이 닫히는 축(오른쪽 손가락이 움직이는 방향)
    // 손목 각도 오차 잴때도 사용
    Vector3 ActualCloseAxis()
    {
        // 손가락 관절은 앵커의 x축 방향으로 미끄러진다. 그 축을 월드 방향으로 바꾼다
        Vector3 slideAxisLocal = closingFinger.anchorRotation * Vector3.right;
        return (closingFinger.transform.rotation * slideAxisLocal).normalized;
    }
    #endregion

    #region 화면 표시

    // 화면 위쪽에 IK 상태, 마지막 오차, 계산 성공 여부를 보여준다
    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(300, 10, 290, 95), GUI.skin.box);
        GUILayout.Label(!Ready ? "IK: 회전 방향 확인 중..." : IsMoving ? "IK: 이동 중" : "IK: 대기");
        GUILayout.Label($"마지막 오차: {PositionError * 100f:F2} cm / 수직 {AngleError:F1}도 / 손목 {YawError:F1}도");
        GUILayout.Label($"경로 계산: {(LastSolveOk ? "성공" : "닿을 수 없음")}");
        GUILayout.EndArea();
    }

    #endregion
}