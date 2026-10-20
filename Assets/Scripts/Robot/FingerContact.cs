using UnityEngine;

// 손가락 하나가 "지금 부품에 닿아있는지" 알려주는 역할
// FingerContact "닿았다"만 알려주고, "잡았다" -> GripperController에서 실행
// GripperController에서 손가락 두개가 같은 부품을 잡았는지, 한쪽만 닿은 것은 잡은 것이 아니다라고 판단

public class FingerContact : MonoBehaviour
{
    public float holdTime = 0.1f;   // 마지막 접촉 후 이 시간까지는 "닿아있음"으로 인정
    private Rigidbody touching; // 마지막으로 닿은 부품
    private float lastTouchTime = -1f;  // 마지막으로 닿은 물리 시간. -1 이면 한번도 안닿았다.

    // 최근 holdTime 안에 닿았던 부품을 돌려주는
    public Rigidbody TouchingPart
    {
        get
        {
            // 닿은 부품이 없거나, 마지막 접촉이 holdTime 보다 오래전이다 => 안닿은 것
            // null 리턴
            if (touching == null || Time.fixedTime - lastTouchTime > holdTime) return null;
            return touching;
        }
    }

    // 충돌 함수
    // 충돌 함수 처음 닿는 순간 한번, 닿아있는 동안 매번
    // => 는 한 줄 짜리 함수 표기
    private void OnCollisionEnter(Collision collision) => Record(collision);
    
    private void OnCollisionStay(Collision collision) => Record(collision);

    // 닿은 것이 "부품" 이면 무엇이, 언제 닿았는지를 기록
    private void Record(Collision collision)
    {
        // 부품 본체를 가져오기
        // 부품은 자식 메시 콜라이더가 있을 수 있으니, 본체면 Rigidbody를 기준으로 기록
        Rigidbody rb = collision.collider.attachedRigidbody;

        // 물체가 존재해야하고, 태그가 Part 인지 체크
        if(rb != null && rb.CompareTag("Part"))
        {
            touching = rb;
            lastTouchTime = Time.fixedTime; // fixedTiime (충돌함수가 물리 스텝 FixedUpdate에서 불린다. 같은 물리시간을 사용하면 좋음)

        }

    }

    public void Clear()
    {
        // 기록 초기화
        touching = null;
        lastTouchTime = -1f;
    }

}
