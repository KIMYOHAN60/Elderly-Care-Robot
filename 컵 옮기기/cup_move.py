import rclpy
import DR_init
import time

# 로봇 기본 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 40, 40

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("force_control", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            set_tool,
            set_tcp,
            movej,
            movel,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
            set_digital_output,
            wait,
            get_current_posx,
            release_force,
            get_tool_force,
            get_workpiece_weight,   # ← 여기!
        )
        from DR_common2 import posx
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    # ✅ 바닥 접촉 감지 함수 (Z축 힘이 일정 이상 증가하면 접촉으로 간주)
    def wait_until_contact(threshold_force=1.5, timeout=5.0):
        print("[C 위치: 바닥 접촉 감지 시작]")
        start_time = time.time()
        while time.time() - start_time < timeout:
            # 여긴 get_tool_force 그대로 사용 (힘 감지니까)
            force_z = get_tool_force(DR_BASE)[2]
            print(f"[감지 중] 현재 Z축 Force: {force_z:.2f}N")
            if force_z >= threshold_force:
                print("📌 컵이 바닥에 닿음 감지됨!")
                return True
            time.sleep(0.05)
        print("❌ 바닥 감지 실패 (시간 초과)")
        return False

    # ✅ 무게 변화를 감지하는 함수 (get_workpiece_weight로 단순화)
    def wait_for_weight(threshold_delta=0.05, timeout=10.0, require_count=3):
        baseline = get_workpiece_weight()
        print(f"\n[기준 무게] {baseline:.3f} kg\n")

        start_time = time.time()
        detected_count = 0
        last_log_time = start_time

        print("[감지 대기] 무게 변화(증가) 기다림...")

        while time.time() - start_time < timeout:
            weight = get_workpiece_weight()
            delta = weight - baseline

            now = time.time()
            if now - last_log_time >= 0.2:
                print(f"[무게 상태] 현재: {weight:.3f} kg | Δ무게: {delta:.3f} kg | count: {detected_count}")
                last_log_time = now

            if delta >= threshold_delta:
                detected_count += 1
            else:
                detected_count = 0

            if detected_count >= require_count:
                print("\n[감지 성공] 유의미한 무게 변화 감지됨!\n")
                return True

            time.sleep(0.1)

        print("\n[감지 실패] 시간 초과: 유의미한 무게 변화 없음\n")
        return False

    def grasp():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.5)

    def release():
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(0.5)

    def gripper_close():
        grasp()

    def gripper_open():
        release()

    def approach(pos, dz=50):
        return posx(pos[0], pos[1], pos[2] + dz, pos[3], pos[4], pos[5])

    def high_approach(pos, dz=150):
        return posx(pos[0], pos[1], pos[2] + dz, pos[3], pos[4], pos[5])

    A_pos = posx(357.370, 17.760, 66.380, 150.8, -179.49, 148.6)
    B_pos = posx(541.980, 430.670, 52.250, 50.98, 176.93, 13.38)
    C_pos = posx(480.800, 20.380, 70.270, 163.55, -177.71, 161.06)
    JReady = [0, 0, 90, 0, 90, 0]

    print("[이동] 시작 위치(JReady)로 이동 중...")
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")
    gripper_open()
    movej(JReady, vel=VELOCITY, acc=ACC)

    print("[작업] A 위치 접근 및 컵 집기")
    movel(approach(A_pos), v=VELOCITY, a=ACC)
    movel(A_pos, v=VELOCITY, a=ACC)
    gripper_close()
    wait(2.0)
    movel(approach(A_pos), v=VELOCITY, a=ACC)
    movel(high_approach(A_pos), v=VELOCITY, a=ACC)

    print("[이동] B 위치로 이동 중...")
    movel(approach(B_pos), v=VELOCITY, a=ACC)
    movel(B_pos, v=VELOCITY, a=ACC)

    print("[대기] B 위치에서 물체 투입 대기 중...")
    wait(5.0)

    print("[감지] 무게 변화 감지 시도 중...")
    if not wait_for_weight(threshold_delta=0.05, timeout=10.0, require_count=3):  # 50g 변화 감지 (환경에 따라 조절)
        print("[실패] 감지 실패 → 복귀")
        movel(approach(B_pos), v=VELOCITY, a=ACC)
        movel(high_approach(B_pos), v=VELOCITY, a=ACC)
        movej(JReady, vel=VELOCITY, acc=ACC)
        node.destroy_node()
        rclpy.shutdown()
        return

    print("[이동] 감지 성공 → C 위치로 이동")
    movel(approach(B_pos), v=VELOCITY, a=ACC)
    movel(high_approach(B_pos), v=VELOCITY, a=ACC)
    movel(approach(C_pos), v=20, a=20)  # 조금 느리게 접근
    movel(C_pos, v=10, a=10)  # 천천히 바닥으로

    print("[감지] 컵이 바닥에 닿는 순간 감지 중...")
    if wait_until_contact(threshold_force=1.5, timeout=5.0):
        gripper_open()
        wait(0.5)
        movel(approach(C_pos), v=VELOCITY, a=ACC)
    else:
        print("[경고] 바닥 감지 실패 → 기본 릴리즈 수행")
        gripper_open()
        wait(0.5)
        movel(approach(C_pos), v=VELOCITY, a=ACC)

    print("[완료] 작업 종료")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
