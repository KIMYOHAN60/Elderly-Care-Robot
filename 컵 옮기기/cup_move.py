import rclpy
import DR_init
import time

# ë¡œë´‡ ê¸°ë³¸ ì„¤ì •
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 40, 40

# DR_init ì„¤ì •
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
            get_workpiece_weight,   # â† ì—¬ê¸°!
        )
        from DR_common2 import posx
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    # âœ… ë°”ë‹¥ ì ‘ì´‰ ê°ì§€ í•¨ìˆ˜ (Zì¶• íž˜ì´ ì¼ì • ì´ìƒ ì¦ê°€í•˜ë©´ ì ‘ì´‰ìœ¼ë¡œ ê°„ì£¼)
    def wait_until_contact(threshold_force=1.5, timeout=5.0):
        print("[C ìœ„ì¹˜: ë°”ë‹¥ ì ‘ì´‰ ê°ì§€ ì‹œìž‘]")
        start_time = time.time()
        while time.time() - start_time < timeout:
            # ì—¬ê¸´ get_tool_force ê·¸ëŒ€ë¡œ ì‚¬ìš© (íž˜ ê°ì§€ë‹ˆê¹Œ)
            force_z = get_tool_force(DR_BASE)[2]
            print(f"[ê°ì§€ ì¤‘] í˜„ìž¬ Zì¶• Force: {force_z:.2f}N")
            if force_z >= threshold_force:
                print("ðŸ“Œ ì»µì´ ë°”ë‹¥ì— ë‹¿ìŒ ê°ì§€ë¨!")
                return True
            time.sleep(0.05)
        print("âŒ ë°”ë‹¥ ê°ì§€ ì‹¤íŒ¨ (ì‹œê°„ ì´ˆê³¼)")
        return False

    # âœ… ë¬´ê²Œ ë³€í™”ë¥¼ ê°ì§€í•˜ëŠ” í•¨ìˆ˜ (get_workpiece_weightë¡œ ë‹¨ìˆœí™”)
    def wait_for_weight(threshold_delta=0.05, timeout=10.0, require_count=3):
        baseline = get_workpiece_weight()
        print(f"\n[ê¸°ì¤€ ë¬´ê²Œ] {baseline:.3f} kg\n")

        start_time = time.time()
        detected_count = 0
        last_log_time = start_time

        print("[ê°ì§€ ëŒ€ê¸°] ë¬´ê²Œ ë³€í™”(ì¦ê°€) ê¸°ë‹¤ë¦¼...")

        while time.time() - start_time < timeout:
            weight = get_workpiece_weight()
            delta = weight - baseline

            now = time.time()
            if now - last_log_time >= 0.2:
                print(f"[ë¬´ê²Œ ìƒíƒœ] í˜„ìž¬: {weight:.3f} kg | Î”ë¬´ê²Œ: {delta:.3f} kg | count: {detected_count}")
                last_log_time = now

            if delta >= threshold_delta:
                detected_count += 1
            else:
                detected_count = 0

            if detected_count >= require_count:
                print("\n[ê°ì§€ ì„±ê³µ] ìœ ì˜ë¯¸í•œ ë¬´ê²Œ ë³€í™” ê°ì§€ë¨!\n")
                return True

            time.sleep(0.1)

        print("\n[ê°ì§€ ì‹¤íŒ¨] ì‹œê°„ ì´ˆê³¼: ìœ ì˜ë¯¸í•œ ë¬´ê²Œ ë³€í™” ì—†ìŒ\n")
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

    print("[ì´ë™] ì‹œìž‘ ìœ„ì¹˜(JReady)ë¡œ ì´ë™ ì¤‘...")
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")
    gripper_open()
    movej(JReady, vel=VELOCITY, acc=ACC)

    print("[ìž‘ì—…] A ìœ„ì¹˜ ì ‘ê·¼ ë° ì»µ ì§‘ê¸°")
    movel(approach(A_pos), v=VELOCITY, a=ACC)
    movel(A_pos, v=VELOCITY, a=ACC)
    gripper_close()
    wait(2.0)
    movel(approach(A_pos), v=VELOCITY, a=ACC)
    movel(high_approach(A_pos), v=VELOCITY, a=ACC)

    print("[ì´ë™] B ìœ„ì¹˜ë¡œ ì´ë™ ì¤‘...")
    movel(approach(B_pos), v=VELOCITY, a=ACC)
    movel(B_pos, v=VELOCITY, a=ACC)

    print("[ëŒ€ê¸°] B ìœ„ì¹˜ì—ì„œ ë¬¼ì²´ íˆ¬ìž… ëŒ€ê¸° ì¤‘...")
    wait(5.0)

    print("[ê°ì§€] ë¬´ê²Œ ë³€í™” ê°ì§€ ì‹œë„ ì¤‘...")
    if not wait_for_weight(threshold_delta=0.05, timeout=10.0, require_count=3):  # 50g ë³€í™” ê°ì§€ (í™˜ê²½ì— ë”°ë¼ ì¡°ì ˆ)
        print("[ì‹¤íŒ¨] ê°ì§€ ì‹¤íŒ¨ â†’ ë³µê·€")
        movel(approach(B_pos), v=VELOCITY, a=ACC)
        movel(high_approach(B_pos), v=VELOCITY, a=ACC)
        movej(JReady, vel=VELOCITY, acc=ACC)
        node.destroy_node()
        rclpy.shutdown()
        return

    print("[ì´ë™] ê°ì§€ ì„±ê³µ â†’ C ìœ„ì¹˜ë¡œ ì´ë™")
    movel(approach(B_pos), v=VELOCITY, a=ACC)
    movel(high_approach(B_pos), v=VELOCITY, a=ACC)
    movel(approach(C_pos), v=20, a=20)  # ì¡°ê¸ˆ ëŠë¦¬ê²Œ ì ‘ê·¼
    movel(C_pos, v=10, a=10)  # ì²œì²œížˆ ë°”ë‹¥ìœ¼ë¡œ

    print("[ê°ì§€] ì»µì´ ë°”ë‹¥ì— ë‹¿ëŠ” ìˆœê°„ ê°ì§€ ì¤‘...")
    if wait_until_contact(threshold_force=1.5, timeout=5.0):
        gripper_open()
        wait(0.5)
        movel(approach(C_pos), v=VELOCITY, a=ACC)
    else:
        print("[ê²½ê³ ] ë°”ë‹¥ ê°ì§€ ì‹¤íŒ¨ â†’ ê¸°ë³¸ ë¦´ë¦¬ì¦ˆ ìˆ˜í–‰")
        gripper_open()
        wait(0.5)
        movel(approach(C_pos), v=VELOCITY, a=ACC)

    print("[ì™„ë£Œ] ìž‘ì—… ì¢…ë£Œ")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
