import rclpy
import DR_init
import time

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("force_response", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            movej,
            movel,
            get_current_posx,
            set_tool,
            set_tcp,
            set_digital_output,
            wait,
            get_current_posj,
            DR_AXIS_Z,
            DR_BASE,
            DR_TOOL,
        )
        from DR_common2 import posx
    except ImportError as e:
        print(f"Import Error: {e}")
        return

    def grip():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.5)

    def release():
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(0.5)

    def approach(pos):
        # Zì¶•ìœ¼ë¡œ 250mm ìœ„ë¡œ ì ‘ê·¼ ìœ„ì¹˜ ìƒì„±
        return posx([pos[0], pos[1], pos[2] + 250, pos[3], pos[4], pos[5]])

    # ì¤€ë¹„ ìžì„¸ ë° ì‚¬ì „ ìœ„ì¹˜ ì§€ì •
    JReady = [0, 0, 90, 0, 90, 0]  # home
    JReady2 = [-2.91, 3.49, 47.36, -0.57, 83.65, 0.70]  # ê±°ì¹˜ ì´ˆê¸°
    JReady3 = [-2.91, 3.49, 47.36, -0.57, 13.17, 0.70]  # ê±°ì¹˜ í›„
    Jmove4 = [-21.65, 4.96, 23.77, 4.68, 132.40, 0.69] # releas í›„ ìœ„ì¹˜1
    Jmove5 = [-70.61, 6.50, 52.20, -3.84, 118.49, -1.38] # releas í›„ ìœ„ì¹˜2

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    
    movej(JReady, vel=VELOCITY, acc=ACC)
    movej(JReady2, vel=VELOCITY, acc=ACC)

    node.get_logger().info("Waiting for external force...")

    # 1. ìˆœì‘ ì œì–´ ì¼œê¸°
    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
    time.sleep(0.5)

    try:
        while rclpy.ok():
            # 2. ë§¤ ë°˜ë³µë§ˆë‹¤ ê·¸ë¦¬í¼ ì—´ê¸°
            release()

            # 3. ê·¸ë¦¬í¼ ì˜¤í”ˆ ì§í›„ ì•ˆì •í™” ëŒ€ê¸° (0.5ì´ˆ)
            wait(0.5)

            # 4. ì™¸ë¶€ ë‹¹ê¹€ ì—¬ë¶€ ì—°ì† íŒì •(3íšŒ í´ë§)
            start_time = time.time()
            timeout = 10.0
            pulled = False
            consecutive_hits = 0

            node.get_logger().info("Checking for pull...")

            while True:
                rclpy.spin_once(node, timeout_sec=0.1)
                if check_force_condition(DR_AXIS_Z, max=5, ref=DR_TOOL):
                    consecutive_hits += 1
                else:
                    consecutive_hits = 0

                # 3íšŒ ì—°ì† ê²€ì¶œë˜ë©´ pulled=True
                if consecutive_hits >= 3:
                    pulled = True
                    node.get_logger().info("Pull detected (3 consecutive).")
                    break

                if time.time() - start_time > timeout:
                    node.get_logger().error("Pull not detected within timeout; aborting loop.")
                    break

            # 5. ìˆœì‘ ì œì–´ í•´ì œ
            release_compliance_ctrl()

            if not pulled:
                node.get_logger().error("No pull detected; exiting without closing gripper.")
                return

            # 6. ì‹¤ì œ ê·¸ë¦¬í¼ í´ë¡œì¦ˆ
            grip()
            node.get_logger().info("Gripper closed. Waiting 4 seconds before lift...")
            wait(4.0)  # ì—¬ìœ  ëŒ€ê¸°

            # 7. í˜„ìž¬ ìœ„ì¹˜ ì½ì–´ì„œ Zì¶• 250mm ìœ„ë¡œ ì ‘ê·¼
            pos, _ = get_current_posx(ref=DR_BASE)
            node.get_logger().info(f"Current pos (xyzrpy): {pos}")
            movel(approach(pos), vel=VELOCITY, acc=ACC)
            node.get_logger().info("Moved up by 250mm after grip.")

            # 8. ë‘ë²ˆì§¸ ìœ„ì¹˜ ì´ë™
            movej(JReady3, vel=VELOCITY, acc=ACC)
            node.get_logger().info("Final joint move completed.")

            # ìž ì‹œê¸°ë‹¤ë¦¬ê¸° 10ì´ˆ
            wait(10.0)

            # 9. ì•ˆì „í•œ ìœ„ì¹˜ ì´ë™()
            release()
            wait(0.5)
            movej(Jmove4, vel=VELOCITY, acc=ACC)
            wait(1)
            movej(Jmove5, vel=VELOCITY, acc=ACC)
            break

    finally:
        release_compliance_ctrl()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
