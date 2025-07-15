#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS2 ê´€ë ¨ ëª¨ë“ˆ ë¶ˆëŸ¬ì˜¤ê¸°
import rclpy                     # ROS2 Python í´ë¼ì´ì–¸íŠ¸ ë¼ì´ë¸ŒëŸ¬ë¦¬
import DR_init                  # Doosan ë¡œë´‡ìš© ROS2 ì´ˆê¸°í™” ëª¨ë“ˆ
import time                     # ì‹œê°„ ì§€ì—°ì„ ìœ„í•œ ëª¨ë“ˆ
import numpy as np              # ê´€ì ˆ ìœ„ì¹˜ ì €ìž¥ì„ ìœ„í•œ numpy
import sys, select              # í‚¤ë³´ë“œ ìž…ë ¥ ê°ì§€ë¥¼ ìœ„í•œ ëª¨ë“ˆ

# ë¡œë´‡ IDì™€ ëª¨ë¸ëª…ì„ ì „ì—­ ë³€ìˆ˜ë¡œ ì„¤ì •
ROBOT_ID    = "dsr01"
ROBOT_MODEL = "m0609"

# ë©”ì¸ í•¨ìˆ˜ ì •ì˜
def main(args=None):
    # ROS2 ì´ˆê¸°í™”
    rclpy.init(args=args)

    # teach_joint_pathë¼ëŠ” ì´ë¦„ì˜ ë…¸ë“œë¥¼ ìƒì„±í•˜ê³ , ë„¤ìž„ìŠ¤íŽ˜ì´ìŠ¤ë¥¼ ë¡œë´‡ IDë¡œ ì„¤ì •
    node = rclpy.create_node("teach_joint_path", namespace=ROBOT_ID)

    # DSR ë¡œë´‡ API ë‚´ë¶€ì—ì„œ ì‚¬ìš©í•  ë…¸ë“œì™€ ë¡œë´‡ ì •ë³´ ë“±ë¡
    DR_init.__dsr__node   = node
    DR_init.__dsr__id     = ROBOT_ID
    DR_init.__dsr__model  = ROBOT_MODEL

    # DSR APIì—ì„œ ì‚¬ìš©í•  ì£¼ìš” í•¨ìˆ˜ ë° ì˜ˆì™¸ í´ëž˜ìŠ¤ ìž„í¬íŠ¸
    from DSR_ROBOT2 import get_current_posj, release_compliance_ctrl, task_compliance_ctrl, DR_Error

    # í˜¹ì‹œ ì´ì „ì— ì¼œì ¸ ìžˆì„ ìˆ˜ ìžˆëŠ” ìˆœì‘ ì œì–´ í•´ì œ
    release_compliance_ctrl()

    # 6ì¶• ëª¨ë‘ stiffness(ê°•ì„±)ë¥¼ 0ìœ¼ë¡œ ì„¤ì •í•˜ì—¬ ì‚¬ìš©ìžê°€ ì†ìœ¼ë¡œ ë¡œë´‡ì„ ì‰½ê²Œ ì›€ì§ì¼ ìˆ˜ ìžˆë„ë¡ ì„¤ì •
    ultra_low_stx = [0, 0, 0, 0, 0, 0]
    task_compliance_ctrl(ultra_low_stx, 0)

    # ê´€ì ˆ ìœ„ì¹˜ë¥¼ ì €ìž¥í•  ë¦¬ìŠ¤íŠ¸ ì´ˆê¸°í™”
    joint_paths = []

    # ì €ìž¥ íšŸìˆ˜ ì¹´ìš´í„° ì´ˆê¸°í™”
    count = 0

    # ì‚¬ìš©ìžì—ê²Œ teach ëª¨ë“œ ì‹œìž‘ ì•ˆë‚´ ë©”ì‹œì§€ ì¶œë ¥
    node.get_logger().info("Teach ëª¨ë“œ ì‹œìž‘: ì—”í„° ëˆ„ë¥´ë©´ ì¢…ë£Œ")

    try:
        # ë¬´í•œ ë£¨í”„ ì‹œìž‘ (ì—”í„° ìž…ë ¥ ì „ê¹Œì§€ ë°˜ë³µ)
        while True:
            # ì‚¬ìš©ìžë¡œë¶€í„° í‚¤ë³´ë“œ ìž…ë ¥ì´ ë“¤ì–´ì™”ëŠ”ì§€ í™•ì¸ (ì—”í„° ìž…ë ¥ ì‹œ ì¢…ë£Œ)
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                _ = sys.stdin.readline()  # ìž…ë ¥ ë²„í¼ ë¹„ìš°ê¸°
                break  # ë£¨í”„ ì¢…ë£Œ

            try:
                # í˜„ìž¬ ë¡œë´‡ì˜ ê´€ì ˆ ìœ„ì¹˜ë¥¼ ê°€ì ¸ì˜¤ê¸°
                joint_pos = get_current_posj()
            except DR_Error:
                # ì—ëŸ¬ ë°œìƒ ì‹œ ê²½ê³  ì¶œë ¥ í›„ ìž ì‹œ ëŒ€ê¸° í›„ ìž¬ì‹œë„
                node.get_logger().warn("get_current_joint_position() ì˜¤ë¥˜, ìž¬ì‹œë„")
                time.sleep(0.01)
                continue

            # ë°›ì€ ê´€ì ˆ ë°ì´í„°ê°€ 6ê°œì˜ ê°’ì„ ê°€ì§„ ë¦¬ìŠ¤íŠ¸ ë˜ëŠ” ë°°ì—´ì¸ì§€ í™•ì¸
            if hasattr(joint_pos, "__len__") and len(joint_pos) == 6:
                count += 1  # ì €ìž¥ íšŸìˆ˜ ì¦ê°€
                node.get_logger().info(f"[{count}] ê´€ì ˆ ìœ„ì¹˜ ì €ìž¥: {joint_pos}")  # ë¡œê·¸ ì¶œë ¥
                joint_paths.append(list(joint_pos))  # ê´€ì ˆ ìœ„ì¹˜ ì €ìž¥
                time.sleep(2.0)  # 2ì´ˆ ê°„ê²©ìœ¼ë¡œ ì €ìž¥
            else:
                # ìž˜ëª»ëœ í˜•ì‹ì˜ ë°ì´í„°ì¸ ê²½ìš° ê²½ê³  ì¶œë ¥ í›„ ìž¬ì‹œë„
                node.get_logger().warn("ìž˜ëª»ëœ ê´€ì ˆ ìœ„ì¹˜ ë°ì´í„°")
                time.sleep(0.01)

    # í‚¤ë³´ë“œ ì¸í„°ëŸ½íŠ¸(Ctrl+C) ì‹œ ì¢…ë£Œ ì²˜ë¦¬
    except KeyboardInterrupt:
        pass

    # ì €ìž¥í•œ ê´€ì ˆ ìœ„ì¹˜ ë°ì´í„°ë¥¼ numpy ë°°ì—´ë¡œ ì €ìž¥
    np.save("joint_path.npy", np.array(joint_paths))

    # ì €ìž¥ ì™„ë£Œ ë©”ì‹œì§€ ì¶œë ¥
    node.get_logger().info(f"joint_path.npy ì €ìž¥ ì™„ë£Œ, ì´ {len(joint_paths)} ì ")

    # ìˆœì‘ ì œì–´ í•´ì œ
    release_compliance_ctrl()

    # ë…¸ë“œ ì¢…ë£Œ
    node.destroy_node()

    # ROS2 ì‹œìŠ¤í…œ ì¢…ë£Œ
    rclpy.shutdown()

# main() í•¨ìˆ˜ ì‹¤í–‰ ì¡°ê±´ í™•ì¸
if __name__ == "__main__":
    main()
