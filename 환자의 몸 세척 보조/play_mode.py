#!/usr/bin/env python3

# ROS2 및 필요한 라이브러리 임포트
import rclpy                          # ROS2 Python 클라이언트 라이브러리
import DR_init                        # DSR 로봇용 ROS2 초기화 모듈
import numpy as np                    # joint path 불러오기용
import time                           # 대기 및 지연을 위한 모듈

# 로봇 기본 설정
ROBOT_ID    = "dsr01"                 # 사용 중인 로봇 ID (namespace로 사용됨)
ROBOT_MODEL = "m0609"                 # 사용 중인 로봇 모델명

FORCE_THRESHOLD = 15.0                # [N] 외력 감지 기준 임계값
STIFFNESS = [100, 100, 100, 20, 20, 20]  # 순응 제어 시 강성 설정 (XYZ + RPY)

def main(args=None):
    # ROS2 시스템 초기화
    rclpy.init(args=args)

    # ROS2 노드 생성 (네임스페이스는 로봇 ID와 동일하게)
    node = rclpy.create_node("joint_and_compliance_with_force_interrupt", namespace=ROBOT_ID)

    # DSR API가 사용할 노드 및 로봇 정보 설정
    DR_init.__dsr__node   = node
    DR_init.__dsr__id     = ROBOT_ID
    DR_init.__dsr__model  = ROBOT_MODEL

    # 필요한 DSR API 함수 및 상수 가져오기
    from DSR_ROBOT2 import (
        amovej,                  # 관절 기반 이동 명령
        DR_Error,                # 에러 처리용 클래스
        wait,                    # 명령 동기화 대기 함수
        get_tool_force,          # 말단 힘 센서 데이터 읽기
        task_compliance_ctrl,    # 순응 제어 활성화
        get_current_pose,        # 현재 위치(Pose) 반환
        release_compliance_ctrl, # 순응 제어 해제
        ROBOT_SPACE_TASK         # task 좌표계 상수
    )

    try:
        # =============================
        # [1단계] joint path 불러오기
        # =============================
        try:
            joint_path = np.load("joint_path.npy")
        except FileNotFoundError:
            node.get_logger().error("❌ joint_path.npy 파일을 찾을 수 없습니다.")
            rclpy.shutdown()
            return

        node.get_logger().info(f"📍 총 {len(joint_path)}개의 관절 위치를 따라 이동을 시작합니다.")

        # ====================================
        # [2단계] joint path를 따라 순차 이동
        # ====================================
        for i, joints in enumerate(joint_path):
            # =========================
            # [2-1] 말단 힘 센서 읽기
            # =========================
            fx, fy, fz, *_ = get_tool_force()  # 말단 힘 정보
            total_force = (fx**2 + fy**2 + fz**2)**0.5  # 힘의 크기 계산 (유클리디안 노름)

            # ======================================
            # [2-2] 외력이 일정 값 이상이면 중단
            # ======================================
            if total_force > FORCE_THRESHOLD:
                node.get_logger().warn(f"⚠️ 외력 감지됨: {total_force:.2f} N → 5초 대기 후 순응 제어 진입")
                time.sleep(5.0)  # 사용자가 로봇을 조작할 시간 확보

                # 현재 위치(task space) 받아오기
                current_pos = get_current_pose(ROBOT_SPACE_TASK)

                # 순응 제어 활성화 (설정된 강성 적용)
                task_compliance_ctrl(STIFFNESS)
                node.get_logger().info(f"✅ 순응 제어 활성화됨 → 강성 설정: {STIFFNESS}")

                # 순응 상태에서 x축 방향으로 +100mm 이동할 목표 위치 설정
                target_pos = [
                    current_pos[0] + 100.0,  # x축 +100mm
                    current_pos[1],
                    current_pos[2],
                    current_pos[3],
                    current_pos[4],
                    current_pos[5]
                ]

                node.get_logger().info("🤖 순응 제어 상태로 x축 +100mm 이동 실행 (amovej)")
                amovej(target_pos, vel=100, acc=100)  # 빠르게 이동

                node.get_logger().info("⏳ 이동 후 대기 중 (10초)...")
                wait(10.0)  # 충분히 위치 유지

                # 순응 제어 해제
                release_compliance_ctrl()
                wait(3.0)
                node.get_logger().info("🛑 순응 제어 해제 및 로봇 정지 완료")

                break  # 이동 중단 및 종료

            # ===============================
            # [2-3] 정상 상태면 관절 이동 수행
            # ===============================
            node.get_logger().info(f"[{i+1}/{len(joint_path)}] 🚗 movej로 관절 이동 중...")

            try:
                amovej(joints.tolist(), v=10, a=10)  # 관절 속도/가속도 설정하여 이동
            except DR_Error as e:
                node.get_logger().warn(f"⚠️ movej 오류 발생: {e}")

    # =======================
    # [3단계] 예외 처리 블록
    # =======================
    except DR_Error as e:
        node.get_logger().error(f"❗ DSR API 오류 발생: {e}")

    # =======================
    # [4단계] 노드 종료 처리
    # =======================
    finally:
        node.get_logger().info("🔚 노드 종료 및 순응 제어 비활성화")
        node.destroy_node()
        rclpy.shutdown()

# main() 함수 실행
if __name__ == '__main__':
    main()
