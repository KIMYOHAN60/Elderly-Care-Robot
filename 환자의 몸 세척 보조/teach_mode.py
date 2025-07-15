#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS2 관련 모듈 불러오기
import rclpy                     # ROS2 Python 클라이언트 라이브러리
import DR_init                  # Doosan 로봇용 ROS2 초기화 모듈
import time                     # 시간 지연을 위한 모듈
import numpy as np              # 관절 위치 저장을 위한 numpy
import sys, select              # 키보드 입력 감지를 위한 모듈

# 로봇 ID와 모델명을 전역 변수로 설정
ROBOT_ID    = "dsr01"
ROBOT_MODEL = "m0609"

# 메인 함수 정의
def main(args=None):
    # ROS2 초기화
    rclpy.init(args=args)

    # teach_joint_path라는 이름의 노드를 생성하고, 네임스페이스를 로봇 ID로 설정
    node = rclpy.create_node("teach_joint_path", namespace=ROBOT_ID)

    # DSR 로봇 API 내부에서 사용할 노드와 로봇 정보 등록
    DR_init.__dsr__node   = node
    DR_init.__dsr__id     = ROBOT_ID
    DR_init.__dsr__model  = ROBOT_MODEL

    # DSR API에서 사용할 주요 함수 및 예외 클래스 임포트
    from DSR_ROBOT2 import get_current_posj, release_compliance_ctrl, task_compliance_ctrl, DR_Error

    # 혹시 이전에 켜져 있을 수 있는 순응 제어 해제
    release_compliance_ctrl()

    # 6축 모두 stiffness(강성)를 0으로 설정하여 사용자가 손으로 로봇을 쉽게 움직일 수 있도록 설정
    ultra_low_stx = [0, 0, 0, 0, 0, 0]
    task_compliance_ctrl(ultra_low_stx, 0)

    # 관절 위치를 저장할 리스트 초기화
    joint_paths = []

    # 저장 횟수 카운터 초기화
    count = 0

    # 사용자에게 teach 모드 시작 안내 메시지 출력
    node.get_logger().info("Teach 모드 시작: 엔터 누르면 종료")

    try:
        # 무한 루프 시작 (엔터 입력 전까지 반복)
        while True:
            # 사용자로부터 키보드 입력이 들어왔는지 확인 (엔터 입력 시 종료)
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                _ = sys.stdin.readline()  # 입력 버퍼 비우기
                break  # 루프 종료

            try:
                # 현재 로봇의 관절 위치를 가져오기
                joint_pos = get_current_posj()
            except DR_Error:
                # 에러 발생 시 경고 출력 후 잠시 대기 후 재시도
                node.get_logger().warn("get_current_joint_position() 오류, 재시도")
                time.sleep(0.01)
                continue

            # 받은 관절 데이터가 6개의 값을 가진 리스트 또는 배열인지 확인
            if hasattr(joint_pos, "__len__") and len(joint_pos) == 6:
                count += 1  # 저장 횟수 증가
                node.get_logger().info(f"[{count}] 관절 위치 저장: {joint_pos}")  # 로그 출력
                joint_paths.append(list(joint_pos))  # 관절 위치 저장
                time.sleep(2.0)  # 2초 간격으로 저장
            else:
                # 잘못된 형식의 데이터인 경우 경고 출력 후 재시도
                node.get_logger().warn("잘못된 관절 위치 데이터")
                time.sleep(0.01)

    # 키보드 인터럽트(Ctrl+C) 시 종료 처리
    except KeyboardInterrupt:
        pass

    # 저장한 관절 위치 데이터를 numpy 배열로 저장
    np.save("joint_path.npy", np.array(joint_paths))

    # 저장 완료 메시지 출력
    node.get_logger().info(f"joint_path.npy 저장 완료, 총 {len(joint_paths)} 점")

    # 순응 제어 해제
    release_compliance_ctrl()

    # 노드 종료
    node.destroy_node()

    # ROS2 시스템 종료
    rclpy.shutdown()

# main() 함수 실행 조건 확인
if __name__ == "__main__":
    main()
