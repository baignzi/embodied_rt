#!/usr/bin/env python3
"""
vla_inference_node.py — VLA异步推理节点

三种运行方式:
  ROS2模式(真实模型):  ros2 run embodied_rt vla_inference_node
  ROS2模式(mock):       ros2 run embodied_rt vla_inference_node --ros-args -p mock:=true
  独立测试:             python vla_inference_node.py --standalone [--mock]
"""
import json
import threading
import queue
import time
import sys
import os
import argparse

import numpy as np

# ========== ROS2导入（可选） ==========
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

STANDALONE = "--standalone" in sys.argv
MOCK_MODE = "--mock" in sys.argv


# ========== 真实模型加载 ==========
def load_model():
    """加载OpenVLA模型（需要GPU + ~14GB显存）"""
    import torch
    from transformers import AutoModelForVision2Seq

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"[INFO] Loading OpenVLA-7B on {device}...")

    model = AutoModelForVision2Seq.from_pretrained(
        'openvla/openvla-7b',
        torch_dtype=torch.float16,
        trust_remote_code=True
    ).to(device).eval()

    print(f"[INFO] Model loaded successfully.")
    return model, device


def create_mock_image():
    """生成模拟图像用于测试"""
    from PIL import Image as PILImage
    img = np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8)
    return PILImage.fromarray(img)


def run_inference(model, device, image, instruction):
    """运行真实VLA推理"""
    import torch
    with torch.no_grad():
        action = model.predict_action(
            image, instruction,
            unnorm_key='bridge_orig',
            do_sample=False
        )
    return action


def mock_inference(step=0):
    """模拟推理：生成平滑变化的动作（更接近真实场景）"""
    # 模拟一个"抓取-移动-放置"的周期动作
    t = step * 0.2  # 时间步
    action = np.zeros(7)
    action[0] = 0.3 + 0.1 * np.sin(t * 0.5)       # x: 0.2~0.4
    action[1] = 0.0 + 0.15 * np.sin(t * 0.3)      # y: -0.15~0.15
    action[2] = 0.2 + 0.1 * (np.sin(t * 0.4) + 1)  # z: 0.2~0.4
    action[3] = 0.0                                # rx
    action[4] = 0.0                                # ry
    action[5] = 0.0                                # rz
    action[6] = 1.0 if (int(step / 5) % 2 == 0) else 0.0  # gripper 周期开合
    latency = np.random.uniform(150, 250)
    time.sleep(latency / 1000)
    return action, latency


# ========== ROS2节点类 ==========
class VLAInferenceNode(Node):
    def __init__(self):
        super().__init__('vla_inference_node')

        # 读取参数：mock 模式（默认 false）
        self.declare_parameter('mock', False)
        self.mock_mode = self.get_parameter('mock').value

        # 指令参数
        self.declare_parameter('instruction',
            'pick up the cup and place it on the shelf')
        self._instruction = self.get_parameter('instruction').value

        # 推理频率（mock模式下的发布频率，Hz）
        self.declare_parameter('rate_hz', 2.0)
        self.rate_hz = self.get_parameter('rate_hz').value

        if self.mock_mode:
            self.get_logger().info(
                f'VLA Inference Node started in MOCK mode ({self.rate_hz} Hz)')
            self._step = 0
            self._mock_timer = self.create_timer(
                1.0 / self.rate_hz, self._mock_tick)
        else:
            # 真实模型模式
            try:
                self.model, self.device = load_model()
            except Exception as e:
                self.get_logger().error(
                    f'Failed to load VLA model: {e}')
                self.get_logger().warn('Falling back to mock mode. '
                    'Use -p mock:=true to skip this warning.')
                self.mock_mode = True
                self._step = 0
                self._mock_timer = self.create_timer(
                    1.0 / self.rate_hz, self._mock_tick)
                return

            # QoS: 观测数据RELIABLE + KEEP_LAST(1)
            obs_qos = QoSProfile(depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST)
            self.obs_sub = self.create_subscription(
                Image, '/perception/observation',
                self._obs_callback, obs_qos)

            # drop-old 异步队列
            self._inference_q = queue.Queue(maxsize=1)
            self._running = threading.Event()
            self._running.set()
            self._worker = threading.Thread(
                target=self._inference_loop, daemon=True)
            self._worker.start()

            self.get_logger().info(
                'VLA Inference Node started (real model)')

        # 动作发布
        act_qos = QoSProfile(depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST)
        self.action_pub = self.create_publisher(
            String, '/vla/action_cmd', act_qos)

    def _mock_tick(self):
        """mock模式：定时发布模拟动作"""
        action, latency = mock_inference(self._step)
        self._step += 1

        msg = String()
        msg.data = json.dumps({
            'action': action.tolist(),
            'timestamp': time.time(),
            'latency_ms': round(latency, 1),
            'mock': True
        })
        self.action_pub.publish(msg)
        self.get_logger().debug(
            f'Mock VLA: {latency:.0f}ms | pos={action[:3].round(3)} '
            f'gripper={action[6]:.0f}')

    def _obs_callback(self, msg):
        img = np.frombuffer(msg.data, dtype=np.uint8)
        img = img.reshape(msg.height, msg.width, -1)[:, :, :3]
        from PIL import Image as PILImage
        pil_img = PILImage.fromarray(img)
        try:
            self._inference_q.get_nowait()
        except queue.Empty:
            pass
        try:
            self._inference_q.put_nowait(pil_img)
        except queue.Full:
            pass

    def _inference_loop(self):
        while self._running.is_set():
            try:
                pil_img = self._inference_q.get(timeout=1.0)
            except queue.Empty:
                continue

            t0 = time.time()
            action = run_inference(self.model, self.device,
                                   pil_img, self._instruction)
            latency_ms = (time.time() - t0) * 1000

            msg = String()
            msg.data = json.dumps({
                'action': action.tolist(),
                'timestamp': time.time(),
                'latency_ms': round(latency_ms, 1)
            })
            self.action_pub.publish(msg)
            self.get_logger().info(
                f'VLA inference: {latency_ms:.0f}ms | '
                f'action={np.array(action)[:3].round(3)}')


# ========== 独立测试模式 ==========
def standalone_test():
    """不依赖ROS2的独立测试"""
    print("=" * 55)
    print("EmbodiedRT VLA Inference — Standalone Test Mode")
    print("=" * 55)

    try:
        model, device = load_model()
    except Exception as e:
        print(f"[ERROR] Failed to load model: {e}")
        print("\n[INFO] 没有GPU或模型未下载？使用 mock 模式:")
        print("  python vla_inference_node.py --standalone --mock")
        return

    instruction = "pick up the cup and place it on the shelf"
    image = create_mock_image()

    print(f"\n[INFO] Instruction: {instruction}")
    print(f"[INFO] Image: mock random image (224x224)")

    for i in range(3):
        t0 = time.time()
        action = run_inference(model, device, image, instruction)
        latency = (time.time() - t0) * 1000

        print(f"\n[Run {i+1}/3] Latency: {latency:.1f}ms")
        print(f"  Action (7-DOF): {np.array(action).round(4)}")
        print(f"  Position (x,y,z): {np.array(action)[:3].round(4)}")
        print(f"  Gripper: {action[6]:.4f}")

    print("\n[OK] VLA inference test passed.")


def mock_test():
    """mock模式独立测试"""
    print("=" * 55)
    print("EmbodiedRT VLA Inference — Mock Mode (no GPU needed)")
    print("=" * 55)

    instruction = "pick up the cup and place it on the shelf"
    print(f"\n[INFO] Instruction: {instruction}")
    print(f"[INFO] Mock mode: simulating VLA inference pipeline\n")

    for i in range(5):
        action, latency = mock_inference(i)
        print(f"[Step {i+1}/5] Latency: {latency:.1f}ms")
        print(f"  Position (x,y,z): {action[:3].round(4)}")
        print(f"  Gripper: {action[6]:.1f} (open/close)\n")

    print("[OK] Mock pipeline test passed.")
    print("     ROS2 mock mode: ros2 run embodied_rt vla_inference_node "
          "--ros-args -p mock:=true")


# ========== 主入口 ==========
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="EmbodiedRT VLA Inference Node")
    parser.add_argument("--standalone", action="store_true",
                        help="Run without ROS2 (direct test)")
    parser.add_argument("--mock", action="store_true",
                        help="Mock mode (no GPU/model needed)")
    args, _ = parser.parse_known_args()

    if args.mock and args.standalone:
        mock_test()
    elif args.standalone or not HAS_ROS2:
        if args.mock:
            mock_test()
        else:
            standalone_test()
    else:
        # ROS2 模式
        rclpy.init()
        node = VLAInferenceNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
