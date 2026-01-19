#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
7轴机械臂控制器 - 图形界面版
电机ID: 51-57
"""
import can
import struct
import time
import threading
import json
import os
import math
import tkinter as tk
from tkinter import ttk, messagebox
import signal
import sys
import atexit


# 机械臂电机ID配置
MOTOR_IDS = [51, 52, 53, 54, 55, 56, 57]

# 配置文件路径
CONFIG_DIR = os.path.dirname(os.path.abspath(__file__))
ZERO_OFFSET_FILE = os.path.join(CONFIG_DIR, "zero_offset.json")
TRAJECTORY_FILE = os.path.join(CONFIG_DIR, "trajectory.json")

# 命令码
CMD_HANDSHAKE = 0x00
CMD_READ_POSITION = 0x06
CMD_SET_MODE = 0x07
CMD_SET_POSITION = 0x0A
CMD_ENABLE = 0x2A
CMD_CLEAR_ALARM = 0xFE
CMD_READ_PT_V = 0x1C
CMD_SET_PT_V = 0x1F
CMD_SET_PT_A = 0x20
CMD_SET_PT_D = 0x21

# 控制模式
MODE_PROFILE_POSITION = 6


class Motor:
    """单个电机"""
    def __init__(self, motor_id, bus):
        self.motor_id = motor_id
        self.bus = bus
        self.position = 0.0
        self.enabled = False
    
    def send_cmd(self, cmd, data=None):
        if data is None:
            data = []
        msg_data = [cmd] + list(data) + [0] * (7 - len(data))
        msg = can.Message(
            arbitration_id=self.motor_id,
            data=msg_data[:8],
            is_extended_id=False
        )
        try:
            self.bus.send(msg)
        except:
            pass
    
    def float_to_bytes(self, value):
        return list(struct.pack('<f', value))
    
    def bytes_to_float(self, data):
        if len(data) >= 4:
            return struct.unpack('<f', bytes(data[:4]))[0]
        return 0.0


class ArmController:
    """机械臂控制器"""
    
    def __init__(self, motor_ids=None, can_channel='can0'):
        self.motor_ids = motor_ids or MOTOR_IDS
        self.can_channel = can_channel
        self.bus = None
        self.motors = {}
        self.running = False
        self.recv_thread = None
        self.connected = False
        
        # 零点偏移
        self.zero_offsets = {mid: 0.0 for mid in self.motor_ids}
        self._load_zero_offsets()
        
        # 示教轨迹
        self.trajectory = []
        self._load_trajectory()
    
    def connect(self):
        """连接CAN总线"""
        try:
            self.bus = can.interface.Bus(
                channel=self.can_channel,
                interface='socketcan',
                bitrate=1000000
            )
            for motor_id in self.motor_ids:
                self.motors[motor_id] = Motor(motor_id, self.bus)
            
            self.running = True
            self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
            self.recv_thread.start()
            self.connected = True
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False
    
    def _recv_loop(self):
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg and msg.arbitration_id in self.motors:
                    motor = self.motors[msg.arbitration_id]
                    data = list(msg.data)
                    if data[0] == CMD_READ_POSITION and len(data) >= 5:
                        motor.position = motor.bytes_to_float(data[1:5])
                    elif data[0] == CMD_ENABLE:
                        motor.enabled = data[1] == 1 if len(data) > 1 else False
            except:
                pass
    
    def _load_zero_offsets(self):
        if os.path.exists(ZERO_OFFSET_FILE):
            try:
                with open(ZERO_OFFSET_FILE, 'r') as f:
                    data = json.load(f)
                    for mid in self.motor_ids:
                        if str(mid) in data:
                            self.zero_offsets[mid] = data[str(mid)]
            except:
                pass
    
    def _save_zero_offsets(self):
        try:
            with open(ZERO_OFFSET_FILE, 'w') as f:
                json.dump({str(k): v for k, v in self.zero_offsets.items()}, f, indent=2)
        except:
            pass
    
    def _load_trajectory(self):
        if os.path.exists(TRAJECTORY_FILE):
            try:
                with open(TRAJECTORY_FILE, 'r') as f:
                    self.trajectory = json.load(f)
            except:
                pass
    
    def _save_trajectory(self):
        try:
            with open(TRAJECTORY_FILE, 'w') as f:
                json.dump(self.trajectory, f, indent=2)
        except:
            pass
    
    def clear_all_alarms(self):
        for motor_id in self.motor_ids:
            self.motors[motor_id].send_cmd(CMD_CLEAR_ALARM)
            time.sleep(0.02)
    
    def set_all_modes(self, mode):
        for motor_id in self.motor_ids:
            self.motors[motor_id].send_cmd(CMD_SET_MODE, [mode])
            time.sleep(0.02)
    
    def enable_all(self, enable=True):
        for motor_id in self.motor_ids:
            self.motors[motor_id].send_cmd(CMD_ENABLE, [1 if enable else 0])
            time.sleep(0.02)
    
    def read_all_positions(self):
        for motor_id in self.motor_ids:
            self.motors[motor_id].send_cmd(CMD_READ_POSITION)
            time.sleep(0.02)
    
    def set_position(self, motor_id, position):
        motor = self.motors[motor_id]
        data = motor.float_to_bytes(position)
        motor.send_cmd(CMD_SET_POSITION, data)
    
    def set_all_positions(self, positions):
        for motor_id, pos in zip(self.motor_ids, positions):
            self.set_position(motor_id, pos)
            time.sleep(0.02)
    
    def set_speed(self, velocity, accel, decel):
        for motor_id in self.motor_ids:
            motor = self.motors[motor_id]
            motor.send_cmd(CMD_SET_PT_V, motor.float_to_bytes(velocity))
            time.sleep(0.02)
            motor.send_cmd(CMD_SET_PT_A, motor.float_to_bytes(accel))
            time.sleep(0.02)
            motor.send_cmd(CMD_SET_PT_D, motor.float_to_bytes(decel))
            time.sleep(0.02)
    
    def init_arm(self):
        self.clear_all_alarms()
        time.sleep(0.2)
        self.read_all_positions()
        time.sleep(0.3)
        self.set_all_modes(MODE_PROFILE_POSITION)
        time.sleep(0.2)
        self.enable_all(True)
        time.sleep(0.2)
    
    def enable_freedrive(self):
        self.clear_all_alarms()
        time.sleep(0.1)
        self.enable_all(False)
        time.sleep(0.1)
    
    def disable_freedrive(self):
        self.read_all_positions()
        time.sleep(0.3)
        self.set_all_modes(MODE_PROFILE_POSITION)
        time.sleep(0.2)
        self.enable_all(True)
        time.sleep(0.2)
    
    def calibrate_zero(self):
        self.read_all_positions()
        time.sleep(0.3)
        for mid in self.motor_ids:
            self.zero_offsets[mid] = self.motors[mid].position
        self._save_zero_offsets()
    
    def go_to_zero(self, callback=None):
        """回到零点位置"""
        for motor_id in self.motor_ids:
            self.set_position(motor_id, self.zero_offsets[motor_id])
            time.sleep(0.02)
    
    def record_point(self, name):
        self.read_all_positions()
        time.sleep(0.3)
        point = {
            "name": name,
            "positions": {str(mid): self.motors[mid].position for mid in self.motor_ids}
        }
        self.trajectory.append(point)
        self._save_trajectory()
        return point
    
    def clear_trajectory(self):
        self.trajectory = []
        self._save_trajectory()
    
    def safe_shutdown(self, go_zero=True):
        """安全关机流程"""
        if not self.connected:
            return
        
        print("执行安全关机...")
        
        if go_zero:
            # 先回到零点
            print("  回到零点...")
            self.go_to_zero()
            time.sleep(3)  # 等待运动完成
        
        # 关闭电机
        print("  关闭电机...")
        self.enable_all(False)
        time.sleep(0.2)
        
        # 关闭CAN连接
        self.running = False
        if self.recv_thread:
            self.recv_thread.join(timeout=1)
        if self.bus:
            self.bus.shutdown()
        
        self.connected = False
        print("安全关机完成")
    
    def close(self):
        self.running = False
        if self.recv_thread:
            self.recv_thread.join(timeout=1)
        if self.bus:
            self.bus.shutdown()
        self.connected = False


class ArmControlGUI:
    """机械臂控制图形界面"""
    
    def __init__(self):
        self.arm = ArmController()
        self.freedrive_mode = False
        self.is_closing = False
        
        # 创建主窗口
        self.root = tk.Tk()
        self.root.title("7轴机械臂控制器")
        self.root.geometry("800x700")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # 注册信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # 状态变量
        self.position_vars = {}
        self.offset_vars = {}
        
        self.create_widgets()
        self.update_positions()
    
    def signal_handler(self, signum, frame):
        """处理系统信号"""
        self.on_closing()
    
    def create_widgets(self):
        """创建界面组件"""
        
        # 顶部状态栏
        status_frame = ttk.Frame(self.root)
        status_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.status_label = ttk.Label(status_frame, text="状态: 未连接", font=('Arial', 12, 'bold'))
        self.status_label.pack(side=tk.LEFT)
        
        self.connect_btn = ttk.Button(status_frame, text="连接", command=self.connect)
        self.connect_btn.pack(side=tk.RIGHT, padx=5)
        
        # 创建Notebook（标签页）
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # === 标签页1: 主控制 ===
        main_frame = ttk.Frame(notebook)
        notebook.add(main_frame, text="主控制")
        
        # 位置显示
        pos_frame = ttk.LabelFrame(main_frame, text="电机位置")
        pos_frame.pack(fill=tk.X, padx=10, pady=5)
        
        headers = ["电机ID", "绝对位置(rad)", "相对零点(rad)", "角度(°)"]
        for i, h in enumerate(headers):
            ttk.Label(pos_frame, text=h, font=('Arial', 10, 'bold')).grid(row=0, column=i, padx=10, pady=5)
        
        for row, mid in enumerate(MOTOR_IDS, 1):
            ttk.Label(pos_frame, text=str(mid)).grid(row=row, column=0, padx=10, pady=2)
            
            self.position_vars[mid] = tk.StringVar(value="0.0000")
            ttk.Label(pos_frame, textvariable=self.position_vars[mid]).grid(row=row, column=1, padx=10, pady=2)
            
            self.offset_vars[mid] = tk.StringVar(value="0.0000")
            ttk.Label(pos_frame, textvariable=self.offset_vars[mid]).grid(row=row, column=2, padx=10, pady=2)
            
            deg_var = tk.StringVar(value="0.00")
            ttk.Label(pos_frame, textvariable=deg_var).grid(row=row, column=3, padx=10, pady=2)
            self.offset_vars[f"{mid}_deg"] = deg_var
        
        # 控制按钮
        ctrl_frame = ttk.LabelFrame(main_frame, text="控制")
        ctrl_frame.pack(fill=tk.X, padx=10, pady=5)
        
        btn_frame1 = ttk.Frame(ctrl_frame)
        btn_frame1.pack(fill=tk.X, pady=5)
        
        self.init_btn = ttk.Button(btn_frame1, text="初始化机械臂", command=self.init_arm, width=15)
        self.init_btn.pack(side=tk.LEFT, padx=5)
        
        self.disable_btn = ttk.Button(btn_frame1, text="关闭电机", command=self.disable_arm, width=15)
        self.disable_btn.pack(side=tk.LEFT, padx=5)
        
        self.zero_btn = ttk.Button(btn_frame1, text="回到零点", command=self.go_to_zero, width=15)
        self.zero_btn.pack(side=tk.LEFT, padx=5)
        
        # 自由拖动
        drag_frame = ttk.LabelFrame(main_frame, text="自由拖动模式 (零点标定)")
        drag_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.freedrive_btn = ttk.Button(drag_frame, text="进入自由拖动", command=self.toggle_freedrive, width=15)
        self.freedrive_btn.pack(side=tk.LEFT, padx=5, pady=5)
        
        self.calibrate_btn = ttk.Button(drag_frame, text="标定当前为零点", command=self.calibrate_zero, width=15)
        self.calibrate_btn.pack(side=tk.LEFT, padx=5, pady=5)
        
        self.freedrive_label = ttk.Label(drag_frame, text="", foreground="red")
        self.freedrive_label.pack(side=tk.LEFT, padx=10)
        
        # 单电机控制
        motor_frame = ttk.LabelFrame(main_frame, text="单电机控制")
        motor_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(motor_frame, text="电机ID:").pack(side=tk.LEFT, padx=5)
        self.motor_combo = ttk.Combobox(motor_frame, values=MOTOR_IDS, width=5)
        self.motor_combo.set(MOTOR_IDS[0])
        self.motor_combo.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(motor_frame, text="目标位置(rad):").pack(side=tk.LEFT, padx=5)
        self.target_entry = ttk.Entry(motor_frame, width=10)
        self.target_entry.insert(0, "0.0")
        self.target_entry.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(motor_frame, text="移动", command=self.move_motor).pack(side=tk.LEFT, padx=5)
        
        # === 标签页2: 速度设置 ===
        speed_frame = ttk.Frame(notebook)
        notebook.add(speed_frame, text="速度设置")
        
        preset_frame = ttk.LabelFrame(speed_frame, text="速度预设")
        preset_frame.pack(fill=tk.X, padx=10, pady=10)
        
        presets = [
            ("极慢 (0.2 rad/s)", 0.2),
            ("慢速 (0.5 rad/s)", 0.5),
            ("中速 (1.0 rad/s)", 1.0),
            ("快速 (2.0 rad/s)", 2.0),
        ]
        
        for text, speed in presets:
            btn = ttk.Button(preset_frame, text=text, 
                           command=lambda s=speed: self.set_speed_preset(s), width=20)
            btn.pack(side=tk.LEFT, padx=10, pady=10)
        
        custom_frame = ttk.LabelFrame(speed_frame, text="自定义速度")
        custom_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(custom_frame, text="最大速度 (rad/s):").grid(row=0, column=0, padx=10, pady=5)
        self.velocity_entry = ttk.Entry(custom_frame, width=10)
        self.velocity_entry.insert(0, "1.0")
        self.velocity_entry.grid(row=0, column=1, padx=10, pady=5)
        
        ttk.Label(custom_frame, text="加速度 (rad/s²):").grid(row=1, column=0, padx=10, pady=5)
        self.accel_entry = ttk.Entry(custom_frame, width=10)
        self.accel_entry.insert(0, "1.0")
        self.accel_entry.grid(row=1, column=1, padx=10, pady=5)
        
        ttk.Label(custom_frame, text="减速度 (rad/s²):").grid(row=2, column=0, padx=10, pady=5)
        self.decel_entry = ttk.Entry(custom_frame, width=10)
        self.decel_entry.insert(0, "1.0")
        self.decel_entry.grid(row=2, column=1, padx=10, pady=5)
        
        ttk.Button(custom_frame, text="应用", command=self.apply_custom_speed).grid(row=3, column=0, columnspan=2, pady=10)
        
        # === 标签页3: 示教轨迹 ===
        traj_frame = ttk.Frame(notebook)
        notebook.add(traj_frame, text="示教轨迹")
        
        record_frame = ttk.LabelFrame(traj_frame, text="记录点位")
        record_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(record_frame, text="点位名称:").pack(side=tk.LEFT, padx=5)
        self.point_name_entry = ttk.Entry(record_frame, width=20)
        self.point_name_entry.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(record_frame, text="记录当前位置", command=self.record_point).pack(side=tk.LEFT, padx=10)
        
        # 轨迹列表
        list_frame = ttk.LabelFrame(traj_frame, text="已记录的轨迹")
        list_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.traj_listbox = tk.Listbox(list_frame, height=10)
        self.traj_listbox.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        traj_btn_frame = ttk.Frame(list_frame)
        traj_btn_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(traj_btn_frame, text="播放轨迹", command=self.play_trajectory).pack(side=tk.LEFT, padx=5)
        ttk.Button(traj_btn_frame, text="移动到选中点", command=self.go_to_point).pack(side=tk.LEFT, padx=5)
        ttk.Button(traj_btn_frame, text="删除选中点", command=self.delete_point).pack(side=tk.LEFT, padx=5)
        ttk.Button(traj_btn_frame, text="清空所有", command=self.clear_trajectory).pack(side=tk.LEFT, padx=5)
        
        # 播放设置
        play_frame = ttk.Frame(list_frame)
        play_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(play_frame, text="点间隔时间(秒):").pack(side=tk.LEFT, padx=5)
        self.interval_entry = ttk.Entry(play_frame, width=5)
        self.interval_entry.insert(0, "2.0")
        self.interval_entry.pack(side=tk.LEFT, padx=5)
        
        # 更新轨迹列表
        self.update_trajectory_list()
        
        # === 标签页4: 安全设置 ===
        safety_frame = ttk.Frame(notebook)
        notebook.add(safety_frame, text="安全设置")
        
        exit_frame = ttk.LabelFrame(safety_frame, text="退出保护")
        exit_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.safe_exit_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(exit_frame, text="退出时先回到零点再关闭电机", 
                       variable=self.safe_exit_var).pack(anchor=tk.W, padx=10, pady=10)
        
        ttk.Label(exit_frame, text="注意: 开启此选项后，关闭程序时机械臂会先移动到零点位置，\n"
                                   "然后再关闭电机。请确保零点位置是安全的！",
                 foreground="red").pack(padx=10, pady=10)
        
        # 底部状态
        bottom_frame = ttk.Frame(self.root)
        bottom_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.info_label = ttk.Label(bottom_frame, text="提示: 请先点击'连接'按钮")
        self.info_label.pack(side=tk.LEFT)
    
    def connect(self):
        """连接机械臂"""
        if self.arm.connected:
            self.arm.close()
            self.status_label.config(text="状态: 未连接")
            self.connect_btn.config(text="连接")
            self.info_label.config(text="已断开连接")
        else:
            if self.arm.connect():
                self.status_label.config(text="状态: 已连接")
                self.connect_btn.config(text="断开")
                self.info_label.config(text="连接成功，请点击'初始化机械臂'")
            else:
                messagebox.showerror("错误", "连接CAN总线失败")
    
    def init_arm(self):
        """初始化机械臂"""
        if not self.arm.connected:
            messagebox.showwarning("警告", "请先连接机械臂")
            return
        
        self.info_label.config(text="正在初始化...")
        self.root.update()
        
        self.arm.init_arm()
        
        self.info_label.config(text="初始化完成")
    
    def disable_arm(self):
        """关闭电机"""
        if self.arm.connected:
            self.arm.enable_all(False)
            self.freedrive_mode = False
            self.freedrive_label.config(text="")
            self.freedrive_btn.config(text="进入自由拖动")
            self.info_label.config(text="电机已关闭")
    
    def go_to_zero(self):
        """回到零点"""
        if not self.arm.connected:
            return
        
        if messagebox.askyesno("确认", "确定要回到零点位置吗?"):
            self.info_label.config(text="正在移动到零点...")
            self.root.update()
            self.arm.go_to_zero()
            self.info_label.config(text="已移动到零点")
    
    def toggle_freedrive(self):
        """切换自由拖动模式"""
        if not self.arm.connected:
            messagebox.showwarning("警告", "请先连接机械臂")
            return
        
        if self.freedrive_mode:
            # 退出自由拖动
            self.arm.disable_freedrive()
            self.freedrive_mode = False
            self.freedrive_btn.config(text="进入自由拖动")
            self.freedrive_label.config(text="")
            self.info_label.config(text="已退出自由拖动，电机已锁定")
        else:
            # 进入自由拖动
            self.arm.enable_freedrive()
            self.freedrive_mode = True
            self.freedrive_btn.config(text="退出并锁定")
            self.freedrive_label.config(text="自由拖动中 - 可手动移动机械臂")
            self.info_label.config(text="已进入自由拖动模式，可手动移动机械臂")
    
    def calibrate_zero(self):
        """标定零点"""
        if not self.arm.connected:
            return
        
        if messagebox.askyesno("确认", "确定将当前位置标定为零点吗?"):
            self.arm.calibrate_zero()
            self.info_label.config(text="零点标定完成")
            messagebox.showinfo("成功", "零点标定完成")
    
    def move_motor(self):
        """移动单个电机"""
        if not self.arm.connected:
            return
        
        try:
            motor_id = int(self.motor_combo.get())
            target = float(self.target_entry.get())
            self.arm.set_position(motor_id, target)
            self.info_label.config(text=f"电机 {motor_id} 移动到 {target:.4f} rad")
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数值")
    
    def set_speed_preset(self, speed):
        """设置速度预设"""
        if not self.arm.connected:
            messagebox.showwarning("警告", "请先连接机械臂")
            return
        
        self.arm.set_speed(speed, speed, speed)
        self.info_label.config(text=f"速度已设置为 {speed} rad/s")
    
    def apply_custom_speed(self):
        """应用自定义速度"""
        if not self.arm.connected:
            return
        
        try:
            v = float(self.velocity_entry.get())
            a = float(self.accel_entry.get())
            d = float(self.decel_entry.get())
            self.arm.set_speed(v, a, d)
            self.info_label.config(text=f"速度参数已应用")
        except ValueError:
            messagebox.showerror("错误", "请输入有效的数值")
    
    def record_point(self):
        """记录点位"""
        if not self.arm.connected:
            return
        
        name = self.point_name_entry.get().strip()
        if not name:
            name = f"point_{len(self.arm.trajectory)}"
        
        self.arm.record_point(name)
        self.update_trajectory_list()
        self.point_name_entry.delete(0, tk.END)
        self.info_label.config(text=f"已记录点位: {name}")
    
    def update_trajectory_list(self):
        """更新轨迹列表"""
        self.traj_listbox.delete(0, tk.END)
        for i, point in enumerate(self.arm.trajectory):
            self.traj_listbox.insert(tk.END, f"{i+1}. {point['name']}")
    
    def go_to_point(self):
        """移动到选中点"""
        if not self.arm.connected:
            return
        
        selection = self.traj_listbox.curselection()
        if not selection:
            messagebox.showwarning("警告", "请选择一个点位")
            return
        
        idx = selection[0]
        point = self.arm.trajectory[idx]
        
        for mid in MOTOR_IDS:
            pos = point['positions'].get(str(mid), 0)
            self.arm.set_position(mid, pos)
            time.sleep(0.02)
        
        self.info_label.config(text=f"移动到: {point['name']}")
    
    def delete_point(self):
        """删除选中点"""
        selection = self.traj_listbox.curselection()
        if not selection:
            return
        
        idx = selection[0]
        del self.arm.trajectory[idx]
        self.arm._save_trajectory()
        self.update_trajectory_list()
    
    def clear_trajectory(self):
        """清空轨迹"""
        if messagebox.askyesno("确认", "确定清空所有轨迹点吗?"):
            self.arm.clear_trajectory()
            self.update_trajectory_list()
            self.info_label.config(text="轨迹已清空")
    
    def play_trajectory(self):
        """播放轨迹"""
        if not self.arm.connected or not self.arm.trajectory:
            return
        
        try:
            interval = float(self.interval_entry.get())
        except:
            interval = 2.0
        
        def play():
            for i, point in enumerate(self.arm.trajectory):
                self.info_label.config(text=f"播放: {i+1}/{len(self.arm.trajectory)} - {point['name']}")
                self.root.update()
                
                for mid in MOTOR_IDS:
                    pos = point['positions'].get(str(mid), 0)
                    self.arm.set_position(mid, pos)
                    time.sleep(0.02)
                
                time.sleep(interval)
            
            self.info_label.config(text="轨迹播放完成")
        
        threading.Thread(target=play, daemon=True).start()
    
    def update_positions(self):
        """更新位置显示"""
        if self.arm.connected:
            self.arm.read_all_positions()
            
            for mid in MOTOR_IDS:
                pos = self.arm.motors[mid].position
                offset = pos - self.arm.zero_offsets[mid]
                deg = offset * 180 / math.pi
                
                self.position_vars[mid].set(f"{pos:.4f}")
                self.offset_vars[mid].set(f"{offset:.4f}")
                self.offset_vars[f"{mid}_deg"].set(f"{deg:.2f}")
        
        if not self.is_closing:
            self.root.after(200, self.update_positions)
    
    def on_closing(self):
        """关闭窗口时的处理"""
        if self.is_closing:
            return
        
        self.is_closing = True
        
        if self.arm.connected:
            go_zero = self.safe_exit_var.get()
            
            if go_zero:
                self.info_label.config(text="正在安全关机，请等待...")
                self.root.update()
                
                # 在新线程中执行安全关机
                def safe_close():
                    self.arm.safe_shutdown(go_zero=True)
                    self.root.after(100, self.root.destroy)
                
                threading.Thread(target=safe_close, daemon=True).start()
            else:
                self.arm.enable_all(False)
                time.sleep(0.2)
                self.arm.close()
                self.root.destroy()
        else:
            self.root.destroy()
    
    def run(self):
        """运行GUI"""
        self.root.mainloop()


def main():
    app = ArmControlGUI()
    app.run()


if __name__ == "__main__":
    main()
