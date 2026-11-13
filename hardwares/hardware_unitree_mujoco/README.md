# Hardware Unitree Mujoco

This package contains the hardware interface based on [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) to control the Unitree robot in Mujoco simulator. 

In theory, it also can communicate with real robot, but it is not tested yet. You can use go2 simulation in [unitree_mujoco](https://github.com/legubiao/unitree_mujoco). In this simulation, I add foot force sensor support.

*[x] **[2025-01-16]** Add odometer states for simulation. 

## 1. Interfaces

Required hardware interfaces:

* command:
  * joint position
  * joint velocity
  * joint effort
  * KP
  * KD
* state:
  * joint effort
  * joint position
  * joint velocity
  * imu sensor
    * linear acceleration
    * angular velocity
    * orientation
  * foot force sensor

## 2. Build

Tested environment:
* Ubuntu 24.04
    * ROS2 Jazzy
* Ubuntu 22.04
    * ROS2 Humble

Build Command:
```bash
cd ~/ros2_ws
colcon build --packages-up-to hardware_unitree_mujoco --symlink-install
```

## 3. Config network and domain
Since the real unitree robot has different network and domain name, you need to set the network and domain name in the xacro file.
```xml
<hardware>
    <plugin>hardware_unitree_mujoco/HardwareUnitree</plugin>
    <param name="domain">1</param>
    <param name="network_interface">lo</param>
</hardware>
```

After modified the config, you can tried to visualize the robot info from real robot by following command:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch hardware_unitree_mujoco visualize.launch.py
```
## 4. Low-level safety（ローレベル安全対策）

`include/hardware_unitree_mujoco/LowLevelSafety.h`（ROS に依存しないヘッダのみのクラス）を
`read()` / `write()` から毎周期呼ぶ．動的メモリ確保と例外は使わない．コントローラに依存しないので，
ocs2 / unitree_guide / rl のどのコントローラでも同じように効く．

### 4.1 毎周期の指令の制限（write()）

1. q, dq, kp, kd, tau のどれかが有限でない → 停止（ダンピング，ラッチ）
2. kp を [0, kp_max]，kd を [0, kd_max] に制限
3. q_cmd を [q_min + position_command_margin, q_max − position_command_margin] に制限
4. モータが出すトルクの合計 `tau + kp (q_cmd − q) + kd (dq_cmd − dq)` を ±(torque_max × torque_limit_scale) に
   収まるように tau を変える（q, dq は最新の LowState）．何回制限したかを 5 s ごとに INFO で出す

### 4.2 状態の監視（異常ならダンピングにしてラッチ）

| 監視 | 条件 |
|---|---|
| 実測トルク | \|tau_est\| > tau_est_ratio × 有効トルク上限 が tau_est_duration 続いた |
| 関節角 | q < q_min − position_monitor_tolerance または q > q_max + position_monitor_tolerance |
| 関節角速度 | \|dq\| > velocity_ratio × velocity_max |
| モータ温度 | temperature ≥ temperature_warn で WARN（関節ごとに 1 回）．temperature_stop は**既定で無効**（下記） |
| 指令のウォッチドッグ | command_timeout の間，どの command interface も書かれなかった（最初の指令の後から有効） |
| LowState | lowstate_timeout の間 rt/lowstate を受信しない（最初の受信の後から有効） |
| リモコン | wireless_remote のボタンが estop_buttons をすべて含む（既定 L2 + B） |
| 制御ループの停止 | loop_stall_timeout の間 write() が呼ばれない（別スレッドで監視し，そのスレッドからダンピングを送る） |
| 状態の非有限値 | LowState の q, dq, tau_est が NaN / Inf |
| LowState の CRC | CRC が一致しないメッセージは使わない．lowstate_crc_max_failures 回連続で不一致なら停止 |

**温度による停止について**：MotorState_.temperature の単位と Go2 のモータの許容温度は，Unitree の資料でまだ確認できていない
（IDL では uint8 で，°C と仮定）．そのため既定では警告だけにしている（`temperature_stop` = `none`）．
Unitree の仕様が確認できたら，ros2_control.xacro の `temperature_stop` に停止温度を設定して有効にすること．

**CRC について**：送信側（unitree_sdk2 の dds_wrapper `go2_pub.h`，unitree_mujoco もこれを使う）と同じく，
`crc32_core(メッセージ, sizeof(LowState_)/4 − 1)` と `crc` を比べる．unitree_mujoco では不一致 0 件を確認した．
CRC を付けないシミュレータなどでは `lowstate_crc_check` を false にする．

- **ダンピング**：全関節 tau = 0, kp = 0, kd = damping_kd, q = dq = 0．一度止まると自動では戻らない．
  解除は ros2_control_node の再起動のみ．
- 止めた理由（どの関節の何が，いくつで，しきい値はいくつか）を 1 回だけ `RCLCPP_ERROR` で出す．
- 最初の LowState を受信するまでは，ラッチせずにダンピングを送る（コントローラが 0 の状態で計算した指令を送らない）．
- ハードウェアの deactivate / shutdown のときもダンピングを送る．

### 4.3 ウォッチドッグの仕組み

`write()` の最後に，command interface のバッファを特別な NaN（payload が `0x7ff85afec0de0001`）で埋める．
次の `write()` でこの値のままのバッファは「この周期にコントローラが書かなかった」と判定し，前回の値を使う
（500 Hz のコントローラを 1 kHz の controller_manager で動かしても問題ない）．コントローラが計算で作った NaN は
payload が違うので，非有限の指令として検出される．

そのため **コントローラは update() のたびに少なくとも 1 つの command interface を書く必要がある**．
controller_common の StatePassive と ocs2 の StateOCS2（MPC の起動前）は，このために毎周期書くように変更した．

### 4.4 安全停止の状態

次の state interface を出す．

| interface | 内容 |
|---|---|
| safety/stop | 1 = 停止中（ラッチ） |
| safety/reason | 停止の理由のコード（下記） |
| safety/torque_limit_count_&lt;joint&gt; | トルク合計の制限で tau を変えた周期数（関節ごと，起動からの累積）．&lt;joint&gt; は FR_hip … RL_calf |
| safety/lowstate_crc_errors | CRC 不一致で捨てた LowState の数（累積） |

- joint_state_broadcaster が `/dynamic_joint_states` に出すので，`ros2 topic echo /dynamic_joint_states` で見える．
- ocs2_quadruped_controller は `hardware_safety_interface: "safety/stop"`（robot_control.yaml）で読み，
  停止中は PASSIVE に移ってそこに留まる．

reason のコード：0 なし，1 指令が非有限，2 状態が非有限，3 tau_est の持続超過，4 関節角が範囲外，
5 関節角速度，6 モータ温度，7 指令のウォッチドッグ，8 LowState のタイムアウト，9 リモコンの非常停止，10 制御ループの停止，11 LowState の CRC の連続不一致

### 4.5 パラメータ

ros2_control.xacro の `<hardware>` の `<param>` で設定する．書かなかったパラメータは下の既定値（Go2）になる．

関節の上限は Unitree 公式の Go2 URDF（unitreerobotics/unitree_ros の
`robots/go2_description/urdf/go2_description.urdf`．unitree_mujoco の `go2.xml` と同じ値）による．
go2_description では const.xacro にこの値があり（後脚の thigh は `rear_thigh_position_*`），
ros2_control.xacro の安全用パラメータは `${...}` で const.xacro を参照する．

OCS2 の WBC のトルク上限（`config/ocs2/task.info` の `torqueLimitsTask`）は，安全層の有効上限
（*_torque_max × torque_limit_scale：hip/thigh 16.59，calf 31.80 Nm）と同じ値にしてある．
torque_limit_scale や const.xacro を変えたら task.info も手で合わせること．

`<name>` は 4 本の脚すべてに設定し，`front_<name>`（FR, FL）と `rear_<name>`（RR, RL）で上書きできる
（例：`rear_thigh_position_min`）．関節の上限のパラメータ（*_torque_max，*_velocity_max，*_position_min/max）すべてで使える．

| パラメータ | 既定値 | 説明 |
|---|---|---|
| safety_enabled | true | false で安全対策を無効化（指令をそのまま送る） |
| torque_limit_scale | 0.7 | 有効トルク上限 = *_torque_max × これ（hip/thigh 16.59，calf 31.80 Nm） |
| hip_torque_max / thigh_torque_max / calf_torque_max | 23.7 / 23.7 / 45.43 Nm | Go2 URDF |
| hip_velocity_max / thigh_velocity_max / calf_velocity_max | 30.1 / 30.1 / 15.70 rad/s | Go2 URDF |
| hip_position_min, max | −1.0472, 1.0472 rad | Go2 URDF |
| front_thigh_position_min, max | −1.5708, 3.4907 rad | Go2 URDF（前脚） |
| rear_thigh_position_min, max | −0.5236, 4.5379 rad | Go2 URDF（後脚） |
| calf_position_min, max | −2.7227, −0.83776 rad | Go2 URDF |
| position_command_margin | 0.05 rad | q_cmd を可動域のこれだけ内側に制限 |
| position_monitor_tolerance | 0.05 rad | 実測 q が可動域をこれだけ超えたら停止 |
| kp_max / kd_max | 200 / 10 | |
| velocity_ratio | 0.8 | |
| tau_est_ratio / tau_est_duration | 0.9 / 0.2 s | |
| temperature_warn | 70 | MotorState_.temperature（単位は未確認，°C と仮定） |
| temperature_stop | none（無効） | 数値を書くと有効 |
| command_timeout | 0.02 s | |
| lowstate_timeout | 0.05 s | |
| lowstate_crc_check | true | false で CRC を確認しない |
| lowstate_crc_max_failures | 10 | 連続不一致の回数．0 で停止しない（捨てるだけ） |
| loop_stall_timeout | 0.1 s | OCS2 状態に入るときの初期方策待ち（制御ループを 15〜40 ms 止める）より長くする |
| estop_buttons | L2+B | R1 L1 Start Select R2 L2 F1 F2 A B X Y up right down left を + でつなぐ．none で無効 |
| damping_kd | 3.0 | |

wireless_remote のバイト配置は unitree_sdk2 の `include/unitree/dds_wrapper/common/unitree_joystick.hpp`
（`REMOTE_DATA_RX`）による：byte 0–1 head，byte 2–3 ボタン（uint16，bit 0 から R1, L1, Start, Select, R2, L2, f1, f2,
A, B, X, Y, up, right, down, left）．

### 4.6 単体テスト

```bash
cd ~/go2_mpc_ws
colcon build --packages-select hardware_unitree_mujoco --symlink-install
colcon test --packages-select hardware_unitree_mujoco --ctest-args -R test_low_level_safety
colcon test-result --test-result-base build/hardware_unitree_mujoco
# または直接： ./build/hardware_unitree_mujoco/test_low_level_safety
```

### 4.7 MuJoCo での確認手順

各ターミナルで：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=/opt/openrobots/lib:/opt/unitree_robotics/lib:$LD_LIBRARY_PATH
source ~/go2_mpc_ws/install/setup.bash
```

起動時のログに `[LowLevelSafety] torque limit (scale 0.70) hip 16.59 thigh 16.59 calf 31.80 Nm ...` と脚ごとの上限（`FR: q hip [...]`）が出ることを確認する．

1. **通常の歩行で誤停止しないこと**
   ```bash
   # ターミナル 1
   cd ~/unitree_mujoco/simulate/build && ./unitree_mujoco -r go2
   # ターミナル 2
   ros2 launch go2_stride_test stride_tracker_test.launch.py stride_length:=0.40
   ```
   `SAFETY STOP` が出ず，`go2_trajectory_tracker: stop: goal reached` まで進むこと．v = 0.8 m/s では歩幅 0.3〜0.7 m で確認済み（0.8 m は calf の角速度の監視で停止する）．
   やり直す前に MuJoCo の Load key で初期姿勢に戻すか，unitree_mujoco を起動し直す．

2. **コントローラを止めたときにウォッチドッグで停止すること**（1 の歩行中に）
   ```bash
   ros2 control switch_controllers --deactivate ocs2_quadruped_controller
   ```
   `SAFETY STOP ... command watchdog timeout: value 0.02xx, limit 0.0200` が出て，ダンピングになること．`ros2 topic echo --once /dynamic_joint_states` の `safety` が `stop: 1.0, reason: 7.0` になること．

3. **ロボットを押し倒したときにコントローラ層の姿勢チェックで PASSIVE になること**（1 の歩行中か立っているときに）
   MuJoCo のウィンドウでロボットの胴体をダブルクリックして選択し，Ctrl + 右ドラッグで横から力を加えて倒す．
   `[SafetyChecker] Orientation safety check failed: roll ... deg` と `Switched from OCS2 State to passive` が出ること．
   高さのチェックは `[SafetyChecker] Base height safety check failed` が出る．
   しきい値は robot_control.yaml の `ocs2_quadruped_controller.safety.*`．

   SafetyChecker による停止はラッチされる：`SafetyChecker stop LATCHED` が出て，
   `ros2 topic echo /ocs2_quadruped_controller/safety_latched` が `data: true` になり，command 2 を送っても OCS2 に入らない
   （`ignoring command 2` の WARN）．command 1 を一度送ると解除される：
   ```bash
   ros2 topic pub -t 5 /control_input control_input_msgs/msg/Inputs "{command: 1}"
   ```
