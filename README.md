# UR Robot Workspace

Workspace ROS 2 Humble gồm nền tảng Universal Robots, Robotiq 2F-85 và hai
package ứng dụng được quản lý bằng branch riêng.

## Cấu trúc branch

| Branch | Nội dung |
|---|---|
| `main` | Universal Robots, Robotiq, Serial và hướng dẫn workspace |
| `ur_drawing` | Package vẽ chữ P bằng MoveIt 2 |
| `ur_llm_control` | Package điều khiển pick/place bằng Gemini |

Hai branch ứng dụng được gắn vào `main` bằng Git worktree. Sau khi thiết lập,
workspace có cấu trúc:

```text
ur_robot_drawing/
├── Universal_Robots_ROS2_GZ_Simulation/
├── ros2_robotiq_gripper/
├── serial/
├── ur_drawing/          # worktree của branch ur_drawing
└── ur_llm_control/      # worktree của branch ur_llm_control
```

## 1. Clone main và dependency

```bash
git clone --recurse-submodules \
  https://github.com/phuwcc/ur_robot_drawing.git
cd ur_robot_drawing

git submodule update --init --recursive
```

Repo lưu một patch tương thích ROS 2 Humble/Gazebo cho Robotiq. Áp dụng patch
một lần sau khi clone:

```bash
git -C ros2_robotiq_gripper apply \
  ../patches/ros2_robotiq_gripper_humble.patch
```

## 2. Gắn hai branch vào workspace

Tạo local branch theo remote rồi gắn chúng thành hai thư mục con:

```bash
git fetch origin
git branch --track ur_drawing origin/ur_drawing
git branch --track ur_llm_control origin/ur_llm_control

git worktree add ./ur_drawing refs/heads/ur_drawing
git worktree add ./ur_llm_control refs/heads/ur_llm_control
```

Kiểm tra:

```bash
git worktree list
colcon list
```

Nếu local branch đã tồn tại, bỏ qua hai lệnh `git branch --track` và chỉ chạy
hai lệnh `git worktree add`. Dùng cú pháp `refs/heads/...` như trên để Git không
nhầm tên branch với tên thư mục.

## 3. Build workspace

```bash
cd ~/ur_robot_drawing
source /opt/ros/humble/setup.bash

rosdep install --ignore-src --from-paths . -r -y
colcon build --symlink-install
source install/setup.bash
```

Build riêng package khi phát triển:

```bash
colcon build --packages-select ur_drawing --symlink-install
colcon build --packages-select ur_llm_control --symlink-install
```

Hướng dẫn chạy chi tiết nằm trong README của từng worktree:

- `ur_drawing/README.md`
- `ur_llm_control/README.md`

## 4. Làm việc với từng branch

Mỗi thư mục ứng dụng là một Git worktree độc lập:

```bash
cd ~/ur_robot_drawing/ur_drawing
git status
git add -A
git commit -m "Update drawing"

cd ~/ur_robot_drawing/ur_llm_control
git status
git add -A
git commit -m "Update LLM control"
```

Không checkout `ur_drawing` hoặc `ur_llm_control` trực tiếp trong thư mục main.
Chuyển branch và commit ngay trong worktree tương ứng.

## 5. Đẩy ba branch lên GitHub

Sau khi kiểm tra nội dung local:

```bash
cd ~/ur_robot_drawing
git push origin main
git push -u origin ur_drawing
git push -u origin ur_llm_control
```
