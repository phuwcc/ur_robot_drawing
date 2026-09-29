# UR3e Natural-Language Skill Control

Package ROS 2 Humble cho bài thực hành điều khiển UR3e bằng ngôn ngữ tự nhiên,
LLM và skill-based planning. Package này độc lập với `ur_drawing`.

## 1. Tổng quan

Luồng xử lý của hệ thống:

```text
Lệnh người dùng (/user_command)
        ↓
9Router / LLM
        ↓
Phản hồi thô (/llm/raw_response)
        ↓
Trích xuất JSON + strict validator
        ↓
Kế hoạch hợp lệ (/llm/validated_plan)
        ↓
SkillExecutor: pick / place / home
        ↓
MoveIt 2 → UR3e + Robotiq 2F-85 trong Gazebo
```

LLM chỉ được tạo kế hoạch symbolic với ba skill:

```json
{
  "plan": [
    {"skill": "pick", "object": "red_cube"},
    {"skill": "place", "object": "red_cube", "zone": "zone_b"},
    {"skill": "home"}
  ]
}
```

LLM không được cung cấp tọa độ, góc khớp, vận tốc, gia tốc, quỹ đạo, cấu hình
MoveIt hoặc lệnh ROS tùy ý. Các giá trị chuyển động đều lấy từ code và
`config/world.yaml`; validator là lớp quyết định cuối cùng trước khi robot chạy.

## 2. Thành phần chính

- `ur_llm_control/node.py`: nhận lệnh, gọi planner, validate và điều phối thực thi.
- `ur_llm_control/planner.py`: client OpenAI-compatible cho 9Router và mock planner.
- `ur_llm_control/validator.py`: trích xuất JSON và kiểm tra whitelist, schema,
  thứ tự `pick`/`place` và bước `home` cuối cùng.
- `ur_llm_control/skills.py`: triển khai `pick`, `place` và `home`.
- `ur_llm_control/moveit_client.py`: MoveIt action client và Planning Scene.
- `ur_llm_control/moveit_cartesian_client.py`: chuyển động thẳng khi hạ/nâng gripper.
- `ur_llm_control/physical_gripper.py`: điều khiển Robotiq 2F-85 và attach/detach vật.
- `config/world.yaml`: whitelist object/zone, pose và tham số chuyển động.
- `worlds/pick_place.sdf`: bàn, ba cube và ba vùng đặt vật.
- `launch/simulation.launch.py`: Gazebo, robot, controller, MoveIt và RViz.
- `launch/llm_control.launch.py`: planner, validator và skill executor.

## 3. Yêu cầu

- Ubuntu 22.04.
- ROS 2 Humble.
- MoveIt 2, Gazebo Sim và `ros_gz`.
- Source của `Universal_Robots_ROS2_GZ_Simulation` và
  `ros2_robotiq_gripper` trong workspace.
- Node.js 20 trở lên nếu cần cài 9Router trên máy này.
- Một provider/model hoạt động trong 9Router.

## 4. Build và test package

Từ thư mục workspace:

```bash
cd /home/phuc/ur_robot_drawing
source /opt/ros/humble/setup.bash

rosdep install --ignore-src --from-paths . -r -y
colcon build --packages-select robotiq_description ur_llm_control --symlink-install
source install/setup.bash
```

Chạy unit test:

```bash
cd /home/phuc/ur_robot_drawing
source /opt/ros/humble/setup.bash

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python3 -m pytest -q ur_llm_control/test
```

Sau khi sửa code, phải build lại và source lại `install/setup.bash` trong mỗi
terminal ROS 2 đang dùng.

## 5. Thiết lập 9Router

Nếu 9Router của bạn đã chạy tại `http://127.0.0.1:20128` và request
`/v1/chat/completions` đã trả về thành công, có thể bỏ qua mục 5.1 và bắt đầu từ
mục 5.2.

Tài liệu tham khảo chính thức:

- [9Router repository](https://github.com/decolua/9router)
- [Hướng dẫn cài đặt 9Router](https://github.com/decolua/9router/blob/master/gitbook/content/vi/getting-started/installation.md)

### 5.1. Cài và khởi động 9Router

Kiểm tra Node.js:

```bash
node --version
npm --version
```

Nếu chưa có 9Router, cài theo quick start của dự án:

```bash
npm install -g 9router
9router
```

Dashboard mặc định:

```text
http://127.0.0.1:20128
```

Nếu đã cài nhưng dashboard không mở tự động, mở địa chỉ trên bằng trình duyệt.
Không cần đặt source 9Router bên trong ROS workspace.

### 5.2. Kết nối provider và chọn model

Trong dashboard 9Router:

1. Mở phần **Providers**.
2. Kết nối một provider bằng OAuth hoặc API key theo hướng dẫn của provider đó.
3. Chạy chức năng test connection trong dashboard.
4. Mở danh sách model và xác nhận model muốn dùng đang khả dụng.
5. Mở **Settings → API Keys**, tạo hoặc copy API key cho client ROS.

Model đã được kiểm tra với project này:

```text
oc/muse-spark-1.3-contributor-free
```

Model hiển thị trong `/v1/models` chưa chắc đã chạy được nếu provider chưa có
credential hoạt động. Luôn test trực tiếp chat completion trước khi chạy robot.

### 5.3. Export API key cho ROS

Project đọc API key từ biến môi trường `NINE_ROUTER_API_KEY`. Export key trong
đúng terminal sẽ chạy `test_llm` hoặc `ros2 launch`:

```bash
read -s -p "9Router API key: " NINE_ROUTER_API_KEY
echo
export NINE_ROUTER_API_KEY
```

Kiểm tra biến đã tồn tại mà không in giá trị bí mật:

```bash
test -n "$NINE_ROUTER_API_KEY" && echo "API key: SET" || echo "API key: MISSING"
```

Không hardcode hoặc commit API key vào source, YAML, launch file hay README.

### 5.4. Kiểm tra server và model

Kiểm tra server:

```bash
curl -fsS http://127.0.0.1:20128/health
```

Liệt kê model:

```bash
curl -fsS http://127.0.0.1:20128/v1/models \
  -H "Authorization: Bearer $NINE_ROUTER_API_KEY"
```

Kiểm tra đúng endpoint và model mà package sẽ dùng:

```bash
curl -sS http://127.0.0.1:20128/v1/chat/completions \
  -H "Authorization: Bearer $NINE_ROUTER_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "model": "oc/muse-spark-1.3-contributor-free",
    "messages": [
      {"role": "user", "content": "Reply with exactly: OK"}
    ],
    "stream": false
  }'
```

Chỉ chuyển sang bước ROS khi request trả HTTP 200 và nội dung completion không
rỗng.

### 5.5. Tham số kết nối trong ROS

| Launch argument | Giá trị mặc định | Ý nghĩa |
|---|---|---|
| `router_base_url` | `http://127.0.0.1:20128/v1` | OpenAI-compatible base URL |
| `router_model` | `oc/muse-spark-1.3-contributor-free` | Model gửi trong HTTP request |
| `router_api_key_env` | `NINE_ROUTER_API_KEY` | Tên biến môi trường chứa key |
| `router_timeout` | `30.0` | HTTP timeout, đơn vị giây |
| `mock_llm` | `false` | `true`: mock local; `false`: gọi 9Router |

## 6. Kiểm tra riêng tầng LLM

Bước này không cần Gazebo, MoveIt, `ros2_control` hoặc robot. Nó kiểm tra đúng
chuỗi:

```text
command → NineRouterPlanner → raw response → JSON extraction → validator
```

```bash
cd /home/phuc/ur_robot_drawing
source /opt/ros/humble/setup.bash
source install/setup.bash

read -s -p "9Router API key: " NINE_ROUTER_API_KEY
echo
export NINE_ROUTER_API_KEY

ros2 run ur_llm_control test_llm \
  "Hãy đưa khối màu đỏ vào vùng B" \
  --model oc/muse-spark-1.3-contributor-free
```

Kết quả hợp lệ có hai phần:

```text
RAW RESPONSE:
...

VALIDATED PLAN:
...
```

Nếu bước này thất bại, chưa cần khởi động simulation. Sửa kết nối 9Router,
provider, model, API key hoặc phản hồi JSON trước.

## 7. Chạy mock mode để kiểm tra robot

Mock mode không gọi 9Router. Dùng chế độ này để tách lỗi Gazebo/MoveIt/gripper
khỏi lỗi LLM.

### Terminal 1: simulation

```bash
cd /home/phuc/ur_robot_drawing
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur_llm_control simulation.launch.py
```

Chờ robot, controller, `/move_action` và RViz khởi động xong.

### Terminal 2: skill node ở mock mode

```bash
cd /home/phuc/ur_robot_drawing
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur_llm_control llm_control.launch.py \
  student_name:="Dinh Van Phuc" \
  student_id:="23020756" \
  mock_llm:=true
```

### Terminal 3: gửi lệnh

```bash
source /opt/ros/humble/setup.bash
source /home/phuc/ur_robot_drawing/install/setup.bash

ros2 topic pub --once /user_command std_msgs/msg/String \
  "{data: 'Hãy đưa khối màu đỏ vào vùng B'}"
```

## 8. Chạy real LLM mode

Giữ simulation ở Terminal 1. Dừng node mock nếu nó còn chạy, sau đó mở Terminal
2 mới:

```bash
cd /home/phuc/ur_robot_drawing
source /opt/ros/humble/setup.bash
source install/setup.bash

read -s -p "9Router API key: " NINE_ROUTER_API_KEY
echo
export NINE_ROUTER_API_KEY

ros2 launch ur_llm_control llm_control.launch.py \
  student_name:="Dinh Van Phuc" \
  student_id:="23020756" \
  router_model:="oc/muse-spark-1.3-contributor-free" \
  mock_llm:=false
```

Gửi lệnh từ Terminal 3:

```bash
ros2 topic pub --once /user_command std_msgs/msg/String \
  "{data: 'Đưa vật màu đỏ sang vùng B'}"

ros2 topic pub --once /user_command std_msgs/msg/String \
  "{data: 'Hãy lấy khối màu vàng và đặt nó vào ô A'}"

ros2 topic pub --once /user_command std_msgs/msg/String \
  "{data: 'Move the blue cube to zone C'}"
```

Không chạy đồng thời node mock và node real vì cả hai cùng subscribe
`/user_command` và publish các topic chẩn đoán giống nhau.

## 9. Topic và trạng thái chẩn đoán

| Topic | Nội dung |
|---|---|
| `/user_command` | Lệnh ngôn ngữ tự nhiên |
| `/llm/raw_response` | Nội dung thô do LLM trả về |
| `/llm/validated_plan` | JSON đã qua strict validator |
| `/execution_status` | Trạng thái planner và từng bước robot |

Mở ba terminal để quan sát:

```bash
ros2 topic echo /llm/raw_response
```

```bash
ros2 topic echo /llm/validated_plan
```

```bash
ros2 topic echo /execution_status
```

Các nhóm lỗi chính:

| Status | Tầng lỗi | Kiểm tra trước |
|---|---|---|
| `LLM_FAILED` | API key, HTTP, router, provider hoặc response API | Mục 5.4 và mục 6 |
| `INVALID_PLAN` | Không trích được JSON hoặc plan không qua validator | `/llm/raw_response` |
| `PLANNING_FAILED` | MoveIt không tìm được quỹ đạo | pose, collision scene, RViz |
| `EXECUTION_FAILED` | Quỹ đạo/gripper thất bại khi thực thi | controller và action server |
| `GRASP_FAILED` | Gripper không giữ được vật | gripper controller và attach joint |
| `BUSY` | Robot đang chạy plan khác | Chờ plan hiện tại kết thúc |
| `SUCCESS` | Bước hoặc toàn bộ plan thành công | Không cần xử lý |

`INVALID_PLAN` dừng pipeline trước `SkillExecutor`. Lỗi MoveIt hoặc gripper
không được báo thành `LLM_FAILED`.

## 10. Troubleshooting

### `Environment variable NINE_ROUTER_API_KEY is not set.`

Export key trong chính terminal chạy node:

```bash
read -s -p "9Router API key: " NINE_ROUTER_API_KEY
echo
export NINE_ROUTER_API_KEY
```

### `Connection refused` hoặc timeout

```bash
curl -fsS http://127.0.0.1:20128/health
ss -ltn | grep 20128
```

Nếu không có process lắng nghe port `20128`, khởi động lại 9Router.

### HTTP 401 hoặc 403

- Kiểm tra đã export đúng API key chưa.
- Tạo lại client API key trong dashboard nếu cần.
- Kiểm tra credential/OAuth của provider và quota upstream.
- Không đăng API key lên issue, log công khai hoặc Git.

### HTTP 404 hoặc model không tồn tại

```bash
curl -fsS http://127.0.0.1:20128/v1/models \
  -H "Authorization: Bearer $NINE_ROUTER_API_KEY"
```

Copy chính xác model ID từ response và truyền qua `router_model:=...`.

### `No active credentials for provider`

Model có trong danh sách nhưng provider chưa có credential hoạt động. Vào
dashboard, reconnect provider và chạy test connection trước khi gọi lại API.

### HTTP 429 hoặc 503

Provider có thể hết quota, rate-limit hoặc tạm thời không khả dụng. Kiểm tra
response body, `Retry-After`, quota và trạng thái connection trong dashboard.
Đây không phải lỗi JSON parser hay MoveIt.

### `INVALID_PLAN`

Xem phản hồi gốc:

```bash
ros2 topic echo /llm/raw_response
```

Plan phải chỉ có `pick`, `place`, `home`; object/zone phải nằm trong
`config/world.yaml`; `place` phải có `pick` tương ứng và bước cuối phải là
`home`.

### `PLANNING_FAILED`

Tất cả pose nằm trong `config/world.yaml`. Khi robot không đạt được pose:

1. Kiểm tra Planning Scene trong RViz.
2. Kiểm tra `frame_id`, `ee_link` và pose object/zone.
3. Giảm khoảng cách X để mục tiêu gần robot hơn.
4. Giữ tâm cube cao hơn mặt bàn một nửa kích thước cube.
5. Không sửa prompt để LLM sinh tọa độ thay thế.

## 11. Cá nhân hóa bằng MSSV

Node lấy hai chữ số cuối của `student_id`, tính `P = XX mod 6` và in mapping
khi khởi động. Ví dụ với MSSV có hai số cuối là `23`:

```text
23 mod 6 = 5
zone_a = Blue
zone_b = Yellow
zone_c = Red
```

Thay `student_name` và `student_id` trong lệnh launch bằng thông tin thật trước
khi demo. Giá trị mặc định chỉ là placeholder.

## 12. Ghi chú về Robotiq 2F-85

Package dùng Robotiq 2F-85 vì hành trình 85 mm phù hợp với cube 50 mm. Gazebo
và MoveIt dùng cùng mô tả robot ghép; MoveIt điều khiển nhóm
`ur_manipulator`, còn gripper dùng trajectory controller riêng.

`motion.tool_length`, `motion.grasp_height_offset` và
`motion.grasp_orientation` trong `config/world.yaml` xác định TCP và hướng kẹp.
Sau khi gripper đóng, Gazebo `DetachableJoint` gắn cube vào robot; khi thả, joint
được xóa. Không có timer teleport cube.

Tham khảo:

- [Robotiq product catalogue](https://blog.robotiq.com/hubfs/Robotiq%20Kits%20for%20Partners/UR%20Robotiq%20Kit%20for%20Partners/Robotiq%20Essentials/Catalogue/Robotiq%20Product%20Catalogue_2021_10.pdf)
- [Robotiq ROS packages](https://github.com/robotiq/ros)
- [Gazebo DetachableJoint](https://gazebosim.org/api/sim/9/detachablejoints.html)
