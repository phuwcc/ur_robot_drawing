# Bài 03 — UR3e + Robotiq + Camera + LLM Skill Planning

Package ROS 2 Humble/Gazebo Fortress độc lập, phát triển từ `ur_llm_control`.

```text
/user_command → Gemini → validator cú pháp
                              ↓
/camera/image → nhận diện → trạng thái bàn → giải phóng zone → validator kế hoạch
                                                            ↓
                            camera xác nhận ← Gazebo ← MoveIt 2 ← skills
```

## Build

Workspace đang dùng nằm tại `/home/phuc/ur_robot_drawing`.
Dùng đường dẫn tuyệt đối dưới đây, kể cả khi terminal đang mở trong workspace.

Từ thư mục gốc workspace, áp dụng patch Robotiq và cài dependency như README gốc.

```bash
cd /home/phuc/ur_robot_drawing
source /opt/ros/humble/setup.bash
rosdep install --ignore-src --from-paths ur_llm_camera -r -y
colcon build --packages-select ur_llm_camera --symlink-install
source install/setup.bash
```

## Demo zone bị chiếm

Terminal 1: 

```bash
cd ~/ur_robot_drawing
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur_llm_camera simulation.launch.py
```

Terminal 2 (chờ MoveIt và các controller active):

```bash
cd ~/ur_robot_drawing
source /opt/ros/humble/setup.bash
source install/setup.bash
export GEMINI_API_KEY='YOUR_KEY'
ros2 launch ur_llm_camera llm_control.launch.py \
  student_name:="Dinh Van Phuc" student_id:="23020756"
```
 
Terminal 3:

```bash
cd ~/ur_robot_drawing
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic pub --once /user_command std_msgs/msg/String \
  "{data: 'Put the red cube in Zone B.'}"
```

World ban đầu có blue_cube trong zone_b, bốn block còn lại trên bàn. Camera
phải nhận đủ 5 block mới cho phép lập kế hoạch. Kế hoạch mở rộng mong đợi:

```json
{"plan":[
  {"skill":"pick","object":"blue_cube"},
  {"skill":"place","object":"blue_cube","zone":"temporary_3"},
  {"skill":"pick","object":"red_cube"},
  {"skill":"place","object":"red_cube","zone":"zone_b"},
  {"skill":"home"}
]}
```

`temporary_3` được tính từ ảnh, kích thước block, biên bàn, khoảng hở và vùng
làm việc; không phải pose cố định. Các điểm tạm còn phải qua MoveIt IK
ở cả độ cao tiếp cận và độ cao thả với collision checking. Nếu không tìm được chỗ trống, robot dừng.
Sau mỗi `place`, robot lùi lên rồi đi thẳng tới vật tiếp theo. Camera xác nhận
các vật vừa thả ngay khi chúng hiện lại trong ảnh; nếu cánh tay vẫn che, kết quả
được giữ ở trạng thái chờ thay vì dừng chuỗi. Sau bước `home` cuối cùng, camera
bắt buộc phải thấy và xác nhận tất cả kết quả còn chờ. Khi chạy lại, trạng thái
mới tiếp tục lấy từ ảnh.
Trong chuyển tiếp, tối đa hai vật đã định danh có thể tạm bị che: vật vừa thả và
vật sắp gắp. Nếu một vật không liên quan cũng biến mất khỏi ảnh, robot vẫn dừng.

## Camera và hiệu chuẩn

- Camera RGB thật trong Gazebo, 800×800, 10 Hz; không đọc model pose/topic pose.
- OpenCV tách màu HSV, kiểm tra diện tích/hình dạng, loại màu xuất hiện nhiều
  ứng viên. Zone dùng màu xám để không bị nhận nhầm là block.
- Camera đặt tại `[0.43, 0, 1.3]`, pitch `pi/2`, yaw 0 trong world/base_link.
  Chiếu pinhole pixel lên mặt trên block, rồi suy ra tâm block bằng kích thước
  đã biết. Đây là hiệu chuẩn hình học cho bàn phẳng và cube nằm thẳng, không
  phải hệ thống 6D pose cho vật nghiêng, chồng lên nhau hoặc camera tùy ý.
- `config/world.yaml` chỉ chứa kích thước block, hình học bàn/zone, hiệu chuẩn
  camera và giới hạn robot. Pose spawn chỉ có trong SDF, không dùng để lập kế hoạch.
- Mất ảnh, ảnh cũ, thiếu block, nhiều ứng viên cùng màu hoặc chưa ổn định đều
  chặn chuyển động. Vật bị che không được coi là zone trống.
- Nếu đổi pose/FOV/resolution camera, sửa đồng thời SDF và cấu hình calibration.
  Nếu đổi kích thước bàn/cube, sửa cả hai. Không dùng cấu hình này cho robot thật.

Theo dõi:

```bash
ros2 topic echo /perception/world_state
ros2 topic echo /llm/validated_plan
ros2 topic echo /execution_status
# Nếu đã cài rqt_image_view:
ros2 run rqt_image_view rqt_image_view /camera/detections
```

`zones` liệt kê block có footprint xâm phạm vùng đặt (có margin). Danh sách rỗng
chỉ có ý nghĩa khi `complete=true` và ảnh còn mới. `objects` là tọa độ đo từ ảnh.

## Gripper và collision checking

Robotiq 2F-85 có controller thật trong mô phỏng. Khi đã đến pose gắp đo bởi
camera và FK xác nhận vị trí/hướng, robot đóng ngón, tạo fixed joint bằng
Gazebo DetachableJoint, nâng và di chuyển block. Khi thả, ngón mở, joint được
tháo ở độ cao 1 cm trên mặt đặt và block rơi xuống bàn bằng vật lý Gazebo.
Khoảng hở này tránh ép cube xuyên mặt bàn khi có sai số camera/IK. Không gọi set_pose hoặc teleport object.
Đây là grasp có hỗ trợ joint, không phải mô hình ma sát ngón tay thuần túy.
Với cube 50 mm, controller đóng tới 0.35 rad (khẩu độ mesh khoảng
50.7 mm), không ép tới trạng thái đóng kín. Collision object khi gắp giữ
nguyên pose camera trong frame tool bằng FK thực tế tại thời điểm attach.

Fortress trong workspace khởi tạo các detachable joint ở trạng thái attached.
Executor tháo cả năm và nhận ACK **trước khi điều khiển gripper/robot**, rồi mới
lấy ảnh ổn định, dựng planning scene và về home. Khi mất ACK, không tự toggle sang attach để tránh gắn vật từ xa.
Nếu node hoặc thao tác gặp lỗi, khởi động lại cả simulation và node trước demo tiếp.

MoveIt giữ collision box cho bàn và toàn bộ block đo được. Chỉ cho phép contact
của block đang gắp với touch links của gripper; các vật khác vẫn được collision
check. Khi giữ, collision object gắn với tool; khi thả, nó trở lại world trước
khi robot rút lên. Cartesian path và fallback pose planning đều kiểm tra va chạm. Đoạn Cartesian
có bước nhảy joint trên 0.15 rad bị loại trước khi thực thi để tránh nhảy nhánh IK.

## LLM, validator và skill

- `planner.py`: Gemini chỉ sinh `pick(object)`, `place(object, zone)`, `home()`.
  Camera state được đưa vào context; LLM không được sinh pose, joint hay trajectory.
- `validator.py`: whitelist skill/object/zone, cấm khóa thừa, kiểm tra cầm/thả và
  home cuối kế hoạch.
- `scene.py`: mô phỏng trạng thái qua từng bước, tìm các vật cản, chọn vị trí tạm,
  chèn transfer và validate lại toàn bộ chuỗi trước khi chạy.
- `skills.py`: kiểm tra camera/occupancy trước transfer và trước place, cập nhật
  planning scene, thực thi và dùng camera xác nhận kết quả.
- `perception.py`, `camera_state.py`: nhận diện, timestamp, kiểm tra ảnh ổn định.

Khi scene đổi trong lúc Gemini trả lời, executor lấy ảnh mới và mở rộng kế hoạch
lại trước khi thực thi. Khi scene đổi trong lúc robot đang chạy, robot dừng nếu
precondition không còn đúng; không tự tiếp tục với pose cũ. Workspace demo giả
định không có người hoặc vật mới đi vào bàn giữa một đoạn chuyển động.

## Nội dung video và báo cáo

1. Hiển thị đủ UR3e, gripper, camera, bàn, 3 zone và 5 block; blue_cube trong B.
2. Hiển thị ảnh detection và world_state; giải thích hiệu chuẩn pixel → tọa độ.
3. Gửi lệnh, hiển thị raw LLM plan và validated expanded plan.
4. Quay liên tục blue_cube → chỗ tạm, red_cube → B, robot về home.
5. Hiển thị `PLACEMENT_VERIFIED`/`SUCCESS` và trạng thái camera cuối.
6. Trình bày xử lý ảnh thiếu/cũ, hết chỗ, plan sai, lỗi MoveIt hoặc gripper.
7. Ghi rõ giới hạn nhận diện màu và grasp hỗ trợ fixed joint; nộp link GitHub,
   video thực tế và báo cáo. Package không tự tạo video hoặc đẩy repository.

Tài liệu Gazebo: [Sensors Fortress](https://gazebosim.org/docs/fortress/sensors/),
[DetachableJoint Fortress](https://github.com/gazebosim/gz-sim/tree/ign-gazebo6/src/systems/detachable_joint).
