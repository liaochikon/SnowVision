import snowvision as sv

camera_count = 4
camera_name_list = [10, 1, 2, 3, 4]
width = 1280
height = 720
fps = 30
wait_sec = 1

sv.RecordVideo("light_long", [0, -10, 1, 2, 3], [800, 450], 30, 1)