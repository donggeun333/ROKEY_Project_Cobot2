import trimesh

# 파일 경로 지정
file_path = "/home/dg/cobot_ws/data/alpha_shape/good_multitap_smoothing_alpha0p2_iter2_alpha_0p010.ply"

# PLY 메쉬 로드
mesh = trimesh.load(file_path)

# 정보 출력 및 뷰어 실행
print(mesh)
mesh.show()
