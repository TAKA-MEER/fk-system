# 正本は docs/plan/detailed/DetailedDesign-names.md §4.1。
std_msgs/Header header
string map_instance_id
Panel[] panels
float32[] unseen_from_rad   # 見ていない方向。map フレーム
float32[] unseen_to_rad
string survey_reason     # 配置図が空・途中で終わった理由。ss が返した終わり方のまま（completed は空）。結果が来ない・使えないときは ss_failed
