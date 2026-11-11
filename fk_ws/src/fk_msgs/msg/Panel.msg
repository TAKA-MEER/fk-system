# 正本は docs/plan/detailed/DetailedDesign-names.md §4.1。
string panel_id
float64 x                # 天面の中心。map フレーム [m]
float64 y
float64 front_yaw        # 正面の向き。盤から外向き [rad]
float32 width_m
float32 depth_m
float32 top_height_m
float32 pos_std_m
float32 yaw_std_deg
int32 lugs               # ss の確からしさ。使った吊り金具の数
bool confident           # しきい値を満たすか
bool ss_front_known      # ss が「正面が分かっている」と返した
float64 ss_front_yaw     # そのときの ss の正面の向き。参考
bool front_doubt         # ss の正面と、機体に向いていた辺が違う
string row_id
